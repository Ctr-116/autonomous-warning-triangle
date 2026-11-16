// Native ROS2 Humble PCL mapper port. Pixel projection, voxelization and
// orientation-preserving accumulation follow the original algorithm. GPL-3.0-or-later.
#include "yahboomcar_slam/PCLMapper.h"

#include <chrono>
#include <cmath>
#include <limits>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2_ros/transform_listener.h>

namespace yahboomcar_slam
{

namespace
{
bool finitePose(const geometry_msgs::msg::Pose & p)
{
  const double vals[7] = {
    p.position.x, p.position.y, p.position.z,
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w};
  for (double v : vals) {
    if (!std::isfinite(v)) {return false;}
  }
  const double qn = p.orientation.x * p.orientation.x + p.orientation.y * p.orientation.y +
    p.orientation.z * p.orientation.z + p.orientation.w * p.orientation.w;
  return qn > 1e-12;
}

bool validIntrinsics(double fx, double fy)
{
  return std::isfinite(fx) && std::isfinite(fy) && fx > 0.0 && fy > 0.0;
}
}  // namespace

PCLMapper::PCLMapper(rclcpp::Node * node, const MapperConfig & cfg)
: node_(node), cfg_(cfg)
{
  global_map_ = PointCloud::Ptr(new PointCloud());
  local_map_ = PointCloud::Ptr(new PointCloud());
  if (cfg_.resolution <= 0.0f) {cfg_.resolution = 0.04f;}
  voxel_.setLeafSize(cfg_.resolution, cfg_.resolution, cfg_.resolution);
  if (cfg_.pixel_step < 1) {cfg_.pixel_step = 1;}
  if (cfg_.max_points < 1) {cfg_.max_points = 1;}
}

PCLMapper::~PCLMapper() = default;

void PCLMapper::initializeSubscriptions()
{
  auto sensor_qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();

  color_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
    node_, cfg_.color_topic, sensor_qos);
  depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
    node_, cfg_.depth_topic, sensor_qos);
  info_filter_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(
    node_, cfg_.camera_info_topic, sensor_qos);
  pose_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(
    node_, cfg_.pose_topic, sensor_qos);

  using Policy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo,
    geometry_msgs::msg::PoseStamped>;
  sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(
    Policy(static_cast<uint32_t>(cfg_.sync_queue)), *color_sub_, *depth_sub_,
    *info_filter_sub_, *pose_sub_);
  sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(cfg_.sync_slop));
  sync_->registerCallback(
    std::bind(&PCLMapper::colorDepthCameraInfoCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
}

void PCLMapper::initializePublishersAndServices()
{
  pub_global_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "Global/PointCloudOutput", rclcpp::QoS(1).transient_local());
  pub_local_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "Local/PointCloudOutput", rclcpp::QoS(10));
  clear_srv_ = node_->create_service<std_srvs::srv::Empty>(
    "clear_map",
    [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {
      this->resetMap();
      RCLCPP_INFO(node_->get_logger(), "Map cleared by service request");
    });
}

bool PCLMapper::validatePose(const geometry_msgs::msg::PoseStamped & pose) const
{
  if (!finitePose(pose.pose)) {return false;}
  if (pose.header.frame_id.empty()) {return false;}
  if (pose.header.stamp.sec == 0 && pose.header.stamp.nanosec == 0) {return false;}
  return true;
}

bool PCLMapper::validateAndConvertColor(const cv::Mat & raw, cv::Mat & out_bgr)
{
  if (raw.empty()) {return false;}
  if (raw.type() == CV_8UC3) {
    out_bgr = raw.clone();
    return true;
  }
  if (raw.type() == CV_8UC1) {
    cv::cvtColor(raw, out_bgr, cv::COLOR_GRAY2BGR);
    return true;
  }
  if (raw.type() == CV_16UC1) {
    cv::Mat tmp;
    raw.convertTo(tmp, CV_8U, 0.02);
    cv::cvtColor(tmp, out_bgr, cv::COLOR_GRAY2BGR);
    return true;
  }
  return false;
}

bool PCLMapper::validateAndConvertDepth(const cv::Mat & raw, cv::Mat & out_float_m)
{
  if (raw.empty()) {return false;}
  if (cfg_.depth_unit == 1) {
    if (raw.type() != CV_16UC1) {return false;}
    // Forced 16U depth requires a strictly positive, finite scale divisor.
    if (!std::isfinite(cfg_.depth_scale) || cfg_.depth_scale <= 0.0f) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "Rejecting depth: depth_unit=1 requires finite positive depth_scale (got %f)",
        static_cast<double>(cfg_.depth_scale));
      return false;
    }
    raw.convertTo(out_float_m, CV_32F, 1.0 / static_cast<double>(cfg_.depth_scale));
    return true;
  }
  if (cfg_.depth_unit == 2) {
    if (raw.type() != CV_32FC1) {return false;}
    raw.convertTo(out_float_m, CV_32F, 1.0);
    return true;
  }
  // Auto: 16UC1 (mm) divided by depth_scale, 32FC1 assumed metres.
  if (raw.type() == CV_16UC1) {
    double scale = 1000.0;
    if (std::isfinite(cfg_.depth_scale) && cfg_.depth_scale > 0.0f) {
      scale = static_cast<double>(cfg_.depth_scale);
    }
    raw.convertTo(out_float_m, CV_32F, 1.0 / scale);
    return true;
  }
  if (raw.type() == CV_32FC1) {
    raw.convertTo(out_float_m, CV_32F, 1.0);
    return true;
  }
  return false;
}

Eigen::Isometry3d PCLMapper::convertPose(const geometry_msgs::msg::PoseStamped & pose) const
{
  Eigen::Quaterniond q(
    pose.pose.orientation.w, pose.pose.orientation.x,
    pose.pose.orientation.y, pose.pose.orientation.z);
  q.normalize();
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = q.toRotationMatrix();
  T.translation() = Eigen::Vector3d(
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  // Contract: this method always returns Tcw (world->camera).
  // If the configured input is already Tcw, return it as-is. If the input is
  // Twc (camera->world), invert it exactly once to obtain Tcw.
  if (cfg_.input_pose_is_twc) {
    T = T.inverse();
  }
  return T;
}

void PCLMapper::insertKeyFrame(const cv::Mat & color, const cv::Mat & depth, const Eigen::Isometry3d & Tcw)
{
  std::unique_lock<std::mutex> lck(mutex_);
  if (pending_.size() >= static_cast<std::size_t>(cfg_.max_buffer)) {
    pending_.erase(pending_.begin());
  }
  PendingFrame f;
  if (!validateAndConvertColor(color, f.color)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "Rejecting keyframe: unsupported color image");
    return;
  }
  if (!validateAndConvertDepth(depth, f.depth)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "Rejecting keyframe: unsupported depth image/unit");
    return;
  }
  // Contract: stored Tcw is always world->camera.
  f.Tcw = Tcw;
  pending_.push_back(std::move(f));
}

PCLMapper::PointCloud::Ptr PCLMapper::generatePointCloud(
  const cv::Mat & color, const cv::Mat & depth, const Eigen::Isometry3d & T_wc)
{
  PointCloud::Ptr tmp(new PointCloud());
  const int step = cfg_.pixel_step;
  const float fx = static_cast<float>(cfg_.fx);
  const float fy = static_cast<float>(cfg_.fy);
  const float cx = static_cast<float>(cfg_.cx);
  const float cy = static_cast<float>(cfg_.cy);

  if (!validIntrinsics(cfg_.fx, cfg_.fy)) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "Invalid camera intrinsics fx=%f fy=%f; skipping point cloud generation",
      cfg_.fx, cfg_.fy);
    return tmp;
  }

  const int rows = std::min(color.rows, depth.rows);
  const int cols = std::min(color.cols, depth.cols);
  for (int m = 0; m < rows; m += step) {
    const float * drow = depth.ptr<float>(m);
    const uchar * crow = color.ptr<uchar>(m);
    for (int n = 0; n < cols; n += step) {
      const float d = drow[n];
      if (!std::isfinite(d) || d < cfg_.min_depth || d > cfg_.max_depth) {
        continue;
      }
      PointT p;
      p.z = d;
      p.x = (static_cast<float>(n) - cx) * p.z / fx;
      p.y = (static_cast<float>(m) - cy) * p.z / fy;
      p.b = crow[n * 3];
      p.g = crow[n * 3 + 1];
      p.r = crow[n * 3 + 2];
      p.a = 255;
      tmp->points.push_back(p);
    }
  }
  tmp->width = static_cast<uint32_t>(tmp->points.size());
  tmp->height = 1;
  tmp->is_dense = false;

  if (tmp->points.empty()) {
    return tmp;
  }

  PointCloud::Ptr filtered(new PointCloud());
  voxel_.setInputCloud(tmp);
  voxel_.setLeafSize(cfg_.resolution, cfg_.resolution, cfg_.resolution);
  voxel_.filter(*filtered);

  PointCloud::Ptr transformed(new PointCloud());
  const Eigen::Matrix4f Tf = T_wc.matrix().cast<float>();
  pcl::transformPointCloud(*filtered, *transformed, Tf);
  return transformed;
}

void PCLMapper::processPending()
{
  std::vector<PendingFrame> batch;
  {
    std::unique_lock<std::mutex> lck(mutex_);
    if (pending_.empty()) {
      return;
    }
    batch.swap(pending_);
  }

  const auto now = node_->now();
  for (auto & f : batch) {
    if (f.color.empty() || f.depth.empty()) {
      continue;
    }
    // Contract: f.Tcw is world->camera; generatePointCloud expects Twc.
    // Invert exactly once here.
    const Eigen::Isometry3d T_wc = f.Tcw.inverse();
    PointCloud::Ptr cloud = generatePointCloud(f.color, f.depth, T_wc);
    if (!cloud || cloud->points.empty()) {
      continue;
    }
    {
      std::unique_lock<std::mutex> lck(mutex_);
      *global_map_ += *cloud;

      sensor_msgs::msg::PointCloud2 local_msg;
      pcl::toROSMsg(*cloud, local_msg);
      local_msg.header.stamp = now;
      local_msg.header.frame_id = cfg_.map_frame;
      pub_local_->publish(local_msg);

      if (global_map_->points.size() > cfg_.max_points) {
        PointCloud::Ptr bounded(new PointCloud());
        voxel_.setInputCloud(global_map_);
        voxel_.filter(*bounded);
        global_map_.swap(bounded);
        if (global_map_->points.size() > cfg_.max_points) {
          global_map_->points.resize(cfg_.max_points);
          global_map_->width = static_cast<uint32_t>(global_map_->points.size());
          global_map_->height = 1;
        }
      }
    }
  }

  {
    std::unique_lock<std::mutex> lck(mutex_);
    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(*global_map_, out);
    out.header.stamp = now;
    out.header.frame_id = cfg_.map_frame;
    pub_global_->publish(out);
    last_keyframe_size_ = global_map_->points.size();
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "Global map size=%zu", global_map_->points.size());
  }
}

void PCLMapper::resetMap()
{
  {
    std::unique_lock<std::mutex> lck(mutex_);
    global_map_->clear();
    local_map_->clear();
    pending_.clear();
    last_keyframe_size_ = 0;
  }

  // Publish a valid empty global map to flush the TRANSIENT_LOCAL latched
  // publisher so late-joining subscribers do not receive the old map.
  if (pub_global_) {
    sensor_msgs::msg::PointCloud2 empty_global;
    pcl::toROSMsg(*global_map_, empty_global);
    empty_global.header.stamp = node_->now();
    empty_global.header.frame_id = cfg_.map_frame;
    pub_global_->publish(empty_global);
  }
  if (pub_local_) {
    sensor_msgs::msg::PointCloud2 empty_local;
    pcl::toROSMsg(*local_map_, empty_local);
    empty_local.header.stamp = node_->now();
    empty_local.header.frame_id = cfg_.map_frame;
    pub_local_->publish(empty_local);
  }
}

std::size_t PCLMapper::mapSize() const
{
  std::unique_lock<std::mutex> lck(mutex_);
  return global_map_->points.size();
}

void PCLMapper::colorDepthCameraInfoCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & color,
  const sensor_msgs::msg::Image::ConstSharedPtr & depth,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info,
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr & pose)
{
  if (!color || !depth || !pose || !info) {
    return;
  }

  if (!validatePose(*pose)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "Dropping sync set: pose missing frame_id, zero stamp or non-finite");
    return;
  }

  // Stale-data protection: reject poses whose stamp is far behind the images.
  const auto image_stamp = rclcpp::Time(color->header.stamp);
  const auto pose_stamp = rclcpp::Time(pose->header.stamp);
  const double dt = std::abs((image_stamp - pose_stamp).seconds());
  if (dt > cfg_.sync_slop) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "Dropping sync set: pose/image timestamp gap %.3fs exceeds slop %.3fs", dt, cfg_.sync_slop);
    return;
  }

  // Update intrinsics from CameraInfo only when valid and config says to use it.
  if (validIntrinsics(info->k[0], info->k[4])) {
    cfg_.fx = info->k[0];
    cfg_.fy = info->k[4];
    cfg_.cx = info->k[2];
    cfg_.cy = info->k[5];
  }

  cv_bridge::CvImageConstPtr cv_color;
  cv_bridge::CvImageConstPtr cv_depth;
  try {
    cv_color = cv_bridge::toCvShare(color, color->encoding);
    cv_depth = cv_bridge::toCvShare(depth, depth->encoding);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "cv_bridge conversion failed: %s", e.what());
    return;
  }

  const Eigen::Isometry3d T = convertPose(*pose);
  insertKeyFrame(cv_color->image, cv_depth->image, T);
  processPending();
}

}  // namespace yahboomcar_slam
