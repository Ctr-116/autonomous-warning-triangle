// Optional real ORB_SLAM2 SDK tracker wrapper (ROS2 Humble).
//
// This file is ONLY compiled when -DENABLE_ORB_SLAM2=ON and the real external
// ORB_SLAM2 SDK is provided via -DORB_SLAM2_ROOT=<path>. It uses the real
// ORB_SLAM2::System API (TrackRGBD/TrackMonocular) returning cv::Mat Tcw, which
// is inverted to Twc PoseStamped + TF. Viewer is forced false. This wrapper is
// NOT validated in this environment (SDK/Pangolin unavailable) and the build is
// pending the SDK; see MIGRATION.md.
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <ORB_SLAM2/System.h>

namespace
{
class OrbSlam2Tracker : public rclcpp::Node
{
public:
  OrbSlam2Tracker()
  : rclcpp::Node("orb_slam2_tracker")
  {
    const std::string vocab = declare_parameter<std::string>("vocabulary_file", "");
    const std::string settings = declare_parameter<std::string>("settings_file", "");
    mode_ = declare_parameter<std::string>("mode", "rgbd");
    depth_map_factor_ = declare_parameter<double>("depth_map_factor", 5000.0);
    parent_frame_ = declare_parameter<std::string>("map_frame", "map");
    child_frame_ = declare_parameter<std::string>("camera_frame", "camera");
    const std::string color_topic = declare_parameter<std::string>("topicColor", "/camera/color/image_raw");
    const std::string depth_topic = declare_parameter<std::string>("topicDepth", "/camera/depth/image_raw");
    const bool viewer = declare_parameter<bool>("viewer", false);

    if (vocab.empty() || settings.empty()) {
      RCLCPP_FATAL(get_logger(), "vocabulary_file and settings_file are required for ORB_SLAM2 tracker");
      throw std::runtime_error("missing ORB_SLAM2 configuration");
    }
    if (viewer) {
      RCLCPP_WARN(get_logger(), "viewer=true requested but forced false for headless operation");
    }

    slam_ = std::make_unique<ORB_SLAM2::System>(
      vocab, settings,
      mode_ == "mono" ? ORB_SLAM2::System::MONOCULAR : ORB_SLAM2::System::RGBD,
      /*bUseViewer=*/false);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("orb_pose", 10);

    auto qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();
    color_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      this, color_topic, qos);
    if (mode_ != "mono") {
      depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, depth_topic, qos);
      using Policy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
      depth_sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(
        Policy(10), *color_sub_, *depth_sub_);
      depth_sync_->registerCallback(
        std::bind(&OrbSlam2Tracker::rgbdCallback, this, std::placeholders::_1, std::placeholders::_2));
    } else {
      mono_sub_ = create_subscription<sensor_msgs::msg::Image>(
        color_topic, rclcpp::SensorDataQoS(),
        std::bind(&OrbSlam2Tracker::monoCallback, this, std::placeholders::_1));
    }
    RCLCPP_INFO(get_logger(), "ORB_SLAM2 tracker started mode=%s (SDK build pending validation)", mode_.c_str());
  }

  ~OrbSlam2Tracker() override
  {
    if (slam_) {
      slam_->Shutdown();
      slam_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
      slam_.reset();
    }
  }

private:
  void publishPose(const cv::Mat & Tcw, const builtin_interfaces::msg::Time & stamp)
  {
    if (Tcw.empty() || Tcw.rows != 4 || Tcw.cols != 4) {
      return;
    }
    if (Tcw.type() != CV_32F && Tcw.type() != CV_64F) {
      return;
    }
    cv::Mat64d Tdot;
    Tcw.convertTo(Tdot, CV_64F);
    Eigen::Matrix4d Tcw_e = Eigen::Matrix4d::Identity();
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        Tcw_e(r, c) = Tdot.at<double>(r, c);
      }
    }
    const Eigen::Matrix4d Twc = Tcw_e.inverse();

    Eigen::Matrix3d R = Twc.block<3, 3>(0, 0);
    Eigen::Quaterniond q(R);
    q.normalize();
    Eigen::Vector3d t = Twc.block<3, 1>(0, 3);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = parent_frame_;
    pose.pose.position.x = t.x();
    pose.pose.position.y = t.y();
    pose.pose.position.z = t.z();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    pose_pub_->publish(pose);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = parent_frame_;
    tf.child_frame_id = child_frame_;
    tf.transform.translation.x = t.x();
    tf.transform.translation.y = t.y();
    tf.transform.translation.z = t.z();
    tf.transform.rotation = pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }

  void rgbdCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & color,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth)
  {
    if (color->data.empty() || depth->data.empty()) {
      return;
    }
    cv_bridge::CvImageConstPtr cv_color, cv_depth;
    try {
      cv_color = cv_bridge::toCvShare(color, color->encoding);
      cv_depth = cv_bridge::toCvShare(depth, depth->encoding);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge: %s", e.what());
      return;
    }
    if (cv_color->image.type() != CV_8UC3) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "RGB-D mode expects BGR8 color image");
      return;
    }
    if (cv_depth->image.type() != CV_16UC1 && cv_depth->image.type() != CV_32FC1) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Unsupported depth encoding for ORB_SLAM2");
      return;
    }
    const double stamp = rclcpp::Time(color->header.stamp).seconds();
    try {
      cv::Mat Tcw = slam_->TrackRGBD(cv_color->image, cv_depth->image, stamp);
      publishPose(Tcw, color->header.stamp);
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "TrackRGBD failed: %s", e.what());
    }
  }

  void monoCallback(const sensor_msgs::msg::Image::ConstSharedPtr & color)
  {
    if (color->data.empty()) {
      return;
    }
    cv_bridge::CvImageConstPtr cv_color;
    try {
      cv_color = cv_bridge::toCvShare(color, color->encoding);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge: %s", e.what());
      return;
    }
    if (cv_color->image.type() != CV_8UC3) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Monocular mode expects BGR8 color image");
      return;
    }
    const double stamp = rclcpp::Time(color->header.stamp).seconds();
    try {
      cv::Mat Tcw = slam_->TrackMonocular(cv_color->image, stamp);
      publishPose(Tcw, color->header.stamp);
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "TrackMonocular failed: %s", e.what());
    }
  }

  std::unique_ptr<ORB_SLAM2::System> slam_;
  std::string mode_;
  std::string parent_frame_;
  std::string child_frame_;
  double depth_map_factor_{5000.0};

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> color_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<
    message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image>>> depth_sync_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mono_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<OrbSlam2Tracker>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("orb_slam2_tracker"), "Fatal: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
