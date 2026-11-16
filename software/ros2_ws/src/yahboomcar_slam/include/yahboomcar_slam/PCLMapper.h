// Native ROS2 Humble / PCL mapping class ported from yahboomcar_slam.
// Algorithm (pixel projection, voxel grid, statistical/orientation-preserving
// accumulation) is preserved on generic cv::Mat + Eigen input, decoupled from
// ORB_SLAM2. GPL-3.0-or-later, matching the original source.
#ifndef YAHBOOMCAR_SLAM__PCLMAPPER_H_
#define YAHBOOMCAR_SLAM__PCLMAPPER_H_

#include <condition_variable>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <rclcpp/rclcpp.hpp>

namespace yahboomcar_slam
{

// Pose-frame contract: input poses are Tcw (world/camera -> camera), i.e. they
// map a point p_world to p_camera = Tcw * p_world. This matches the original
// ORB_SLAM2 wrapper which published Tcw. Internally we invert to Twc to lift
// camera-frame points into the world/map frame. `input_pose_is_twc` can be set
// true to accept Twc directly (no inversion).
struct MapperConfig
{
  std::string color_topic{"/RGBD/RGB/Image"};
  std::string depth_topic{"/RGBD/Depth/Image"};
  std::string pose_topic{"/RGBD/CameraPose"};
  std::string camera_info_topic{ "/camera/color/camera_info"};
  std::string map_frame{"map"};
  std::string camera_frame{"camera"};

  double fx{517.306408};
  double fy{516.469215};
  double cx{318.643040};
  double cy{255.313989};
  float resolution{0.04f};
  float depth_scale{1000.0f};         // divisor applied to depth values
  int depth_unit{0};                    // 0 = auto-detect 16UC1(mm)/32FC1(m), 1=force 16U mm, 2=force 32F m
  int queue_size{10};
  int sync_queue{10};
  double sync_slop{0.05};
  int pixel_step{3};                   // original sampled every 3rd pixel
  double min_depth{0.01};
  double max_depth{10.0};
  std::size_t max_points{1500000};    // bounded map growth
  std::size_t max_buffer{150};         // bounded keyframe buffer
  bool input_pose_is_twc{false};
  std::string save_path{"./resultPointCloudFile.pcd"};
};

class PCLMapper
{
public:
  using PointT = pcl::PointXYZRGBA;
  using PointCloud = pcl::PointCloud<PointT>;

  explicit PCLMapper(rclcpp::Node * node, const MapperConfig & cfg);
  ~PCLMapper();

  void initializeSubscriptions();
  void initializePublishersAndServices();

  // Decoupled core API (plain cv::Mat + Eigen), callable without ROS or ORB SDK.
  void insertKeyFrame(const cv::Mat & color, const cv::Mat & depth, const Eigen::Isometry3d & Tcw);

  PointCloud::Ptr generatePointCloud(const cv::Mat & color, const cv::Mat & depth, const Eigen::Isometry3d & T);

  Eigen::Isometry3d convertPose(const geometry_msgs::msg::PoseStamped & pose) const;

  void processPending();

  void resetMap();

  std::size_t mapSize() const;

private:
  void colorDepthCameraInfoCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & color,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr & pose);

  bool validateAndConvertDepth(const cv::Mat & raw, cv::Mat & out_float_m);
  bool validateAndConvertColor(const cv::Mat & raw, cv::Mat & out_bgr);
  bool validatePose(const geometry_msgs::msg::PoseStamped & pose) const;

  rclcpp::Node * node_;
  MapperConfig cfg_;

  mutable std::mutex mutex_;
  PointCloud::Ptr global_map_;
  PointCloud::Ptr local_map_;
  pcl::VoxelGrid<PointT> voxel_;

  struct PendingFrame
  {
    cv::Mat color;
    cv::Mat depth;
    Eigen::Isometry3d Tcw{Eigen::Isometry3d::Identity()};
    builtin_interfaces::msg::Time stamp;
    std::string frame_id;
  };
  std::vector<PendingFrame> pending_;

  std::size_t last_keyframe_size_{0};

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_global_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_local_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_srv_;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> color_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>> info_filter_sub_;
  std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>> pose_sub_;
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo,
    geometry_msgs::msg::PoseStamped>>> sync_;
};

}  // namespace yahboomcar_slam

#endif  // YAHBOOMCAR_SLAM__PCLMAPPER_H_
