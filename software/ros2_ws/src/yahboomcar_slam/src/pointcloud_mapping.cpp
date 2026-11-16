// Headless native ROS2 Humble pointcloud mapping node (no ORB_SLAM2 SDK).
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "yahboomcar_slam/PCLMapper.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("pointcloud_mapping");

  yahboomcar_slam::MapperConfig cfg;
  cfg.color_topic = node->declare_parameter<std::string>("topicColor", cfg.color_topic);
  cfg.depth_topic = node->declare_parameter<std::string>("topicDepth", cfg.depth_topic);
  cfg.pose_topic = node->declare_parameter<std::string>("topicTcw", cfg.pose_topic);
  cfg.camera_info_topic = node->declare_parameter<std::string>("topicCameraInfo", cfg.camera_info_topic);
  cfg.map_frame = node->declare_parameter<std::string>("map_frame", cfg.map_frame);
  cfg.camera_frame = node->declare_parameter<std::string>("local_frame_id", cfg.camera_frame);
  cfg.fx = node->declare_parameter<double>("fx", cfg.fx);
  cfg.fy = node->declare_parameter<double>("fy", cfg.fy);
  cfg.cx = node->declare_parameter<double>("cx", cfg.cx);
  cfg.cy = node->declare_parameter<double>("cy", cfg.cy);
  cfg.resolution = static_cast<float>(node->declare_parameter<double>("resolution", cfg.resolution));
  cfg.depth_scale = static_cast<float>(node->declare_parameter<double>("depthfactor", cfg.depth_scale));
  cfg.depth_unit = node->declare_parameter<int>("depth_unit", cfg.depth_unit);
  cfg.sync_slop = node->declare_parameter<double>("sync_slop", cfg.sync_slop);
  cfg.pixel_step = node->declare_parameter<int>("pixel_step", cfg.pixel_step);
  cfg.min_depth = node->declare_parameter<double>("min_depth", cfg.min_depth);
  cfg.max_depth = node->declare_parameter<double>("max_depth", cfg.max_depth);
  cfg.max_points = static_cast<std::size_t>(node->declare_parameter<int>("max_points", static_cast<int>(cfg.max_points)));
  cfg.input_pose_is_twc = node->declare_parameter<bool>("input_pose_is_twc", cfg.input_pose_is_twc);

  RCLCPP_INFO(node->get_logger(), "Starting native PCL mapper (no ORB_SLAM2 SDK required)");
  RCLCPP_INFO(node->get_logger(), "  color=%s depth=%s pose=%s info=%s",
    cfg.color_topic.c_str(), cfg.depth_topic.c_str(), cfg.pose_topic.c_str(), cfg.camera_info_topic.c_str());
  RCLCPP_INFO(node->get_logger(), "  map_frame=%s fx=%.3f fy=%.3f cx=%.3f cy=%.3f",
    cfg.map_frame.c_str(), cfg.fx, cfg.fy, cfg.cx, cfg.cy);
  RCLCPP_INFO(node->get_logger(), "  pose contract: %s",
    cfg.input_pose_is_twc ? "Twc" : "Tcw (world->camera; inverted internally)");

  yahboomcar_slam::PCLMapper mapper(node.get(), cfg);
  mapper.initializePublishersAndServices();
  mapper.initializeSubscriptions();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
