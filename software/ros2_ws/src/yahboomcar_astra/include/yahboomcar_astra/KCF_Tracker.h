//
// ROS2 Humble migration of the KCF tracking wrapper.
//

#ifndef TRANSBOT_ASTRA_KCF_TRACKER_H
#define TRANSBOT_ASTRA_KCF_TRACKER_H

#include <array>
#include <memory>
#include <string>

#include <opencv2/core/core.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include "yahboomcar_msgs/msg/position.hpp"

#include "kcftracker.h"
#include "PID.h"

class KCFTrackerNode : public rclcpp::Node {
public:
    explicit KCFTrackerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~KCFTrackerNode() override;

private:
    void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void depthCb(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void cameraInfoCb(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
    void joyCb(const std_msgs::msg::Bool::ConstSharedPtr msg);
    void reset();
    void cancel();
    bool trackingCallback(const cv::Mat &image, const std_msgs::msg::Header &header);

    rclcpp::Publisher<yahboomcar_msgs::msg::Position>::SharedPtr pub_position_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmdVel_;
    image_transport::Publisher image_pub_;

    image_transport::Subscriber image_sub_;
    image_transport::Subscriber depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr joy_sub_;

    std::unique_ptr<PID> linear_PID_;
    std::unique_ptr<PID> angular_PID_;

    // Configuration
    double linear_Kp_ = 0.9, linear_Ki_ = 0.0, linear_Kd_ = 0.1;
    double angular_Kp_ = 0.5, angular_Ki_ = 0.0, angular_Kd_ = 0.2;
    double minDist_ = 1.0;  // metres
    double linear_max_ = 0.2;
    double angular_max_ = 0.6;
    double image_center_x_ = 320.0;
    int image_width_ = 640;
    int image_height_ = 480;

    bool show_gui_ = false;
    bool enable_get_depth_ = false;
    bool select_flag_ = false;
    bool bRenewROI_ = false;
    bool bBeginKCF_ = false;

    // Initial ROI from parameters (usable without mouse).
    bool has_roi_param_ = false;
    int roi_x_ = 0, roi_y_ = 0, roi_w_ = 0, roi_h_ = 0;

    cv::Rect selectRect_;
    cv::Point origin_;
    bool gui_initialized_ = false;
    bool using_gui_ = false;
    static constexpr const char *RGB_WINDOW = "rgb_img";

    cv::Mat rgbimage_;
    cv::Mat depthimage_;
    std::array<float, 5> dist_val_{};
    float linear_speed_ = 0.0f;
    float rotation_speed_ = 0.0f;
    bool robot_run_ = false;
    double last_image_stamp_ = 0.0;
    double last_depth_stamp_ = 0.0;
    double depth_stale_timeout_ = 0.5;

    KCFTracker tracker_;

    void onMouse(int event, int x, int y);
    void initGUI();
};

#endif //TRANSBOT_ASTRA_KCF_TRACKER_H
