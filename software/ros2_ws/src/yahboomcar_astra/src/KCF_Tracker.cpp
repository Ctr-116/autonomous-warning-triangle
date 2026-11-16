//
// ROS2 Humble migration of the KCF tracking wrapper (was src/KCF_Tracker.cpp).
//

#include "KCF_Tracker.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "sensor_msgs/image_encodings.hpp"

KCFTrackerNode::KCFTrackerNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("KCF_Tracker", options),
      tracker_(true, false, true, false) {
    // Declare params replacing KCFTracker.cfg dynamic_reconfigure.
    linear_Kp_ = this->declare_parameter("linear_Kp", 0.9);
    linear_Ki_ = this->declare_parameter("linear_Ki", 0.0);
    linear_Kd_ = this->declare_parameter("linear_Kd", 0.1);
    angular_Kp_ = this->declare_parameter("angular_Kp", 0.5);
    angular_Ki_ = this->declare_parameter("angular_Ki", 0.0);
    angular_Kd_ = this->declare_parameter("angular_Kd", 0.2);
    minDist_ = this->declare_parameter("minDist", 1.0);
    linear_max_ = this->declare_parameter("linear_max", 0.2);
    angular_max_ = this->declare_parameter("angular_max", 0.6);
    image_center_x_ = this->declare_parameter("image_center_x", 320.0);
    image_width_ = this->declare_parameter("image_width", 640);
    image_height_ = this->declare_parameter("image_height", 480);
    show_gui_ = this->declare_parameter("show_gui", false);
    depth_stale_timeout_ = this->declare_parameter("depth_stale_timeout", 0.5);

    roi_x_ = this->declare_parameter("initial_roi_x", 0);
    roi_y_ = this->declare_parameter("initial_roi_y", 0);
    roi_w_ = this->declare_parameter("initial_roi_w", 0);
    roi_h_ = this->declare_parameter("initial_roi_h", 0);
    has_roi_param_ = (roi_w_ > 0 && roi_h_ > 0);

    linear_PID_ = std::make_unique<PID>(static_cast<float>(linear_Kp_),
                                        static_cast<float>(linear_Ki_),
                                        static_cast<float>(linear_Kd_));
    angular_PID_ = std::make_unique<PID>(static_cast<float>(angular_Kp_),
                                         static_cast<float>(angular_Ki_),
                                         static_cast<float>(angular_Kd_));

    pub_position_ = this->create_publisher<yahboomcar_msgs::msg::Position>("/Position", 10);
    pub_cmdVel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    auto sensor_qos = rclcpp::SensorDataQoS();

    image_transport::ImageTransport it(shared_from_this());
    image_pub_ = image_transport::create_publisher(this, "/KCF_image",
                                                   rclcpp::SystemDefaultsQoS().get_rmw_qos_profile());
    image_sub_ = image_transport::create_subscription(
        this, "/camera/rgb/image_raw",
        std::bind(&KCFTrackerNode::imageCb, this, std::placeholders::_1),
        "raw", sensor_qos.get_rmw_qos_profile());
    depth_sub_ = image_transport::create_subscription(
        this, "/camera/depth/image_raw",
        std::bind(&KCFTrackerNode::depthCb, this, std::placeholders::_1),
        "raw", sensor_qos.get_rmw_qos_profile());

    caminfo_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/depth/camera_info", sensor_qos,
        std::bind(&KCFTrackerNode::cameraInfoCb, this, std::placeholders::_1));

    joy_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/JoyState", 10, std::bind(&KCFTrackerNode::joyCb, this, std::placeholders::_1));

    if (show_gui_) {
        initGUI();
    }

    RCLCPP_INFO(this->get_logger(), "KCF tracker started (show_gui=%s, initial_roi=%s)",
                show_gui_ ? "true" : "false", has_roi_param_ ? "set" : "unset");
}

KCFTrackerNode::~KCFTrackerNode() {
    cancel();
}

void KCFTrackerNode::initGUI() {
    if (gui_initialized_) return;
    try {
        cv::namedWindow(RGB_WINDOW, cv::WINDOW_AUTOSIZE);
        cv::setMouseCallback(RGB_WINDOW, [](int event, int x, int y, int, void *userdata) {
            static_cast<KCFTrackerNode *>(userdata)->onMouse(event, x, y);
        }, this);
        using_gui_ = true;
        gui_initialized_ = true;
    } catch (const cv::Exception &e) {
        RCLCPP_WARN(this->get_logger(), "GUI unavailable (%s); continuing headless", e.what());
        show_gui_ = false;
        using_gui_ = false;
    }
}

void KCFTrackerNode::onMouse(int event, int x, int y) {
    if (select_flag_) {
        selectRect_.x = std::min(origin_.x, x);
        selectRect_.y = std::min(origin_.y, y);
        selectRect_.width = std::abs(x - origin_.x);
        selectRect_.height = std::abs(y - origin_.y);
        if (!rgbimage_.empty()) {
            selectRect_ &= cv::Rect(0, 0, rgbimage_.cols, rgbimage_.rows);
        }
    }
    if (event == 1) {
        bBeginKCF_ = false;
        select_flag_ = true;
        origin_ = cv::Point(x, y);
        selectRect_ = cv::Rect(x, y, 0, 0);
    } else if (event == 4) {
        select_flag_ = false;
        bRenewROI_ = true;
    }
}

void KCFTrackerNode::reset() {
    bRenewROI_ = false;
    bBeginKCF_ = false;
    selectRect_ = cv::Rect();
    linear_speed_ = 0.0f;
    rotation_speed_ = 0.0f;
    enable_get_depth_ = false;
    robot_run_ = false;
    linear_PID_->reset();
    angular_PID_->reset();
    pub_cmdVel_->publish(geometry_msgs::msg::Twist());
}

void KCFTrackerNode::cancel() {
    reset();
    if (gui_initialized_) {
        try {
            cv::destroyWindow(RGB_WINDOW);
        } catch (const cv::Exception &) {
        }
        gui_initialized_ = false;
        using_gui_ = false;
    }
}

void KCFTrackerNode::imageCb(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    cv_ptr->image.copyTo(rgbimage_);
    if (rgbimage_.empty()) {
        return;
    }
    last_image_stamp_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    if (show_gui_ && !gui_initialized_) {
        initGUI();
    }

    if (has_roi_param_ && !bBeginKCF_ && !bRenewROI_) {
        selectRect_ = cv::Rect(roi_x_, roi_y_, roi_w_, roi_h_);
        if (selectRect_.width > 0 && selectRect_.height > 0 &&
            selectRect_.x >= 0 && selectRect_.y >= 0 &&
            selectRect_.x + selectRect_.width <= rgbimage_.cols &&
            selectRect_.y + selectRect_.height <= rgbimage_.rows) {
            tracker_.init(selectRect_, rgbimage_);
            bBeginKCF_ = true;
            enable_get_depth_ = true;
            has_roi_param_ = false;  // only bootstrap once
        }
    }

    if (bRenewROI_) {
        if (selectRect_.width <= 0 || selectRect_.height <= 0) {
            bRenewROI_ = false;
            return;
        }
        if (selectRect_.x < 0) selectRect_.x = 0;
        if (selectRect_.y < 0) selectRect_.y = 0;
        if (selectRect_.x + selectRect_.width > rgbimage_.cols)
            selectRect_.width = rgbimage_.cols - selectRect_.x;
        if (selectRect_.y + selectRect_.height > rgbimage_.rows)
            selectRect_.height = rgbimage_.rows - selectRect_.y;
        if (selectRect_.width <= 0 || selectRect_.height <= 0) {
            bRenewROI_ = false;
            return;
        }
        tracker_.init(selectRect_, rgbimage_);
        bBeginKCF_ = true;
        bRenewROI_ = false;
        enable_get_depth_ = false;
    }

    if (bBeginKCF_) {
        cv::Rect result = tracker_.update(rgbimage_);
        result &= cv::Rect(0, 0, rgbimage_.cols, rgbimage_.rows);
        cv::rectangle(rgbimage_, result, cv::Scalar(0, 255, 255), 1, 8);
        cv::circle(rgbimage_, cv::Point(result.x + result.width / 2, result.y + result.height / 2),
                   3, cv::Scalar(0, 0, 255), -1);
        selectRect_ = result;
    } else {
        cv::rectangle(rgbimage_, selectRect_, cv::Scalar(255, 0, 0), 2, 8, 0);
    }

    // Publish annotated image with original timestamp.
    try {
        std_msgs::msg::Header header = msg->header;
        sensor_msgs::msg::Image::SharedPtr out =
            cv_bridge::CvImage(header, "bgr8", rgbimage_).toImageMsg();
        image_pub_.publish(*out);
    } catch (const cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "publish failed: %s", e.what());
    }

    int action = -1;
    if (using_gui_) {
        try {
            action = cv::waitKey(1) & 0xFF;
        } catch (const cv::Exception &) {
            using_gui_ = false;
            action = -1;
        }
    }
    if (action == 'q' || action == 27) {
        cancel();
    } else if (action == 'r') {
        reset();
    } else if (action == 32) {
        enable_get_depth_ = true;
    }
}

bool KCFTrackerNode::trackingCallback(const cv::Mat &image, const std_msgs::msg::Header &header) {
    if (image.empty()) {
        return false;
    }
    // Non-blocking single-shot depth processing: called once per RGB frame
    // provided the most recent depth image and camera info are available.
    (void)header;
    return true;
}

void KCFTrackerNode::depthCb(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            cv_ptr->image.convertTo(depthimage_, CV_32FC1, 1.0 / 1000.0);
        } else if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            cv_ptr->image.copyTo(depthimage_);
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Unsupported depth encoding '%s'", msg->encoding.c_str());
            return;
        }
    } catch (const cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "depth cv_bridge exception: %s", e.what());
        return;
    }
    last_depth_stamp_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    // If not tracking or no valid ROI, publish zero once and return.
    if (!enable_get_depth_ || !bBeginKCF_ || selectRect_.width <= 0 || selectRect_.height <= 0 ||
        depthimage_.empty()) {
        if (robot_run_) {
            pub_cmdVel_->publish(geometry_msgs::msg::Twist());
            robot_run_ = false;
        }
        return;
    }

    const double now = this->get_clock()->now().seconds();
    if (now - last_image_stamp_ > depth_stale_timeout_ || now - last_depth_stamp_ > depth_stale_timeout_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "stale image/depth; stopping");
        pub_cmdVel_->publish(geometry_msgs::msg::Twist());
        robot_run_ = false;
        return;
    }

    const int center_x = static_cast<int>(selectRect_.x + selectRect_.width / 2);
    const int center_y = static_cast<int>(selectRect_.y + selectRect_.height / 2);
    const int rows = depthimage_.rows;
    const int cols = depthimage_.cols;

    if (center_x - 5 < 0 || center_x + 5 >= cols || center_y - 5 < 0 || center_y + 5 >= rows) {
        pub_cmdVel_->publish(geometry_msgs::msg::Twist());
        robot_run_ = false;
        return;
    }

    const float nan_val = std::numeric_limits<float>::quiet_NaN();
    auto sample = [&](int yy, int xx) -> float {
        if (yy < 0 || yy >= rows || xx < 0 || xx >= cols) return nan_val;
        return depthimage_.at<float>(yy, xx);
    };

    dist_val_[0] = sample(center_y - 5, center_x - 5);
    dist_val_[1] = sample(center_y - 5, center_x + 5);
    dist_val_[2] = sample(center_y + 5, center_x + 5);
    dist_val_[3] = sample(center_y + 5, center_x - 5);
    dist_val_[4] = sample(center_y, center_x);

    float distance = 0.0f;
    int num_depth_points = 5;
    for (int i = 0; i < 5; i++) {
        if (std::isfinite(dist_val_[i]) && dist_val_[i] > 0.4f && dist_val_[i] < 10.0f) {
            distance += dist_val_[i];
        } else {
            num_depth_points--;
        }
    }
    if (num_depth_points == 0) {
        pub_cmdVel_->publish(geometry_msgs::msg::Twist());
        robot_run_ = false;
        return;
    }
    distance /= static_cast<float>(num_depth_points);
    if (!std::isfinite(distance)) {
        pub_cmdVel_->publish(geometry_msgs::msg::Twist());
        robot_run_ = false;
        return;
    }

    if (std::abs(distance - static_cast<float>(minDist_)) < 0.1f) {
        linear_speed_ = 0.0f;
    } else {
        linear_speed_ = -linear_PID_->compute(static_cast<float>(minDist_), distance);
    }
    rotation_speed_ = angular_PID_->compute(static_cast<float>(image_center_x_ / 100.0),
                                            static_cast<float>(center_x / 100.0));
    if (std::abs(rotation_speed_) < 0.1f) rotation_speed_ = 0.0f;

    if (!std::isfinite(linear_speed_) || !std::isfinite(rotation_speed_)) {
        pub_cmdVel_->publish(geometry_msgs::msg::Twist());
        robot_run_ = false;
        return;
    }

    linear_speed_ = std::clamp(linear_speed_, static_cast<float>(-linear_max_),
                               static_cast<float>(linear_max_));
    rotation_speed_ = std::clamp(rotation_speed_, static_cast<float>(-angular_max_),
                                 static_cast<float>(angular_max_));

    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear_speed_;
    twist.angular.z = rotation_speed_;
    pub_cmdVel_->publish(twist);
    robot_run_ = true;

    yahboomcar_msgs::msg::Position position;
    position.angle_x = static_cast<float>(center_x);
    position.angle_y = static_cast<float>(center_y);
    position.distance = distance;
    pub_position_->publish(position);

    RCLCPP_DEBUG(this->get_logger(), "linear = %.3f, angular = %.3f, distance = %.3f, center_x = %d",
                 linear_speed_, rotation_speed_, distance, center_x);

    if (using_gui_) {
        try {
            cv::imshow(RGB_WINDOW, rgbimage_);
            cv::waitKey(1);
        } catch (const cv::Exception &) {
            using_gui_ = false;
        }
    }
}

void KCFTrackerNode::cameraInfoCb(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
    if (msg->width > 0 && msg->height > 0) {
        image_width_ = static_cast<int>(msg->width);
        image_height_ = static_cast<int>(msg->height);
    }
    if (msg->k[2] > 0.0) {
        image_center_x_ = msg->k[2];
    }
}

void KCFTrackerNode::joyCb(const std_msgs::msg::Bool::ConstSharedPtr msg) {
    // Preserved ROS1 behaviour: bool state gates the depth-driven velocity.
    enable_get_depth_ = msg->data;
    if (!enable_get_depth_) {
        pub_cmdVel_->publish(geometry_msgs::msg::Twist());
        robot_run_ = false;
    }
}
