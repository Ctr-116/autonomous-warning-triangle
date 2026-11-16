// Native ROS2 Humble port of yahboomcar_visual/src/IR_transform.cpp
// Normalizes MONO16 / 16UC1 IR image to MONO8 (0..255) and republishes.
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <string>

namespace {

class IrTransform : public rclcpp::Node {
 public:
  IrTransform() : Node("IR_transform") {
    in_topic_ = declare_parameter<std::string>("image_topic", "camera/ir/image_raw");
    out_topic_ = declare_parameter<std::string>("output_topic", "camera/ir/image_mono8");
    auto qos = rclcpp::SensorDataQoS();
    pub_ = create_publisher<sensor_msgs::msg::Image>(out_topic_, qos);
    sub_ = create_subscription<sensor_msgs::msg::Image>(
        in_topic_, qos,
        [this](const sensor_msgs::msg::Image::ConstSharedPtr & msg) { callback(msg); });
  }

 private:
  void callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
    if (msg->encoding != sensor_msgs::image_encodings::TYPE_16UC1 &&
        msg->encoding != sensor_msgs::image_encodings::MONO16) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Unsupported image encoding '%s'. Expected 16UC1 or mono16.",
          msg->encoding.c_str());
      return;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "cv_bridge exception: %s", e.what());
      return;
    }

    if (cv_ptr->image.type() != CV_16UC1) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Unexpected image type %d, expected CV_16UC1 (%d).",
          cv_ptr->image.type(), CV_16UC1);
      return;
    }

    cv::Mat img;
    cv::normalize(cv_ptr->image, img, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv_bridge::CvImage out(msg->header, sensor_msgs::image_encodings::MONO8, img);
    pub_->publish(*out.toImageMsg());
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  std::string in_topic_;
  std::string out_topic_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IrTransform>());
  rclcpp::shutdown();
  return 0;
}
