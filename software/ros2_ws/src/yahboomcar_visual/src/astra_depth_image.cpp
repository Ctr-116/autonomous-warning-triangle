// Native ROS2 Humble port of yahboomcar_visual/src/astra_depth_image.cpp
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <string>

namespace {

class AstraDepthImage : public rclcpp::Node {
 public:
  AstraDepthImage() : Node("astra_depth_image_cpp") {
    image_topic_ = declare_parameter<std::string>("image_topic", "camera/depth/image_raw");
    display_ = declare_parameter<bool>("display", false);
    auto qos = rclcpp::SensorDataQoS();
    sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { depth_callback(*msg); });
  }

 private:
  void depth_callback(const sensor_msgs::msg::Image& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    const std::string encoding = msg.encoding;
    try {
      if (encoding == "16UC1" || encoding == "mono16") {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
      } else {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
      }
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    if (display_ && !cv_ptr->image.empty()) {
      cv::imshow("depth_image", cv_ptr->image);
      cv::waitKey(1);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  std::string image_topic_;
  bool display_{false};
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AstraDepthImage>());
  rclcpp::shutdown();
  return 0;
}
