//
// Created by yahboom on 2021/4/29.
//

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <random>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

/**
 * @brief 显式合成点云演示节点 (Explicit Synthetic Point Cloud Demo)
 *
 * 注意：本节点仅生成用于演示的合成数据（在 10m 立方体内随机生成 100 个 XYZRGB 点，默认 2Hz），
 * 并非真实机器人的地图建图或传感器反馈数据。
 */
class PointCloudPublisher : public rclcpp::Node {
public:
    explicit PointCloudPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("pub_pcl", options) {
        // 声明参数
        this->declare_parameter<int>("num_points", 100);
        this->declare_parameter<double>("rate", 2.0);
        this->declare_parameter<std::string>("frame_id", "map");
        this->declare_parameter<int64_t>("seed", -1);  // -1 表示采用真随机数；>= 0 时启用确定性伪随机种子

        // 获取参数
        int raw_num_points = this->get_parameter("num_points").as_int();
        double raw_rate = this->get_parameter("rate").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();
        int64_t seed = this->get_parameter("seed").as_int();

        // 校验有限正数参数 (finite positive params)
        if (raw_num_points <= 0) {
            RCLCPP_WARN(this->get_logger(),
                "参数 'num_points' 必须为正整数，当前输入为 %d，已回退为默认值 100", raw_num_points);
            num_points_ = 100;
        } else {
            num_points_ = static_cast<size_t>(raw_num_points);
        }

        if (!std::isfinite(raw_rate) || raw_rate <= 0.0) {
            RCLCPP_WARN(this->get_logger(),
                "参数 'rate' 必须为有限正浮点数，当前输入无效，已回退为默认值 2.0 Hz");
            rate_hz_ = 2.0;
        } else {
            rate_hz_ = raw_rate;
        }

        // 初始化随机数生成器（可选确定性随机种子）
        if (seed >= 0) {
            rng_.seed(static_cast<unsigned int>(seed));
            RCLCPP_INFO(this->get_logger(), "已启用确定性随机种子: %ld", seed);
        } else {
            std::random_device rd;
            rng_.seed(rd());
        }

        // 建立相对话题发布者 (topic: "color_cloud")
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("color_cloud", 10);

        RCLCPP_INFO(this->get_logger(),
            "[合成演示 Demo] 开始发布合成点云 (点数: %zu, 范围: 10m立方体, 频率: %.2f Hz, frame_id: '%s', 话题: 'color_cloud')",
            num_points_, rate_hz_, frame_id_.c_str());

        // 创建定时器发布点云
        auto period = std::chrono::duration<double>(1.0 / rate_hz_);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&PointCloudPublisher::timer_callback, this));
    }

private:
    void timer_callback() {
        // 创建 PCL 点云
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cloud.points.resize(num_points_);

        std::uniform_real_distribution<float> coord_dist(0.0f, 10.0f);
        std::uniform_int_distribution<int> color_dist(0, 255);

        for (size_t i = 0; i < num_points_; ++i) {
            // 点云中每个点位于一个 10*10*10 的方块内随机分布，颜色也随机
            cloud.points[i].x = coord_dist(rng_);
            cloud.points[i].y = coord_dist(rng_);
            cloud.points[i].z = coord_dist(rng_);
            cloud.points[i].r = static_cast<uint8_t>(color_dist(rng_));
            cloud.points[i].g = static_cast<uint8_t>(color_dist(rng_));
            cloud.points[i].b = static_cast<uint8_t>(color_dist(rng_));
        }

        // 将点云转化为 ROS2 PointCloud2 消息
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(cloud, output_msg);
        output_msg.header.stamp = this->now();
        output_msg.header.frame_id = frame_id_;

        // 发布点云消息
        pub_->publish(output_msg);
        RCLCPP_INFO_ONCE(this->get_logger(), "Published first point cloud successfully.");
    }

    size_t num_points_{100};
    double rate_hz_{2.0};
    std::string frame_id_{"map"};
    std::mt19937 rng_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
