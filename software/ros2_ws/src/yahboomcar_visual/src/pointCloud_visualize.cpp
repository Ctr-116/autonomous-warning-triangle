//
// Created by yahboom on 2021/8/21.
//

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>

/**
 * @brief 点云可视化节点 (Point Cloud Visualizer)
 *
 * 参数说明：
 * - gui (bool, 默认 false): 无头模式 (headless)，设为 false 时不会创建任何 VTK/Viewer 窗口；设为 true 时创建 CloudViewer。
 * - topic (string, 默认 "color_cloud"): 订阅的点云相对话题名。
 */
class CloudHandler : public rclcpp::Node {
public:
    explicit CloudHandler(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("pcl_visualize", options) {
        // 声明参数
        this->declare_parameter<bool>("gui", false);
        this->declare_parameter<std::string>("topic", "color_cloud");

        gui_ = this->get_parameter("gui").as_bool();
        std::string topic_name = this->get_parameter("topic").as_string();

        if (gui_) {
            RCLCPP_INFO(this->get_logger(), "GUI 模式已启用，正在初始化 PCL CloudViewer 窗口...");
            viewer_ = std::make_unique<pcl::visualization::CloudViewer>("Cloud Viewer");

            // 创建有界定时器，检查可视化窗口状态，窗口关闭时主动退出节点
            viewer_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&CloudHandler::timerCB, this));
        } else {
            RCLCPP_INFO(this->get_logger(),
                "当前处于默认无头模式 (gui=false，未创建图形界面及 VTK 渲染窗口)");
        }

        // 使用传感器 QoS BEST_EFFORT 订阅点云数据
        auto sensor_qos = rclcpp::SensorDataQoS();
        sensor_qos.best_effort();

        // 创建点云订阅者
        pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name,
            sensor_qos,
            std::bind(&CloudHandler::cloudCB, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "已创建点云订阅者，监听话题: '%s' (QoS: Best Effort)", topic_name.c_str());
    }

    ~CloudHandler() override {
        // 有界平稳析构 (Bounded destruction)
        if (viewer_timer_) {
            viewer_timer_->cancel();
            viewer_timer_.reset();
        }
        if (viewer_) {
            viewer_.reset();
        }
    }

private:
    void cloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr input) {
        // 创建 PCL 点云
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        // 从话题消息转成 PCL 点云
        pcl::fromROSMsg(*input, cloud);

        if (gui_ && viewer_) {
            // 使用 PCL 可视化工具显示出来
            viewer_->showCloud(cloud.makeShared());
        } else {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "[Headless] 成功接收点云帧: 点数=%zu, frame_id='%s'",
                cloud.points.size(), input->header.frame_id.c_str());
        }
    }

    void timerCB() {
        // 关闭可视化工具窗口时，安全退出 ROS2 节点
        if (viewer_ && viewer_->wasStopped()) {
            RCLCPP_INFO(this->get_logger(), "PCL 可视化窗口已关闭，正在停止节点...");
            if (viewer_timer_) {
                viewer_timer_->cancel();
            }
            rclcpp::shutdown();
        }
    }

    bool gui_{false};
    std::unique_ptr<pcl::visualization::CloudViewer> viewer_{nullptr};
    rclcpp::TimerBase::SharedPtr viewer_timer_{nullptr};
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_{nullptr};
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CloudHandler>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
