#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include "KCF_Tracker.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<KCFTrackerNode>();
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        std::cerr << "KCF_Tracker fatal: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
