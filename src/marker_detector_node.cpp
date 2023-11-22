#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "detector_controller/detector_controller.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectorController>());
    rclcpp::shutdown();
    return 0;
}
