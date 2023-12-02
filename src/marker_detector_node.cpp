#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "params/cam_params.h"

#include "detector_controller/detector_controller.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectorController>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
