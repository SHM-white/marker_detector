//
// Created by mijiao on 23-11-25.
//

#include "buff_params.h"

void BuffParams::init(rclcpp::Node::SharedPtr _node) {
    node = _node;
    node->declare_parameter("/detector/buff_detector/model_path",
                            "./install/marker_detector/lib/marker_detector/buff_detector/model/dafu.xml");
}

std::string BuffParams::getModelPath() const {
    return node->get_parameter("/detector/buff_detector/model_path").as_string();
}
