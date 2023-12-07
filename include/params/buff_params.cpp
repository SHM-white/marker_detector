//
// Created by mijiao on 23-11-25.
//

#include "buff_params.h"

void BuffParams::init(rclcpp::Node::SharedPtr _node) {
    node = _node;
    node->declare_parameter("detector/buff_detector/model_path",
                            "./install/marker_detector/lib/marker_detector/buff_detector/model/dafu.xml");
    node->declare_parameter("detector/buff_detector/height");
    node->declare_parameter("detector/buff_detector/width");
    node->declare_parameter("detector/buff_detector/offset_H");
    node->declare_parameter("detector/buff_detector/offset_W");
    node->declare_parameter("detector/buff_detector/exp_time");
}

std::string BuffParams::getModelPath() const {
    return node->get_parameter("detector/buff_detector/model_path").as_string();
}

long BuffParams::getHeight() const {
    return node->get_parameter("detector/buff_detector/height").as_int();
}

long BuffParams::getWidth() const {
    return node->get_parameter("detector/buff_detector/width").as_int();
}

long BuffParams::getExpTime() const {
    return node->get_parameter("detector/buff_detector/exp_time").as_int();
}
