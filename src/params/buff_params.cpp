//
// Created by mijiao on 23-11-25.
//

#include "params/buff_params.h"

void BuffParams::init(const rclcpp::Node::SharedPtr& _node) {
    node = _node;
    node->declare_parameter("buff_detector.model_path",
                            "./install/marker_detector/lib/marker_detector/buff_detector/model/dafu.xml");
    node->declare_parameter("buff_detector.height", 700);
    node->declare_parameter("buff_detector.width", 700);
    node->declare_parameter("buff_detector.offset_H", 162);
    node->declare_parameter("buff_detector.offset_W", 290);
    node->declare_parameter("buff_detector.exp_time", 1750);
    node->declare_parameter("buff_detector.analysis_points_num", 150);
    node->declare_parameter("buff_detector.delay_time", 0.35);
}

bool BuffParams::getIsRed() const {
    return node->get_parameter("is_red").as_bool();
}

long BuffParams::getHeight() const {
    return node->get_parameter("buff_detector.height").as_int();
}

long BuffParams::getWidth() const {
    return node->get_parameter("buff_detector.width").as_int();
}

long BuffParams::getExpTime() const {
    return node->get_parameter("buff_detector.exp_time").as_int();
}

long BuffParams::getAnalysisPointsNum() const {
    return node->get_parameter("buff_detector.analysis_points_num").as_int();
}

double BuffParams::getDelayTime() const {
    return node->get_parameter("buff_detector.delay_time").as_double();
}

std::string BuffParams::getModelPath() const {
    return node->get_parameter("buff_detector.model_path").as_string();
}

cv::Point2f BuffParams::getOffset() const {
    return {(float) node->get_parameter("buff_detector.offset_W").as_double(),
            (float) node->get_parameter("buff_detector.offset_H").as_double()};
}