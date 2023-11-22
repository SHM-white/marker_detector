//
// Created by mijiao on 23-11-19.
//

#ifndef MARKER_DETECTOR_DETECT_RESULT_H
#define MARKER_DETECTOR_DETECT_RESULT_H

#include <Eigen/Dense>
#include <rclcpp/time.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "marker_detector/msg/detect_result.hpp"
#include "marker_detector/msg/detect_results.hpp"

struct DetectResult {
    Eigen::Vector3d point;
    Eigen::Quaterniond quat;
    double roll, pitch, yaw;
    bool selected;
    bool big;
    int id{0};
    bool detectSuccess{false};

    marker_detector::msg::DetectResult toRosMsg();
};

struct DetectResults {
    rclcpp::Time time;
    std::vector<DetectResult> results;

    marker_detector::msg::DetectResults toRosMsg();
};

#endif //MARKER_DETECTOR_DETECT_RESULT_H
