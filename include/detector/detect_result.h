//
// Created by mijiao on 23-11-19.
//

#ifndef MARKER_DETECTOR_DETECT_RESULT_H
#define MARKER_DETECTOR_DETECT_RESULT_H

#include <rclcpp/time.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/quaternion.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "marker_detector/msg/detect_result.hpp"
#include "marker_detector/msg/detect_results.hpp"

struct DetectResult {
    cv::Point3d point;
    cv::Quatd quaternion;
    int id{0};
    bool detectSuccess{false};

    [[nodiscard]] marker_detector::msg::DetectResult toRosMsg() const;
};

using DetectResults = std::vector<DetectResult>;

struct DetectResultsStamped {
    rclcpp::Time time;
    DetectResults results;

    marker_detector::msg::DetectResults toRosMsg();
};

#endif //MARKER_DETECTOR_DETECT_RESULT_H
