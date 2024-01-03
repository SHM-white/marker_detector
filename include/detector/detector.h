//
// Created by mijiao on 23-11-19.
//

#ifndef MARKER_DETECTOR_DETECTOR_H
#define MARKER_DETECTOR_DETECTOR_H

#include <string>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>

#include "detect_result.h"

#include "marker_detector/msg/detect_results.hpp"

class Detector {
protected:
    std::string name;
    rclcpp::Time lastEnd;
    std::array<double, 32> recentDurations = {};
    size_t recentDurationsIt = 0;

    void reinitialize();

    virtual DetectResults detectImpl(const cv::Mat& image) = 0;

public:
    explicit Detector(const std::string& _name);

    DetectResultsStamped detect(const cv::Mat& image);

    virtual void reinitialize(const std::vector<uint8_t>&) = 0;

    using SharedPtr = std::shared_ptr<Detector>;

};

#endif //MARKER_DETECTOR_DETECTOR_H
