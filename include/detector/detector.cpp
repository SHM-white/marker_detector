//
// Created by mijiao on 23-11-20.
//

#include "detector.h"

Detector::Detector(const std::string& _name) {
    name = _name;
}

DetectResultsStamped Detector::detect(const cv::Mat& image) {
    auto start = rosClock.now();

    auto detectResults = detectImpl(image);

    auto end = rosClock.now();
    auto duration = start - end;
    RCLCPP_INFO(rclcpp::get_logger(name), "Detect Latency:%f", duration.seconds());

    DetectResultsStamped detectResultsStamped;
    detectResultsStamped.time = start;
    detectResultsStamped.results = detectResults;
    return detectResultsStamped;
}