//
// Created by mijiao on 23-11-20.
//

#include "detector.h"

Detector::Detector(const std::string& _name) {
    name = _name;
}

DetectResultsStamped Detector::detect(const cv::Mat& image) {
    auto start = rclcpp::Clock().now();

    auto detectResults = detectImpl(image);

    auto end = rclcpp::Clock().now();
    auto duration = end - start;
    RCLCPP_INFO(rclcpp::get_logger(name), "Detect Latency:%fs Fps:%fHz", duration.seconds(),1.0/duration.seconds());

    DetectResultsStamped detectResultsStamped;
    detectResultsStamped.time = start;
    detectResultsStamped.results = detectResults;
    return detectResultsStamped;
}