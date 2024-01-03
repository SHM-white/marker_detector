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
    auto idleDuration = start - lastEnd;
    auto processDuration = end - start;
    auto totalDuration = idleDuration + processDuration;

    RCLCPP_INFO(rclcpp::get_logger(name),
                "\nIdle Latency:%fms\nDetect Latency:%fms\nTotal Latency: %fms\nAverage Fps:%fHz",
                1000 * idleDuration.seconds(),
                1000 * processDuration.seconds(),
                1000 * totalDuration.seconds(),
                (double) recentDurations.size() /
                std::accumulate(recentDurations.begin(), recentDurations.end(), 0.0)
    );

    DetectResultsStamped detectResultsStamped;
    detectResultsStamped.time = start;
    detectResultsStamped.results = detectResults;
    lastEnd = end;
    recentDurations[recentDurationsIt] = totalDuration.seconds();
    recentDurationsIt++;
    recentDurationsIt %= recentDurations.size();
    return detectResultsStamped;
}

void Detector::reinitialize() {
    lastEnd = rclcpp::Clock().now();
    recentDurationsIt = 0;
}
