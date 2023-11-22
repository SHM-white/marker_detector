//
// Created by mijiao on 23-11-19.
//

#ifndef MARKER_DETECTOR_DETECTOR_CONTROLLER_H
#define MARKER_DETECTOR_DETECTOR_CONTROLLER_H

#include <opencv2/opencv.hpp>


#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/msg/image.hpp>

#include "detector/detector.h"

#include "marker_detector/msg/detect_results.hpp"
#include "robot_serial/msg/mode.hpp"

#include "detector/detect_result.h"


class DetectorController : public rclcpp::Node {
private:
    enum class Mode : uint8_t {
        AUTO_AIM, BUFF, OUTPOST
    } mode = Mode::AUTO_AIM;

    std::vector<Detector::SharedPtr> detectorList;

    rclcpp::Publisher<marker_detector::msg::DetectResults>::SharedPtr detectResultsPublisher;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscription;
    rclcpp::Subscription<robot_serial::msg::Mode>::SharedPtr modeSubscription;

    rclcpp::CallbackGroup::SharedPtr callbackGroup;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr rosImage);

    void modeCallback(const robot_serial::msg::Mode::SharedPtr modeMsg);


public:
    DetectorController();
};


#endif //MARKER_DETECTOR_DETECTOR_CONTROLLER_H
