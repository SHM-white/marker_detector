//
// Created by mijiao on 23-11-19.
//

#ifndef MARKER_DETECTOR_DETECTOR_CONTROLLER_H
#define MARKER_DETECTOR_DETECTOR_CONTROLLER_H

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "marker_detector/msg/detect_results.hpp"
#include "robot_serial/msg/mode.hpp"

#include "detector/detector.h"
#include "detector/detect_result.h"

#include "armor_detector/armor_detector.h"
#include "buff_detector/buff_detector.h"

#include "params/cam_params.h"
#include "params/armor_params.h"


class DetectorController : public rclcpp::Node {
private:
    enum class Mode : uint8_t {
        AUTO_AIM, BUFF, OUTPOST, NUM
    } mode = Mode::NUM;

    std::vector<Detector::SharedPtr> detectorList;

    rclcpp::Publisher<marker_detector::msg::DetectResults>::SharedPtr detectResultsPublisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr rawResultPublisher;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscription;
    rclcpp::Subscription<robot_serial::msg::Mode>::SharedPtr modeSubscription;

    rclcpp::CallbackGroup::SharedPtr callbackGroup;

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rosImage);

    void modeCallback(const robot_serial::msg::Mode::ConstSharedPtr& modeMsg);


public:
    DetectorController();

    void init();
};


#endif //MARKER_DETECTOR_DETECTOR_CONTROLLER_H
