//
// Created by mijiao on 23-11-19.
//

#include "detector_controller.h"

DetectorController::DetectorController() : Node("detector_controller"),
                                           detectorList(static_cast<unsigned long>(Mode::NUM)) {
    callbackGroup = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions subOpt = rclcpp::SubscriptionOptions();
    subOpt.callback_group = callbackGroup;
    detectResultsPublisher = create_publisher<marker_detector::msg::DetectResults>("/detect_results", 1);
    rawResultPublisher = create_publisher<geometry_msgs::msg::PoseStamped>("/raw_detect_results",1);
    imageSubscription = create_subscription<sensor_msgs::msg::Image>(
            "image_raw",
            1,
            std::bind(&DetectorController::imageCallback, this, std::placeholders::_1),
            subOpt
    );
    modeSubscription = create_subscription<robot_serial::msg::Mode>(
            "/robot/mode",
            1,
            std::bind(&DetectorController::modeCallback, this, std::placeholders::_1)
    );

}

void DetectorController::imageCallback(const sensor_msgs::msg::Image::SharedPtr rosImage) {
    cv_bridge::CvImagePtr cvImage;
    cvImage = cv_bridge::toCvCopy(rosImage, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cvImage->image;
    auto msg = detectorList[static_cast<unsigned long>(mode)]->detect(image).toRosMsg();
    if(!msg.detect_results.empty()){
        geometry_msgs::msg::PoseStamped poseStamped;
        poseStamped.header = msg.header;
        poseStamped.pose = msg.detect_results[0].pose;
        rawResultPublisher->publish(poseStamped);
    }
    detectResultsPublisher->publish(msg);
}

void DetectorController::modeCallback(const robot_serial::msg::Mode::SharedPtr modeMsg) {
    if (static_cast<Mode>(modeMsg->mode) != mode) {
        mode = static_cast<Mode>(modeMsg->mode);
        detectorList[modeMsg->mode]->reinitialize(modeMsg->config);
    }
}

void DetectorController::init() {
    buffParams.init(shared_from_this());
    camParams.init(shared_from_this());
    detectorList[static_cast<unsigned long>(Mode::BUFF)] = std::make_shared<BuffDetector>();
}
