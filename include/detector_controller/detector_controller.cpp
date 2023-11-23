//
// Created by mijiao on 23-11-19.
//

#include "detector_controller.h"

DetectorController::DetectorController() : Node("detector_controller"),
                                           detectorList(static_cast<unsigned long>(Mode::NUM)) {
    callbackGroup = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions subOpt = rclcpp::SubscriptionOptions();
    subOpt.callback_group = callbackGroup;
    detectResultsPublisher = create_publisher<marker_detector::msg::DetectResults>("/detectResults", 1);
    imageSubscription = create_subscription<sensor_msgs::msg::Image>(
            "/camera/raw", //TODO:参数设置
            1,
            std::bind(&DetectorController::imageCallback, this, std::placeholders::_1),
            subOpt
    );
    modeSubscription = create_subscription<robot_serial::msg::Mode>(
            "/robot/mode",
            1,
            std::bind(&DetectorController::modeCallback, this, std::placeholders::_1)
    );
    declare_parameter("/detector/buff_detector/model_path");
    detectorList[static_cast<unsigned long>(Mode::BUFF)] = std::make_shared<BuffDetector>();
}

void DetectorController::imageCallback(const sensor_msgs::msg::Image::SharedPtr rosImage) {
    cv_bridge::CvImagePtr cvImage;
    cvImage = cv_bridge::toCvCopy(rosImage, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cvImage->image;
    detectResultsPublisher->publish(detectorList[static_cast<unsigned long>(mode)]->detect(image).toRosMsg());
}

void DetectorController::modeCallback(const robot_serial::msg::Mode::SharedPtr modeMsg) {
    if (static_cast<Mode>(modeMsg->mode) != mode) {
        mode = static_cast<Mode>(modeMsg->mode);
        detectorList[modeMsg->mode]->reinitialize(modeMsg->config);
    }
}
