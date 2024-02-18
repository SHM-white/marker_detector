//
// Created by mijiao on 23-11-25.
//

#include "params/cam_params.h"

void CamParams::init(const rclcpp::Node::SharedPtr &_node) {
    node = _node;
    cameraInfoSubscription = node->create_subscription<sensor_msgs::msg::CameraInfo>("camera_info", 1,
                                                                                     std::bind(
                                                                                             &CamParams::cameraInfoCallback,
                                                                                             this,
                                                                                             std::placeholders::_1));
    paramEventClient = node->create_client<camera_interfaces::srv::ParamEvent>("param_event");
    cameraMatrix = cv::Mat(3, 3, CV_64F, 0.0);
    cameraMatrix.at<double>(0, 0) = 1134;
    cameraMatrix.at<double>(1, 1) = 1134;
    cameraMatrix.at<double>(2, 2) = 1;
    cameraMatrix.at<double>(0, 2) = 720;
    cameraMatrix.at<double>(1, 2) = 200;
    distCoeffs = cv::Mat(1, 4, CV_64F, 0.0);
}

void CamParams::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg) {
    cv::Mat tempCameraMatrix(3, 3, CV_64FC1);
    height = (int) msg->height;
    width = (int) msg->width;
    distCoeffs.reserve(msg->d.size());
    memcpy(distCoeffs.data, msg->d.data(), msg->d.size() * sizeof(double));
    memcpy(tempCameraMatrix.data, msg->k.data(), msg->k.size() * sizeof(double));
    if (cv::countNonZero(tempCameraMatrix) == 0) {
        RCLCPP_WARN(rclcpp::get_logger("camera"), "Camera matrix is all of zero!");
    } else {
        cameraMatrix = tempCameraMatrix;
    }
}

void CamParams::setSize(int _height, int _width) {
    using namespace std::chrono_literals;
    while (!paramEventClient->wait_for_service(1s)) {
        RCLCPP_WARN(node->get_logger(), "Camera service offline! Waiting...");
    }
    camera_interfaces::srv::ParamEvent::Request::SharedPtr heightRequest = std::make_shared<camera_interfaces::srv::ParamEvent::Request>();
    camera_interfaces::srv::ParamEvent::Request::SharedPtr widthRequest = std::make_shared<camera_interfaces::srv::ParamEvent::Request>();
    heightRequest->param_name = heightRequest->CAMERA_HEIGHT;
    heightRequest->value = _height;
    widthRequest->param_name = widthRequest->CAMERA_WIDTH;
    widthRequest->value = _width;
    bool success = true;
    paramEventClient->async_send_request(
            heightRequest,
            [this, &success, &_height](
                    const rclcpp::Client<camera_interfaces::srv::ParamEvent>::SharedFuture heightResponse) {
                if (heightResponse.get()->camera_height != _height) {
                    success = false;
                }
            });
    paramEventClient->async_send_request(
            widthRequest,
            [this, &success, &_width](
                    const rclcpp::Client<camera_interfaces::srv::ParamEvent>::SharedFuture widthResponse) {
                if (widthResponse.get()->camera_width != _width) {
                    success = false;
                }
            });
    if (success) {
        RCLCPP_INFO(node->get_logger(), "Resizing camera success!");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Resizing camera failed!");
    }
}

void CamParams::setExpTime(int expTime) {
    camera_interfaces::srv::ParamEvent::Request::SharedPtr expRequest = std::make_shared<camera_interfaces::srv::ParamEvent::Request>();
    expRequest->param_name = expRequest->CAMERA_EXP;
    expRequest->value = expTime;
    auto expResponse = paramEventClient->async_send_request(expRequest);
    if (expResponse.get()->camera_exp == expTime) {
        RCLCPP_INFO(node->get_logger(), "Setting camera exp time success!");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Setting camera exp time failed!");
    }
}

double CamParams::getFx() {
    return cameraMatrix.at<double>(0, 0);
}

double CamParams::getCx() {
    return cameraMatrix.at<double>(0, 2);
}

double CamParams::getFy() {
    return cameraMatrix.at<double>(1, 1);
}

double CamParams::getCy() {
    return cameraMatrix.at<double>(1, 2);
}
