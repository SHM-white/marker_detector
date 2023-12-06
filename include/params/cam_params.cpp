//
// Created by mijiao on 23-11-25.
//

#include "cam_params.h"

void CamParams::init(rclcpp::Node::SharedPtr _node) {
    node = _node;
    node->create_subscription<sensor_msgs::msg::CameraInfo>("/camera_info", 1,
                                                            std::bind(&CamParams::cameraInfoCallback, this,
                                                                      std::placeholders::_1));
    cameraMatrix = cv::Mat(3, 3, CV_64F, 0.0);
    cameraMatrix.at<float>(0, 0) = 1000;
    cameraMatrix.at<float>(1, 1) = 1000;
    cameraMatrix.at<float>(2, 2) = 1;
    cameraMatrix.at<float>(0, 3) = 350;
    cameraMatrix.at<float>(1, 3) = 350;
    distCoeffs = cv::Mat(1, 4, CV_64F, 0.0);
}

void CamParams::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    height = (int) msg->roi.height;
    width = (int) msg->roi.width;
    offsetH = (int) msg->roi.y_offset;
    offsetW = (int) msg->roi.x_offset;
    distCoeffs.reserve(msg->d.size());
    memcpy(distCoeffs.data, msg->d.data(), msg->d.size() * sizeof(double));
    memcpy(cameraMatrix.data, msg->k.data(), msg->k.size() * sizeof(double));
}

void CamParams::setSize(int _height, int _width) {
    camera_interfaces::srv::ParamEvent::Request::SharedPtr heightRequest;
    camera_interfaces::srv::ParamEvent::Request::SharedPtr widthRequest;
    heightRequest->param_name = heightRequest->CAMERA_HEIGHT;
    heightRequest->value = _height;
    widthRequest->param_name = widthRequest->CAMERA_WIDTH;
    widthRequest->value = _width;
    auto heightResponse = paramEventClient->async_send_request(heightRequest);
    auto widthResponse = paramEventClient->async_send_request(widthRequest);
    if (heightResponse.get()->camera_height == height && widthResponse.get()->camera_width == width) {
        RCLCPP_INFO(node->get_logger(), "Resizing camera success!");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Resizing camera failed!");
    }
}

void CamParams::setExpTime(int expTime) {
    camera_interfaces::srv::ParamEvent::Request::SharedPtr expRequest;
    expRequest->param_name = expRequest->CAMERA_EXP;
    expRequest->value = expTime;
    auto expResponse = paramEventClient->async_send_request(expRequest);
    if (expResponse.get()->camera_exp == expTime) {
        RCLCPP_INFO(node->get_logger(), "Setting camera exp time success!");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Setting camera exp time failed!");
    }
}
