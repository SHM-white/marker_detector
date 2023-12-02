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
