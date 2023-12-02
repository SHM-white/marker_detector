//
// Created by mijiao on 23-11-25.
//

#ifndef MARKER_DETECTOR_CAM_PARAMS_H
#define MARKER_DETECTOR_CAM_PARAMS_H


#include <iostream>
#include <boost/serialization/singleton.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>


class CamParams : public boost::serialization::singleton<CamParams> {
private:
    rclcpp::Node::SharedPtr node;

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

public:
    std::string cameraName;
    cv::Mat cameraMatrix, distCoeffs;
    int height, width, offsetW, offsetH;

    int CAMERA_OFFSET_X;//摄像头相对云台的位置补偿的单位为毫米
    int CAMERA_OFFSET_Z;

    void init(rclcpp::Node::SharedPtr _node);
};


#define camParams CamParams::get_mutable_instance()

#endif //MARKER_DETECTOR_CAM_PARAMS_H
