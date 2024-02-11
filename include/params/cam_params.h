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
#include <camera_interfaces/srv/param_event.hpp>


class CamParams : public boost::serialization::singleton<CamParams> {
private:
    rclcpp::Node::SharedPtr node;
    rclcpp::Client<camera_interfaces::srv::ParamEvent>::SharedPtr paramEventClient;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoSubscription;

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg);

public:
    std::string cameraName;
    cv::Mat cameraMatrix, distCoeffs;
    int height, width, offsetW, offsetH;

    int CAMERA_OFFSET_X;//摄像头相对云台的位置补偿的单位为毫米
    int CAMERA_OFFSET_Z;

    void init(const rclcpp::Node::SharedPtr& _node);

    void setSize(int height, int width);

    void setExpTime(int expTime);

    double getFx();

    double getCx();

    double getFy();

    double getCy();
};


#define camParams CamParams::get_mutable_instance()

#endif //MARKER_DETECTOR_CAM_PARAMS_H
