//
// Created by mijiao on 23-11-25.
//

#include "cam_params.h"

void CamParams::init(rclcpp::Node::SharedPtr _node) {
    node = _node;
    cameraMatrix = cv::Mat(3,3,CV_32F,0.0f);
    cameraMatrix.at<float>(0,0) = 1000;
    cameraMatrix.at<float>(1,1) = 1000;
    cameraMatrix.at<float>(2,2) = 1;
    cameraMatrix.at<float>(0,3) = 350;
    cameraMatrix.at<float>(1,3) = 350;
    distCoeffs = cv::Mat(1,4,CV_32F,0.0f);
//    std::string cameraNameSpace;
//    node->declare_parameter("camera","/md_camera_node");
//    cameraNameSpace = node->get_parameter("camera").as_string();
////    ros::ServiceClient client = nh.serviceClient<md_camera::GetCameraInfo>(cameraNameSpace + "/get_camera_info");
//    infoSub = nh.subscribe(cameraNameSpace + "/camera_info", 1, &CamParams::infoCB, this);
//    client.waitForExistence();
//    md_camera::GetCameraInfo srv;
//    if (client.call(srv)) {
//        cameraName = srv.response.camera_name;
//        auto& in = srv.response.camera_info;
//        cameraMatrix =
//                (cv::Mat_<double>(3, 3) << in.K[0], in.K[1], in.K[2], in.K[3], in.K[4], in.K[5], in.K[6], in.K[7], in.K[8]);
//        distCoeffs = (cv::Mat_<double>(1, 4) << in.D[0], in.D[1], in.D[2], in.D[3]);
//        height = (int)in.roi.height;
//        width = (int)in.roi.width;
//        offsetH = (int)in.roi.y_offset;
//        offsetW = (int)in.roi.x_offset;
//        std::string s_cox = "/robots/" + cameraName + "/CAMERA_OFFSET_X";
//        std::string s_coz = "/robots/" + cameraName + "/CAMERA_OFFSET_Z";
//        ros::param::get(s_cox, CAMERA_OFFSET_X);
//        ros::param::get(s_coz, CAMERA_OFFSET_Z);
//        CAMERA_OFFSET_X /= 1000;
//        CAMERA_OFFSET_Z /= 1000;
//    } else {
//        ROS_ERROR("Load Camera Info Failed!!");
//        ros::shutdown();
//    }
}

[[maybe_unused]] void CamParams::infoCB(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    height = (int)msg->roi.height;
    width = (int)msg->roi.width;
    offsetH = (int)msg->roi.y_offset;
    offsetW = (int)msg->roi.x_offset;
}
