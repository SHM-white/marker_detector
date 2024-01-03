//
// Created by mijiao on 23-12-25.
//

#ifndef MARKER_DETECTOR_ARMOR_PARAMS_H
#define MARKER_DETECTOR_ARMOR_PARAMS_H

#include <boost/serialization/singleton.hpp>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/header.hpp>

#define RAD2DEG 57.32

class ArmorParams : public boost::serialization::singleton<ArmorParams> {
private:
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr showImagePublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr showRoiPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr showNumPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr showCoordinatePublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr showBinaryPublisher;
public:
    void init(const rclcpp::Node::SharedPtr& _node);

    double getArmorTypeThreshold();

    bool getIsRed();

    double getGrayThreshold();

    cv::Scalar getBlueChMin();

    cv::Scalar getBlueChMax();

    cv::Scalar getRedChMin();

    cv::Scalar getRedChMax();

    double getBin3Threshold1();

    double getBin3Threshold2();

    double getBin3Threshold3();

    bool getIfShow();

    bool getIfGetAngle();

    bool getIsNumber();

    double getContoursLength1Min();

    double getContoursLength1Max();

    double getLedHeightMin();

    double getLedHeightMax();

    double getCosMarkerParallelRadian();

    double getCosMarkerDirectionRadian();

    double getCosMarkerVerticalRadian();

    double getLedRatioMin();

    double getLedRatioMax();

    double getMarkerSizeMin();

    double getMarkerSizeMax();

    double getMarkerRatioMin();

    double getMarkerRatioMax();

    long getBinaryMethod();

    double getLedDiff();

    void showInfo(const cv::Mat& image, const cv::Mat& roi, const cv::Mat& num, const cv::Mat& coordinate, const cv::Mat& binary);

    bool getIsBig(int i);

    long getOffsetH();

    long getOffsetW();

    long getHeight();

    long getWidth();

    long getExpTime();
};

#define armorParams ArmorParams::get_mutable_instance()

#endif //MARKER_DETECTOR_ARMOR_PARAMS_H
