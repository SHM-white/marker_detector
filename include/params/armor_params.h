//
// Created by mijiao on 23-12-25.
//

#ifndef MARKER_DETECTOR_ARMOR_PARAMS_H
#define MARKER_DETECTOR_ARMOR_PARAMS_H

#include <boost/serialization/singleton.hpp>

#include <opencv2/core/types.hpp>

#include <rclcpp/rclcpp.hpp>

#define RAD2DEG 57.32

class ArmorParams : public boost::serialization::singleton<ArmorParams> {
private:
    rclcpp::Node::SharedPtr node;
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
};

#define armorParams ArmorParams::get_mutable_instance()

#endif //MARKER_DETECTOR_ARMOR_PARAMS_H
