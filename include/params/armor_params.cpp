//
// Created by mijiao on 23-12-25.
//

#include "armor_params.h"

void ArmorParams::init(const rclcpp::Node::SharedPtr& _node) {
    node = _node;

//    node->declare_parameter("/MarkerParams/led_vertical_diff");
//
//
//    node->declare_parameter("/MarkerParams/led_real_height");
//
//
//    node->declare_parameter("/AlgoriParams/is_auto");
//    node->declare_parameter("/AlgoriParams/is3big");
//    node->declare_parameter("/AlgoriParams/is4big");
//    node->declare_parameter("/AlgoriParams/is5big");
//    node->declare_parameter("/AlgoriParams/is_switch");
//
//    node->declare_parameter("/AlgoriParams/pitch_bias");
//
//    node->declare_parameter("/AlgoriParams/numdis_method");
//    node->declare_parameter("/AlgoriParams/dafu_gray_threthold");
//
//    node->declare_parameter("/AlgoriParams/kf_predict_gain");
//    node->declare_parameter("/AlgoriParams/time_gain");
//
//    node->declare_parameter("/dbg_img_path");
//    node->declare_parameter("/templates_path");
//    node->declare_parameter("/num_model_path");


    node->declare_parameter("/ifshow");

    node->declare_parameter("/if_get_angle");

    node->declare_parameter("detector/armor_detector/ch1_min_r");
    node->declare_parameter("detector/armor_detector/ch1_max_r");
    node->declare_parameter("detector/armor_detector/ch3_min_r");
    node->declare_parameter("detector/armor_detector/ch3_max_r");

    node->declare_parameter("detector/armor_detector/ch1_min_b");
    node->declare_parameter("detector/armor_detector/ch1_max_b");
    node->declare_parameter("detector/armor_detector/ch3_min_b");
    node->declare_parameter("detector/armor_detector/ch3_max_b");

    node->declare_parameter("detector/armor_detector/ch2_min");
    node->declare_parameter("detector/armor_detector/ch2_max");

    node->declare_parameter("/AlgoriParams/bina3_threshold_1");
    node->declare_parameter("/AlgoriParams/bina3_threshold_2");
    node->declare_parameter("/AlgoriParams/bina3_threshold_3");

    node->declare_parameter("detector/armor_detector/armor_type_threshold");
    node->declare_parameter("detector/armor_detector/gray_threshold");

    node->declare_parameter("detector/armor_detector/is_number");

    node->declare_parameter("detector/armor_detector/contours_length1_min");
    node->declare_parameter("detector/armor_detector/contours_length1_max");

    node->declare_parameter("detector/armor_detector/led_height_min");
    node->declare_parameter("detector/armor_detector/led_height_max");

    node->declare_parameter("detector/armor_detector/marker_parallel_angle");
    node->declare_parameter("detector/armor_detector/marker_direction_angle");
    node->declare_parameter("detector/armor_detector/led_vertical_angle");

    node->declare_parameter("detector/armor_detector/led_ratio_min");
    node->declare_parameter("detector/armor_detector/led_ratio_max");

    node->declare_parameter("detector/armor_detector/marker_size_min");
    node->declare_parameter("detector/armor_detector/marker_size_max");

    node->declare_parameter("detector/armor_detector/marker_ratio_min");
    node->declare_parameter("detector/armor_detector/marker_ratio_max");

    node->declare_parameter("detector/armor_detector/led_diff");

    node->declare_parameter("detector/armor_detector/binary_method");
}

double ArmorParams::getArmorTypeThreshold() {
    return node->get_parameter("detector/armor_detector/armor_type_threshold").as_double();
}

bool ArmorParams::getIsRed() {
    return node->get_parameter("detector/is_red").as_bool();
}

double ArmorParams::getGrayThreshold() {
    return node->get_parameter("detector/armor_detector/gray_threshold").as_double();
}

cv::Scalar ArmorParams::getBlueChMin() {
    return {node->get_parameter("detector/armor_detector/ch1_min_b").as_double(),
            node->get_parameter("detector/armor_detector/ch2_min").as_double(),
            node->get_parameter("detector/armor_detector/ch3_min_b").as_double()};
}

cv::Scalar ArmorParams::getBlueChMax() {
    return {node->get_parameter("detector/armor_detector/ch1_max_b").as_double(),
            node->get_parameter("detector/armor_detector/ch2_max").as_double(),
            node->get_parameter("detector/armor_detector/ch3_max_b").as_double()};
}

cv::Scalar ArmorParams::getRedChMin() {
    return {node->get_parameter("detector/armor_detector/ch1_min_r").as_double(),
            node->get_parameter("detector/armor_detector/ch2_min").as_double(),
            node->get_parameter("detector/armor_detector/ch3_min_r").as_double()};
}

cv::Scalar ArmorParams::getRedChMax() {
    return {node->get_parameter("detector/armor_detector/ch1_max_r").as_double(),
            node->get_parameter("detector/armor_detector/ch2_max").as_double(),
            node->get_parameter("detector/armor_detector/ch3_max_r").as_double()};
}

double ArmorParams::getBin3Threshold1() {
    return node->get_parameter("/AlgoriParams/bin3_threshold_1").as_double();
}

double ArmorParams::getBin3Threshold2() {
    return node->get_parameter("/AlgoriParams/bin3_threshold_2").as_double();
}

double ArmorParams::getBin3Threshold3() {
    return node->get_parameter("/AlgoriParams/bin3_threshold_3").as_double();
}

bool ArmorParams::getIfShow() {
    return node->get_parameter("/ifshow").as_bool();
}

bool ArmorParams::getIfGetAngle() {
    return node->get_parameter("/if_get_angle").as_bool();
}

bool ArmorParams::getIsNumber() {
    return node->get_parameter("detector/armor_detector/is_number").as_bool();
}

double ArmorParams::getContoursLength1Min() {
    return node->get_parameter("detector/armor_detector/contours_length1_min").as_double();
}

double ArmorParams::getContoursLength1Max() {
    return node->get_parameter("detector/armor_detector/contours_length1_max").as_double();
}

double ArmorParams::getLedHeightMin() {
    return node->get_parameter("detector/armor_detector/led_height_min").as_double();
}

double ArmorParams::getLedHeightMax() {
    return node->get_parameter("detector/armor_detector/led_height_max").as_double();
}

double ArmorParams::getCosMarkerParallelRadian() {
    return cos(node->get_parameter("detector/armor_detector/marker_parallel_angle").as_double() / RAD2DEG);
}

double ArmorParams::getCosMarkerDirectionRadian() {
    return cos(node->get_parameter("detector/armor_detector/marker_direction_angle").as_double() / RAD2DEG);
}

double ArmorParams::getCosMarkerVerticalRadian() {
    return cos((90 - node->get_parameter("detector/armor_detector/led_vertical_angle").as_double()) / RAD2DEG);
}

double ArmorParams::getLedRatioMin() {
    return node->get_parameter("detector/armor_detector/led_ratio_min").as_double();
}

double ArmorParams::getLedRatioMax() {
    return node->get_parameter("detector/armor_detector/led_ratio_max").as_double();
}

double ArmorParams::getMarkerSizeMin() {
    return node->get_parameter("detector/armor_detector/marker_size_min").as_double();
}

double ArmorParams::getMarkerSizeMax() {
    return node->get_parameter("detector/armor_detector/marker_size_max").as_double();
}

double ArmorParams::getMarkerRatioMin() {
    return node->get_parameter("detector/armor_detector/marker_ratio_min").as_double();
}

double ArmorParams::getMarkerRatioMax() {
    return node->get_parameter("detector/armor_detector/marker_ratio_max").as_double();
}

double ArmorParams::getLedDiff() {
    return node->get_parameter("detector/armor_detector/led_diff").as_double();
}

long ArmorParams::getBinaryMethod() {
    return node->get_parameter("detector/armor_detector/binary_method").as_int();
}