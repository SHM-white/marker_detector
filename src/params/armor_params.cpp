//
// Created by mijiao on 23-12-25.
//

#include "params/armor_params.h"

void ArmorParams::init(const rclcpp::Node::SharedPtr& _node) {
    node = _node;
    node->declare_parameter("show", true);
    node->declare_parameter("if_get_angle", true);

    node->declare_parameter("is_red", false);

    node->declare_parameter("armor_detector.ch1_min_r", 0.0);
    node->declare_parameter("armor_detector.ch1_max_r", 255.0);
    node->declare_parameter("armor_detector.ch3_min_r", 100.0);
    node->declare_parameter("armor_detector.ch3_max_r", 255.0);

    node->declare_parameter("armor_detector.ch1_min_b", 100.0);
    node->declare_parameter("armor_detector.ch1_max_b", 255.0);
    node->declare_parameter("armor_detector.ch3_min_b", 0.0);
    node->declare_parameter("armor_detector.ch3_max_b", 255.0);

    node->declare_parameter("armor_detector.ch2_min", 0.0);
    node->declare_parameter("armor_detector.ch2_max", 255.0);

    node->declare_parameter("armor_detector.bin3_threshold_1", 130.0);
    node->declare_parameter("armor_detector.bin3_threshold_2", 50.0);
    node->declare_parameter("armor_detector.bin3_threshold_3", 0.3);

    node->declare_parameter("armor_detector.armor_type_threshold", 3.6);
    node->declare_parameter("armor_detector.gray_threshold", 40.0);

    node->declare_parameter("armor_detector.is_number", true);

    node->declare_parameter("armor_detector.contours_length1_min", 7.0);
    node->declare_parameter("armor_detector.contours_length1_max", 300.0);

    node->declare_parameter("armor_detector.led_height_min", 5.0);
    node->declare_parameter("armor_detector.led_height_max", 100.0);

    node->declare_parameter("armor_detector.marker_parallel_angle", 30.0);
    node->declare_parameter("armor_detector.marker_direction_angle", 30.0);
    node->declare_parameter("armor_detector.led_vertical_angle", 60.0);

    node->declare_parameter("armor_detector.led_ratio_min", 1.0);
    node->declare_parameter("armor_detector.led_ratio_max", 20.0);

    node->declare_parameter("armor_detector.marker_size_min", 1.0);
    node->declare_parameter("armor_detector.marker_size_max", 2000.0);

    node->declare_parameter("armor_detector.marker_ratio_min", 0.8);
    node->declare_parameter("armor_detector.marker_ratio_max", 6.0);

    node->declare_parameter("armor_detector.led_diff", 0.3);

    node->declare_parameter("armor_detector.binary_method", 3);

    node->declare_parameter("armor_detector.is3big", false);
    node->declare_parameter("armor_detector.is4big", false);
    node->declare_parameter("armor_detector.is5big", false);

    node->declare_parameter("armor_detector.offset_h", 256);
    node->declare_parameter("armor_detector.offset_w", 0);

    node->declare_parameter("armor_detector.height", 400);
    node->declare_parameter("armor_detector.width", 1440);

    node->declare_parameter("armor_detector.exp_time", 3200);

    showRoiPublisher = node->create_publisher<sensor_msgs::msg::Image>("detector/armor_detector/roi_image", 1);
    showBinaryPublisher = node->create_publisher<sensor_msgs::msg::Image>("detector/armor_detector/binary_image", 1);
    showImagePublisher = node->create_publisher<sensor_msgs::msg::Image>("detector/armor_detector/output_image", 1);
    showNumPublisher = node->create_publisher<sensor_msgs::msg::Image>("detector/armor_detector/num_image", 1);
    showCoordinatePublisher = node->create_publisher<sensor_msgs::msg::Image>(
            "detector/armor_detector/coordinate_image", 1);
}

bool ArmorParams::getIsRed() {
    return node->get_parameter("is_red").as_bool();
}

bool ArmorParams::getIfShow() {
    return node->get_parameter("show").as_bool();
}

bool ArmorParams::getIfGetAngle() {
    return node->get_parameter("if_get_angle").as_bool();
}

bool ArmorParams::getIsNumber() {
    return node->get_parameter("armor_detector.is_number").as_bool();
}

bool ArmorParams::getIsBig(int i) {
    rclcpp::Parameter ret;
    if (node->get_parameter_or("armor_detector.is" + std::to_string(i) + "big", ret, rclcpp::Parameter())) {
        return ret.as_bool();
    } else {
        return false;
    }
}

long ArmorParams::getBinaryMethod() {
    return node->get_parameter("armor_detector.binary_method").as_int();
}

long ArmorParams::getOffsetH() {
    return node->get_parameter("armor_detector.offset_h").as_int();
}

long ArmorParams::getOffsetW() {
    return node->get_parameter("armor_detector.offset_w").as_int();
}

long ArmorParams::getHeight() {
    return node->get_parameter("armor_detector.height").as_int();
}

long ArmorParams::getWidth() {
    return node->get_parameter("armor_detector.width").as_int();
}

long ArmorParams::getExpTime() {
    return node->get_parameter("armor_detector.exp_time").as_int();
}

double ArmorParams::getGrayThreshold() {
    return node->get_parameter("armor_detector.gray_threshold").as_double();
}

double ArmorParams::getArmorTypeThreshold() {
    return node->get_parameter("armor_detector.armor_type_threshold").as_double();
}

double ArmorParams::getBin3Threshold1() {
    return node->get_parameter("armor_detector.bin3_threshold_1").as_double();
}

double ArmorParams::getBin3Threshold2() {
    return node->get_parameter("armor_detector.bin3_threshold_2").as_double();
}

double ArmorParams::getBin3Threshold3() {
    return node->get_parameter("armor_detector.bin3_threshold_3").as_double();
}

double ArmorParams::getContoursLength1Min() {
    return node->get_parameter("armor_detector.contours_length1_min").as_double();
}

double ArmorParams::getContoursLength1Max() {
    return node->get_parameter("armor_detector.contours_length1_max").as_double();
}

double ArmorParams::getLedHeightMin() {
    return node->get_parameter("armor_detector.led_height_min").as_double();
}

double ArmorParams::getLedHeightMax() {
    return node->get_parameter("armor_detector.led_height_max").as_double();
}

double ArmorParams::getCosMarkerParallelRadian() {
    return cos(node->get_parameter("armor_detector.marker_parallel_angle").as_double() / RAD2DEG);
}

double ArmorParams::getCosMarkerDirectionRadian() {
    return cos(node->get_parameter("armor_detector.marker_direction_angle").as_double() / RAD2DEG);
}

double ArmorParams::getCosMarkerVerticalRadian() {
    return cos((90 - node->get_parameter("armor_detector.led_vertical_angle").as_double()) / RAD2DEG);
}

double ArmorParams::getLedRatioMin() {
    return node->get_parameter("armor_detector.led_ratio_min").as_double();
}

double ArmorParams::getLedRatioMax() {
    return node->get_parameter("armor_detector.led_ratio_max").as_double();
}

double ArmorParams::getMarkerSizeMin() {
    return node->get_parameter("armor_detector.marker_size_min").as_double();
}

double ArmorParams::getMarkerSizeMax() {
    return node->get_parameter("armor_detector.marker_size_max").as_double();
}

double ArmorParams::getMarkerRatioMin() {
    return node->get_parameter("armor_detector.marker_ratio_min").as_double();
}

double ArmorParams::getMarkerRatioMax() {
    return node->get_parameter("armor_detector.marker_ratio_max").as_double();
}

double ArmorParams::getLedDiff() {
    return node->get_parameter("armor_detector.led_diff").as_double();
}

cv::Scalar ArmorParams::getBlueChMin() {
    return {node->get_parameter("armor_detector.ch1_min_b").as_double(),
            node->get_parameter("armor_detector.ch2_min").as_double(),
            node->get_parameter("armor_detector.ch3_min_b").as_double()};
}

cv::Scalar ArmorParams::getBlueChMax() {
    return {node->get_parameter("armor_detector.ch1_max_b").as_double(),
            node->get_parameter("armor_detector.ch2_max").as_double(),
            node->get_parameter("armor_detector.ch3_max_b").as_double()};
}

cv::Scalar ArmorParams::getRedChMin() {
    return {node->get_parameter("armor_detector.ch1_min_r").as_double(),
            node->get_parameter("armor_detector.ch2_min").as_double(),
            node->get_parameter("armor_detector.ch3_min_r").as_double()};
}

cv::Scalar ArmorParams::getRedChMax() {
    return {node->get_parameter("armor_detector.ch1_max_r").as_double(),
            node->get_parameter("armor_detector.ch2_max").as_double(),
            node->get_parameter("armor_detector.ch3_max_r").as_double()};
}

void ArmorParams::showInfo(const cv::Mat& image, const cv::Mat& roi, const cv::Mat& num, const cv::Mat& coordinate,
                           const cv::Mat& binary) {
    std_msgs::msg::Header header;
    header.set__stamp(rclcpp::Clock().now()).set__frame_id("armor");
    if (!image.empty()) {
        showImagePublisher->publish(*cv_bridge::CvImage(header, "bgr8", image).toImageMsg());
        cv::imshow("output", image);
    }
    if (!roi.empty()) {
        showRoiPublisher->publish(*cv_bridge::CvImage(header, "bgr8", roi).toImageMsg());
        cv::imshow("roi", roi);
    }
    if (!num.empty()) {
        cv::Mat show;
        cvtColor(num, show, cv::COLOR_GRAY2BGR);
        showNumPublisher->publish(*cv_bridge::CvImage(header, "bgr8", num).toImageMsg());
        cv::imshow("num", num);
    }
    if (!coordinate.empty()) {
        showCoordinatePublisher->publish(*cv_bridge::CvImage(header, "bgr8", coordinate).toImageMsg());
        cv::imshow("coordinate", coordinate);
    }
    if (!binary.empty()) {
        cv::Mat show;
        cvtColor(binary, show, cv::COLOR_GRAY2BGR);
        showBinaryPublisher->publish(*cv_bridge::CvImage(header, "bgr8", show).toImageMsg());
        cv::imshow("binary", binary);
    }
    cv::waitKey(1);
}
