//
// Created by mijiao on 23-11-25.
//

#ifndef MARKER_DETECTOR_BUFF_PARAMS_H
#define MARKER_DETECTOR_BUFF_PARAMS_H


#include <boost/serialization/singleton.hpp>

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

class BuffParams : public boost::serialization::singleton<BuffParams> {
private:
    rclcpp::Node::SharedPtr node;
public:
    void init(rclcpp::Node::SharedPtr _node);

    [[nodiscard]] std::string getModelPath() const;

    [[nodiscard]] long getHeight() const;

    [[nodiscard]] long getWidth() const;

    [[nodiscard]] long getExpTime() const;

    [[nodiscard]] long getAnalysisPointsNum() const;

    [[nodiscard]] double getDelayTime() const;

    [[nodiscard]] cv::Point2f getOffset() const;

    bool getIsRed() const;
};


#define buffParams BuffParams::get_mutable_instance()

#endif //MARKER_DETECTOR_BUFF_PARAMS_H
