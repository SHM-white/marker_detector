//
// Created by mijiao on 23-11-25.
//

#ifndef MARKER_DETECTOR_BUFF_PARAMS_H
#define MARKER_DETECTOR_BUFF_PARAMS_H


#include <boost/serialization/singleton.hpp>

#include <rclcpp/rclcpp.hpp>

class BuffParams : public boost::serialization::singleton<BuffParams> {
private:
    rclcpp::Node::SharedPtr node;
public:
    [[nodiscard]] std::string getModelPath() const;

    void init(rclcpp::Node::SharedPtr _node);
};


#define buffParams BuffParams::get_mutable_instance()

#endif //MARKER_DETECTOR_BUFF_PARAMS_H
