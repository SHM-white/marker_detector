//
// Created by mijiao on 23-11-20.
//

#include "detector/detect_result.h"

marker_detector::msg::DetectResult DetectResult::toRosMsg() const {
    marker_detector::msg::DetectResult ret;
    ret.pose.position.set__x(point.x).set__y(point.y).set__z(point.z);
    ret.pose.orientation.set__x(quaternion.x).set__y(quaternion.y).set__z(quaternion.z).set__w(quaternion.w);
    ret.set__id(id).set__is_big(isBig).set__detect_success(detectSuccess);
    return ret;
}

marker_detector::msg::DetectResults DetectResultsStamped::toRosMsg() {
    marker_detector::msg::DetectResults ret;
    ret.header.stamp = time;
    ret.header.frame_id = "markers";
    for (size_t i = 0; i < results.size(); i++) {
        ret.detect_results.push_back(results[i].toRosMsg());
    }
    return ret;
}