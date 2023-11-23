//
// Created by mijiao on 23-11-20.
//

#include "detect_result.h"

marker_detector::msg::DetectResult DetectResult::toRosMsg() {
    marker_detector::msg::DetectResult ret;
    ret.pose.position.set__x(point[0]).set__y(point[1]).set__z(point[2]);
    ret.pose.orientation.set__x(quat.x()).set__y(quat.y()).set__z(quat.z()).set__w(quat.w());
    ret.set__id(id).set__big(big).set__detect_success(detectSuccess).set__selected(selected);
    return ret;
}

marker_detector::msg::DetectResults DetectResultsStamped::toRosMsg() {
    marker_detector::msg::DetectResults ret;
    ret.header.stamp = time;
    ret.header.frame_id = "markers";
    for (size_t i = 0; i < results.size(); i++) {
        ret.detect_results[i] = results[i].toRosMsg();
    }
    return ret;
}