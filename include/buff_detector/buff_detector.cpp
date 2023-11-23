//
// Created by mijiao on 23-11-23.
//

#include "buff_detector.h"

BuffDetector::BuffDetector() : Detector("buff_detector") {

}

DetectResults BuffDetector::detectImpl(const cv::Mat& image) {
    df::Outputs outputs = modelManager.infer(image);
    if (outputs.empty()) {
        return {};
    }
    float rPrecision = 0.0f;
    float aimPrecision = 0.0f;
    cv::Point2f rPoint;
    cv::Point2f aimPoint;
    for (auto& output: outputs) {
        switch (output.id) {
            case df::ClassId::R:
                if (output.precision > rPrecision) {
                    rPrecision = output.precision;
                    rPoint = (output.vertex[0] + output.vertex[1] + output.vertex[2] + output.vertex[3] +
                              output.vertex[4]) / 5.0f;
                }
                break;
            case df::ClassId::INACTIVED:
                if (output.precision > aimPrecision) {
                    aimPrecision = output.precision;
                    aimPoint = (output.vertex[0] + output.vertex[1] + output.vertex[2] + output.vertex[4]) / 4.0f;
                }
                break;
            case df::ClassId::ACTIVED:
                break;
        }
    }
    if (rPrecision > 0.0f && aimPrecision > 0.0f) {
        return {{}};
    } else {
        return {};
    }
}

void BuffDetector::reinitialize(std::vector<uint8_t>) {

}


