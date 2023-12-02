//
// Created by mijiao on 23-11-23.
//

#ifndef MARKER_DETECTOR_BUFF_DETECTOR_H
#define MARKER_DETECTOR_BUFF_DETECTOR_H

#include "detector/detector.h"

#include "model_manager/ModelManager.h"

#include "params/buff_params.h"
#include "params/cam_params.h"

class BuffDetector : public Detector {
private:
    df::ModelManager modelManager;
    DetectResults detectImpl(const cv::Mat& image) override;
    std::vector<cv::Point3f> realPoints = {
            cv::Point3f(-0.160, 0.158, 0),
            cv::Point3f(0.160, 0.158, 0),
            cv::Point3f(0.186, -0.158, 0),
            cv::Point3f(-0.186, -0.158, 0)
    };

public:
    BuffDetector();
    ~BuffDetector() = default;

    void reinitialize(std::vector<uint8_t>) override;
};


#endif //MARKER_DETECTOR_BUFF_DETECTOR_H
