//
// Created by mijiao on 23-11-23.
//

#ifndef MARKER_DETECTOR_BUFF_DETECTOR_H
#define MARKER_DETECTOR_BUFF_DETECTOR_H

#include "detector/detector.h"

#include "model_manager/ModelManager.h"

class BuffDetector : public Detector {
private:
    df::ModelManager modelManager;
    DetectResults detectImpl(const cv::Mat& image) override;

public:
    BuffDetector();
    ~BuffDetector() = default;

    void reinitialize(std::vector<uint8_t>) override;
};


#endif //MARKER_DETECTOR_BUFF_DETECTOR_H
