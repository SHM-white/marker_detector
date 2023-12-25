//
// Created by mijiao on 23-12-25.
//

#ifndef MARKER_DETECTOR_ARMOR_DETECTOR_H
#define MARKER_DETECTOR_ARMOR_DETECTOR_H

#include <detector/detector.h>
#include "MarkerSensor.h"

class ArmorDetector : public Detector {
private:
    MarkerSensor markerSensor;

    DetectResults detectImpl(const cv::Mat& image) override;

public:
    ArmorDetector();

    void reinitialize(std::vector<uint8_t>) override;
};


#endif //MARKER_DETECTOR_ARMOR_DETECTOR_H
