//
// Created by mijiao on 23-12-25.
//

#include "armor_detector.h"

ArmorDetector::ArmorDetector() : Detector("armor") {

}

DetectResults ArmorDetector::detectImpl(const cv::Mat& image) {
    double x, y, z;
    cv::Quatd quat;
    int target_num;
    bool result = markerSensor.ProcessFrameLEDXYZ(image, z, x, y, target_num, quat);
    return {{cv::Point3d(x, y, z), quat, target_num, result}};
}

void ArmorDetector::reinitialize(std::vector<uint8_t>) {

}
