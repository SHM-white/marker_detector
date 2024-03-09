//
// Created by mijiao on 23-12-25.
//

#include "armor_detector/armor_detector.h"

ArmorDetector::ArmorDetector() : Detector("armor") {

}

DetectResults ArmorDetector::detectImpl(const cv::Mat& image) {
    double x{}, y{}, z{};
    int target_num{};
    cv::Quatd quat;
    bool result = markerSensor.ProcessFrameLEDXYZ(image, z, x, y, target_num, quat);
    armorParams.showInfo(markerSensor.img_show, markerSensor.ROI_bgr, markerSensor.NUM_bgr, markerSensor.coordinate,
                         markerSensor.led_mask);
    DetectResults ret(9);
    for (int i = 0; i < 9; i++) {
        ret[i].id = i;
    }
    if (target_num != 0) {
        ret[target_num] = {cv::Point3d(x, y, z), quat, target_num, armorParams.getIsBig(target_num), result};
    }
    return ret;
}

void ArmorDetector::reinitialize(const std::vector<uint8_t>&) {
    Detector::reinitialize();
    camParams.setSize(armorParams.getHeight(), armorParams.getWidth());
}
