//
// Created by mijiao on 23-11-23.
//

#ifndef MARKER_DETECTOR_BUFF_DETECTOR_H
#define MARKER_DETECTOR_BUFF_DETECTOR_H

#include "detector/detector.h"

#include "model_manager/ModelManager.h"
#include "CurveSolver.h"

#include "params/buff_params.h"
#include "params/cam_params.h"

#define SPEED_INTERVAL 3

class BuffDetector : public Detector {
private:
    std::vector<cv::Point3f> realPoints = {
            cv::Point3f(-0.160, 0.158, 0),
            cv::Point3f(0.160, 0.158, 0),
            cv::Point3f(0.186, -0.158, 0),
            cv::Point3f(-0.186, -0.158, 0)
    };

    struct PositionData {
        cv::Point2f aimPoint;
        cv::Point2f rPoint;
        rclcpp::Time timeStamp;
    };

    enum class Mode {
        DAFU, XIAOFU
    } mode = Mode::DAFU;

    df::ModelManager modelManager;

    bool finish{false};
    float direction{0};
    float A{0}, B{0}, W{0}, Phi{0};
    std::deque<PositionData> dataList;
    std::vector<std::pair<float, float>> speedList;
    std::vector<std::pair<float, float>> distanceList;

    DetectResults detectImpl(const cv::Mat& image) override;

public:
    BuffDetector();

    ~BuffDetector() = default;

    void reinitialize(const std::vector<uint8_t>&) override;

    std::optional<cv::Point3f> detectOnce(const cv::Mat& image);

    bool analysis();
};


#endif //MARKER_DETECTOR_BUFF_DETECTOR_H
