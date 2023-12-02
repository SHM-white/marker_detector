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
    std::vector<cv::Point2f> edgePoints(4);
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
                    edgePoints[0] = output.vertex[0];
                    edgePoints[1] = output.vertex[1];
                    edgePoints[2] = output.vertex[2];
                    edgePoints[3] = output.vertex[4];
                }
                break;
            case df::ClassId::ACTIVED:
                break;
        }
    }
    if (rPrecision > 0.0f && aimPrecision > 0.0f) {
        cv::Mat rVec, tVec;
        cv::solvePnP(realPoints, edgePoints, camParams.cameraMatrix, camParams.distCoeffs, rVec, tVec, false,
                     cv::SOLVEPNP_IPPE);
        cv::Mat worldInCamera = cv::Mat::zeros(4, 4, CV_64F);  //外参矩阵
        cv::Rodrigues(rVec, worldInCamera(cv::Rect(0, 0, 3, 3)));
        tVec.copyTo((worldInCamera(cv::Rect(3, 0, 1, 3))));
        worldInCamera.at<double>(3,3) = 1;
        cv::Quatd quat = cv::Quatd::createFromRotMat(worldInCamera(cv::Rect(0, 0, 3, 3)));
        RCLCPP_WARN(rclcpp::get_logger("POINT"), "%f %f %f",tVec.at<double>(0),tVec.at<double>(1),tVec.at<double>(2));
        return {{cv::Point3d(tVec.at<double>(0), tVec.at<double>(1), tVec.at<double>(2)), quat, -1, true}};
    } else {
        return {};
    }
}

void BuffDetector::reinitialize(std::vector<uint8_t>) {

}


