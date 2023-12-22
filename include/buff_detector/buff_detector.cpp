//
// Created by mijiao on 23-11-23.
//

#include "buff_detector.h"

BuffDetector::BuffDetector() : Detector("buff_detector") {

}

void BuffDetector::reinitialize(std::vector<uint8_t> config) {
    camParams.setSize(buffParams.getHeight(), buffParams.getWidth());
    if (config[0] == 0) {
        mode = Mode::DAFU;
    } else {
        mode = Mode::XIAOFU;
    }
}

DetectResults BuffDetector::detectImpl(const cv::Mat& image) {
    std::vector<cv::Mat> channels;
    cv::split(image, channels);
    cv::Mat bin;
    if (buffParams.getIsRed()) {
        bin = channels[2];
    } else {
        bin = channels[0];
    }
    cv::cvtColor(bin, bin, cv::COLOR_GRAY2BGR);
    auto result = detectOnce(image);
    if (result.has_value()) {
        return {{result.value(), cv::Quatd(), -1, true}};
    } else {
        return {{}};
    }
}

std::optional<cv::Point3f> BuffDetector::detectOnce(const cv::Mat& image) {
    rclcpp::Time now = rclcpp::Clock().now();
    df::Outputs outputs = modelManager.infer(image);
    if (outputs.empty()) {
        return std::nullopt;
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
        if (finish) {
            auto timePoint = (float) now.seconds();
            auto theta = [this](float time) { return -A / W * cos(W * time + Phi) + B * time; };

            float deltaTheta;
            if (mode == Mode::DAFU) {
                deltaTheta = theta(timePoint + (float) buffParams.getDelayTime()) - theta(timePoint);
            } else {
                deltaTheta = B * (float) buffParams.getDelayTime();
            }

            if (direction < 0) {
                deltaTheta = -deltaTheta;
            }


#ifdef DEBUG_DETAIL
            std::cout << "predict Speed:" << A * sin(W * timePoint + Phi) + B << std::endl;
#endif

            auto rotate = [&rPoint](const cv::Vec2f& _vec, float theta) {
                auto vec = _vec - cv::Vec2f(rPoint);
                return cv::Point2f(vec.val[0] * cos(theta) - vec.val[1] * sin(theta),
                                   vec.val[0] * sin(theta) + vec.val[1] * cos(theta)) + rPoint;
            };

            for (auto& point: edgePoints) {
                point = rotate(point, deltaTheta);
            }

            for (auto& point: edgePoints) {
                point += buffParams.getOffset();
            }

            std::vector<float> rVec;
            std::vector<float> tVec;
            cv::solvePnP(realPoints, edgePoints, camParams.cameraMatrix, camParams.distCoeffs, rVec, tVec, false,
                         cv::SOLVEPNP_IPPE);
            return cv::Point3f(tVec[0], tVec[1], tVec[2]);

#ifdef DEBUG
            cv::Point2f target2 =
                    cv::Point2f(OA.val[0] * cos(deltaTheta) - OA.val[1] * sin(deltaTheta),
                                OA.val[0] * sin(deltaTheta) + OA.val[1] * cos(deltaTheta)) + rPoint;
            cv::circle(matWithTimeStamp.mat, target2, 2, cv::Scalar(0, 255, 0), -1);
            cv::imshow("target", matWithTimeStamp.mat);
            //assert(matWithTimeStamp.timeStamp == result.timeStamp);
            cv::waitKey(1);
#endif

        } else {
            dataList.push_back({aimPoint, rPoint, now});
            if (dataList.size() > (size_t) buffParams.getAnalysisPointsNum()) {
                finish = analysis();
            }
            return std::nullopt;
        }
    }
    return std::nullopt;
}

bool BuffDetector::analysis() {
    if (dataList.size() < SPEED_INTERVAL) {
        return false;
    }

    for (size_t i = 0; i < dataList.size() - SPEED_INTERVAL; i++) {
        cv::Vec2f OA = dataList[i].aimPoint - dataList[i].rPoint;
        cv::Vec2f OB = dataList[i + SPEED_INTERVAL].aimPoint - dataList[i + SPEED_INTERVAL].rPoint;
        rclcpp::Duration duration = dataList[i + SPEED_INTERVAL].timeStamp - dataList[i].timeStamp;

        float deltaAngle = acos(normalize(OA).dot(normalize(OB)));

        cv::Vec3f OA3(OA.val[0], OA.val[1], 0);
        cv::Vec3f OB3(OB.val[0], OB.val[1], 0);
        cv::Vec3f _direction = OA3.cross(OB3);

        if (i < dataList.size() - SPEED_INTERVAL - 1) {
            direction = _direction.val[2] + direction;
        }

        auto deltaTime = (float) duration.seconds();

        float speed = deltaAngle / deltaTime;
        float timePoint = (float) dataList[i].timeStamp.seconds() + deltaTime / 2;
        if (speed < 5) {
            speedList.emplace_back(timePoint, speed);
        }


#ifdef DEBUG_DETAIL
        std::string s;
            for (float j = 0; j < speed; j += 0.1f) {
                s.push_back('x');
            }
            std::cout << std::setw(80) << s << std::setw(10) << "Speed:" << std::setw(10) <<speed << std::setw(10)
                      << " Time:" << std::setw(10) << timePoint << std::setw(10) << std::endl;
#endif
    }


    auto timePoint0 = (float) dataList[0].timeStamp.seconds();
    distanceList.emplace_back(timePoint0, 0);

    for (size_t i = 1; i < dataList.size(); i++) {
        cv::Vec2f OA = dataList[i - 1].aimPoint - dataList[i - 1].rPoint;
        cv::Vec2f OB = dataList[i].aimPoint - dataList[i].rPoint;
        float deltaAngle = acos(normalize(OA).dot(normalize(OB)));

        cv::Vec3f OA3(OA.val[0], OA.val[1], 0);
        cv::Vec3f OB3(OB.val[0], OB.val[1], 0);
        cv::Vec3f _direction = OA3.cross(OB3);
        if (_direction.val[2] < 0) {
            deltaAngle = -deltaAngle;
        }

        auto timePoint = (float) dataList[i].timeStamp.seconds();
        float angle = distanceList[i - 1].second + deltaAngle;

        distanceList.emplace_back(timePoint, angle);
    }


    df::CurveSolver curveSolver(speedList);
    df::Ans ans = curveSolver.solve();
    A = (float) ans.a;
    B = (float) ans.b;
    W = (float) ans.w;
    Phi = (float) ans.phi;

    return true;
}
