//
// Created by robin on 23-3-8.
//

#ifndef SRC_MARK_H
#define SRC_MARK_H

#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <fstream>
#include <numeric>

//Eigen头文件必须在opencv2/core/eigen.hpp前
#include<Eigen/Core>

#include <opencv4/opencv2/opencv.hpp>
#include "RotRect.h"
#include "DigitalRecognition/infer.h"

using namespace cv;

class Marker {
public:
    enum MarkerType {
        ALL = 0,
        SMALL = 1,
        BIG = 2
    };
    enum RotatingDir {
        NOT_AVAILABLE = 0,
        COUNTER_CLOCK_WISE = 1,
        CLOCK_WISE = 2
    };
    RotRect LEDs[2];
    cv::Point2f kpts[4];
    cv::Point2f num_kpts[4];
    Rect bbox;
    int roi_x, roi_y;
    float number_count[9]{0};
    float number_count_sum{0};
    int armor_cnt = 0;
    int life = 10;

    float yaw{};
    float length{};
    float decision_points{};
    int number{0};
    MarkerType armor_type = SMALL;
    InferResultAsync typeCall;
    int hp = 0;
    float depth = 0;
    int armor_count = 4;
    RotatingDir rotatingDir = NOT_AVAILABLE;

    int ComputeKeyPoints();

    int ComputeBBox();

    int Draw(Mat& img);

    float operator-(Marker& other);

    void number_update(Marker& marker_old);

    void type_update(Marker& marker_old);

    void init_type();

    Rect getAbsBbox() const;
};

#endif //SRC_MARK_H
