//
// Created by robin on 23-3-13.
//

#ifndef SRC_ROTRECT_H
#define SRC_ROTRECT_H
#include <opencv4/opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<opencv2/calib3d/calib3d.hpp>

class RotRect {
public:
    cv::Point2f center;
    cv::Point2f dir;
    float width;
    float height;

    RotRect() : width(0), height(0) {};
    explicit RotRect(const cv::Rect & rect);
};

#endif //SRC_ROTRECT_H
