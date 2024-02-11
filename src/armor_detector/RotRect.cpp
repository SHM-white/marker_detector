//
// Created by robin on 23-3-13.
//
#include "armor_detector/RotRect.h"

RotRect::RotRect(const cv::Rect &rect): width(rect.width), height(rect.height) {
    center.x = rect.x + rect.width * 0.5f;
    center.y = rect.y + rect.height * 0.5f;
    dir.x = 1;
    dir.y = 0;
}
