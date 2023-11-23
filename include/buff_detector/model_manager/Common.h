//
// Created by mijiao on 23-5-21.
//

#ifndef DAFUDETECT_COMMON_H
#define DAFUDETECT_COMMON_H

#include <opencv2/opencv.hpp>

namespace df {

    constexpr int tensorLen = 3549;
    constexpr int colorNum = 1;
    constexpr int classNum = 3;
    constexpr int size = 416;
    constexpr float scoreThres = 0.5;
    constexpr float nmsThres = 0.5;

    enum ClassId {
        R, INACTIVED, ACTIVED
    };


} // df


#endif //DAFUDETECT_COMMON_H
