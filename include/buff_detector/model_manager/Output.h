//
// Created by mijiao on 23-3-26.
//

#ifndef SIGNSDETECT_OUTPUT_H
#define SIGNSDETECT_OUTPUT_H

#include <vector>
#include <opencv2/opencv.hpp>

namespace df {

    struct Output {
        int id = -1;
        float precision = 0;
        std::array<cv::Point2f, 5> vertex{};
    };

    typedef std::vector<Output> Outputs;

}

#endif //SIGNSDETECT_OUTPUT_H
