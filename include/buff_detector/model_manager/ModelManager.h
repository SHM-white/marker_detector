//
// Created by mijiao on 23-5-21.
//

#ifndef DAFUDETECT_MODELMANAGER_HPP
#define DAFUDETECT_MODELMANAGER_HPP

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>

#include "params/buff_params.h"

#include "Common.h"
#include "Output.h"

namespace df {

    class ModelManager {
    private:
        ov::Core core;
        ov::CompiledModel model;
        ov::InferRequest inferRequest;

        std::vector<std::pair<float, float>> grids;
        std::vector<float> strides;

        Outputs postProcess(const ov::Tensor& outputTensor, float scaleRate);

        void buildGirdsAndStrides();

    public:
        ModelManager();

        Outputs infer(const cv::Mat& img);

    };

}
#endif //DAFUDETECT_MODELMANAGER_HPP
