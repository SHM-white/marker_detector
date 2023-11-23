//
// Created by mijiao on 23-5-21.
//

#include "ModelManager.h"

namespace df {
    ModelManager::ModelManager() {
        model = core.compile_model(
                "./install/marker_detector/lib/marker_detector/buff_detector/model/dafu.xml",
                "CPU",
                {{ov::hint::performance_mode.name(), ov::hint::PerformanceMode::THROUGHPUT}}
        );
        inferRequest = model.create_infer_request();
        buildGirdsAndStrides();
    }

    void ModelManager::buildGirdsAndStrides() {
        const int hw[] = {52, 26, 13};
        const int _strides[] = {8, 16, 32};
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < hw[i]; j++) {
                for (int k = 0; k < hw[i]; k++) {
                    grids.emplace_back(k, j);
                    strides.push_back((float) _strides[i]);
                }
            }
        }
    }

    Outputs ModelManager::postProcess(const ov::Tensor& outputTensor, float scaleRate) {
        std::vector<cv::Rect> boxes(tensorLen);
        std::vector<float> scores(tensorLen);
        std::vector<int> indexes;
        for (int i = 0; i < tensorLen; i++) {
            float* x = 15 * i + outputTensor.data<float>();
            for (int j = 0; j < 10; j += 2) {
                x[j] = (x[j] + grids[i].first) * strides[i];
                x[j + 1] = (x[j + 1] + grids[i].second) * strides[i];
            }
            std::vector<cv::Point2f> bbox = {
                    {x[0], x[1]},
                    {x[2], x[3]},
                    {x[4], x[5]},
                    {x[6], x[7]},
                    {x[8], x[9]}
            };

            boxes[i] = cv::boundingRect(bbox);
            scores[i] = x[10];
        }
        cv::dnn::NMSBoxes(boxes, scores, scoreThres, nmsThres, indexes);

        std::vector<Output> outputs(indexes.size());

        for (size_t i = 0; i < indexes.size(); i++) {
            int idx = indexes[i];
            float* x = 15 * idx + outputTensor.data<float>();

            Output output;
            output.precision = x[10];

            for (int j = 0; j < 10; j += 2) {
                output.vertex[j / 2] = cv::Point2f(x[j], x[j + 1]);
            }

            int maxClassIndex = 0;
            float classValue = 0;
            for (int j = 0; j < colorNum; j++) {
                float color = x[11 + j];
                for (int k = 0; k < classNum; k++) {
                    float cls = x[11 + colorNum + k];
                    float colorClass = (color + cls) / 2;
                    if (colorClass > classValue) {
                        classValue = colorClass;
                        maxClassIndex = j * classNum + k;
                    }
                }
            }
            output.id = maxClassIndex;

            for (auto& point: output.vertex) {
                point.x /= scaleRate;
                point.y /= scaleRate;
            }
            outputs[i] = output;
        }
        return outputs;
    }

    Outputs ModelManager::infer(const cv::Mat& img) {
        float scale = (float) size / (float) img.rows;
        cv::Mat blob;
        cv::dnn::blobFromImage(img, blob, scale, cv::Size(416, 416));

        auto tensor = inferRequest.get_input_tensor();
        memcpy(tensor.data(), blob.data, tensor.get_byte_size());
        inferRequest.infer();
        return postProcess(inferRequest.get_output_tensor(), scale);
    }
}
