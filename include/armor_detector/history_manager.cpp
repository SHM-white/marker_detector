//
// Created by robin on 23-3-8.
//
#include "Marker.h"
#include "history_manager.h"

using namespace std;

void history_manager::add_history(vector<Marker>& new_markers) {
    if (markers.empty()) {
        markers = new_markers;
        for (auto& marker: markers) {
            marker.init_type();
        }
        return;
    }
    if (!new_markers.empty()) {
        double mMin;
        cv::Point minP;
        int row = (int) new_markers.size();
        int col = (int) markers.size();
        bool* is_select = new bool[row];
        cv::Mat h(row, col, CV_32FC1);

        for (int i = 0; i < row; i++) {
            for (int j = 0; j < col; j++) {
                h.at<float>(i, j) = new_markers[i] - markers[j];
            }
        }
        cv::minMaxLoc(h, &mMin, NULL, &minP, NULL);
        while (mMin < thresh) {
            h.col(minP.x) = thresh;
            h.row(minP.y) = thresh;
            new_markers[minP.y].number_update(markers[minP.x]);
            new_markers[minP.y].type_update(markers[minP.x]);
            markers[minP.x] = new_markers[minP.y];
            is_select[minP.y] = true;
            markers[minP.x].life = maxLife + 1;
            cv::minMaxLoc(h, &mMin, NULL, &minP, NULL);
        }
        for (int s = 0; s < row; s++) {
            if (!is_select[s]) {
                new_markers[s].init_type();
                markers.push_back(new_markers[s]);
                markers.end()->life = maxLife + 1;
            }
        }
        delete[] is_select;
    }
    for (auto& marker: markers) {
        marker.life--;
    }
    markers.erase(
            std::remove_if(
                    markers.begin(),
                    markers.end(),
                    [](const Marker& marker) {
                        return marker.life <= 0;
                    }),
            markers.end()
    );
}

