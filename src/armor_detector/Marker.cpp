//
// Created by robin on 23-3-8.
//
#include "armor_detector/Marker.h"
#include "armor_detector/utils.h"

using namespace std;

int Marker::ComputeKeyPoints() {
    int is_dir0_down = (LEDs[0].dir.dot(Point2f(0, 1))) > 0 ? 1 : -1;
    int is_dir1_down = (LEDs[1].dir.dot(Point2f(0, 1))) > 0 ? 1 : -1;
    float expand = 1.9f;
    kpts[0] = LEDs[0].center - is_dir0_down * LEDs[0].dir * LEDs[0].height * 0.5f; //top left
    kpts[2] = LEDs[0].center + is_dir0_down * LEDs[0].dir * LEDs[0].height * 0.5f; //bottom left
    kpts[1] = LEDs[1].center - is_dir1_down * LEDs[1].dir * LEDs[1].height * 0.5f; //top right
    kpts[3] = LEDs[1].center + is_dir1_down * LEDs[1].dir * LEDs[1].height * 0.5f; //bottom right
    num_kpts[0] = LEDs[0].center - is_dir0_down * LEDs[0].dir * LEDs[0].height * 0.5f * expand;
    num_kpts[2] = LEDs[0].center + is_dir0_down * LEDs[0].dir * LEDs[0].height * 0.5f * expand;
    num_kpts[1] = LEDs[1].center - is_dir1_down * LEDs[1].dir * LEDs[1].height * 0.5f * expand;
    num_kpts[3] = LEDs[1].center + is_dir1_down * LEDs[1].dir * LEDs[1].height * 0.5f * expand;

    return 0;
}

int Marker::ComputeBBox() {
    float max_x = 0, max_y = 0;
    float min_x = 9999, min_y = 9999;
    for (auto kpt: kpts) {
        // may be wrong
        if (kpt.x < min_x) {
            min_x = kpt.x;
        }
        if (kpt.x > max_x) {
            max_x = kpt.x;
        }
        if (kpt.y < min_y) {
            min_y = kpt.y;
        }
        if (kpt.y > max_y) {
            max_y = kpt.y;
        }
    }
    bbox.x = (int) min_x;
    bbox.y = (int) min_y;
    bbox.width = (int) (max_x - min_x);
    bbox.height = (int) (max_y - min_y);
    return 0;
}

int Marker::Draw(Mat& img) {
    ComputeKeyPoints();
    // cv::line(img, kpts[0], kpts[1], cv::Scalar(255, 0, 0), 1);
    // cv::line(img, kpts[1], kpts[2], cv::Scalar(0, 255, 0), 1);
    // cv::line(img, kpts[2], kpts[3], cv::Scalar(0, 0, 255), 1);
    // cv::line(img, kpts[3], kpts[0], cv::Scalar(255, 255, 0), 1);
    circle(img, kpts[0], 2, Scalar(255, 255, 255), 2);
    circle(img, kpts[1], 2, Scalar(255, 255, 255), 2);
    circle(img, kpts[2], 2, Scalar(255, 255, 255), 2);
    circle(img, kpts[3], 2, Scalar(255, 255, 255), 2);
    return 0;
}

float Marker::operator-(Marker& other) {
    float deviation;
    other.yaw = paraDistance(other.LEDs[0], other.LEDs[1]) / other.LEDs->height;
    other.length = other.LEDs->height;
    yaw = paraDistance(LEDs[0], LEDs[1]) / LEDs->height;
    length = LEDs->height;

    Rect bbox1 = getAbsBbox();
    Rect bbox2 = other.getAbsBbox();
    float distance = (float) (bbox1 & bbox2).area() / (float) (bbox1 | bbox2).area();
    cout << "distance::::::::;" << distance << endl;
    // TODO: 上面计算了交并比，需要把交并比也纳入考量范围
    deviation = float(abs(other.yaw - yaw) * 2.3
                      + abs(other.length - length) * 1.9
                      + (10 - other.life) * 0.5 + distance * 400);

    return deviation;
}

void Marker::number_update(Marker& marker_old) {
    marker_old.number_count[typeCall().id] += typeCall().confidence;
    number_count_sum = marker_old.number_count_sum + 1;
    cout << "id: " << typeCall().id << "\t\tconf: " << typeCall().confidence << endl;
    number = typeCall().id;
//    for (int i = 1; i <= 8; i++) {
//        number_count[i] = marker_old.number_count[i];
//        if (marker_old.number_count[i] / number_count_sum >= 0.5) {
//            number = i;
//        }
//    }
    if (typeCall().confidence < 0.85) {
        number = 0;
    }
    cout << "final id: " << number << endl;
}

void Marker::type_update(Marker& marker_old) {
    if (typeCall().big) {
        armor_type = BIG;
    } else {
        armor_type = SMALL;
    }
    if (armor_type != marker_old.armor_type) {
        marker_old.armor_cnt++;
    } else {
        marker_old.armor_cnt = 0;
    }

    if (marker_old.armor_cnt >= 5) {
        marker_old.armor_type = armor_type;
        marker_old.armor_cnt = 0;
    } else {
        armor_type = marker_old.armor_type;
    }
}

void Marker::init_type() {
    if (typeCall().confidence >= 0.8) {
        if (typeCall().big) {
            armor_type = BIG;
        } else {
            armor_type = SMALL;
        }
        number = typeCall().id;
    } else {
        number = 0;
    }
    cout << "id: " << typeCall().id << "\t\tconf: " << typeCall().confidence << endl;

}


Rect Marker::getAbsBbox() const {
    cv::Rect box = bbox;
    box.x += roi_x;
    box.y += roi_y;
    return box;
}