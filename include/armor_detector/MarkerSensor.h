#ifndef MARKERSSENSOR_H
#define MARKERSSENSOR_H

#include <iostream>
#include <vector>
#include<map>
#include <string>
#include <fstream>
#include <numeric>
#include "angles/angles.h"

//Eigen头文件必须在opencv2/core/eigen.hpp前
#include<Eigen/Core>

#include <opencv4/opencv2/opencv.hpp>

#include <opencv2/core/quaternion.hpp>

#include <opencv2/core/eigen.hpp>

#include "Marker.h"
#include "RotRect.h"
#include "history_manager.h"

#include "utils.h"
#include "DigitalRecognition/infer.h"
#include "params/armor_params.h"
#include "armor_detector/DigitalRecognition/openvino/modelManager.h"

#define LABEL_NUM 4

#define cameraMatrix camParams.cameraMatrix
#define distCoeffs camParams.distCoeffs

class MarkerSensor {
public :
    enum SensorStatus {
        STATUS_SUCCESS = 0,
        STATUS_TRACKING,
        STATUS_TRACKLOST0,
        STATUS_TRACKLOST1,
        STATUS_TRACKLOST2 = 8,
        STATUS_DETECTING,
        STATUS_TOP_REDETECT
    };

    enum EnemyStatus {
        STATIC_POS = 0,
        MOVING = 1,
        SWAGGING = 2,
        ROTATING = 3
    };

    using mMarkerType = Marker::MarkerType;

    MarkerSensor();

    int ProcessFrameLEDXYZ(const cv::Mat& img, double& z, double& x, double& y, int& target_num, cv::Quatd& qu);

    int DetectLEDMarker(const cv::Mat& img, Marker& res_marker);

    int TrackLEDMarker(const cv::Mat& img, Marker& res_marker);

    int tgt_selector(std::vector<Marker>& markers);

    int GetLEDMarker(const cv::Mat& roi_mask, Marker& res_marker, int roi_x, int roi_y);

    int GetLEDStrip(const cv::Mat& roi_mask, std::vector<RotRect>& LEDs);

    void NumberDis(Marker& marker);

    bool SetGimbalAngularVelocity(float& angular_velocity);

    //Mat cameraMatrix_inv;
    Mat RT_inv;
    Mat RT;
    //Mat OptcameraMatrix;
    //Mat distCoeffs;

    history_manager _history;
    ModelManager modelManager;

    SensorStatus status = STATUS_DETECTING;
    EnemyStatus enemyStatus = MOVING;
    Marker marker;
    Mat img_gray, img_bgr, img_hsv, img_h, led_mask, img_out;
    Mat target_coor;
    Mat img_show, ROI_bgr, coordinate, NUM_bgr, Image;
    Point2f target;
    mMarkerType manualMT = mMarkerType::ALL;

    std::map<int, int> HP;

    int track_fail_cnt[3];
    bool lock = false;

    //for motion analyse
    std::vector<float> dis_list;

    //for calc angle(via solvePnP)
    std::vector<Point3d> big_armor;
    std::vector<Point3d> small_armor;
    std::vector<Point3d> small_armor_1;
    std::vector<Point2f> img_points;
    std::atomic<bool> if_roi_predict;
    std::atomic<int> predict_roi_cx, predict_roi_cy;
    Mat rvec, tvec;
    double armor_x, armor_y, armor_z, armor_yaw, armor_pitch, armor_roll;
    double gimbal_yaw, gimbal_pitch, gimbal_roll;
    int center_in_rect = 0;
    Point2f offset;
    std::atomic<bool> gimbal_angular_velocity;//向上是True

    int last_number = 0;
};

#endif
