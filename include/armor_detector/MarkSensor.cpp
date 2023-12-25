#include "MarkerSensor.h"
#include <string>
#include <opencv2/core/quaternion.hpp>

using namespace cv;
using namespace std;

int otsuThreshold(const Mat& src) {
    cv::Mat img = src.clone();
    return cv::threshold(img, img, 0, 0, cv::THRESH_OTSU);
}

/**
 * @brief Construct a new Mark Sensor:: Mark Sensor object
 * MarkSensor构造函数
 * @param ap_  算法参数
 * @param cp_  相机参数
 * @param mp_  装甲板参数
 */
MarkerSensor::MarkerSensor() {
    cameraMatrix = camParams.cameraMatrix;
    invert(cameraMatrix, cameraMatrix_inv, DECOMP_SVD);
    distCoeffs = camParams.distCoeffs;
    OptcameraMatrix = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, Size(640, 480), 1);

    //init real armor size, for solvePnP
    big_armor = {Point3d(-0.1135, 0.031, 0), Point3d(0.1135, 0.031, 0), Point3d(0.1135, -0.031, 0),
                 Point3d(-0.1135, -0.031, 0)};
    small_armor = {Point3d(-0.0685, 0.031, 0), Point3d(0.0685, 0.031, 0), Point3d(0.0685, -0.031, 0),
                   Point3d(-0.0685, -0.031, 0)};
    small_armor_1 = {Point3d(-0.0675, 0.02898, -0.007764), Point3d(0.0675, 0.02898, -0.007764),
                     Point3d(0.0675, -0.02898, 0.007764), Point3d(-0.0675, -0.02898, 0.007764)};
    offset = Point2f(camParams.offsetW, camParams.offsetH);
    if_roi_predict = false;
    modelManager.init();
}

/**
 * @brief MarkerSensor::SetGimbalAngularVelocity
 * 获取卡尔曼中云台角速度信息，放入gimbal_angular_velocity中
 * @param angular_velocity 云台角速度
 * @return 转动方向，向上为True
 */
bool MarkerSensor::SetGimbalAngularVelocity(float& angular_velocity) {
    if (angular_velocity < 0) {
        gimbal_angular_velocity.store(false);
    } else {
        gimbal_angular_velocity.store(true);
    }
    return gimbal_angular_velocity.load();
}


/**
 * @brief MarkerSensor::NumberDis
 * 识别装甲版上的数字，结果存放在Marker对象的no成员中
 * @param marker 待识别的装甲版
 */
void MarkerSensor::NumberDis(Marker& marker) {
    Mat img;
    Point2f res_pts[4];
    const int heightCrop = 0, widthCrop = 6;
    const float targetValue = 160.f;
    const float maxThresh = targetValue / 4.f;
    const float a = 0.25;
    res_pts[0] = Point(-widthCrop, -heightCrop);
    res_pts[2] = Point(-widthCrop, 32 + heightCrop);
    res_pts[1] = Point(32 + widthCrop, -heightCrop);
    res_pts[3] = Point(32 + widthCrop, 32 + heightCrop);
    Mat transform = getPerspectiveTransform(marker.num_kpts, res_pts);
    if (status == STATUS_DETECTING) {
        warpPerspective(MarkerSensor::Image, img, transform, Size(32, 32));
    } else {
        warpPerspective(MarkerSensor::ROI_bgr, img, transform, Size(32, 32));
    }
    cv::Mat img_gray(img.rows, img.cols, CV_8UC1, cv::Scalar(0));
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
    float threshold = targetValue / (float) otsuThreshold(img_gray);
    // 低通滤波
    static float thresh_ = threshold;
    if (::isnan(threshold) || ::isinf(threshold)) {
        threshold = thresh_;
    } else {
        threshold = a * thresh_ + (1 - a) * threshold;
    }

    if (threshold > maxThresh) {
        threshold = maxThresh;
    }

    float thresh = targetValue / threshold;
    for (int _row = 0; _row < img_gray.rows; _row++) {
        for (int _col = 0; _col < img_gray.cols; _col++) {
            if (img_gray.at<uchar>(_row, _col) < thresh)
                img_gray.at<uchar>(_row, _col) = 0;
        }
    }
    img_gray *= threshold;
    if (armorParams.getIfShow()) {
        NUM_bgr = img_gray.clone();
//         保存装甲板图片用于训练神经网络
//        static unsigned long long noooo = 916949;
//            cv::imwrite(std::string("/home/nuaa/5/") + std::to_string(noooo) + ".png", NUM_bgr);
//        noooo++;
    }
    marker.typeCall = modelManager.infer_async(img_gray);
}


/**
 * @brief MarkerSensor::tgt_selector
 * 计算装甲板权重，在识别到多个装甲时决定击打目标
 * @param markers 识别到的所有装甲
 * @return int 要击打的装甲索引
 */
int MarkerSensor::tgt_selector(vector<Marker>& markers) {
    int res_idx = -1;
    float minDist = 9999;
    for (int i = 0; i < markers.size(); i++) {
        Point2f camera_c(led_mask.cols / 2.0, led_mask.rows / 2.0);
        Point2f marker_c((markers[i].LEDs[0].center + markers[i].LEDs[1].center) * 0.5);
        float dist2c = norm(camera_c - marker_c);

        float decide_p1 = MAX(1 - 0.002 * dist2c, 0);

        if (status != STATUS_DETECTING) {
            markers[i].decision_points = decide_p1;
            if (armorParams.getIsNumber()) {
                if (markers[i].number == 0) {
                    markers[i].decision_points = -1e20;
                }
            }
        } else {
            // second param: area of marker
            int area = markers[i].bbox.area();
            float decide_p2 = MIN(0.001 * area + 0.1, 1);
            markers[i].decision_points = 0.5f * decide_p1 + decide_p2;

            if (marker.rotatingDir == Marker::COUNTER_CLOCK_WISE &&
                markers[i].LEDs[0].height < markers[i].LEDs[1].height ||
                marker.rotatingDir == Marker::CLOCK_WISE && markers[i].LEDs[0].height > markers[i].LEDs[1].height) {
                markers[i].decision_points *= 80;//TODO 具体数值需要调参
            }
        }

        if (lock && last_number != 0 && markers[i].number != last_number) {
            markers[i].decision_points = -1e21;
        }

//        if (markers[i].number != 7) {
//            markers[i].decision_points = -1e21;
//        }

    }

    /*
     * Only For Sentry
     * TODO: 添加Sentry参数且添加判断条件后解除注释
     */
//        if (markers.size() >= 1) {
//            res_idx = FindLowestHP(markers, HP);
//            if (res_idx) {
//                markers[res_idx - 1].decision_points += 20; //the res_idx was added 1 in the end of FindLowestHP
//                tracked_num = markers[res_idx - 1].number;
//            }
//        }


    float max_points = -1e20;
    for (int j = 0; j < markers.size(); j++) {
        if (markers[j].decision_points > max_points) {
            max_points = markers[j].decision_points;
            res_idx = j;
        }
    }


    if ((!lock || last_number == 0) && res_idx != -1) {
        last_number = markers[res_idx].number;
    }

    return res_idx;
}


/**
 * @brief MarkerSensor::GetLEDStrip
 * 在图像中识别灯条
 * @param roi_mask 图像（需二值化）
 * @param LEDs 找到的所有灯条
 * @return int
 */
int MarkerSensor::GetLEDStrip(cv::Mat& roi_mask, vector<RotRect>& LEDs) {
    vector<vector<Point>> tmp_countours;
    vector<vector<Point>*> pContours;
    // 3 rad
    findContours(roi_mask, tmp_countours, RETR_EXTERNAL, CHAIN_APPROX_NONE);//找轮廓
    checkContoursCompleteness(tmp_countours, roi_mask);

    for (auto& contour: tmp_countours) {
        int cont_sz = static_cast<int>(contour.size());//强制类型转换
        if (cont_sz >= armorParams.getContoursLength1Min() && cont_sz <= armorParams.getContoursLength1Max())
            pContours.push_back(&contour);
        else
            cout << "led size failed: size= " << cont_sz << endl;;
    }

    //PCA
    for (auto& pContour: pContours) {
        RotRect LED;
        if (PCALEDStrip(*pContour, LED) == STATUS_SUCCESS) {
            // check length
            if (LED.height < armorParams.getLedHeightMin() || LED.height > armorParams.getLedHeightMax()) {
                cout << "led height failed: height= " << LED.height << endl;
                continue;
            }
            cout << "<<<<<<<<<<<<<<<<<<" << LED.center << ">>>>>>>>>>>>>>>>>" << endl;
            // vertical to ground
            if (fabs(LED.dir.dot(cv::Point2f(0, 1))) < armorParams.getCosMarkerVerticalRadian()) {
                cout << "led angle failed: angle= " << fabs(LED.dir.dot(cv::Point2f(0, 1))) << endl;
                continue;
            }


            //check ratio
            float ratio = LED.height / LED.width;   //>1

            if (ratio < armorParams.getLedRatioMin() || ratio > armorParams.getLedRatioMax()) {
                cout << "Height:" << LED.height << " Width:" << LED.width << endl;
                cout << "led ratio failed: ratio= " << ratio << endl;
                continue;
            }
            LEDs.push_back(LED);
        }
    }
}


/**
 * @brief MarkerSensor::GetLEDMarker
 * 在图像中识别装甲
 * @param roi_mask 图像（需二值化）
 * @param res_marker 输出的装甲板
 * @return int 0成功 -1失败
 */
int MarkerSensor::GetLEDMarker(cv::Mat& roi_mask, Marker& res_marker, int roi_x, int roi_y) {
    vector<RotRect> LEDs;
    GetLEDStrip(roi_mask, LEDs);//找灯条
    cout << "led size:" << LEDs.size() << endl;
    if (LEDs.size() < 2) {
        printf("LED num < 2 ! \n");
        return -1;
    }

    sort(LEDs.begin(), LEDs.end(), SortLEDs);
    // search marker
    vector<Marker> markers;

    for (int i = 0; i < LEDs.size(); i++)//循环匹配装甲
    {
        for (int j = i + 1; j < LEDs.size(); j++) {
            cv::Point2f c2c = LEDs[i].center - LEDs[j].center;//centre to centre建立一个向量
            float para_dist = paraDistance(LEDs[i], LEDs[j]);//计算平行线间的距离
            float distance_hori = para_dist;

            //check x distance
            //cout<<"distance_hori"<<distance_hori<<endl;
            if (distance_hori > armorParams.getMarkerSizeMax() || distance_hori < armorParams.getMarkerSizeMin()) {
                printf("LED horizontal not satisfied !\n");
                continue;
            }

            float max_height = max(LEDs[i].height, LEDs[j].height);
            float led_diff = fabs(LEDs[i].height - LEDs[j].height) / max_height;
            //check y distance
            if (abs(c2c.y) > max_height) {
                printf("LED vertical diff not satisfied !\n");
                continue;
            }

            // check height difference
            if (led_diff > armorParams.getLedDiff()) {
                printf("LED difference not satisfied !\n"); // 491,408,514,510// 56,170 ,91,175
                continue;
            }

            //check LED parallel
            if (fabs(LEDs[i].dir.dot(LEDs[j].dir)) < armorParams.getCosMarkerParallelRadian()) {
                printf("LED parallel not satisfied !\n");
                continue;
            }

            // check direction
            float distance = norm(c2c);
            cv::Point2f direction = c2c / distance;
            if (fabs(direction.dot(cv::Point2f(1, 0))) < armorParams.getCosMarkerDirectionRadian()) {
                printf("Marker direction not satisfied !\n");
                continue;
            }

            // build marker
            Marker tmp_marker;

            // check marker width/height ratio
            float marker_width = para_dist;
            float marker_height = (LEDs[i].height + LEDs[j].height) * 0.5f;
            float marker_size_ratio = marker_width / marker_height;
            //cout<<"chang div kuan:"<<marker_size_ratio <<endl;
            if (marker_size_ratio > armorParams.getMarkerSizeMax() ||
                marker_size_ratio < armorParams.getMarkerSizeMin()) {
                //printf("Marker size ratio not satisfied !\n");
                continue;
            }

            if (c2c.x > 0) {
                tmp_marker.LEDs[0] = LEDs[j];//0 on the left
                tmp_marker.LEDs[1] = LEDs[i];
            } else {
                tmp_marker.LEDs[0] = LEDs[i];
                tmp_marker.LEDs[1] = LEDs[j];
            }
            if (j > i + 1) {
                if (abs((LEDs[j - 1].center - LEDs[j].center).y) < LEDs[j].height ||
                    abs((LEDs[j - 2].center - LEDs[j - 1].center).y) < LEDs[j].height) {
                    break;
                }
            }
            tmp_marker.roi_x = roi_x;
            tmp_marker.roi_y = roi_y;
            tmp_marker.ComputeKeyPoints();
            tmp_marker.ComputeBBox();
            NumberDis(tmp_marker);
            markers.push_back(tmp_marker);
            break;
        }
    }

    if (markers.empty()) {
        cout << "markers is empty" << endl;
        return -1;
    }

    // decide which marker to shoot
    for (auto& marker_: markers) {
        // TODO: 目前为方便调试暂时不启动装甲板类型识别（默认为小）
//        markerTypeIdentify(marker_, cp, mp, manualMT);
    }
    //_history.add_history(markers);

//    //这一段放到了tracker中
//    for (int i = 0; i < markers.size(); i++) {
//        markers[i].init_type();
//        if (markers[i].number == 3) {
//            if (ap.is3big) {
//                markers[i].armor_type = Marker::BIG;
//                markers[i].armor_count = 2;
//            } else {
//                markers[i].armor_type = Marker::SMALL;
//                markers[i].armor_count = 4;
//            }
//        } else if (markers[i].number == 4) {
//            if (ap.is4big) {
//                markers[i].armor_type = Marker::BIG;
//                markers[i].armor_count = 2;
//            } else {
//                markers[i].armor_type = Marker::SMALL;
//                markers[i].armor_count = 4;
//            }
//        } else if (markers[i].number == 5) {
//            if (ap.is5big) {
//                markers[i].armor_type = Marker::BIG;
//                markers[i].armor_count = 2;
//            } else {
//                markers[i].armor_type = Marker::SMALL;
//                markers[i].armor_count = 4;
//            }
//        } else if (markers[i].number == 1 || markers[i].number == 2 || markers[i].number == 6) {
//            markers[i].armor_count = 4;
//        } else if (markers[i].number == 7) {
//            markers[i].armor_count = 3;
//        }
//
//    }

    int res_idx = tgt_selector(markers);
    if (markers[res_idx].number == 0)
        return -1;
    if (res_idx == -1) {
        cout << "without number or deviation is too big" << endl;
        return -1;
    }
    res_marker = markers[res_idx];

    cout << "marker type is" << res_marker.armor_type << endl;

    return 0;
}


/**
 * @brief MarkerSensor::DetectLEDMarker
 * 识别装甲板（对相机得到的全图做处理）
 * @param img 源图像（彩色）
 * @param res_marker 要击打的装甲板
 * @return int 0成功 
 */
int MarkerSensor::DetectLEDMarker(const Mat& img, Marker& res_marker) {

    //bgr2binary
    bgr2binary(img, led_mask, armorParams.getBinaryMethod());

    //形态学处理
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    //morphologyEx(led_mask, led_mask, MORPH_CLOSE, element, Point(-1, -1), 1);

    //主要工作，识别装甲版
    bool is_detected = GetLEDMarker(led_mask, res_marker, 0, 0);
    return is_detected;
}


/**
 * @brief MarkerSensor::TrackLEDMarker
 * 跟踪装甲板（对相机得到的全图做ROI再处理）
 * @param img 源图像（彩色）
 * @param res_marker 要击打的装甲板
 * @return int 0成功
 */
int MarkerSensor::TrackLEDMarker(const Mat& img, Marker& res_marker) {
    //获取 ROI
    Rect& box = res_marker.bbox;
    float left, right, top, bot;
    if (if_roi_predict) {
        left = predict_roi_cx - 0.5 * (status + 1) * box.width;
        right = predict_roi_cx + 0.5 * box.width * (status + 1);
        top = predict_roi_cy - 0.5 * (status + 1) * box.height;
        bot = predict_roi_cy + 0.5 * box.height * (status + 1);
    } else {
        left = box.x - status * box.width;
        right = box.x + box.width * (status + 1);
        top = box.y - status * box.height;
        bot = box.y + box.height * (status + 1);
    }

    left = left < 0 ? 0 : left;
    right = right >= img.cols ? img.cols : right;
    top = top < 0 ? 0 : top;
    bot = bot >= img.rows ? img.rows : bot;
    Rect ROI(left, top, (right - left), (bot - top));

    // Get Mask
    ROI_bgr = img(ROI).clone();

    cv::Mat ROI_led_mask;

    //check if empty
    if (ROI_bgr.empty()) {
        printf("no marker for tracking!!");
        status = STATUS_DETECTING;
        marker = Marker();
        return -1;
    }

    bgr2binary(ROI_bgr, led_mask, armorParams.getBinaryMethod());


    // Get Marker
    if (GetLEDMarker(led_mask, res_marker, ROI.x, ROI.y) != STATUS_SUCCESS) {
        printf("Get no marker!\n");
        return -1;
    }

    // add coordinate bias
    res_marker.LEDs[0].center.x += ROI.x;
    res_marker.LEDs[0].center.y += ROI.y;
    res_marker.LEDs[1].center.x += ROI.x;
    res_marker.LEDs[1].center.y += ROI.y;
    //draw the best marker
    res_marker.ComputeKeyPoints();
    res_marker.ComputeBBox();

    if (armorParams.getIfShow()) {
        //    img_out=img_show(res_marker.bbox);
        if (res_marker.armor_type == Marker::SMALL) {
            rectangle(img_show, res_marker.bbox, Scalar(0, 255, 0), 3);
            putText(img_show, to_string(res_marker.number),
                    Point(res_marker.bbox.x + res_marker.bbox.width / 3, res_marker.bbox.y + res_marker.bbox.height),
                    FONT_HERSHEY_SIMPLEX, 5.5f, Scalar(255), 4, 8);
        }
        if (res_marker.armor_type == Marker::BIG) {
            rectangle(img_show, res_marker.bbox, Scalar(0, 0, 255), 3);
            putText(img_show, to_string(res_marker.number),
                    Point(res_marker.bbox.x + res_marker.bbox.width / 3, res_marker.bbox.y + res_marker.bbox.height),
                    FONT_HERSHEY_SIMPLEX, 5.5f, Scalar(255), 4, 8);
        }
    }

    return 0;
}


/**
 * @brief MarkerSensor::ProcessFrameLEDXYZ
 * 普通自瞄入口
 * @param img 相机获取的图像
 * @param z 目标z轴坐标*1000（相机坐标系）
 * @param x 目标x轴坐标*1000
 * @param y 目标y轴坐标*1000
 * @param qu 目标四元数
 * @return int 0失败
 */
int MarkerSensor::ProcessFrameLEDXYZ(const Mat& img, double& z, double& x, double& y, int& target_num, cv::Quatd& qu) {
    if (armorParams.getIfShow())
        img.copyTo(img_show);

    //---------------
    img.copyTo(Image);
    //----------------

    if (status == STATUS_DETECTING) {
        cout << "----------DETECTING---------" << endl;
        if (DetectLEDMarker(img, marker) == STATUS_SUCCESS) {
            status = STATUS_TRACKING;
        } else {
            printf("Detect No target!\n");
            return 0;
        }
    } else if (status == STATUS_TRACKING) {
        cout << "----------TRACKING---------" << endl;
        //-------------------------
        cout << "the last number is " << marker.number << endl;
        //-------------------------
        cout << "center_x:"
             << ((marker.kpts[0].x + marker.kpts[1].x) / 2 + (marker.kpts[2].x + marker.kpts[3].x) / 2) / 2 << endl;
        cout << "center_y:"
             << ((marker.kpts[0].y + marker.kpts[1].y) / 2 + (marker.kpts[2].y + marker.kpts[3].y) / 2) / 2 << endl;
        cout << "yaw:" << paraDistance(marker.LEDs[0], marker.LEDs[1]) / marker.LEDs->height;
        cout << "the led length:" << marker.LEDs->height << endl;

        if (TrackLEDMarker(img, marker) == STATUS_SUCCESS) {
            printf("Track Success!\n");
        } else {
            status = STATUS_TRACKLOST0;
            printf("Track No target!\n");
            track_fail_cnt[0] = 0;
            return 0;
        }
    } else if (status == STATUS_TRACKLOST0) {
        cout << "----------TRACKLOST0---------" << endl;
        if (TrackLEDMarker(img, marker) == STATUS_SUCCESS) {
            printf("Track 0 Success!\n");
            status = STATUS_TRACKING;
        } else {
            printf("Track 0 No target!\n");
            track_fail_cnt[0]++;
            if (track_fail_cnt[0] > 5)   // you can modify this constant
            {
                status = STATUS_TRACKLOST1;
                printf("enlarge ROI!");
                track_fail_cnt[0] = 0;
                track_fail_cnt[1] = 0;
            }
            return 0;
        }
    } else if (status == STATUS_TRACKLOST1) {
        cout << "----------TRACKLOST1---------" << endl;
        if (TrackLEDMarker(img, marker) == STATUS_SUCCESS) {
            printf("Track 0 Success!\n");
            status = STATUS_TRACKING;
        } else {
            printf("Track 1 No target!\n");
            track_fail_cnt[1]++;
            if (track_fail_cnt[1] > 10)// you can modify this constant
            {
                status = STATUS_TRACKLOST2;
                printf("ROI enlarge again!");
                track_fail_cnt[1] = 0;
                track_fail_cnt[2] = 0;
            }
            return 0;
        }
    } else if (status == STATUS_TRACKLOST2) {
        cout << "----------TRACKLOST2---------" << endl;
        if (TrackLEDMarker(img, marker) == STATUS_SUCCESS) {
            printf("Track 0 Success!\n");
            status = STATUS_TRACKING;
        } else {
            printf("Track 0 No target!\n");
            track_fail_cnt[2]++;
            if (track_fail_cnt[2] > 10) {
                status = STATUS_DETECTING;
                printf("failed to find marker in ROI");
                track_fail_cnt[2] = 0;
                marker = Marker();
            }
            return 0;
        }
    }
//    static int last_target_num;
//    if (status == STATUS_TRACKING && last_target_num != marker.number){
//        marker.number = last_target_num;
//        marker.init_type();
//    }
    target_num = marker.number;
    //last_target_num = marker.number;

    marker.ComputeKeyPoints();
    marker.ComputeBBox();
    target = (marker.LEDs[0].center + marker.LEDs[1].center) * 0.5f;

    if (armorParams.getIfGetAngle()) {
        //将像素坐标转为相机坐标系下的真实坐标


        if (armorParams.getIfShow()) {
            if (status == STATUS_TRACKING)
                circle(img_show, target, 4, Scalar(20, 20, 255), 3);
            else
                circle(img_show, target, 4, Scalar(255, 20, 20), 3);
        }

        //solvePnP
        img_points = {marker.kpts[2], marker.kpts[3], marker.kpts[1], marker.kpts[0]};
        addImageOffset(img_points, offset);

        if (marker.armor_type == Marker::BIG)
            solvePnP(big_armor, img_points, cameraMatrix, distCoeffs, rvec, tvec, false, SOLVEPNP_ITERATIVE);
        else
            solvePnP(small_armor, img_points, cameraMatrix, distCoeffs, rvec, tvec, false, SOLVEPNP_ITERATIVE);
        armor_x = tvec.at<double>(0, 0);
        armor_y = tvec.at<double>(0, 1);
        armor_z = tvec.at<double>(0, 2);
        cout << "X =" << armor_x << endl;
        cout << "Y =" << armor_y << endl;
        cout << "Z =" << armor_z << endl;

        double theta = cv::norm(rvec);
        Eigen::Quaterniond _quat;
        _quat.w() = cos(theta * 0.5);
        double _sin = sin(theta * 0.5) / theta;
        _quat.x() = rvec.at<double>(0, 2) * _sin;
        _quat.y() = -rvec.at<double>(0, 0) * _sin;
        _quat.z() = -rvec.at<double>(0, 1) * _sin;
        auto angles_ = ToEulerAngles(_quat);
        armor_yaw = angles_(2);
        armor_pitch = angles_(1);
        armor_roll = angles_(0);
        if (abs(angles::shortest_angular_distance(armor_yaw, M_PI / 2)) < M_PI / 5 ||
            abs(angles::shortest_angular_distance(armor_yaw, -M_PI / 2)) < M_PI / 5) {
            status = STATUS_TRACKLOST2;
            return 0;
        }
        cout << "Yaw =" << armor_yaw << endl;
        cout << "Pitch =" << armor_pitch << endl;
        cout << "Roll =" << armor_roll << endl;

        /*if(armor_z>10)
        {
            status=STATUS_DETECTING;
            return -1;
        }*/


        if (armorParams.getIfShow()) {
            marker.Draw(img_show);
            coordinate = Mat::zeros(Size(500, 500), CV_8UC3);
            line(coordinate, Point(250, 0), Point(250, 500), Scalar(255, 255, 255), 1);
            line(coordinate, Point(0, 250), Point(500, 250), Scalar(255, 255, 255), 1);
            int scale_factor = 125;//显示在坐标轴时X Y坐标放大倍数
            if (armorParams.getIsRed())
                circle(coordinate, Point(armor_x * scale_factor + 250, 250 - armor_y * scale_factor), 2,
                       Scalar(255, 114, 5), 2);
            else
                circle(coordinate, Point(armor_x * scale_factor + 250, 250 - armor_y * scale_factor), 2,
                       Scalar(5, 114, 255), 2);
        }

        x = armor_x;
        y = armor_y;
        z = armor_z;
        qu.x = _quat.x();
        qu.y = _quat.y();
        qu.z = _quat.z();
        qu.w = _quat.w();

    } else {
        x = target.x;
        y = target.y;
        z = 0;
    }

    //tell if it is time to shoot
    float extend_yaw; //  you will modify this var on different machines
    if (marker.armor_type == Marker::BIG) {
        extend_yaw = 0.6;
    } else {
        extend_yaw = 1.2;
    }
    /*Point img_center=Point(0.5*img.cols,0.5*img.rows+ap.pitch_bias);
    if(img_center.y>(marker.kpts[0].y-marker.bbox.height)&&img_center.y<(marker.kpts[2].y+marker.bbox.height)&&
       img_center.x>(marker.LEDs[0].center.x-extend_yaw*marker.bbox.width)&&img_center.x<(marker.LEDs[1].center.x+extend_yaw*marker.bbox.width))
    {
      center_in_rect=0;
      ROS_WARN("shoot the target!");
    }
    else
    {
      center_in_rect=0;
    }

    if(mp.ifShow)
    {
      if(center_in_rect==1)
          circle(img_show,target,4,Scalar(20,20,255),3);
      else
          circle(img_show,target,4,Scalar(255,20,20),3);
    }*/

    return 1;
}
