//
// Created by robin on 23-3-12.
//
#include "Marker.h"
#include "utils.h"

using namespace std;

__inline__ string num2str(double i) {
    stringstream ss;
    ss << i;
    return ss.str();
}

/**
 * @brief ComputeLengthAlongDir
 * 计算灯条长度
 * @param contour 轮廓
 * @param dir PCA获得的灯条方向
 * @return float 灯条长度
 */
float ComputeLengthAlongDir(vector<cv::Point>& contour, cv::Point2f& dir) {
    float max_range = -999999;
    float min_range = 999999;
    for (auto& pt: contour) {
        float x = pt.x * dir.x + pt.y * dir.y;//dot(x,dir)=x*y*cos(theta), project pix on dir
        if (x < min_range) min_range = x;
        if (x > max_range) max_range = x;
    }
    return (max_range - min_range);
}

/**
 * @brief PCALEDStrip
 *  将边缘（vector<Point>）转换为自定义的LED灯条（RotRect）结构
 * @param contour cv::findCountours函数获得的单个边缘
 *  注意得到的height是长边，width是短边
 * @param LED 转换后的灯条
 * @return int 成功返回0
 */
int PCALEDStrip(vector<cv::Point>& contour, RotRect& LED) {
    int sz = static_cast<int>(contour.size());
    cv::Mat data_pts(sz, 2, CV_64FC1);
    double* _data_pts = (double*) data_pts.data;
    for (int i = 0; i < data_pts.rows; ++i, _data_pts += 2) {
        _data_pts[0] = contour[i].x;
        _data_pts[1] = contour[i].y;
    }
    cv::PCA pca_analysis(data_pts, cv::Mat(), PCA::DATA_AS_ROW);
    LED.center.x = static_cast<float>(pca_analysis.mean.at<double>(0, 0));
    LED.center.y = static_cast<float>(pca_analysis.mean.at<double>(0, 1));
    cv::Point2f dir1, dir2;
    Mat eig = pca_analysis.eigenvectors;
    dir1.x = static_cast<float>(pca_analysis.eigenvectors.at<double>(0, 0));
    dir1.y = static_cast<float>(pca_analysis.eigenvectors.at<double>(0, 1));
    dir2.x = static_cast<float>(pca_analysis.eigenvectors.at<double>(1, 0));
    dir2.y = static_cast<float>(pca_analysis.eigenvectors.at<double>(1, 1));

    dir1 = dir1 * (1 / cv::norm(dir1));// norm(dir1)=1, so make no sense
    dir2 = dir2 * (1 / cv::norm(dir2));

    LED.dir = dir1;

    //height是长边，width是短边
    LED.height = ComputeLengthAlongDir(contour, dir1);
    LED.width = ComputeLengthAlongDir(contour, dir2);

    // RotatedRect RRect = minAreaRect(contour);

    // LED.center = RRect.center;
    // LED.height = RRect.size.height > RRect.size.width ? RRect.size.height : RRect.size.width;
    // LED.width = RRect.size.width < RRect.size.height ? RRect.size.width : RRect.size.height;
    // Point2f vtx[4];
    // RRect.points(vtx);
    // LED.dir = (vtx[3]-vtx[2])*(1/cv::norm((vtx[3]-vtx[2])));
    //  cout<<"LED.dir "<<LED.dir<<endl;
    return 0;

}

/**
 * @brief paraDistance
 * 计算两平行灯条间的距离
 * @param LED1 灯条1
 * @param LED2 灯条2
 * @return int 计算出的距离
 */
float paraDistance(RotRect& LED1, RotRect& LED2) {
    float distance = 0;
    float tgt_theta = LED1.dir.y / LED1.dir.x;
    float theta = atan(tgt_theta);
    float cx2_para = (LED1.center.y - LED2.center.y) / tgt_theta + LED2.center.x;
    distance = fabs((LED1.center.x - cx2_para) * sin(theta));
    return distance;
}

/**
 * @brief FindLowestHP
 * 确定识别到的所有装甲中血量最低的目标
 * @param markers 识别到的所有装甲
 * @return int 血量最低的装甲索引
 */
int FindLowestHP(vector<Marker>& markers, map<int, int>& HP) {
    int res_idx = 0;
    int lowestHP = 1000;
    if (HP.size() < 5) {
//        ROS_WARN("The HP array is empty");
        HP[1] = 50;
        HP[2] = 60;
        HP[3] = 70;
        HP[4] = 80;
        HP[5] = 100;
        return 0;
    }
    /*for(int i=1;i<=HP.size();i++)
        cout<<HP[i]<<endl;*/
    int idx = 0;
    for (auto& marker: markers) {
        if (marker.number != 0) {
            if (HP[marker.number] < lowestHP) {
                lowestHP = HP[marker.number];
                res_idx = idx;
            }
        }
        idx++;
    }
    if (markers[res_idx].number == 0)
        return 0;
    return res_idx + 1;
}

/**
 * @brief bgr2binary
 * 图像二值化
 *
 * @param srcImg 彩色图，CV_8UC3
 * @param img_out 二值化后的图像
 * @param method 二值化方法，1为红蓝通道分离相减 2直接使用inRange函数
 * @return int
 */
int bgr2binary(const Mat& srcImg, Mat& img_out, int method) {
    if (srcImg.empty())
        return -1;
    if (method == 1) {
        //method 1: split channels and substract
        vector<Mat> imgChannels;
        split(srcImg, imgChannels);
        Mat red_channel = imgChannels.at(2);
        Mat blue_channel = imgChannels.at(0);
        Mat mid_chn_img;
        if (!armorParams.getIsRed()) {
            mid_chn_img = red_channel - blue_channel;

        } else {
            mid_chn_img = blue_channel - red_channel;
        }
        threshold(mid_chn_img, img_out, armorParams.getGrayThreshold(), 255, THRESH_BINARY);
    } else if (method == 2) {
        //method 2: 3 channel threthold
        if (!armorParams.getIsRed()) {
            cv::inRange(srcImg, armorParams.getBlueChMin(),
                        armorParams.getBlueChMax(), img_out);

        } else {
            cv::inRange(srcImg, armorParams.getRedChMin(),
                        armorParams.getRedChMax(), img_out);
        }

    } else if (method == 3) {
        vector<Mat> imgChannels;
        split(srcImg, imgChannels);
        Mat red_channel = imgChannels.at(2);
        Mat blue_channel = imgChannels.at(0);
        Mat green_channel = imgChannels.at(1);
        Mat single_bina;
        Mat double_bina;
        Mat mid_chn_img;
        vector<vector<Point>> countours;
        if (!armorParams.getIsRed()) {
            threshold(red_channel, single_bina, armorParams.getBin3Threshold1(), 255, THRESH_BINARY);
            mid_chn_img = red_channel - blue_channel;
            threshold(mid_chn_img, double_bina, armorParams.getBin3Threshold2(), 255, THRESH_BINARY);
            findContours(single_bina, countours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
            for (int i = 0; i < countours.size(); i++) {
                Rect sin_rect = boundingRect(countours[i]);
                cout << mean(double_bina(sin_rect))[0] << endl;
                if (mean(double_bina(sin_rect))[0] < armorParams.getBin3Threshold3() * 255 ||
                    mean(green_channel(sin_rect))[0] > 200) {
                    drawContours(single_bina, countours, i, Scalar(0), FILLED);
                    //single_bina(sin_rect) = Mat(sin_rect.height, sin_rect.width, CV_8UC1, 0);
                }
            }
        } else {
            threshold(blue_channel, single_bina, armorParams.getBin3Threshold1(), 255, THRESH_BINARY);
            mid_chn_img = blue_channel - red_channel;
            threshold(mid_chn_img, double_bina, armorParams.getBin3Threshold2(), 255, THRESH_BINARY);
            findContours(single_bina, countours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
            for (int i = 0; i < countours.size(); i++) {
                Rect sin_rect = boundingRect(countours[i]);
                if (mean(double_bina(sin_rect))[0] < armorParams.getBin3Threshold3() * 255 ||
                    mean(green_channel(sin_rect))[0] > 200)
                    drawContours(single_bina, countours, i, Scalar(0), FILLED);
                //single_bina(sin_rect) = Mat(sin_rect.height,sin_rect.width,CV_8UC1,0);
            }
        }
        img_out = single_bina;
    } else
        return -1;

    return 0;
}

/**
 * @brief addImageOffset
 * 在获取的原始图像经过裁切时将坐标转换到完整的图像坐标系
 * @param marker
 * @param points
 */
void addImageOffset(vector<Point2f>& points, Point2f& offset) {
    for (auto& p: points) {
        p = p + offset;
    }
}

void checkContoursCompleteness(vector<vector<Point>>& contours, Mat& img) {
    for (auto contour = contours.begin(); contour < contours.end(); contour++) {
        Rect bbox = boundingRect(*contour);
        if (bbox.br().x == img.rows || bbox.br().y == img.cols || bbox.tl().x == 0 || bbox.tl().y == 0) {
            contour = contours.erase(contour);
        }
    }
}

void markerTypeIdentify(Marker& marker, mMarkerType manualMT) {
    float avg_height = (marker.LEDs[0].height + marker.LEDs[1].height) / 2 / (float) camParams.getFy();
    float armor_w = paraDistance(marker.LEDs[0], marker.LEDs[1]) / (float) camParams.getFx();
    float wid_div_height = armor_w / avg_height;
    float leaky_angle = MIN(marker.LEDs[0].height / marker.LEDs[1].height,
                            marker.LEDs[1].height / marker.LEDs[0].height);
    cout << "leaky_angle= " << leaky_angle << endl;
    // cout<<"armor_type_param= "<<(wid_div_height/avg_height)/(leaky_angle/avg_height)<<endl;

    Point2f tgt = (marker.LEDs[0].center + marker.LEDs[1].center) * 0.5f;
    if (manualMT == mMarkerType::ALL) {
        if (abs(tgt.x - camParams.getCx()) > 300) {
            if (wid_div_height / leaky_angle > armorParams.getArmorTypeThreshold()) {
                //marker.armor_type = Marker::BIG;
                marker.typeCall.setMarkerType(true);
                cout << "big armor!!" << endl;
            } else {
                marker.typeCall.setMarkerType(false);
                cout << "small armor!!" << endl;
            }
        } else {
            if (wid_div_height / leaky_angle > armorParams.getArmorTypeThreshold())
                //if(wid_div_height>mp.armor_type_threshold)
            {
                //marker.armor_type = Marker::BIG;
                marker.typeCall.setMarkerType(true);
                cout << "big armor!!" << endl;
            } else {
                marker.typeCall.setMarkerType(false);
                cout << "small armor!!" << endl;
            }
        }
        cout << "wid_div_height/leaky_angle:" << wid_div_height / leaky_angle << endl;
        cout << "wid_div_height:" << wid_div_height << endl;
    } else {
        if (manualMT == mMarkerType::BIG)
            marker.typeCall.setMarkerType(true);
        else if (manualMT == mMarkerType::SMALL)
            marker.typeCall.setMarkerType(false);
    }
}

Eigen::Vector3d ToEulerAngles(const Eigen::Quaterniond& q) {
    Eigen::Vector3d angles;    //yaw pitch roll
    const auto x = q.x();
    const auto y = q.y();
    const auto z = q.z();
    const auto w = q.w();

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    angles[0] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        angles[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles[1] = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    angles[2] = std::atan2(siny_cosp, cosy_cosp);
    return angles;
}

Point2i camera2pixel(const Point3d& p3d) {
    Point2i p2i;
    float x = p3d.x * camParams.getFx() / p3d.z + camParams.getCx();
    float y = p3d.y * camParams.getFy() / p3d.z + camParams.getCy();
    float r = sqrt((x - 640 + camParams.offsetW) * (x - 640 + camParams.offsetW) +
                   (y - 512 + camParams.offsetH) * (y - 512 + camParams.offsetH));
    p2i.x = int(
            x * (1 + camParams.distCoeffs.at<double>(1) * r * r + camParams.distCoeffs.at<double>(2) * r * r * r * r));
    p2i.y = int(
            y * (1 + camParams.distCoeffs.at<double>(1) * r * r + camParams.distCoeffs.at<double>(2) * r * r * r * r));
    return p2i;
}

Point3d world2camera(const Point3d& p3d, const Point3d& gimbalPRY, const Point3d& offset) {
    Point3d p3d_c;
    Eigen::Quaterniond q = Eigen::AngleAxisd(gimbalPRY.x, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(gimbalPRY.y, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(gimbalPRY.z, Eigen::Vector3d::UnitX());
    Eigen::Vector3d p3d_w(p3d.x, p3d.y, p3d.z);
    Eigen::Vector3d p3d_c_eigen = q * p3d_w + Eigen::Vector3d(offset.x, offset.y, offset.z);
    p3d_c.x = p3d_c_eigen.x();
    p3d_c.y = p3d_c_eigen.y();
    p3d_c.z = p3d_c_eigen.z();
    return p3d_c;
}

double sigmoid(double x) {
    return 1.0 / (1.0 + exp(-1.0 * x));
}

bool SortLEDs(RotRect LED1, RotRect LED2) {
    return LED1.center.x < LED2.center.x;
}