#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <opencv2/opencv.hpp>

#include <opencv2/core/ocl.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Geometry>

#include "RotRect.h"
#include "Marker.h"
#include "params/armor_params.h"
#include "params/cam_params.h"

using mMarkerType = Marker::MarkerType;


__inline__ std::string num2str(double i);

float ComputeLengthAlongDir(std::vector<cv::Point>& contour, cv::Point2f& dir);

int PCALEDStrip(std::vector<cv::Point>& contour, RotRect& LED);

float paraDistance(RotRect& LED1, RotRect& LED2);

int FindLowestHP(std::vector<Marker>& markers, std::map<int, int>& HP);

int bgr2binary(const Mat& srcImg, Mat& img_out, int method);

void addImageOffset(std::vector<Point2f>& points, Point2f& offset);

void checkContoursCompleteness(std::vector<std::vector<Point>>& contours, Mat& img);

void markerTypeIdentify(Marker& marker, mMarkerType manualMT);

double sigmoid(double x);

bool SortLEDs(RotRect LED1, RotRect LED2);

Eigen::Vector3d ToEulerAngles(const Eigen::Quaterniond& q);

Point2i camera2pixel(const Point3d& p3d);

Point3d world2camera(const Point3d& p3d, const Point3d& gimbalPRY, const Point3d& offset);

#endif // UTILS_H
