//
// Created by chiliu on 5/12/18.
//

#ifndef MYSLAM_COMMON_INCLUDE_H
#define MYSLAM_COMMON_INCLUDE_H

// define the commonly included file to avoid a long include list
// for Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;

// for Sophus
#include <sophus/se3.h>
using Sophus::SE3;

// for cv
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
using cv::Mat;

// std
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>

using namespace std;
#endif //MYSLAM_COMMON_INCLUDE_H
