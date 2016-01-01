#ifndef _UTILITIES_H_
#define _UTILITIES_H_

#include <vector>
#include <cmath>
#include <algorithm>    // std::random_shuffle
#include <cstdlib>      // std::rand, std::srand
#include "Types.h"

float euclidianDistance(const Vec3 x, const Vec3 y);

float euclidianDistance(const Vec2 x, const Vec2 y);

float euclidianDistance(const cv::Mat& x, const cv::Mat& y);

float euclidianDistance(const cv::Point current, const cv::Point mostRecent);

uint max(uint a, uint b);

uint min(uint a, uint b);

void determineMinAndMax(std::vector<cv::DMatch> matches, float* min, float* max);

Pose mat4ToPose(const Mat4& m);

Mat4 poseToMat4(const Pose& p);

Mat4 estimateRigidTransform(const std::vector<Vec3>& begin, const std::vector<Vec3>& end);

#endif