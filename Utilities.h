#ifndef _UTILITIES_H_
#define _UTILITIES_H_

#include <vector>
#include <cmath>
#include <algorithm>    // std::random_shuffle
#include <cstdlib>      // std::rand, std::srand
#include "Types.h"

/////////////////////////////////////////////////////////////////////////////////////////
//                            EUCLIDEAN DISTANCE FUNCTIONS                             //
/////////////////////////////////////////////////////////////////////////////////////////
template <typename Derived>
float euclidianDistance(const Eigen::MatrixBase<Derived>& x, const Eigen::MatrixBase<Derived>& y) {

	float sumOfSquares = 0.0f;
	for(int i = 0; i < x.rows(); ++i)
		sumOfSquares += (x(i)-y(i)) * (x(i)-y(i));

	return std::sqrt(sumOfSquares);

}
float euclidianDistance(const cv::Mat& x, const cv::Mat& y) {

	float sumOfSquares = 0.0f;
	cv::Mat d = x - y;
	for(int i = 0; i < d.rows; ++i)
		sumOfSquares += d.at<float>(i) * d.at<float>(i);

	return std::sqrt(sumOfSquares);

}
float euclidianDistance(const cv::Point current, const cv::Point mostRecent) {

	float dx = current.x - mostRecent.x;
	float dy = current.y - mostRecent.y;
	return std::sqrt(dx * dx + dy * dy);

}
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

uint max(uint a, uint b) { return a > b ? a : b; }
uint min(uint a, uint b) { return a > b ? b : a; }

void determineMinAndMax(std::vector<cv::DMatch> matches, float* min, float* max) {

	// Thresholds for matching.
	*min = 100; 
	*max = 0;

	// Quick calculation of max and min distances between descriptors.
	for(uint i = 0; i < matches.size(); i++ ) { 

		double dist = matches[i].distance;
		if(dist < *min) *min = dist;
		if(dist > *max) *max = dist;

	}

}

Pose mat4ToPose(const Mat4& m) {

	float r = std::atan2(m(2,1), m(2,2));
	float p = std::atan2(-m(2,0), std::sqrt(m(2,1)*m(2,1) + m(2,2)*m(2,2)));
	float w = std::atan2(m(1,0), m(0,0));
	float x = m(0,3);
	float y = m(1,3);
	float z = m(2,3);

	Pose pose;
	pose << x, y, z, r, p, w;
	return pose;

}

Mat4 poseToMat4(const Pose& p){

	// Calculate the rotation angles.
	float A = sin(p(SE3_W)); // sin of yaw about z-axis.
	float B = cos(p(SE3_W)); // cos of yaw about z-axis.
	float C = sin(p(SE3_P)); // sin of pitch about x-axis.
	float D = cos(p(SE3_P)); // cos of pitch about x-axis.
	float E = sin(p(SE3_R)); // sin of roll about y-axis.
	float F = cos(p(SE3_R)); // cos of roll aobut y-axis.

	// Construct the matrix.
	Mat4 T;
	T << D*B, B*C*E - F*A, B*C*F + E*A, p(SE3_X),
	     D*A, A*C*E + F*B, A*C*F + E*B, p(SE3_Y),
	      -C,         E*D,         F*D, p(SE3_Z),
	     0.0f,       0.0f,        0.0f,     1.0f; 
	return T;

}

Mat4 estimateRigidTransform(const std::vector<Vec3>& begin, const std::vector<Vec3>& end) {

	// Number of points being sampled.
	uint count = begin.size();

	if (count == 0) return Mat4::Identity();

	// Sum up corresponding components of the vectors.
	Vec3 begin_mean(0.0f, 0.0f, 0.0f);
	Vec3 end_mean(0.0f, 0.0f, 0.0f);

	for(uint i = 0; i < count; ++i) {
		begin_mean += begin[i];
		end_mean += end[i];
	}

	begin_mean /= (float) count;
	end_mean /= (float) count;

	Mat3 K = Mat3::Zero();
	for(uint i = 0; i < count; ++i) {

		Vec3 new_end = end[i] - end_mean;
		Vec3 new_begin = begin[i] - begin_mean;
		K += new_begin * new_end.transpose();
	}

	Eigen::JacobiSVD<MatX> svd(K, Eigen::ComputeThinU | Eigen::ComputeThinV);

	Mat3 U = svd.matrixU();
	Mat3 V = svd.matrixV();
	Mat3 R = V * U.transpose();
	Mat3 I = Mat3::Identity();

	I(2,2) = R.determinant();
	R = V * I * U.transpose();

	Vec3 t = end_mean - (R * begin_mean);

	Mat4 trans = Mat4::Identity();
	trans.block(0,0,3,3) = R;
	trans.block(0,3,3,1) = t;

	return trans;

}

#endif