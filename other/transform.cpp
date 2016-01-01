#include "opencv2/core/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/opencv_modules.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include <Eigen/Dense>
#include <iostream>	
#include <ctime>
#include <cmath>
#include <algorithm>    // std::random_shuffle
#include <cstdlib>      // std::rand, std::srand
#include <random>

typedef Eigen::Matrix<float, 2, 1> Vec2;
typedef Eigen::Matrix<float, 3, 1> Vec3;
typedef Eigen::Matrix<float, 4, 1> Vec4;
typedef Eigen::Matrix<float, 6, 1> Pose;
typedef Eigen::Matrix<float, 3, 3> Mat3;
typedef Eigen::Matrix<float, 4, 4> Mat4;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MatX;

#define SE3_X 0
#define SE3_Y 1
#define SE3_Z 2
#define SE3_R 3
#define SE3_P 4
#define SE3_W 5

int myrandom (int i) { std::srand(std::time(NULL)); return std::rand()%i;}
uint max(uint a, uint b) { return a > b ? a : b; }
uint min(uint a, uint b) { return a > b ? b : a; }


class TransformHypothesis {

public:
	Mat3 R;
	Vec3 t;
	uint matches;
	float distance;

};

Mat4 RANSAC(const std::vector<Vec3>& start, const std::vector<Vec3>& end) {

	// Constants used for tuning.
	const uint n = 6;
	const uint k = 100;
	const float thresh = 0.25;
	const uint maxMatches = 70;

	// For convenience...
	uint numMatches = start.size();
	if(numMatches == 0) {

		printf("No matches to compute.\n");
		return Mat4::Identity();

	} else if (numMatches != end.size()) {

		printf("Lists must match in size!\n");
		return Mat4::Identity();

	}

	// Determine the centroids of all the points.
	Vec3 startAvg = Vec3::Zero(), endAvg = Vec3::Zero();
	for(uint i = 0; i < numMatches; ++i) {

		startAvg += start[i];
		endAvg += end[i];

	}

	// Average.
	startAvg /= numMatches;
	endAvg /= numMatches;

	// Compute the translation.
	Vec3 t = endAvg - startAvg;

	// Normalize and Homogenize the coordinates.
	std::vector<Vec3> startNorm(numMatches), endNorm(numMatches);
	for(uint i = 0; i < numMatches; ++i) {

		startNorm[i] = start[i] - startAvg;
		endNorm[i] = end[i] - endAvg;

	}

	// Initialie the indices.
	std::vector<uint> indices(numMatches);
	for(uint i = 0; i < numMatches; ++i) {
		indices[i] = i;
	}

	// Perform RANSAC for k iterations.
	std::vector<TransformHypothesis> hypotheses;
	int bestMatchIndex = -1;
	for(uint guess = 0; guess < k; ++guess) {

		// Shuffle the indices.
		std::random_shuffle(indices.begin(), indices.end(), myrandom);
		printf("Fitting landmarks {%d, %d, %d, %d, %d, %d}\n",
			indices[0], indices[1], indices[2], indices[3], indices[4], indices[5]);

		// Create the source and destination matrices.
		Eigen::Matrix<float, 3, n> source;
		Eigen::Matrix<float, 3, n> dest;

		// Grab 'n' random points.
		for(uint i = 0; i < n; ++i) {

			uint index = indices[i];
			source.col(i) = startNorm[index];
			dest.col(i) = endNorm[index];

		}

		// Estimate Transform using the 'n' points (SVD)
		MatX S = source * dest.transpose();
		Eigen::JacobiSVD<MatX> svd(S, Eigen::ComputeThinU | Eigen::ComputeThinV);
		MatX U = svd.matrixU();
		MatX V = svd.matrixV();
		MatX R = U * V.transpose();
		Mat3 T = Mat3::Identity();
		T(2,2) = R.determinant();
		R = U * T * V.transpose(); 

		// Create new hypothesis.
		hypotheses.push_back(TransformHypothesis{R, t, 0, 0.0f});

		// See how well we did.
		for(uint i = 0; i < numMatches; ++i) {

			// Predict where the start point should really be...
			Vec3 predicted = (hypotheses[guess].R * start[i]) + hypotheses[guess].t;

			// ... and compute how far it is.
			float d = euclidianDistance(end[i], predicted);

			// If it is close enough, call it a match.
			if(d < thresh) {
				hypotheses[guess].matches++;
			}

			// Add to the total distance.
			hypotheses[guess].distance += d;

			// If we've made enough matches, we can stop.
			if(hypotheses[guess].matches >= maxMatches) {

				bestMatchIndex = guess;
				break;

			}

		}

	}

	// Determine the best match.
	if(bestMatchIndex == -1) {

		uint mostMatches = 0;
		for(uint i = 0; i < hypotheses.size(); ++i) {

			if(hypotheses[i].matches > mostMatches) {

				mostMatches = hypotheses[i].matches;
				bestMatchIndex = i;

			}

		}

		printf("Most Matches: %d (%4.2f)\n", mostMatches, mostMatches/(float)numMatches);

	}

	// Construct the 3D transformation (Rotation and translation).
	Mat4 T = Mat4::Zero();
	T.block(0,0,3,3) = hypotheses[bestMatchIndex].R;
	T.block(0,3,3,1) = hypotheses[bestMatchIndex].t;
	T(3,3) = 1;

	return T;

}


void generatePointCloud(std::vector<Vec3>& v, uint n) {

	std::srand(std::time(NULL));

	for(uint i = 0; i < n; ++i) {

		float x = ((std::rand() % 2000) / 100.0f) - 9.5f;
		float y = ((std::rand() % 2000) / 100.0f) - 9.5f;
		float z = ((std::rand() % 2000) / 100.0f) - 9.5f;
		v.push_back(Vec3(x, y, z));

	}

}

void transformPoints(std::vector<Vec3>& begin, std::vector<Vec3>& end,
	const Mat4& T) {

	uint numPoints = begin.size();
		
	std::default_random_engine generator;
	std::normal_distribution<float> distribution(0.0,2.0);

	for(uint i = 0; i < numPoints; ++i) {

		Vec4 homogenous;
		homogenous.block(0,0,3,1) = begin[i];
		homogenous(3) = 1.0f;
		Vec4 transformed = T * homogenous;
		transformed /= transformed(3);
		//transformed(0) += distribution(generator);
		//transformed(1) += distribution(generator);
		//transformed(2) += distribution(generator);
		end.push_back(transformed.block(0,0,3,1));

	}

}


Mat4 estimateTransform(const std::vector<Vec3>& begin, const std::vector<Vec3>& end) {

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

int main() {

	std::vector<Vec3> begin;
	generatePointCloud(begin, 50);

	Pose p;
	p << 5.0f, -2.4f, 0.3f, -0.8f, 1.0f, 0.4f;
	Mat4 T = poseToMat4(p);

	std::cout << T << std::endl;
	
	Pose p_guess = mat4ToPose(T);

	std::cout << p_guess << std::endl;

	T << 0.54f, -0.84f,  0.0f,   5.0f,
		 0.84f,  0.54f,  0.0f,  -2.4f,
		  0.0f,   0.0f,  1.0f,   0.3f,
		  0.0f,   0.0f,  0.0f,   1.0f;

	std::vector<Vec3> end;
	transformPoints(begin, end, T);

	Mat4 T_guess = estimateTransform(begin, end);

	std::cout << T_guess << std::endl;

	p_guess = mat4ToPose(T_guess);

	std::cout << p_guess << std::endl;

	Vec3 e = Vec3::Zero();
	for(uint i = 0; i < begin.size(); ++i) {

		Vec3 p = (T.block(0,0,3,3) * begin[i]) + T.block(0,3,3,1);
		printf("Predicted {%4.2f, %4.2f, %4.2f} Actual {%4.2f, %4.2f, %4.2f}\n",
			p(0), p(1), p(2), end[i](0), end[i](1), end[i](2));
		e += p - end[i];

	}


	e /= begin.size();

	printf("Tranformation error: {%4.2f, %4.2f, %4.2f}\n", e(0), e(1), e(2));

}