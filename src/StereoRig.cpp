#include <iostream>

#include "StereoRig.h"

StereoRig::StereoRig() :

		left(FOCAL_LENGTH, CENTER_X, CENTER_Y),
		right(FOCAL_LENGTH, CENTER_X, CENTER_Y),
		baseline(BASELINE),
		disparityConstant(FOCAL_LENGTH * BASELINE),
		capture(SEQUENCE_PATH),
		good(true) {

		if(!capture.isOpened()) {

	        std::cout << "Could not open or find the image." << std::endl ;
	        good = false;

		}

	}

Vec3 StereoRig::positionFromPixels(const Vec2& leftPoint, 
	const Vec2& rightPoint) const {

	// Calculate the position of the Landmark relative to this KeyFrame.
	float z = disparityConstant / (leftPoint(0) - rightPoint(0));
	float y = z * (leftPoint(SE3_Y) - left.getCenter(SE3_Y)) / left.getFocalLength();
	float x = z * (rightPoint(SE3_X) - left.getCenter(SE3_X)) / left.getFocalLength();

	// Convert from camera coords to robot coords.
	return Vec3(x, z, -y);

}

Vec2 StereoRig::pixelFromPosition(const Vec3& pos) const {

	// Swap around the values to covert to camera coordinates.
	Vec3 p = pos;
	
	// X remains the same.
	// Swap Y with Z, negating Z.
	float temp = p(SE3_Y);
	p(SE3_Y) = -p(SE3_Z);
	p(SE3_Z) = temp;

	// Perform homogenous transformation.
	Vec3 pixels = left.getK() * p;
	return Vec2(pixels(0)/pixels(2), pixels(1)/pixels(2));

}

bool StereoRig::grabFrames(cv::Mat& _leftFrame, cv::Mat& _rightFrame) {

	// Copy the previous frames back.
	leftFrame.copyTo(prevLeftFrame);
	rightFrame.copyTo(prevRightFrame);

	// Read in the left and right images, return false if problem.
	capture >> leftFrame;  if(leftFrame.empty()) return false;
	capture >> rightFrame; if(rightFrame.empty()) return false;

	// Indicate that read was successful.
	leftFrame.copyTo(_leftFrame);
	rightFrame.copyTo(_rightFrame);
	return true;

}

void StereoRig::copyFrames(cv::Mat& _leftFrame, cv::Mat& _rightFrame) {

	// Copy the previous frames back.
	leftFrame.copyTo(_leftFrame);
	rightFrame.copyTo(_rightFrame);

}

bool StereoRig::isGood() const {

	return good;

}
