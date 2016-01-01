#ifndef _STEREORIG_H_
#define _STEREORIG_H_

#include "Types.h"
#include "Camera.h"

#define FOCAL_LENGTH  389.956085f
#define CAMERA_WIDTH  512
#define CAMERA_HEIGHT 384
#define CENTER_X      254.903519f
#define CENTER_Y      201.899490f
#define BASELINE      0.120005f
#define SEQUENCE_PATH "../../data/block1/%06d.pnm"

class StereoRig {

private:

	Camera left;
	Camera right;
	cv::Mat leftFrame, rightFrame, prevLeftFrame, prevRightFrame;
	float  baseline;
	float  disparityConstant;
	cv::VideoCapture capture;
	bool   good;

public:

	StereoRig();

	Vec3 positionFromPixels(const Vec2& left, const Vec2& right) const;

	Vec2 pixelFromPosition(const Vec3& pos) const;

	bool grabFrames(cv::Mat& _leftFrame, cv::Mat& _rightFrame);

	void copyFrames(cv::Mat& _leftFrame, cv::Mat& _rightFrame);

	bool isGood() const;

};

#endif