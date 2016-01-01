#ifndef _SLAM_H_
#define _SLAM_H_

#include "Types.h"
#include "KeyFrameState.h"
#include "StereoRig.h"

#define DEFAULT_MAX_KEYFRAMES       10
#define DEFAULT_MIN_OBSERVATIONS    10
#define SUBFRAMES_PER_FRAME         10
#define SECONDS_PER_FRAME        0.05f

class Slam {

public:

	// Start with no keyframe.
    uint keyFrameCount;
    uint landmarkCount;
    int  lastKeyFrame;
    uint maxKeyFrames;
    uint minObservations;

	// Rig of two calibrated stereo cameras.
	StereoRig rig;

	// Primary backend data structure.
	SRBA bundleAdjuster;

	// All of the landmark descriptors, with each row i describing the i'th landmark.
	cv::Mat allDescriptors;

	// Slam constructor.
	Slam();

	// Create and add a new Keyframe using KeyFrameState data.
	void constructSearchSpace(KeyFrameState& keyframe, int root);
	void createNewKeyFrame(KeyFrameState& keyframe);
	void initialize();
	void run();
	void display(const cv::Mat& left, const cv::Mat& right);

};

#endif