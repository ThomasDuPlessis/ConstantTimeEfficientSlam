#ifndef _OBSERVATIONSET_H_
#define _OBSERVATIONSET_H_

#include <vector>

#include "Types.h"
#include "Observation.h"
#include "StereoRig.h"

class ObservationSet {

public:

	static const     uint    scanRadius     =     4;
	static const     uint    maxFeatures    =   300;
	static const     uint    maxAttempts    =     5;
	static constexpr float   margin         = 0.15f;
	static const     uint    maxBadMatches  =    50;
	static constexpr float   minDistance    =  1.0f;
	static constexpr float   maxDistance    = 20.0f;
	static           float   fastThresh[];
 
	std::vector<Observation> list;
	cv::Mat                  descriptors;
	std::vector<bool>        matches;
	cv::Ptr<cv::Feature2D>   extractor;

	ObservationSet();

	uint extractFromStereoRig(StereoRig& rig);

	void generateKeypoints(const cv::Mat& img, std::vector<cv::KeyPoint>& keypoints,
		uint side);

	Vec2 findRightPoint(const cv::Mat& leftImg, const cv::Mat&  rightImg, 
		const Vec2& leftPoint);

	Vec2 matchTemplateInScanline(cv::Mat& templ, cv::Mat& scanline);

	uint size() const;

	cv::Mat getMask(uint width) const;

};

#endif