#ifndef _KEYFRAMESTATE_H_
#define _KEYFRAMESTATE_H_

#include <vector>

#include "Types.h"
#include "Observation.h"
#include "ObservationSet.h"
#include "LandmarkSet.h"
#include "StereoRig.h"
#include "SearchSpaceConstructor.h"

class KeyFrameState {

public:

	int                   index;
	std::vector<uint>     oldIndices;
	std::vector<Vec3>     oldPositions;
	    
	LandmarkSet           oldSet;
	LandmarkSet           newSet;

	// Using a Fast Linear Nearest-Neighbor descriptor matcher.
	cv::FlannBasedMatcher matcher;

	KeyFrameState(int _index);

	uint matchOld(ObservationSet& observations, const StereoRig& rig, 
		uint subframe, cv::Mat& drawOld);
	uint matchNew(ObservationSet& observations, const StereoRig& rig, 
		uint subframe, cv::Mat& drawNew);
	uint addNew(ObservationSet& observations, uint numMatches, uint subframe);
	void updateOldSetCount();

};

#endif