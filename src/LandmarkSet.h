#ifndef _LANDMARKSET_H_
#define _LANDMARKSET_H_

#include <vector>
#include <unordered_map>

#include "Types.h"
#include "Observation.h"

class LandmarkSet {

public:

	uint						  count;
	std::vector<
		std::vector<Observation>> observations;
	cv::Mat 					  descriptors;
	std::vector<uint> 			  lastSeen;

	LandmarkSet() :

		count(0),
		observations(),
		descriptors(),
		lastSeen() {

		// Empty.
	}

	void addLandmark(const Observation& obs, const cv::Mat& descr, uint _lastSeen) {

		// Add the observation.
		observations.push_back(std::vector<Observation>());
		observations.back().push_back(obs);

		// Add the descriptor.
		descriptors.push_back(descr);		

		// Add the last seen index.
		lastSeen.push_back(_lastSeen);

		// Increment count.
		count++;

	}

	uint size() {

		return count;

	}

};

#endif