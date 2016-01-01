#include "KeyFrameState.h"
#include "Utilities.h"

KeyFrameState::KeyFrameState(int _index) : 

	index(_index),
	oldIndices(),
	oldPositions(),
	oldSet(),
	newSet(),
	matcher() { 

	// Empty. 

}

uint KeyFrameState::matchOld(ObservationSet& observations, const StereoRig& rig,
	uint subframe, cv::Mat& drawOld) {

	// Use a FLANN based descriptor matcher.
	uint numMatches = 0;

	// Match old landmarks if there are any.
	if(oldSet.size() > 0 && observations.size() > 0) {

		// Attempt to match to old landmarks.
		cv::Mat mask = observations.getMask(oldSet.size());
		std::vector<cv::DMatch> obsMatches;
		matcher.match(observations.descriptors, oldSet.descriptors, obsMatches,
			mask);

		// Determine the bounds for matching
		float minDist, maxDist;
		determineMinAndMax(obsMatches, &minDist, &maxDist);

		// Look at each match to determine if a landmark has been recognized.
		for(uint i = 0; i < obsMatches.size(); i++ ) {

			// Grab the indices of the matches.
			cv::DMatch match = obsMatches[i];
			uint obsIndex = match.queryIdx;
			uint oldIndex = match.trainIdx;

			// Recognized landmark 'oldIndex'.
			if(match.distance < 3 * minDist) { 

				// Determine where the observation was made in 3-D space.
				Vec3 observedPosition = observations.list[obsIndex].relativePosition;

				// Determine where the landmark was predicted to be in 3-D space.
				Vec3 predictedPosition = oldPositions[oldIndex];
				float maxDistance = 1.0f;

				// If we've already observed the landmark in this keyframe, 
				//  use its last known position.
				if(oldSet.observations[oldIndex].size() > 0) {

					predictedPosition = oldSet.observations[oldIndex].back().relativePosition;
					maxDistance = 0.5f;

				}

				// Calculate the euclidian distance between the two positions.
				float eDist = euclidianDistance(predictedPosition, observedPosition);

				// Only make close matches.
				if(eDist < maxDistance) {

					//printf("   Old match No %3d:", oldIndices[oldIndex]);
					//printf("   Predicted: {% -5.2f, % -5.2f, % -5.2f}", 
					//	predictedPosition(0), predictedPosition(1), 
					//	predictedPosition(2));
					//printf("   Observed: {% -5.2f, % -5.2f, % -5.2f}", 
					//	observedPosition(0), observedPosition(1), 
					//	observedPosition(2));
					//printf("   Distance: % -4.2f meters.\n", eDist);

					Vec2 current = observations.list[obsIndex].leftPixel;
					Vec2 mostRecent = rig.pixelFromPosition(predictedPosition);
				
					// If the new measurement is under 10 pixels away, track it.
					if(euclidianDistance(current, mostRecent) < 7.0f) {

						// Add to the list of observations.
						oldSet.observations[oldIndex].push_back(
							observations.list[obsIndex]
						);

						// Found a match for this observation.
						observations.matches[obsIndex] = true;
						oldSet.lastSeen[oldIndex] = subframe;
						numMatches++;

						cv::Point before(mostRecent(0), mostRecent(1));
						cv::Point after(current(0), current(1));
						cv::circle(drawOld, before, 3, cv::Scalar(0.0, 0.0, 255.0), 2);
						cv::circle(drawOld, after, 3, cv::Scalar(0.0, 255.0, 0.0), 2);
						cv::line(drawOld, before, after, cv::Scalar(255.0, 0.0, 0.0), 2);

					}

				}

			}

		}

	}

	return numMatches;

}


uint KeyFrameState::matchNew(ObservationSet& observations, const StereoRig& rig,
	uint subframe, cv::Mat& drawNew) {

	uint numMatches = 0;

	// Match new landmarks if there are any.
	if(newSet.size() > 0 && observations.size() > 0) {

		// Attempt to match to new landmarks.
		cv::Mat mask = observations.getMask(newSet.size());
		std::vector<cv::DMatch> obsMatches;
		matcher.match(observations.descriptors, newSet.descriptors, obsMatches, 
			mask);

		// Determine the bounds for matching
		float minDist, maxDist;
		determineMinAndMax(obsMatches, &minDist, &maxDist);

		// Look at each match to determine if a landmark has been recognized.
		for(uint i = 0; i < obsMatches.size(); i++) {

			// Grab the indices of the matches.
			cv::DMatch match = obsMatches[i];
			uint obsIndex = match.queryIdx;
			uint newIndex = match.trainIdx;

			// Recognized landmark 'newIndex'.
			if(match.distance < 3 * minDist) { 

				// Determine where the observation was made in 3-D space.
				Vec3 observedPosition = observations.list[obsIndex].relativePosition;

				// Determine where the landmark was predicted to be in 3-D space.
				Vec3 predictedPosition = newSet.observations[newIndex].back().relativePosition;

				float maxDistance = 0.5f;

				// Calculate the euclidian distance between the two positions.
				float eDist = euclidianDistance(predictedPosition, observedPosition);

				// Only make close matches.
				if(eDist < maxDistance) {

					Vec2 current = observations.list[obsIndex].leftPixel;
					Vec2 mostRecent = rig.pixelFromPosition(predictedPosition);
				
					// If the new measurement is under 10 pixels away, track it.
					if(euclidianDistance(current, mostRecent) < 7.0f) {

						// Add to the list of observations.
						newSet.observations[newIndex].push_back(
							observations.list[obsIndex]
						);

						// Found a match for this observation.
						observations.matches[obsIndex] = true;
						newSet.lastSeen[newIndex] = subframe;
						numMatches++;

						cv::Point before(mostRecent(0), mostRecent(1));
						cv::Point after(current(0), current(1));
						cv::circle(drawNew, before, 3, cv::Scalar(0.0, 0.0, 255.0), 2);
						cv::circle(drawNew, after, 3, cv::Scalar(0.0, 255.0, 0.0), 2);
						cv::line(drawNew, before, after, cv::Scalar(255.0, 0.0, 0.0), 2);

					}

				}

			}

		}

	}

	return numMatches;

}


uint KeyFrameState::addNew(ObservationSet& observations, uint numMatches, 
	uint subframe) {

	uint counter = 0;

	if(observations.size() > 0 && numMatches < observations.size()) {

		for(uint i = 0; i < observations.matches.size(); ++i) {

			if(observations.matches[i] == false) {

				// Add the landmark to the set.
				newSet.addLandmark(observations.list[i], 
					observations.descriptors.row(i), subframe);

				// Increment the counter.
				counter++;

			}

		}

	}

	return counter;

}


void KeyFrameState::updateOldSetCount() {

	oldSet.count = oldIndices.size();

}