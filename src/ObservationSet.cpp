#include "ObservationSet.h"
#include "Utilities.h"

// Initialize static members:
float ObservationSet::fastThresh[] = { 70.f, 70.0f, 70.0f, 70.0f };

ObservationSet::ObservationSet() :

	list(),
	descriptors(),
	matches() {

	// Use a SIFT extractor.
	extractor = cv::xfeatures2d::SIFT::create();

}

uint ObservationSet::extractFromStereoRig(StereoRig& rig) {

	// Extract the images from the stereo rig, and terminate if something
	//   goes wrong.
	cv::Mat left, right;
	if(!rig.grabFrames(left, right)) {

		return -1;

	}

	// Detect the keypoints in the left image.
	std::vector<cv::KeyPoint> leftKeypoints;
	generateKeypoints(left, leftKeypoints, 2);

	// Collection of filtered Landmarks.
	std::vector<cv::KeyPoint> goodKeypoints;

	// Keep track of the number of bad matches (out of bounds).
	uint badMatches = 0;

	// Match each keypoint in the right image.
	for(uint i = 0; i < leftKeypoints.size(); ++i) {

		// If too many of the left/right matches are incorrect, skip
		//   this subframe.
		if(badMatches > maxBadMatches) {

			return 0;

		}

		// Grab the point in left image.
		cv::Point leftPt = leftKeypoints[i].pt;
		Vec2 leftPoint(leftPt.x, leftPt.y);

		// Locate the point in right image.
		Vec2 rightPoint = findRightPoint(left, right, leftPoint);

		// Extract the 3-D coordinate.
		Vec3 position = rig.positionFromPixels(leftPoint, rightPoint);

		// Skip bad matches.
		if (position(SE3_Y) < minDistance || position(SE3_Y) > maxDistance) {

			badMatches++;
			continue;

		} else {

			// Add to list of good observations.
			list.push_back(Observation(leftPoint, rightPoint, position));
			matches.push_back(false);
			goodKeypoints.push_back(leftKeypoints[i]);

		}

	}

	// Compute the descriptors for the observations.
	extractor->compute(left, goodKeypoints, descriptors);

	// Make sure the matrix type is correct.
	if(descriptors.type() != CV_32F) {

	    descriptors.convertTo(descriptors, CV_32F);

	}

	//
	// IT IS IMPORTANT TO NOTE THAT 'list' AND 'descriptors' HAVE
	// CORRESPONDING INDICES BY THIS POINT.
	//

	return size();

}


void ObservationSet::generateKeypoints(const cv::Mat& img, 
	std::vector<cv::KeyPoint>& keypoints, uint side) {

	if(img.empty()) return;

	// Divide image into equal size boxes.
	uint width = img.size().width / side;
	uint height = img.size().height / side;

	// Determine the number of squares and the upper and lower
	//  bounds for keypoints number per box.
	uint squares = side * side;
	uint lowerLimit = (maxFeatures / squares) * (1.0f - margin);
	uint upperLimit = (maxFeatures / squares) * (1.0f + margin);

	// Generate an equal number of keypoints per box.
	for(uint i = 0; i < side; ++i) {
		for(uint j = 0; j < side; ++j) {

			// Determine which 
			uint boxNum = (i * side) + j;
			uint xStart = i * width;
			uint yStart = j * height;

			cv::Mat box(img, cv::Rect(xStart, yStart, width, height));

			std::vector<cv::KeyPoint> points;
			uint numAttempts = 0;

			while (numAttempts++ < maxAttempts) {

				// Use a FAST feature detector to detect keypoints.
				cv::Ptr<cv::FeatureDetector> detector = 
					cv::FastFeatureDetector::create(fastThresh[boxNum]);

				detector->detect(box, points);

				// Adjust the threshold if there are too many or too
				//	few keypoints in the current box.
				if(points.size() < lowerLimit) {

					fastThresh[boxNum] -= 2.5f;

				} else if(points.size() > upperLimit) {

					fastThresh[boxNum] += 2.5f;

				} else {

					break;

				}

			}

			// Add the keypoints from the box to the list of all keypoints.
			for (uint i = 0; i < points.size(); ++i) {

				// Adjust the coordinates;
				points[i].pt.x += xStart;
				points[i].pt.y += yStart;
				keypoints.push_back(points[i]);

			}

		}

	}

}

/**
 * @todo Implement sub-pixel refinement.
 */
Vec2 ObservationSet::findRightPoint(const cv::Mat& left, const cv::Mat& 
	right, const Vec2& leftPoint) {

	if(left.empty() || right.empty()) return Vec2(-1, -1);

	// Determine the height of the scanline.
	uint scanDiameter = 2 * scanRadius;

	// Get the size of the images.
	cv::Size leftSize = left.size();
	cv::Size rightSize = right.size();

	// Determine the x and y coordinates.
	uint x = leftPoint(0);
	uint y = leftPoint(1);

	// Make sure that x and y are in the searchable space.
	if (y <= scanRadius || y >= leftSize.height - scanRadius || 
		x <= scanRadius || x >= leftSize.width  - scanRadius) {
	
		// Point not found.
		return Vec2(-1, -1);
	
	}

	// Extract the template of the feature.
	cv::Mat templ(left, cv::Rect(x - scanRadius, y - scanRadius, 
		scanDiameter, scanDiameter));

	// Determine the bounds for the scanline.
	int l = max(0, x - 4 * scanDiameter);
	int r = min(rightSize.width, x + scanRadius);

	// Make sure that the scanline box has valid bounds.
	if(l < 0 || l >= rightSize.width || 
	   r < 0 || r >= rightSize.width ||
	   y-scanRadius < 0 || y - scanRadius >= (uint)rightSize.height || 
	   y+scanRadius < 0 || y + scanRadius >= (uint)rightSize.height) {

		return Vec2(-1, -1);

	}

	// Create the scanline
	cv::Mat scanline = cv::Mat(right, cv::Rect(l, y - scanRadius, r - l, 
		scanDiameter));

	// Match the template and adjust the point position.
	Vec2 rightPoint = matchTemplateInScanline(templ, scanline);

	rightPoint(0) += l + scanRadius;
	rightPoint(1) += leftPoint(1);

	return rightPoint;

}

Vec2 ObservationSet::matchTemplateInScanline(cv::Mat& templ, cv::Mat& scanline) {

	uint fractionFactor = 4;

	// Blow up for sub-pixel accuracy.
	cv::Size templSize = templ.size();
	cv::Size scanlineSize = scanline.size();
	templSize.width *= fractionFactor;
	templSize.height *= fractionFactor;
	scanlineSize.width *= fractionFactor;
	scanlineSize.height *= fractionFactor;
	resize(templ, templ, templSize);
	resize(scanline, scanline, scanlineSize);

	// Match the template.
	int match_method = cv::TM_SQDIFF_NORMED;
	cv::Mat responses;
	cv::matchTemplate(scanline, templ, responses, match_method);

	// Find the minimum and maximum values.
	double min, max;
	cv::Point minPoint, maxPoint;
	cv::minMaxLoc(responses, &min, &max, &minPoint, &maxPoint);

	// Return the appropriate match depending on the matching method.
	if(match_method  == cv::TM_SQDIFF || match_method == cv::TM_SQDIFF_NORMED) 
		return Vec2(minPoint.x/(float)fractionFactor, minPoint.y/(float)fractionFactor);
	else 
		return Vec2(maxPoint.x/(float)fractionFactor, minPoint.y/(float)fractionFactor);

}


cv::Mat ObservationSet::getMask(uint width) const {

	// Create the mask (to avoid matching descriptors more than once).
	cv::Mat matchMask = cv::Mat::ones(list.size(), width, CV_8UC1);
	cv::Mat zeros = cv::Mat::zeros(1, width, CV_8UC1);
	for(uint i = 0; i < matches.size(); ++i) {

		if(matches[i]) zeros.copyTo(matchMask.row(i));

	}

	return matchMask;

}


uint ObservationSet::size() const {

	return list.size();

}