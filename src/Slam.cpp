#include <iostream>	
#include <ctime>

#include "Slam.h"
#include "SearchSpaceConstructor.h"

Slam::Slam() :

	keyFrameCount(0),
    landmarkCount(0),
    lastKeyFrame(-1),
    maxKeyFrames(DEFAULT_MAX_KEYFRAMES),
    minObservations(DEFAULT_MIN_OBSERVATIONS),
	rig(),
	bundleAdjuster(),
	allDescriptors() {

    initialize();

}


void Slam::run() {

	if(!rig.isGood()) {

		std::cout << ">>> ERROR: Rig failed to initialize. Aborting..." << std::endl;

	}

	// Create window for display.
    cv::namedWindow("Stereo", cv::WINDOW_AUTOSIZE);// Create a window for display.

	// Determining the computation time.
    std::clock_t startT = std::clock();
    std::clock_t entT;

    // Perform the algorithm for every keyframe.
	while(keyFrameCount < maxKeyFrames) {

		printf("# Keyframe Number: %2d\n", keyFrameCount);

		// New KeyFrameState for this frame.
		KeyFrameState keyFrame(keyFrameCount++);

		// Construct the search space of previously seen landmarks.
		constructSearchSpace(keyFrame, lastKeyFrame);

		// Perform several iterations to track the landmarks in the current frame.
		for(uint subframe = 1; subframe <= SUBFRAMES_PER_FRAME; ++subframe) {

			// Extract the observations from the image.
			ObservationSet observations;
			observations.extractFromStereoRig(rig);

			// Make sure that we've observed enough landmarks.
			if (observations.size() < 0) {

				std::cout << ">>> ERROR: Rig failed to capture frames. Aborting..."
						  << std::endl;
				return;

			} else if (observations.size() < minObservations) {

				continue;
			
			}

			// Frames used for drawing the landmarks.
			cv::Mat drawNew, drawOld;
			rig.copyFrames(drawNew, drawOld);
			cv::cvtColor(drawNew, drawNew, CV_GRAY2RGB);
			drawNew.copyTo(drawOld);

			// Find old landmarks.
			//printf(" > Matching old landmarks...\n");
			uint numOldMatches = keyFrame.matchOld(observations, rig, subframe, drawOld);
			
			// Find new landmarks.
			//printf(" > Matching new landmarks...\n");
			uint numNewMatches = keyFrame.matchNew(observations, rig, subframe, drawNew);
			
			// Add the rest as new landmarks.
			uint numMatches    = numOldMatches + numNewMatches;
			uint numAdded      = keyFrame.addNew(observations, numMatches, subframe);

			display(drawNew, drawOld);

			// Determine processing time required.
		    entT = std::clock();
		    printf(" > Old matches: %3u   New Matches: %3u  Number Added: %3u",
		    	numOldMatches, numNewMatches, numAdded);
		    printf("  Processing time: %6.5f seconds\n", ((entT - startT) / 
		    	(double) CLOCKS_PER_SEC));

		    while(((std::clock() - startT) / (double) CLOCKS_PER_SEC) < SECONDS_PER_FRAME);
		    startT = std::clock();

		}

		// Add a new KeyFrame to the data structure.
		createNewKeyFrame(keyFrame);

	}

	// Show 3D view of the resulting map:
	SRBA::TOpenGLRepresentationOptions options;
	mrpt::opengl::CSetOfObjectsPtr RBAscene = mrpt::opengl::CSetOfObjects::Create();
	bundleAdjuster.build_opengl_representation( 0, options, RBAscene);


	// Display:
#if MRPT_HAS_WXWIDGETS
	mrpt::gui::CDisplayWindow3D win("SRBA results", 1280, 960); 
	{
		mrpt::opengl::COpenGLScenePtr &scene = win.get3DSceneAndLock();
		scene->insert(RBAscene);
		win.unlockAccess3DScene();
	}
	win.setCameraZoom(1);
	win.repaint();

	std::cout << "Press any key or close window to exit.\n";
	win.waitForKey();
#endif

}


void Slam::constructSearchSpace(KeyFrameState& keyFrame, int root) {

	// Only construct the search space if there is something to traverse.
	if(root != -1) {

		// Traverses the graph in breadth-first fashion.
		SearchSpaceConstructor ssc(&bundleAdjuster, &keyFrame.oldIndices, 
			&keyFrame.oldPositions);

		// Perform Breadth-First Search and project landmarks from the
		// 	Active Region into the current frame.
		bundleAdjuster.bfs_visitor(root, MAX_SEARCH_DEPTH, false, ssc, 
			ssc, ssc, ssc);

		// Update the number of old Landmarks projected.
		keyFrame.updateOldSetCount();

		// Initilize the 'lastSeen' counter as well as the descriptor matrix.
		if(keyFrame.oldSet.size() > 0) {

			// Initlaize the first.
			keyFrame.oldSet.lastSeen.push_back(-1);
			keyFrame.oldSet.observations.push_back(std::vector<Observation>());
			keyFrame.oldSet.descriptors = allDescriptors.row(
				keyFrame.oldIndices[0]
			);

			// Now do this for the rest.
			for(uint i = 1; i < keyFrame.oldSet.size(); ++i) {

				keyFrame.oldSet.lastSeen.push_back(-1);
				keyFrame.oldSet.observations.push_back(std::vector<Observation>());
				keyFrame.oldSet.descriptors.push_back(allDescriptors.row(
					keyFrame.oldIndices[i])
				);

			}

		}

	}

}



void Slam::createNewKeyFrame(KeyFrameState& keyframe) {

	// CREATE A NEW KEYFRAME!!!!
	SRBA::new_kf_observations_t observationList;
	SRBA::new_kf_observation_t currentObservation;

	// Landmarks are unknown, but we have some idea of where they are.
	currentObservation.is_fixed = false;
	currentObservation.is_unknown_with_init_val = false;

	//int oldLandmarkNumber = 0;
	//printf("#### OBSERVED OLD LANDMARKS #####\n");

	// Observed old landmarks.
	for(uint i = 0; i < keyframe.oldSet.observations.size(); ++i) {

		uint n = keyframe.oldSet.observations[i].size();

		// If we observed the landmark in at least 20% of the frames,
		//	mark it as recognized.
		if(n > 0.2 * SUBFRAMES_PER_FRAME && keyframe.oldSet.lastSeen[i] > 
			SUBFRAMES_PER_FRAME - 3) { // 

			//oldLandmarkNumber++;
			Observation obs = keyframe.oldSet.observations[i].back();
			float x = obs.relativePosition(SE3_X);
			float y = obs.relativePosition(SE3_Y);
			float z = obs.relativePosition(SE3_Z);

			//printf(" Old Landmark Obser: %3d  -  {% -5.2f, % -5.2f, % -5.2f}\n", oldLandmarkIndices[i],
			//	x, y, z);

			currentObservation.obs.feat_id = keyframe.oldIndices[i];
			currentObservation.obs.obs_data.pt = mrpt::math::TPoint3D(x, y, z);
			observationList.push_back(currentObservation);

			// Update descriptor:
			allDescriptors.row(keyframe.oldIndices[i]) = 
				keyframe.oldSet.descriptors.row(i);

		}

	}

	//int newLandmarkNumber = 0;
	//printf("#### ADDING NEW LANDMARKS #####\n");

	// Observed new landmarks.
	for(uint i = 0; i < keyframe.newSet.observations.size(); ++i) {

		uint n = keyframe.newSet.observations[i].size();
			
		// If we observed the landmark in at least 20% of the frames,
		//	mark it as recognized.
		if(n > 0.2 * SUBFRAMES_PER_FRAME && keyframe.newSet.lastSeen[i] > 
			SUBFRAMES_PER_FRAME - 3) { //  

			//newLandmarkNumber++;
			Observation obs = keyframe.newSet.observations[i].back();
			float x = obs.relativePosition(SE3_X);
			float y = obs.relativePosition(SE3_Y);
			float z = obs.relativePosition(SE3_Z);

			//printf(" New Landmark Added: %3d  -  {% -5.2f, % -5.2f, % -5.2f}\n", landmarkCount,
			//	x, y, z);

			// Register the new landmark.
			currentObservation.obs.feat_id = landmarkCount++;
			currentObservation.obs.obs_data.pt = mrpt::math::TPoint3D(x, y, z);
			observationList.push_back(currentObservation);
			if(allDescriptors.empty())
				allDescriptors = keyframe.newSet.descriptors.row(i);
			else
				allDescriptors.push_back(keyframe.newSet.descriptors.row(i));

		}

	}

	//printf("################################\n");
	//printf("Number added to keyframe: new=%d old=%d total =%d\n", newLandmarkNumber, 
	//	oldLandmarkNumber, newLandmarkNumber + oldLandmarkNumber);

	// Create the new keyframe.
	SRBA::TNewKeyFrameInfo newKeyFrameInfo;
	std::cout <<  "================================================================" << std::endl;
	bundleAdjuster.define_new_keyframe(observationList, newKeyFrameInfo, true);

	std::cout << " KF #" << newKeyFrameInfo.kf_id
	<< " | # kf-to-kf edges created:" <<  newKeyFrameInfo.created_edge_ids.size() << std::endl

	<< " Optimization error: " << newKeyFrameInfo.optimize_results.total_sqr_error_init 
	<< " -> " << newKeyFrameInfo.optimize_results.total_sqr_error_final << std::endl

	<< "================================================================" << std::endl;

	lastKeyFrame = newKeyFrameInfo.kf_id;

}


void Slam::initialize() {

	// 0: None; 1:Important only; 2:Verbose
	bundleAdjuster.setVerbosityLevel(1);

	bundleAdjuster.parameters.srba.use_robust_kernel = true;
	bundleAdjuster.parameters.obs_noise.std_noise_observations = 0.5;
	bundleAdjuster.parameters.srba.max_tree_depth = MAX_SEARCH_DEPTH;
	bundleAdjuster.parameters.srba.max_optimize_depth = MAX_SEARCH_DEPTH;

}

void Slam::display(const cv::Mat& left, const cv::Mat& right) {

	cv::Size leftSize = left.size();
	cv::Size rightSize = right.size();

	cv::Mat stereo(leftSize.height, leftSize.width + rightSize.width, CV_8UC3);

	cv::Mat leftRect(stereo, cv::Rect(0, 0, leftSize.width, leftSize.height));
	left.copyTo(leftRect);

	cv::Mat rightRect(stereo, cv::Rect(leftSize.width, 0, rightSize.width, 
		rightSize.height));
	right.copyTo(rightRect);

	cv::putText(stereo, "Old Landmarks Matched", cv::Point(leftSize.width/2,
		leftSize.height/2), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8f, 
		cv::Scalar(0, 0, 225));

	cv::putText(stereo, "New Landmarks Matched", cv::Point(leftSize.width + 
		rightSize.width/2, leftSize.height/2), cv::FONT_HERSHEY_COMPLEX_SMALL,
		0.8f, cv::Scalar(0, 0, 225));

	cv::imshow("Stereo", stereo);
	cv::waitKey(1);

}

// if(DEBUG_SHOW) {
// 
// 	// Grab the size of the images.
// 	cv::Size leftSize = left.size();
// 	cv::Size rightSize = right.size();
// 
// 	// Copy over the source images for drawing.
// 	cv::cvtColor(left, drawLeft, CV_GRAY2RGB);
// 	cv::cvtColor(right, drawRight, CV_GRAY2RGB);
// 
// 	// Highlight the keypoint in the left image.
// 	cv::circle(drawLeft, leftPoint, 4, cv::Scalar(0.0, 0.0, 255.0), 2);
// 	cv::circle(drawRight, rightPoint, 4, cv::Scalar(0.0, 100.0, 255.0), 2);
// 	cv::rectangle(drawRight, cv::Point(l, leftPoint.y-half_ep), cv::Point(r, 
// 		leftPoint.y+half_ep), cv::Scalar( 0, 55, 255 ), +1, 4);
// 
// 	// Show the template image.
// 	cv::Mat showTempl;
// 	cv::resize(templ, showTempl, cv::Size(100,100));
// 	imshow("Source Template", showTempl);
// 
// 	// Show the matched template.
// 	cv::Mat templ2(right, cv::Rect(rightPoint.x-half_ep, rightPoint.y-half_ep, epsilon, epsilon));
// 	cv::Mat showTempl2;
// 	cv::resize(templ2, showTempl2, cv::Size(100,100));
// 	imshow("Matched Template", showTempl2);
// 
// 	// Show everything.
//     cv::Mat stereo(leftSize.height, leftSize.width + rightSize.width, CV_8UC3);
//     cv::Mat leftRect(stereo, cv::Rect(0, 0, leftSize.width, leftSize.height));
//     drawLeft.copyTo(leftRect);
//     cv::Mat rightRect(stereo, cv::Rect(leftSize.width, 0, rightSize.width, rightSize.height));
//     drawRight.copyTo(rightRect);
// 	cv::imshow("Stereo", stereo);
// 	cv::waitKey(0);
// 
// }

//Mat3 projection;
//projection << focal_length,            0, center_x, 
//			             0, focal_length, center_y, 
//			             0,            0,        1;
//
//// Copy over the source images for drawing.
//cv::cvtColor(prevLeft, drawLeft, CV_GRAY2RGB);
//cv::cvtColor(left, drawRight, CV_GRAY2RGB);
//
//Vec3 projected;
//Vec2 pixels;
//cv::Point p;
//
//for(uint f = 1; f <= 3; ++f) {
//	if (f > keyFrameIndex) break;
//	for(uint i = 0; i < keyframes[keyFrameIndex-f].landmarkIndices.size(); ++i) {
//
//		uint index = keyframes[keyFrameIndex-f].landmarkIndices[i];
//
//		// Left Image - previous frame.
//		projected = projection * landmarks[index].relativePosition;
//		pixels = Vec2(projected(0)/projected(2), projected(1)/projected(2));
//		p = cv::Point (pixels(0), pixels(1));
//		cv::circle(drawLeft, p, 4, cv::Scalar(0.0, 0.0, 255.0), 2);
//
//		// Right Image - current frame.
//		Mat4 proj = composeTransform(keyFrameIndex-f, keyFrameIndex, keyframes);
//
//		projected = projection * homogenousTransform(proj, landmarks[index].relativePosition);
//		pixels = Vec2(projected(0)/projected(2), projected(1)/projected(2));
//		p = cv::Point(pixels(0), pixels(1));
//		cv::circle(drawRight, p, 4, cv::Scalar(0.0, 0.0, 255.0), 2);
//
//	}
//
//}
//
//// Grab the size of the images.
//cv::Size leftSize = drawLeft.size();
//cv::Size rightSize = drawRight.size();
//cv::Mat stereo(leftSize.height, leftSize.width + rightSize.width, CV_8UC3);
//cv::Mat leftRect(stereo, cv::Rect(0, 0, leftSize.width, leftSize.height));
//drawLeft.copyTo(leftRect);
//cv::Mat rightRect(stereo, cv::Rect(leftSize.width, 0, rightSize.width, rightSize.height));
//drawRight.copyTo(rightRect);
//cv::imshow("Stereo", stereo);
//cv::waitKey(0);
