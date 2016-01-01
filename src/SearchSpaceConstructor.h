#ifndef _SEARCHSPACECONSTRUCTOR_H_
#define _SEARCHSPACECONSTRUCTOR_H_

#include <mrpt/srba.h>
#include <vector>
#include <map>

#include "Types.h"

#define MAX_SEARCH_DEPTH 3

typedef std::map<uint, Mat4, std::less<int>, 
	Eigen::aligned_allocator<std::pair<const uint, Mat4>>> KeyFrameTransformMap;

class SearchSpaceConstructor {

public:

	uint                  root;
 	KeyFrameTransformMap  transformMap;
	SRBA*                 bundleAdjuster;
	std::vector<uint>*    projectedLandmarkIDs;
	std::vector<Vec3>*	  projectedLandmarkPositions;

	SearchSpaceConstructor( SRBA* _bundleAdjuster, std::vector<uint>* 
		_projectedLandmarkIDs, std::vector<Vec3>* _projectedLandmarkPositions) :

		root(-1),
		transformMap(), 
		bundleAdjuster(_bundleAdjuster), 
		projectedLandmarkIDs(_projectedLandmarkIDs),
		projectedLandmarkPositions(_projectedLandmarkPositions) {

		// Nothing more.

	}

	void projectLandmarks(const srba::TKeyFrameID kf) {

		// Grab the keyframe info.

		const auto frame = bundleAdjuster->get_rba_state().keyframes[kf];
		const auto all_lms = bundleAdjuster->get_rba_state().all_lms;

		//printf("Projection from base keyframe %d to current keyframe %d:\n", kf, root);
		//std::cout << transformMap[kf] << std::endl;

		// For each feature edge, project the landmark and add it to
		//  the list of projected elements.
		for(uint i = 0; i < frame.adjacent_k2f_edges.size(); ++i) {

			// Grab the landmark, it's id, and it's relative position to
			//	this keyframe.
			auto* landmark = frame.adjacent_k2f_edges[i];

			uint landmarkID = landmark->get_observed_feature_id();

			auto landmarkPosition = landmark->feat_rel_pos->pos;

			// Proejct the landmark.
			Vec3 relPos(landmarkPosition(0), landmarkPosition(1), 
				landmarkPosition(2));
			Vec4 relativePosition(landmarkPosition(0), landmarkPosition(1), 
				landmarkPosition(2), 1.0f);
			Vec4 proj = transformMap[kf] * relativePosition;
			Vec3 projectedPosition = Vec3(proj(0)/proj(3), proj(1)/proj(3), 
				proj(2)/proj(3));

			//printf("  Projecting Landmark %3d:"
			//	"    Base %3d: {% -5.2f, % -5.2f, % -5.2f}"
			//	"    Current %3d: {% -5.2f, % -5.2f, % -5.2f}"
			//	"    Distance: % -5.2f\n", 
			//	landmarkID, kf, root, relativePosition(0), relativePosition(1), 
			//	relativePosition(2), projectedPosition(0),
			//	projectedPosition(1), projectedPosition(2),
			//	euclidianDistance(projectedPosition, relPos)); // Vec3 must come first.

			// Add the projected landmark to the list of elements.
			projectedLandmarkIDs->push_back(landmarkID);
			projectedLandmarkPositions->push_back(projectedPosition);

		}

	}

	bool visit_filter_k2k (const srba::TKeyFrameID current_kf,
		const srba::TKeyFrameID next_kf, const srba::kf2kf_pose_traits<srba::kf2kf_poses::SE3>::k2k_edge_t* edge,
		const srba::topo_dist_t cur_dist) {

		// Visit all reachable nodes.
		return true;

	}

	void visit_k2k(const srba::TKeyFrameID current_kf, 
		const srba::TKeyFrameID next_kf, 
		const srba::kf2kf_pose_traits<srba::kf2kf_poses::SE3>::k2k_edge_t* edge,
		const srba::topo_dist_t cur_dist) {

		// NOTE: Transforms are stored inversely with respect to
		// edges. 

		// Grab the transformation indicated by this edge.
		mrpt::math::CMatrixDouble44 temp;
		edge->inv_pose.getHomogeneousMatrix(temp);
		Mat4 edgeTransform = temp.cast<float>();
		Mat4 toRoot = transformMap[current_kf];
  
		if(current_kf == edge->to) {

			//   + +   T(c->n)   + +   
			//  + c + <-------- + n + 
			//   + +             + + 
			transformMap[next_kf] = edgeTransform * toRoot;

		} else {

			//   + +   T(n->c)   + +   
			//  + c + --------> + n + 
			//   + +             + +    
			transformMap[next_kf] = edgeTransform.inverse() * toRoot;

		}

		// Project all of the landmarks from the next keyframe to the root.
		projectLandmarks(next_kf);

	}


// Visit KeyFrames.
	bool visit_filter_kf(const srba::TKeyFrameID kf_ID, 
		const srba::topo_dist_t cur_dist) { 

		return false;

	}

	void visit_kf(const srba::TKeyFrameID kf_ID, 
		const srba::topo_dist_t cur_dist) {

		root = kf_ID;

		// First edge in BFS, set map value:
		transformMap[kf_ID] = Mat4::Identity();

		// Add all of the landmarks from the root keyframe (no transformation
		//	yet known).
		projectLandmarks(kf_ID);

	}

// Not used:

// Visit Landmarks:
	bool visit_filter_feat(const srba::TLandmarkID lm_ID, 
		const srba::topo_dist_t cur_dist) { return false; }
	void visit_feat(const srba::TLandmarkID lm_ID, 
		const srba::topo_dist_t cur_dist) { /*empty*/ }
// Visit KeyFrame-Landmark edges.
	bool visit_filter_k2f(const srba::TKeyFrameID current_kf,
		const srba::rba_joint_parameterization_traits_t<
		srba::kf2kf_poses::SE3, srba::landmarks::Euclidean3D, srba::observations::Cartesian_3D
		>::k2f_edge_t* edge, 
		const srba::topo_dist_t cur_dist) { return false; }
	void visit_k2f(const srba::TKeyFrameID current_kf,
		const srba::rba_joint_parameterization_traits_t<
		srba::kf2kf_poses::SE3, srba::landmarks::Euclidean3D, srba::observations::Cartesian_3D
		>::k2f_edge_t* edge, 
		const srba::topo_dist_t cur_dist) { /*empty*/ }

};

#endif
