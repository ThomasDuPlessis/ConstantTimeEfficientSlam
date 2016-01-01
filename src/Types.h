#ifndef _TYPES_
#define _TYPES_

// OpenCV
#include "opencv2/core/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/opencv_modules.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"

// MRPT
#include <mrpt/srba.h>
#include <mrpt/gui.h>

// Eigen
#include <Eigen/Dense>

// Pose Constants
#define SE3_X 0
#define SE3_Y 1
#define SE3_Z 2
#define SE3_R 3
#define SE3_P 4
#define SE3_W 5

typedef Eigen::Matrix<float, 2, 1> Vec2;
typedef Eigen::Matrix<float, 3, 1> Vec3;
typedef Eigen::Matrix<float, 4, 1> Vec4;
typedef Eigen::Matrix<float, 6, 1> Pose;
typedef Eigen::Matrix<float, 3, 3> Mat3;
typedef Eigen::Matrix<float, 4, 4> Mat4;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MatX;

// SRBA Types.
typedef srba::kf2kf_poses::SE3           POSES_TYPE;
typedef srba::landmarks::Euclidean3D     LANDMARKS_TYPE;
typedef srba::observations::Cartesian_3D OBSERVATIONS_TYPE;

struct SbraOptions : public srba::RBA_OPTIONS_DEFAULT {

	typedef srba::options::sensor_pose_on_robot_se3 sensor_pose_on_robot_t;
	typedef srba::options::observation_noise_identity obs_noise_matrix_t;
	typedef srba::ecps::classic_linear_rba edge_creation_policy_t;

};

typedef srba::RbaEngine<POSES_TYPE, LANDMARKS_TYPE, OBSERVATIONS_TYPE, 
	SbraOptions>  SRBA;

#endif
