#ifndef _CAMERA_H_
#define _CAMERA_H_

#include "Types.h"

class Camera {

private:

	float focalLength;
	float center[2];
	float skew;
	Mat3  K;

public:

	Camera(float _focalLength, float _centerX, float _centerY, 
		float _skew = 0) :

		focalLength(_focalLength),
		skew(_skew),
		K() {

		center[SE3_X] = _centerX;
		center[SE3_Y] = _centerY;

		K << focalLength,         skew, center[SE3_X], 
						0, focalLength, center[SE3_Y], 
						0,           0,             1;	

	}

	float getCenter(uint i) const {

		return (i >= 0 && i <= 1) ? center[i] : -1;

	}

	float getFocalLength() const {

		return focalLength;

	}

	float getSkew() const {

		return skew;

	}

	Mat3 getK() const {

		return K;

	}

};

#endif