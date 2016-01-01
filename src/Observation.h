#ifndef _OBSERVATION_H_
#define _OBSERVATION_H_

#include "Types.h"

class Observation {

public:

	Vec2 leftPixel;
	Vec2 rightPixel;
	Vec3 relativePosition;

	Observation(const Vec2& _leftPixel, const Vec2& _rightPixel, 
		const Vec3& _relativePosition) :

		leftPixel(_leftPixel),
		rightPixel(_rightPixel),
		relativePosition(_relativePosition) {

		// Empty.

	}

	// Copy constructor.
	Observation(const Observation& rhs) :

		leftPixel(rhs.leftPixel),
		rightPixel(rhs.rightPixel),
		relativePosition(rhs.relativePosition) {

		// Empty.

	}

};

#endif