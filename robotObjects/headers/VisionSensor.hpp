#ifndef VISIONSENSOR_HPP
#define VISIONSENSOR_HPP

#include "Object.hpp"
#include "RobotVREP.hpp"
#include <iostream>
#include <stdlib.h>

using namespace std;

class VisionSensor : public Object
{
	vector < int > resolution;

	friend class RobotVREP;

public:
	VisionSensor(const char name[]);
	~VisionSensor();

	vector < int > getResolution();
};

#endif