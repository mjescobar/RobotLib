#include "VisionSensor.hpp"

VisionSensor::VisionSensor(const char name[]) : Object(name)
{
	resolution = vector < int > (2,-1);
}

VisionSensor::~VisionSensor()
{

}

vector < int > VisionSensor::getResolution()
{
	return resolution;
}