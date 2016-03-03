#include "robotObject.hpp"

int RobotObject::uniqueRobotObjectIdGenerator = 1;
RobotObject::RobotObject()
{
	uniqueRobotObjectId = uniqueRobotObjectIdGenerator++;
}

int RobotObject::getUniqueRobotObjectId()
{
	return uniqueRobotObjectId;
}