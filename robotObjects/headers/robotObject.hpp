#ifndef ROBOTOBJECT_HPP
#define ROBOTOBJECT_HPP

class RobotObject
{
	int uniqueRobotObjectId;
public:
	static int uniqueRobotObjectIdGenerator;
	int getUniqueRobotObjectId();
	RobotObject();
};

#endif