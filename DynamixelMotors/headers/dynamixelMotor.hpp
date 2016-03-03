#ifndef DYNAMIXELMOTOR_HPP
#define DYNAMIXELMOTOR_HPP

#include "motor.h"
class Joint;

class DynamixelMotor
{
public: 
	virtual ~DynamixelMotor(){};
	virtual void addMotor(Joint * joint, int id)=0;
	virtual void addMotor(Joint * joint, int id, int angleResolution, bool hasCurrentSensor, int velocityResolution, double angleRangeDeg)=0;
	// virtual double getAngle(int id)=0;
	// virtual void refreshAll()=0;
	virtual void move()=0;
	// virtual void setTorque(bool enable)=0;
	// virtual void printValues()=0;
	// virtual bool verifyModel(int model)=0;
	// virtual void setParametersFromModel(int model, Motor &motor)=0;
	// virtual void setNextAngle(int id, double angle_RAD, double velocity)=0;
};

#endif
