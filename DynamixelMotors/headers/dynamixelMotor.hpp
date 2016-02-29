#ifndef DYNAMIXELMOTOR_HPP
#define DYNAMIXELMOTOR_HPP

#include "motor.h"

class DynamixelMotor
{
public: 
	virtual ~DynamixelMotor(){};
	virtual void addMotor(int id)=0;
	virtual void addMotor(int id, int angleResolution, bool hasCurrentSensor, int velocityResolution, double angleRangeDeg)=0;
	virtual double getMotorAngle(int id)=0;
	virtual void refreshAll()=0;
	virtual void move()=0;
	virtual void setTorque(bool enable)=0;
	virtual void printValues()=0;
	virtual bool verifyModel(int model)=0;
	virtual void setMotorParametersFromModel(int model, Motor &motor)=0;
	virtual void setNextMotorAngle(int id, double angle_RAD, double velocity)=0;
};

#endif
