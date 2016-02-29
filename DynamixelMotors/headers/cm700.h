#ifndef CM700_HPP
#define CM700_HPP

#include "usb2dynamixel.h"
#include "serial.h"
#include "motor.h"
#include "dynamixel.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <iostream>
#include <algorithm>    // std::find
#include "dynamixelMotor.hpp"

using namespace std;

class CM700 : public DynamixelMotor
{
	int 					fd;
	uint8_t 				buffer_in [ 255 ];
	char 					buffer_out [ 255 ];
	string 					serialPort;
	std::vector < Motor > 	motors;
	std::map <int , int  > 	idToMotorsVectorPosition_map;
	std::vector < int > 	idMotorsWithNewPosition_Vect;
	char * serial_name;

public:	
	CM700 (string serialPort, speed_t baudrate);
	~CM700(); 
	void addMotor(int id);
	void addMotor(int id, int angleResolution, bool hasCurrentSensor, int velocityResolution, double angleRangeDeg);
	double getMotorAngle(int id);
	void refreshAll();
	void move();
	void setTorque(bool enable);
	void printValues();
	bool verifyModel(int model);
	void setMotorParametersFromModel(int model, Motor &motor);	
	void setNextMotorAngle(int id, double angle_RAD, double velocity);

};


#endif
