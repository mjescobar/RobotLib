#ifndef CM700_HPP
#define CM700_HPP

#include "Joint.hpp"
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
	std::map <int , int  > 	jointIdToId_map;
	std::vector < int > 	idMotorsWithNewPosition_Vect;
	char * serial_name;
	std::vector < Joint * > jointVector;
public:	
	CM700 (string serialPort, speed_t baudrate);
	~CM700(); 
	void addMotor(Joint * joint, int id);
	void addMotor(Joint * joint, int id, int angleResolution, bool hasCurrentSensor, int velocityResolution, double angleRangeDeg);
	double getAngle(int id);
	void refreshAll();
	void move();
	void setTorque(bool enable);
	void printValues();
	bool verifyModel(int model);
	void setParametersFromModel(int model, Motor &motor);	
	//void setNextAngle(int id, double angle_RAD, double velocity);

};


#endif
