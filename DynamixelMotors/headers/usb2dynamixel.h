/* */
#ifndef USB2Dynamixel_H
#define USB2Dynamixel_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <map>
#include "serial.h"
#include "dynamixel.h"
#include <cmath>
#include <iostream>
#include <algorithm>    // std::find
#include "motor.h"
#include "dynamixelMotor.hpp"
#include "Joint.hpp"

/* MEMADDR */
#define P_MODEL_L				0
#define P_TORQUE_ENABLE			24

#define P_GOAL_POSITION_L       30
#define P_GOAL_POSITION_H       31
#define P_MOVING_SPEED_L        32
#define P_MOVING_SPEED_H        33

#define P_PRESENT_POSITION_L    36
#define P_PRESENT_POSITION_H    37
#define P_PRESENT_SPEED_L       38
#define P_PRESENT_SPEED_H       39
#define P_PRESENT_LOAD_L        40
#define P_PRESENT_LOAD_H        41
#define P_PRESENT_VOLTAGE       42
#define P_PRESENT_TEMPERATURE   43
#define P_CURRENT_LOW			(68)

#define TAMANO_BUFFER_COMUNICACION 10000
#define USE_USB2DXL

#define DEF_TIMEOUT		100000
#define SETPOSNDVEL		0x02
#define ASKPOSNDVEL		0x04
#define ASKDATACTID		0x08
#define ASKSENSOR		0x10
#define RPYPOSNVEL		0x40
#define RPYDATAID		0x41
#define SETTORQUE		0x80
#define SETTORQEXID		0x81
#define ERRORMOVING		2
#define ERRORREAD		3
#define _L16(x)			((x >> 0) & 0xFF)
#define _H16(x)			((x >> 8) & 0xFF)
#define _MW(x, y)		(y * 256 + x)



using namespace std;

class USB2Dynamixel : public DynamixelMotor
{
	int fd;
	uint8_t buffer_in [ 255 ];
	char buffer_out [ 255 ];
	string serialPort;
	std::vector <Motor> motors;
	std::map <int , int  > idToMotorsVectorPosition_map;
	std::vector < int > idMotorsWithNewPosition_Vect;
	std::map <int , int  > 	jointIdToId_map;
	std::vector < Joint * > jointVector;

	int identifyModel( int id );

public:	
	USB2Dynamixel (string serialPort, int baudNum);
	~USB2Dynamixel(); 
	void addMotor(Joint * joint, int id);
	void addMotor(Joint * joint, int id, int angleResolution, bool hasCurrentSensor, int velocityResolution, double angleRangeDeg);
	double getAngle(int id);
	void refreshAll();
	void move();
	void setTorque(bool enable);
	void printValues();
	bool verifyModel(int model);
	void setParametersFromModel(int model, Motor &motor);	
	void setNextAngle(int id, double angle_RAD, double velocity);
};
#endif
