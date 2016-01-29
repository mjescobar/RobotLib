/* */
#ifndef CM700_H
#define CM700_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>

#include "serial.h"
#include "dynamixel.h"

#include <iostream>

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

#define TAMANO_BUFFER_COMUNICACION 10000
#define USE_USB2DXL
#define PUERTO_SERIAL "/dev/ttyUSB0"

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

struct actuador {
	int id;

	int cposition;
	int tposition;

	int cspeed;
	int tspeed;

	int load;

	int volt;
	int current;
	int temperature;
};

class CM700 {
	int fd;
	char *serial_name;

	uint8_t buffer_in[255];
	char buffer_out[255];

	int num_actuadores;
	int actuadores_leg;

	struct actuador *actuadores;
	struct actuador servocam;
	time_t timestamp;
public:
	
	CM700(int, int);
	~CM700(void);

	void setMotorPosition(int id, int pos, int vel);
	int getMotorPosition(int id);
	void refreshAll();
	void moveAll();
	void setTorque(bool enable);
	void printValues();
	
	friend class Robot;
	friend class Leg;
};
#endif
