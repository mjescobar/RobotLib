#ifndef MOTOR_DYNAMIXEL_H
#define MOTOR_DYNAMIXEL_H

enum Models 
{
	AX12 = 12,
	MX64 = 54,
	AX18 = 18
};

struct Motor 
{
	int id;
	int jointId;
	int cposition;
	int tposition;
	int cspeed;
	int tspeed;
	int load;
	int volt;
	int current;
	int temperature;
	int angleResolution;
	bool hasCurrentSensor;
	int velocityResolution;
	double angleRangeDeg; // in case that the dynamixel motor have not 360 deg of range like ax12 that have only 300 deg of range.s
};

#endif

