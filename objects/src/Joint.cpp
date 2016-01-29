#ifndef JOINT_CPP
#define JOINT_CPP

#include "Joint.hpp"
#include <iostream>
using namespace ANN_USM;


double Joint::truncValue(double value, int precision)
{
	return (double)floor(value*pow(10,precision))/pow(10,precision);
}

double Joint::RADtoRAD(double value)
{
	return value;
}

double Joint::DEGtoRAD(double value)
{
	return value*RAD_CONV;
}

double Joint::SCALEtoRAD(double value)
{
	return (max_value - min_value )*(value + 1.0)/2.0 + min_value;
}

double Joint::RADtoDEG(double value)
{
	return value/RAD_CONV;
}

double Joint::RADtoSCALE(double value)
{
	return 2.0*(value - min_value)/(max_value - min_value) - 1.0;
}

double Joint::RADtoAX(double value)
{
	return AX_RAD_TO_NUMBER(value);
}

double Joint::AXtoRAD(double value)
{
	double rad = AX_NUMBER_TO_RAD(value);
	return rad;
}

double Joint::Filter(double value)
{
	return (double)((-(W_CONST*H_CONST - 2.0)*filtered_positions[1] + K_CONST*H_CONST*value + K_CONST*H_CONST*positions[1])/(2 + W_CONST*H_CONST));

}

Joint::Joint(RobotSimulator * simulator, char name[], double max_value, double min_value, const char * unit, double position)  : Object(simulator, name)
{
	this->max_value = max_value;
	this->min_value = min_value;

	if (!strcmp(unit,(char *)"RAD"))
	{
		convertToRadFrom = &Joint::RADtoRAD;
		convertFromRadTo = &Joint::RADtoRAD;	
	} 
	else if (!strcmp(unit,(char *)"DEG")) 
	{
		convertToRadFrom = &Joint::DEGtoRAD;	
		convertFromRadTo = &Joint::RADtoDEG;
	}
	else if (!strcmp(unit,(char *)"SCALE"))
	{
		convertToRadFrom = &Joint::SCALEtoRAD;	
		convertFromRadTo = &Joint::RADtoSCALE;
	}
	else
		cerr << "Joint ERROR: Unit not valid" << endl;

	robotInput = &Joint::RADtoRAD;
	robotOutput = &Joint::RADtoRAD;
	
	positions = new double[3];
	filtered_positions = new double[3];

	initial_position = truncValue(position, PRECISION);

	getJointForce(simx_opmode_streaming);
	getJointCurrentPosition(simx_opmode_streaming);

	setJointInitialPosition();

}

Joint::Joint(RobotSimulator * simulator, char name[], double max_value, double min_value, const char * unit) : Object(simulator, name)
{
	this->max_value = max_value;
	this->min_value = min_value;

	if (!strcmp(unit,(char *)"RAD"))
	{
		convertToRadFrom = &Joint::RADtoRAD;
		convertFromRadTo = &Joint::RADtoRAD;	
	} 
	else if (!strcmp(unit,(char *)"DEG")) 
	{
		convertToRadFrom = &Joint::DEGtoRAD;	
		convertFromRadTo = &Joint::RADtoDEG;
	}
	else if (!strcmp(unit,(char *)"SCALE"))
	{
		convertToRadFrom = &Joint::SCALEtoRAD;	
		convertFromRadTo = &Joint::RADtoSCALE;
	}
	else
		cerr << "Joint ERROR: Unit not valid" << endl;

	robotInput = &Joint::RADtoRAD;
	robotOutput = &Joint::RADtoRAD;

	positions = new double[3];
	filtered_positions = new double[3];

	initial_position = truncValue(simulator->simGetJointPosition(handle, simx_opmode_oneshot_wait), PRECISION);

	getJointForce(simx_opmode_streaming);
	getJointCurrentPosition(simx_opmode_streaming);

	setJointInitialPosition();
}


Joint::Joint(CM700 * cm700, char name[], int id, double max_value, double min_value, const char * unit, double position) : Object(cm700, name, id)
{
	this->max_value = max_value;
	this->min_value = min_value;

	if (!strcmp(unit,(char *)"RAD"))
	{
		convertToRadFrom = &Joint::RADtoRAD;
		convertFromRadTo = &Joint::RADtoRAD;	
	} 
	else if (!strcmp(unit,(char *)"DEG")) 
	{
		convertToRadFrom = &Joint::DEGtoRAD;	
		convertFromRadTo = &Joint::RADtoDEG;
	}
	else if (!strcmp(unit,(char *)"SCALE"))
	{
		convertToRadFrom = &Joint::SCALEtoRAD;	
		convertFromRadTo = &Joint::RADtoSCALE;
	}
	else
		cerr << "Joint ERROR: Unit not valid" << endl;

	if (!strcmp(name,(char *)"AX"))
	{
		robotInput = &Joint::RADtoAX;
		robotOutput = &Joint::AXtoRAD;
		motor_velocity = new int(AX_VEL_MOT_DEF);
	} 
	else
		cerr << "Joint ERROR: Name not valid" << endl;

	positions = new double[3];
	filtered_positions = new double[3];

	initial_position = truncValue(position, PRECISION);
	current_position = initial_position;

	cm700->setMotorPosition(id, (this->*robotInput)(initial_position), *motor_velocity);
}

Joint::~Joint()
{

}

void Joint::setJointPosition(double position)
{
	positions[2] = positions[1];
	positions[1] = positions[0];
	positions[0] = truncValue((this->*convertToRadFrom)(position),PRECISION);

	filtered_positions[2] = filtered_positions[1];
	filtered_positions[1] = filtered_positions[0];
	filtered_positions[0] = Filter(positions[0]);
		
	if(simulator != NULL) simulator->simSetJointTargetPosition(handle, positions[0], simx_opmode_oneshot);
	else if (cm700 != NULL) cm700->setMotorPosition(id, (this->*robotInput)(positions[0]), *motor_velocity);
	else clog << "ERROR: Function 'Joint::setJointPosition(double position, const char * unit)' not implemented in other enviroment" << endl;
}

void Joint::setJointInitialPosition()
{
	positions[2] = positions[1] = positions[0] = initial_position;
	filtered_positions[2] = filtered_positions[1] = filtered_positions[0] = initial_position;	
	pass_slope_sign = 1;
	next_slope_sign = 1;
	joint_change_direction = false;

	if(simulator != NULL) simulator->simSetJointTargetPosition(handle, initial_position, simx_opmode_oneshot_wait);
	else if (cm700 != NULL) cm700->setMotorPosition(id, (this->*robotInput)(initial_position), *motor_velocity);
	else clog << "ERROR: Function 'Joint::setJointInitialPosition()' not implemented in other enviroment" << endl;
}

double Joint::getJointCurrentPosition()
{
	if (simulator != NULL) current_position = simulator->simGetJointPosition(handle, simx_opmode_buffer);
	else if (cm700 != NULL) current_position = (this->*robotOutput)(cm700->getMotorPosition(id));
	else clog << "ERROR: Function 'Joint::getJointCurrentPosition()' not implemented in other enviroment" << endl;

	return (this->*convertFromRadTo)(current_position);
}

double Joint::getJointCurrentPosition(simxInt operationMode)
{
	if (simulator != NULL) current_position = simulator->simGetJointPosition(handle, operationMode);
	else if (cm700 != NULL) current_position = (this->*robotOutput)(cm700->getMotorPosition(id));
	else clog << "ERROR: Function 'Joint::getJointCurrentPosition()' not implemented in other enviroment" << endl;

	return (this->*convertFromRadTo)(current_position);
}

double Joint::getJointForce()
{
	double force = 0.0;
	if (simulator != NULL) force = simulator->simGetJointForce(handle, simx_opmode_buffer);
	else if (cm700 != NULL) clog << "Not yet implemented" << endl;
	else clog << "ERROR: Function 'Joint::getJointForce()' not implemented in other enviroment" << endl;

	return force;
}

double Joint::getJointForce(simxInt operationMode)
{
	if (simulator != NULL) force = simulator->simGetJointForce(handle, operationMode);
	else if (cm700 != NULL) clog << "Not yet implemented" << endl;
	else clog << "ERROR: Function 'Joint::getJointForce()' not implemented in other enviroment" << endl;

	return force;
}

double Joint::getJointGoalPosition()
{
	return (this->*convertFromRadTo)(positions[0]);
}

double Joint::getJointFilteredPosition()
{
	return (this->*convertFromRadTo)(filtered_positions[0]);
}

double Joint::getJointLastCurrentPosition()
{
	return (this->*convertFromRadTo)(current_position);
}

double Joint::getJointLastForce()
{
	return force;
}

double Joint::getJointInitialPosition()
{
	return (this->*convertFromRadTo)(initial_position);
}

double Joint::getMaxAngle()
{
	return max_value;
}

double Joint::getMinAngle()
{
	return min_value;
}

bool Joint::getJointChangeDirection()
{	
	/*
	double aux_slope = 0;

	double pass_position = current_position;
	getJointCurrentPosition();
	double next_position = current_position;

	pass_slope_sign = next_slope_sign;
	aux_slope = next_position - pass_position;

	if(abs(aux_slope) > TOLERANCE)
	{
		if(aux_slope < 0) next_slope_sign = -1;
		else next_slope_sign = 1; 
	}		

	if(next_slope_sign != pass_slope_sign) joint_change_direction = true;
	else joint_change_direction = false;
	*/
	
	double aux_slope = 0;

	pass_slope_sign = next_slope_sign;
	aux_slope = filtered_positions[0] - filtered_positions[1];

	if(abs(aux_slope) > TOLERANCE)
	{
		if(aux_slope < 0) next_slope_sign = -1;
		else next_slope_sign = 1; 
	}		

	if(next_slope_sign != pass_slope_sign) joint_change_direction = true;
	else joint_change_direction = false;
	

	return joint_change_direction;
}

void Joint::refreshValues()
{
	if (cm700 != NULL) cm700->refreshAll();
	else clog << "ERROR: Function 'Joint::refreshValues()' not implemented in other enviroment" << endl;
}

#endif

