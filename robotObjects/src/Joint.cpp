#include "Joint.hpp"
#include <iostream>




double truncValue(double value, int precision)
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

Joint::Joint(double max_value, double min_value, const char * unit) : RobotObject()
{
	positions = new double[3];
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
	{
		cerr << "Joint ERROR: Unit not valid" << endl;
		exit(EXIT_FAILURE);
	}
}

Joint::~Joint()
{

}

void Joint::setJointNextPosition(double position)
{
	positions[2] = positions[1];
	positions[1] = positions[0];

	if (position > max_value)
	{
		positions[0] = truncValue((this->*convertToRadFrom)(max_value),PRECISION);
	}
	else if (position < min_value)
	{
		positions[0] = truncValue((this->*convertToRadFrom)(min_value),PRECISION);
	}
	else
	{
		positions[0] = truncValue((this->*convertToRadFrom)(position),PRECISION);
	}

	// Using the interval [-pi,pi] for simetric reasons.
	while(positions[0] >= M_PI || positions[0] <= -M_PI)
	{
		if( positions[0] < -M_PI )
		{
			positions[0] += 2.0 * M_PI;
		}
		else
		{
			positions[0] -= 2.0 * M_PI;
		}
	}
}

double Joint::getJointGoalPosition()
{
	return (this->*convertFromRadTo)(positions[0]);
}

double Joint::getMaxAngle()
{
	return max_value;
}

double Joint::getMinAngle()
{
	return min_value;
}

double Joint::getNextPositionRad()
{
	return positions[0];
}


