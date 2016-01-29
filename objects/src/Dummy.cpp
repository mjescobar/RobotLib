#ifndef DUMMY_CPP
#define DUMMY_CPP

#include "Dummy.hpp"
using namespace ANN_USM;

Dummy::Dummy(RobotSimulator * simulator, char name[]) : Object(simulator, name)
{
	lVelocity = new double[3];
	aVelocity = new double[3];

	getVelocity(NULL,NULL,simx_opmode_streaming);
}

Dummy::Dummy()
{

}

Dummy::~Dummy()
{

}

void Dummy::getVelocity(double lVel[3], double aVel[3])
{
	if(simulator != NULL) simulator->simGetObjectVelocity(handle, lVelocity, aVelocity, simx_opmode_buffer);
	else clog << "ERROR: Function 'Dummy::getVelocity(double lVel[3])' not implemented in other enviroment" << endl;
	
	if (lVel != NULL)
		for (int i = 0; i < 3; i++)
			lVel[i] = lVelocity[i];	

	if (aVel != NULL)
		for (int i = 0; i < 3; i++)
			aVel[i] = aVelocity[i];		
}

void Dummy::getVelocity(double lVel[3], double aVel[3], simxInt operationMode)
{
	if(simulator != NULL) simulator->simGetObjectVelocity(handle, lVelocity, aVelocity, operationMode);
	else clog << "ERROR: Function 'Dummy::getVelocity(double lVel[3])' not implemented in other enviroment" << endl;
	
	if (lVel != NULL)
		for (int i = 0; i < 3; i++)
			lVel[i] = lVelocity[i];	

	if (aVel != NULL)
		for (int i = 0; i < 3; i++)
			aVel[i] = aVelocity[i];		
}

#endif

