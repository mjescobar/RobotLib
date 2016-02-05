#ifndef OBJECT_CPP
#define OBJECT_CPP

#include "Object.hpp"
using namespace ANN_USM;

Object::Object(RobotVREP * simulator, char name[])
{
	this->simulator = simulator;
	this->cm700 = NULL;

	int size = strlen(name)+1;
	this->name = new char[size];
	strncpy(this->name, name, size);

	position = new double[3];
	orientation = new double[3];
	lVelocity = new double[3];
	aVelocity = new double[3];

	simulator->getObjectHandle(this->name, &handle, simx_opmode_oneshot_wait);
	getPosition(-1,NULL,simx_opmode_streaming);
	getOrientation(-1,NULL,simx_opmode_streaming);
	getVelocity(NULL,NULL,simx_opmode_streaming);
}

Object::Object(CM700 * cm700, char name[], int id)
{
	this->simulator = NULL;
	this->cm700 = cm700;

	int size = strlen(name)+1;
	this->name = new char[size];
	strncpy(this->name, name, size);

	position = NULL;
	orientation = NULL;

	this->id = id;
}

Object::Object()
{

}

Object::~Object()
{
	delete name;
	delete position;
	delete orientation;
	delete lVelocity;
	delete aVelocity;
	if (simulator != NULL)	delete simulator;
	else if (cm700 != NULL) delete cm700;
}

int Object::getHandle()
{
	return handle;
}

char * Object::getName()
{
	return name;
}

void Object::getPosition(int relativeTo, double position[3])
{
	if(simulator != NULL) simulator->getObjectPosition(handle, relativeTo, this->position, simx_opmode_buffer);
	else clog << "ERROR: Function 'Object::getPosition(int relativeTo)' not implemented in other enviroment" << endl;

	if (position != NULL)
		for (int i = 0; i < 3; i++)
			position[i] = this->position[i];
}

void Object::getPosition(int relativeTo, double position[3], simxInt operationMode)
{
	if(simulator != NULL) simulator->getObjectPosition(handle, relativeTo, this->position, operationMode);
	else clog << "ERROR: Function 'Object::getPosition(int relativeTo)' not implemented in other enviroment" << endl;

	if (position != NULL)
		for (int i = 0; i < 3; i++)
			position[i] = this->position[i];
}

void Object::getOrientation(int relativeTo, double orientation[3])
{
	if(simulator != NULL) simulator->getObjectOrientation(handle, relativeTo, this->orientation, simx_opmode_buffer);
	else clog << "ERROR: Function 'Object::getOrientation(int relativeTo)' not implemented in other enviroment" << endl;

	if (orientation != NULL)
		for (int i = 0; i < 3; i++)
			orientation[i] = this->orientation[i];
}

void Object::getOrientation(int relativeTo, double orientation[3], simxInt operationMode)
{
	if(simulator != NULL) simulator->getObjectOrientation(handle, relativeTo, this->orientation, operationMode);
	else clog << "ERROR: Function 'Object::getOrientation(int relativeTo)' not implemented in other enviroment" << endl;

	if (orientation != NULL)
		for (int i = 0; i < 3; i++)
			orientation[i] = this->orientation[i];
}

void Object::getVelocity(double lVel[3], double aVel[3])
{
	if(simulator != NULL) simulator->getObjectVelocity(handle, lVelocity, aVelocity, simx_opmode_buffer);
	else clog << "ERROR: Function 'Object::getVelocity(double lVel[3])' not implemented in other enviroment" << endl;
	
	if (lVel != NULL)
		for (int i = 0; i < 3; i++)
			lVel[i] = lVelocity[i];	

	if (aVel != NULL)
		for (int i = 0; i < 3; i++)
			aVel[i] = aVelocity[i];		
}

void Object::getVelocity(double lVel[3], double aVel[3], simxInt operationMode)
{
	if(simulator != NULL) simulator->getObjectVelocity(handle, lVelocity, aVelocity, operationMode);
	else clog << "ERROR: Function 'Object::getVelocity(double lVel[3])' not implemented in other enviroment" << endl;
	
	if (lVel != NULL)
		for (int i = 0; i < 3; i++)
			lVel[i] = lVelocity[i];	

	if (aVel != NULL)
		for (int i = 0; i < 3; i++)
			aVel[i] = aVelocity[i];		
}

#endif