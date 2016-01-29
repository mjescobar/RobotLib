#ifndef OBJECT_CPP
#define OBJECT_CPP

#include "Object.hpp"
using namespace ANN_USM;

Object::Object(RobotSimulator * simulator, char name[])
{
	this->simulator = simulator;
	this->cm700 = NULL;

	int size = strlen(name)+1;
	this->name = new char[size];
	strncpy(this->name, name, size);

	position = new double[3];
	orientation = new double[3];

	simulator->simGetObjectHandle(this->name, &handle, simx_opmode_oneshot_wait);
	getPosition(-1,NULL,simx_opmode_streaming);
	getOrientation(-1,NULL,simx_opmode_streaming);
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
	if(simulator != NULL) simulator->simGetObjectPosition(handle, relativeTo, this->position, simx_opmode_buffer);
	else clog << "ERROR: Function 'Object::getPosition(int relativeTo)' not implemented in other enviroment" << endl;

	if (position != NULL)
		for (int i = 0; i < 3; i++)
			position[i] = this->position[i];
}

void Object::getPosition(int relativeTo, double position[3], simxInt operationMode)
{
	if(simulator != NULL) simulator->simGetObjectPosition(handle, relativeTo, this->position, operationMode);
	else clog << "ERROR: Function 'Object::getPosition(int relativeTo)' not implemented in other enviroment" << endl;

	if (position != NULL)
		for (int i = 0; i < 3; i++)
			position[i] = this->position[i];
}

void Object::getOrientation(int relativeTo, double orientation[3])
{
	if(simulator != NULL) simulator->simGetObjectOrientation(handle, relativeTo, this->orientation, simx_opmode_buffer);
	else clog << "ERROR: Function 'Object::getOrientation(int relativeTo)' not implemented in other enviroment" << endl;

	if (orientation != NULL)
		for (int i = 0; i < 3; i++)
			orientation[i] = this->orientation[i];
}

void Object::getOrientation(int relativeTo, double orientation[3], simxInt operationMode)
{
	if(simulator != NULL) simulator->simGetObjectOrientation(handle, relativeTo, this->orientation, operationMode);
	else clog << "ERROR: Function 'Object::getOrientation(int relativeTo)' not implemented in other enviroment" << endl;

	if (orientation != NULL)
		for (int i = 0; i < 3; i++)
			orientation[i] = this->orientation[i];
}

#endif