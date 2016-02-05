#ifndef COLLISIONOBJECT_CPP
#define COLLISIONOBJECT_CPP

#include "CollisionObject.hpp"
using namespace ANN_USM;

CollisionObject::CollisionObject(RobotVREP * simulator, char name[])
{
	this->simulator = NULL;
	this->simulator = simulator;

	int size = strlen(name)+1;
	this->name = new char[size];
	strncpy(this->name, name, size);
	
	simulator->getCollisionHandle(this->name, &collisionHandle, simx_opmode_oneshot_wait);

	simulator->readCollision(collisionHandle, &collisionState, simx_opmode_streaming);
}

CollisionObject::CollisionObject()
{

}

CollisionObject::~CollisionObject()
{

}

char * CollisionObject::getName()
{
	return name;
}

int CollisionObject::getCollisionHandle()
{
	return collisionHandle;
}

int CollisionObject::getCollisionState()
{
	simulator->readCollision(collisionHandle, &collisionState, simx_opmode_buffer);
	
	return collisionState;
}

#endif