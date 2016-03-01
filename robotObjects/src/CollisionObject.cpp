#include "CollisionObject.hpp"

CollisionObject::CollisionObject(RobotVREP * simulator, char name[])
{
	this->simulator = NULL;
	this->simulator = simulator;

	int size = strlen(name)+1;
	this->name = new char[size];
	strncpy(this->name, name, size);
	
	simulator->simGetCollisionHandle(this->name, &collisionHandle, simx_opmode_oneshot_wait);

	simulator->simReadCollision(collisionHandle, &collisionState, simx_opmode_streaming);
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
	simulator->simReadCollision(collisionHandle, &collisionState, simx_opmode_streaming);
	
	return collisionState;
}
