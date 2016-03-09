#include "CollisionObject.hpp"

CollisionObject::CollisionObject(char name[]) : Object(name)
{
	collisionState = false;
}

CollisionObject::~CollisionObject()
{
	
}

void CollisionObject::setCollisionState(bool collisionState)
{
	this->collisionState = collisionState;
}

bool CollisionObject::getLastCollisionState()
{
	return collisionState;
}
