#ifndef COLLISIONOBJECT_HPP
#define COLLISIONOBJECT_HPP

#include <stdlib.h>
#include <iostream>
#include <cstring>
#include "Object.hpp"

/**
 * \class CollisionObject
 * \brief The CollisionObject class is used to interact with collisionable objects in an virtual environment of simulation named VREP or in real robots, more easily and transparently for the user.
 */
class CollisionObject : public Object
{
	// Collision state of the object. if exist collision its value will be true, otherwise it will be false.
	bool collisionState;

public:
	/**
	 * \brief Constructor with parameters.
	 * \param name Name of the motor.
	 */
	CollisionObject(char name[]);

	/**
	 * \brief Destructor
	 */
	~CollisionObject();

	/**
	 * \brief Sets the collision state of a registered collision object.
	 * \param collisionState The collision state
	 */
	void setCollisionState(bool collisionState);

	/**
	 * \brief Reads the last collision state of a registered collision object.
	 * \return The collision state (0: not colliding).
	 */
	bool getLastCollisionState();
};


#endif