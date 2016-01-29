#ifndef COLLISIONOBJECT_HPP
#define COLLISIONOBJECT_HPP

#include <stdlib.h>
#include <iostream>
#include <cstring>

#include "RobotSimulator.hpp"

using namespace std;

/**
 * \namespace ANN_USM
 * \brief Dedicated to artificial intelligence development in Santa María University.
 */
namespace ANN_USM
{
	/**
	 * \class CollisionObject
	 * \brief The CollisionObject class is used to interact with collisionable objects in an virtual environment of simulation named VREP or in real robots, more easily and transparently for the user.
	 */
	class CollisionObject
	{
		// Object of class RobotSimulator used to interact with VREP simulator.
		RobotSimulator * simulator;
		// Name of a object, needed for use an virtual or real environment workspace.
		char * name;
		// Collision state of the object. if exist collision its value will be 1, otherwise it will be 0.
		int collisionState;
		// Identificator corresponding to an object in VREP simulator.
		int collisionHandle;

	public:
		/**
		 * \brief Constructor with parameters.
		 * \param simulator Object of class simulator used for interact with VREP.
		 * \param name Name of the motor.
		 */
		CollisionObject(RobotSimulator * simulator, char name[]);
		/**
		 * \brief Void constructor
		 */
		CollisionObject();
		/**
		 * \brief Destructor
		 */
		~CollisionObject();	

		/**
		 * \brief Retrieves the name corresponding to object.
		 * \return The object name.
		 */
		char * getName();

		/**
		 * \brief Reads the collision state of a registered collision object.
		 * \return The collision state (0: not colliding).
		 */
		int getCollisionState();

		/**
		 * \brief Retrieves the handle corresponding to collisionObject in VREP simulator.
		 * \return The object handle value.
		 */
		int getCollisionHandle();
	};
}

#endif