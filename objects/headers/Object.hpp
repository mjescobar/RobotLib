#ifndef OBJECT_HPP
#define OBJECT_HPP

#include <stdlib.h>
#include <iostream>
#include <cstring>

#include "RobotSimulator.hpp"
#include "cm700.h"

using namespace std;

/**
 * \namespace ANN_USM
 * \brief Dedicated to artificial intelligence development in Santa María University.
 */
namespace ANN_USM
{
	/**
	 * \class Object
	 * \brief The Object class is used to interact with objects in an virtual environment of simulation named VREP or in real robots, more easily and transparently for the user.
	 */
	class Object
	{
	protected:
		// Name of a object, needed for use an virtual or real environment workspace.
		char * name;
		// Object of class RobotSimulator used to interact with VREP simulator.
		RobotSimulator * simulator;
		// Identificator corresponding to an object in VREP simulator.
		int handle;
		// Variable used for contain the position of the object in VREP environment in cordenates x, y and z.
		double * position;
		// Variable used for contain the orientation of the object in VREP environment in alpha, beta and gamma.
		double * orientation;
		//Object of class cm700 used to interact with real robots.
		CM700 * cm700;
		// Identificator corresponding to a dynamixel real motor
		int id;		

	public:

		/**
		 * \brief Constructor with parameters.
		 * \param simulator Object of class simulator used for interact with VREP.
		 * \param name Name of the object in VREP.
		 */
		Object(RobotSimulator * simulator, char name[]);

		/**
		 * \brief Constructor with parameters.
		 * \param cm700 Object of class cm700 used for interact with real robots.
		 * \param name Name of the motor.
		 */
		Object(CM700 * cm700, char name[], int id);

		/**
		 * \brief Void constructor.
		 */
		Object();

		/**
		 * \brief Destructor.
		 */
		~Object();	

		/**
		 * \brief Retrieves the handle corresponding to object in VREP simulator.
		 * \return The object handle value.
		 */
		int getHandle();

		/**
		 * \brief Retrieves the name corresponding to object.
		 * \return The object name.
		 */
		char * getName();

		/**
		 * \brief Retrieves the position of an object in VREP.
		 * \param relativeTo Indicates relative to which reference frame we want the position. Specify -1 to retrieve the absolute position, sim_handle_parent to retrieve the position relative to the object's parent, or an object handle relative to whose reference frame you want the position.
		 * \param position Array of 3 values receiving the position (x,y and z).
		 */
		void getPosition(int relativeTo, double position[3]);

		/**
		 * \brief Retrieves the position of an object in VREP.
		 * \param relativeTo Indicates relative to which reference frame we want the position. Specify -1 to retrieve the absolute position, sim_handle_parent to retrieve the position relative to the object's parent, or an object handle relative to whose reference frame you want the position.
		 * \param position Array of 3 values receiving the position (x,y and z).
		 * \param operationMode The remote API function operation mode. Recommended operation modes for this function are simx_opmode_streaming (the first call) and simx_opmode_buffer (the following calls).
		 */
		void getPosition(int relativeTo, double position[3], simxInt operationMode);

		/**
		 * \brief Retrieves the orientation (Euler angles) of an object in VREP.
		 * \param relativeTo Indicates relative to which reference frame we want the position. Specify -1 to retrieve the absolute position, sim_handle_parent to retrieve the position relative to the object's parent, or an object handle relative to whose reference frame you want the position.indicates relative to which reference frame we want the orientation. Specify -1 to retrieve the absolute orientation, sim_handle_parent to retrieve the orientation relative to the object's parent, or an object handle relative to whose reference frame you want the orientation.
		 * \param orientation Array of 3 values receiving the Euler angles (alpha, beta and gamma).
		 */		
		void getOrientation(int relativeTo, double orientation[3]);

		/**
		 * \brief Retrieves the orientation (Euler angles) of an object in VREP.
		 * \param relativeTo Indicates relative to which reference frame we want the position. Specify -1 to retrieve the absolute position, sim_handle_parent to retrieve the position relative to the object's parent, or an object handle relative to whose reference frame you want the position.indicates relative to which reference frame we want the orientation. Specify -1 to retrieve the absolute orientation, sim_handle_parent to retrieve the orientation relative to the object's parent, or an object handle relative to whose reference frame you want the orientation.
		 * \param orientation Array of 3 values receiving the Euler angles (alpha, beta and gamma).
		 * \param operationMode The remote API function operation mode. Recommended operation modes for this function are simx_opmode_streaming (the first call) and simx_opmode_buffer (the following calls).
		 */		
		void getOrientation(int relativeTo, double orientation[3], simxInt operationMode);
	};
}

#endif