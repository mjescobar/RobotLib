#ifndef ROBOTSIMULATOR_HPP
#define ROBOTSIMULATOR_HPP

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <unistd.h>

using namespace std;

extern "C" {
    #include "extApi.h"
}

#define PORTNB 19998

/**
 * \namespace ANN_USM
 * \brief Dedicated to artificial intelligence development in Santa María University.
 */
namespace ANN_USM
{
	/**
	 * \class RobotSimulator
	 * \brief The RobotSimulator class is used to interact with an virtual environment of simulation named VREP more easily and transparently for the user.
	 */
	class RobotSimulator
	{
		// Id used for VREP to identify the socket user.
		int clientID;
		// Object used for VREP for error notification to user.
		ofstream vrep_error;

	public:
		/**
		 * \brief Void constructor.
		 */
		RobotSimulator();

		/**
		 * \brief Destructor.
		 */
		~RobotSimulator();

		/**
		 * \brief Starts a communication thread with VREP through default ip address.
		 */
		void simStart();

		/**
		 * \brief Starts a communication thread with VREP through specific ip address.
		 * \param ip The ip address where VREP is located.
		 */
		void simStart(const char * ip);
		/**
		 * \brief Starts a communication thread with VREP through specific ip address and port.
		 * \param ip The ip address where VREP is located.
		 */
		 void simStart(const char * ip, int port);
		 /**
		 * \brief Starts a communication thread with VREP through specific port.
		 * \param ip The ip address where VREP is located.
		 */
		 void simStart(int port);

		/**
		 * \brief Ends the communication thread.
		 */
		void simFinish();

		/**
		 * \brief Returns the ID of the current connection. Use this function to track the connection state to VREP. 
		 * \return a connection ID, or -1 if the client is not connected to VREP. Different connection IDs indicate temporary disconections in-between.
		 */
		int simGetConnectionId();

		/**
		 * \brief Allows to temporarily halt the communication thread from sending data. This can be useful if you need to send several values to V-REP that should be received and evaluated at the same time.
		 * \param action Whether the communication thread should pause or run normally. Use 1 for pause and 0 for run normally.
		 */
		void simPauseCommunication(int action);

		/**
		 * \brief Requests a start of a simulation (or a resume of a paused simulation).
		 * \param operationMode A remote API function operation mode. Recommended operation mode for this function is simx_opmode_oneshot.
		 */
		void simStartSimulation(simxInt operationMode);

		/**
		 * \brief Requests a stop of the running simulation.
		 * \param operationMode A remote API function operation mode. Recommended operation mode for this function is simx_opmode_oneshot.
		 */
		void simStopSimulation(simxInt operationMode);

		/**
		 * \brief Retrieves an object handle based on its name.
		 * \param name Name of the object.
		 * \param handle Pointer to a value that will receive the handle.
		 * \param operationMode A remote API function operation mode. Recommended operation mode for this function is simx_opmode_oneshot.
		 */
		void simGetObjectHandle(char name[], int * handle, simxInt operationMode);

		/**
		 * \brief Retrieves the position of an object.
		 * \param object_handle Handle of the object.
		 * \param relativeTo Indicates relative to which reference frame we want the position. Specify -1 to retrieve the absolute position, sim_handle_parent to retrieve the position relative to the object's parent, or an object handle relative to whose reference frame you want the position.
		 * \param position Pointer to 3 values receiving the position.
		 * \param operationMode A remote API function operation mode. Recommended operation mode for this function is simx_opmode_oneshot_wait.
		 */
		void simGetObjectPosition(int object_handle, int relativeTo, double * position, simxInt operationMode);

		/**
		 * \brief Retrieves the position of an object.
		 * \param object_handle Handle of the object.
		 * \param velocity Retrieves the linear velocity of an object.
		 * \param operationMode A remote API function operation mode. Recommended operation modes for this function are simx_opmode_streaming (the first call) and simx_opmode_buffer (the following calls).
		 */
		void simGetObjectVelocity(int object_handle, double * lVelocity, double * aVelocity, simxInt operationMode);

		/**
		 * \brief Retrieves the orientation (Euler angles) of an object.
		 * \param object_handle Handle of the object.
		 * \param relativeTo Indicates relative to which reference frame we want the orientation. Specify -1 to retrieve the absolute orientation, sim_handle_parent to retrieve the orientation relative to the object's parent, or an object handle relative to whose reference frame you want the orientation.
		 * \param orientation Pointer to 3 values receiving the Euler angles (alpha, beta and gamma).
		 * \param operationMode A remote API function operation mode. Recommended operation modes for this function are simx_opmode_streaming (the first call) and simx_opmode_buffer (the following calls).
		 */
		void simGetObjectOrientation(int object_handle, int relativeTo, double * orientation, simxInt operationMode);

		/**
		 * \brief Retrieves the intrinsic position of a joint.
		 * \param object_handle Handle of the object.
		 * \param operationMode A remote API function operation mode. Recommended operation modes for this function are simx_opmode_streaming (the first call) and simx_opmode_buffer (the following calls).
		 */
		double simGetJointPosition(int object_handle, simxInt operationMode);

		/**
		 * \brief Sets the target position of a joint if the joint is in torque/force mode (also make sure that the joint's motor and position control are enabled).
		 * \param object_handle Handle of the object.
		 * \param joint_pos Target position of the joint (angular or linear value depending on the joint type).
		 * \param operationMode A remote API function operation mode. Recommended operation modes for this function are simx_opmode_streaming (the first call) and simx_opmode_buffer (the following calls).
		 */
		void simSetJointTargetPosition(int object_handle, double joint_pos, simxInt operationMode);

		/**
		 * \brief Retrieves the force or torque applied to a joint along/about its active axis.
		 * \param object_handle Handle of the object.
		 * \param operationMode A remote API function operation mode. Recommended operation modes for this function are simx_opmode_streaming (the first call) and simx_opmode_buffer (the following calls).
		 * \return The force or the torque applied to the joint along/about its z-axis.
		 */
		double simGetJointForce(int object_handle, simxInt operationMode);

		/**
		 * \brief Adds a message to the status bar.
		 * \param message The message to display.
		 * \param operationMode A remote API function operation mode. Recommended operation modes for this function are simx_opmode_oneshot or simx_opmode_streaming.
		 */
		void simAddStatusbarMessage(char * message, simxInt operationMode);

		/**
		 * \brief Reads the collision state of a registered collision object.
		 * \param collisionHandle Handle of the collision object.
		 * \param collisionState A pointer to a value receiving the collision state (0: not colliding).
		 * \param operationMode A remote API function operation mode. Recommended operation modes for this function are simx_opmode_streaming (the first call) and simx_opmode_buffer (the following calls).
		 */
		void simReadCollision(int collisionHandle, int * collisionState, simxInt operationMode);

		/**
		 * \brief Retrieves a collision object handle based on its name.
		 * \param name Name of the collision object.
		 * \param collisionHandle Pointer to a value that will receive the handle.
		 * \param operationMode A remote API function operation mode. Recommended operation mode for this function is simx_opmode_oneshot_wait.
		 */
		void simGetCollisionHandle(char name[], int * collisionHandle, simxInt operationMode);

	};
}

#endif