#ifndef ROBOTVREP_HPP
#define ROBOTVREP_HPP

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <unistd.h>

using namespace std;

extern "C" {
    #include "extApi.h"
}

#define PORTNB 19997

/**
 * \namespace ANN_USM
 * \brief Dedicated to artificial intelligence development in Santa María University.
 */
namespace ANN_USM
{
	/**
	 * \class RobotVREP
	 * \brief The RobotVREP class is used to interact with an virtual environment of simulation named VREP more easily and transparently for the user.
	 */
	class RobotVREP
	{
		// Id used for VREP to identify the socket user.
		int clientID;
		// Object used for VREP for error notification to user.
		ofstream vrep_error;
		// Use this function to track the connection state to VREP. If the client is not connected to VREP, the program ends. If exist temporary disconections in-between, a warning message is emitted
		void trackConnection();

	public:
		/**
		 * \brief Void constructor. Starts a communication thread with VREP through default ip address.
		 */
		RobotVREP();

		/**
		 * \brief Constructor with parameters. Starts a communication thread with VREP through specific ip address.
		 * \param ip The ip address where VREP is located.
		 */
		RobotVREP(const char * ip);

		/**
		 * \brief Constructor with parameters. Starts a communication thread with VREP through specific port.
		 * \param port The port number where to connect.
		 */
		RobotVREP(int port);

		/**
		 * \brief Constructor with parameters. Starts a communication thread with VREP through specific ip address and port.
		 * \param ip The ip address where VREP is located.
		 * \param port The port number where to connect.
		 */
		RobotVREP(const char * ip, int port);

		/**
		 * \brief Destructor. Ends the communication thread.
		 */
		~RobotVREP();
		
		/**
		 * \brief Returns the ID of the current connection. Use this function to track the connection state to VREP. 
		 * \return a connection ID, or -1 if the client is not connected to VREP. Different connection IDs indicate temporary disconections in-between.
		 */
		int GetConnectionId();

		/**
		 * \brief Allows to temporarily halt the communication thread from sending data. This can be useful if you need to send several values to V-REP that should be received and evaluated at the same time.
		 * \param action Whether the communication thread should pause or run normally. Use 1 for pause and 0 for run normally.
		 */
		void PauseCommunication(int action);

		/**
		 * \brief Requests a start of a simulation (or a resume of a paused simulation).
		 * \param operationMode A remote API function operation mode. Default operation mode for this function is simx_opmode_oneshot if the argument is not passed.
		 */
		void StartSimulation(simxInt operationMode = simx_opmode_oneshot);

		/**
		 * \brief Requests a stop of the running simulation.
		 * \param operationMode A remote API function operation mode. Default operation mode for this function is simx_opmode_oneshot if the argument is not passed.
		 */
		void StopSimulation(simxInt operationMode = simx_opmode_oneshot);

		/**
		 * \brief Retrieves an object handle based on its name.
		 * \param name Name of the object.
		 * \param handle Pointer to a value that will receive the handle.
		 * \param operationMode A remote API function operation mode. Default operation mode for this function is simx_opmode_oneshot if the argument is not passed.
		 */
		void GetObjectHandle(char name[], int * handle, simxInt operationMode = simx_opmode_oneshot);

		/**
		 * \brief Retrieves the position of an object.
		 * \param object_handle Handle of the object.
		 * \param relativeTo Indicates relative to which reference frame we want the position. Specify -1 to retrieve the absolute position, sim_handle_parent to retrieve the position relative to the object's parent, or an object handle relative to whose reference frame you want the position.
		 * \param position Pointer to 3 values receiving the position.
		 * \param operationMode A remote API function operation mode. Default operation mode for this function is simx_opmode_oneshot_wait if the argument is not passed.
		 */
		void GetObjectPosition(int object_handle, int relativeTo, double * position, simxInt operationMode = simx_opmode_oneshot_wait);

		/**
		 * \brief Retrieves the position of an object.
		 * \param object_handle Handle of the object.
		 * \param velocity Retrieves the linear velocity of an object.
		 * \param operationMode A remote API function operation mode. Recommended operation modes for this function are simx_opmode_streaming (the first call) and simx_opmode_buffer (the following calls, assigned as default).
		 */
		void GetObjectVelocity(int object_handle, double * lVelocity, double * aVelocity, simxInt operationMode = simx_opmode_buffer);

		/**
		 * \brief Retrieves the orientation (Euler angles) of an object.
		 * \param object_handle Handle of the object.
		 * \param relativeTo Indicates relative to which reference frame we want the orientation. Specify -1 to retrieve the absolute orientation, sim_handle_parent to retrieve the orientation relative to the object's parent, or an object handle relative to whose reference frame you want the orientation.
		 * \param orientation Pointer to 3 values receiving the Euler angles (alpha, beta and gamma).
		 * \param operationMode A remote API function operation mode. Recommended operation modes for this function are simx_opmode_streaming (the first call) and simx_opmode_buffer (the following calls, assigned as default).
		 */
		void GetObjectOrientation(int object_handle, int relativeTo, double * orientation, simxInt operationMode = simx_opmode_buffer);

		/**
		 * \brief Retrieves the intrinsic position of a joint.
		 * \param object_handle Handle of the object.
		 * \param operationMode A remote API function operation mode. Recommended operation modes for this function are simx_opmode_streaming (the first call) and simx_opmode_buffer (the following calls, assigned as default).
		 * \return The intrinsic position of the joint. This is a one-dimensional value: if the joint is revolute, the rotation angle is returned, if the joint is prismatic, the translation amount is returned, etc.
		 */
		double GetJointPosition(int object_handle, simxInt operationMode = simx_opmode_buffer);

		/**
		 * \brief Sets the target position of a joint if the joint is in torque/force mode (also make sure that the joint's motor and position control are enabled).
		 * \param object_handle Handle of the object.
		 * \param joint_pos Target position of the joint (angular or linear value depending on the joint type).
		 * \param operationMode A remote API function operation mode. Recommended operation modes for this function are simx_opmode_oneshot (default) or simx_opmode_streaming.
		 */
		void SetJointTargetPosition(int object_handle, double joint_pos, simxInt operationMode = simx_opmode_oneshot);

		/**
		 * \brief Retrieves the force or torque applied to a joint along/about its active axis. This function retrieves meaningful information only if the joint is prismatic or revolute, and is dynamically enabled. With the Bullet engine, this function returns the force or torque applied to the joint motor (torques from joint limits are not taken into account). With the ODE or Vortex engine, this function returns the total force or torque applied to a joint along/about its z-axis.
		 * \param object_handle Handle of the object.
		 * \param operationMode A remote API function operation mode. Recommended operation modes for this function are simx_opmode_streaming (the first call) and simx_opmode_buffer (the following calls, assigned as default).
		 * \return The force or the torque applied to the joint along/about its z-axis.
		 */
		double GetJointForce(int object_handle, simxInt operationMode = simx_opmode_buffer);

		/**
		 * \brief Adds a message to the status bar.
		 * \param message The message to display.
		 * \param operationMode A remote API function operation mode. Recommended operation modes for this function are simx_opmode_oneshot or simx_opmode_streaming.
		 */
		void AddStatusbarMessage(char * message, simxInt operationMode = simx_opmode_oneshot);

		/**
		 * \brief Reads the collision state of a registered collision object.
		 * \param collisionHandle Handle of the collision object.
		 * \param collisionState A pointer to a value receiving the collision state (0: not colliding).
		 * \param operationMode A remote API function operation mode. Recommended operation modes for this function are simx_opmode_streaming (the first call) and simx_opmode_buffer (the following calls, assigned as default).
		 */
		void ReadCollision(int collisionHandle, int * collisionState, simxInt operationMode = simx_opmode_buffer);

		/**
		 * \brief Retrieves a collision object handle based on its name.
		 * \param name Name of the collision object.
		 * \param collisionHandle Pointer to a value that will receive the handle.
		 * \param operationMode A remote API function operation mode. Default operation mode for this function is simx_opmode_oneshot_wait if the argument is not passed.
		 */
		void GetCollisionHandle(char name[], int * collisionHandle, simxInt operationMode = simx_opmode_oneshot_wait);

	};
}

#endif