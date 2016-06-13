#ifndef ROBOTVREP_HPP
#define ROBOTVREP_HPP

class Joint;
class VisionSensor;

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <string.h>
#include <sstream>

#include <map>
#include <vector>
#include <algorithm>    // std::find
#include "Object.hpp"
#include "Joint.hpp"
#include "VisionSensor.hpp"
#include "CollisionObject.hpp"


using namespace std;

extern "C" {
    #include "extApi.h"
}

#define PORTNB 19997


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
	// Parameter used for video recording mode;
	bool video_recording_flag;
	// If changeVideoName funtion is called before StarSimulation funcion, this parameter is true
	bool video_updated_name;
	// Video Id if the user don't use changeVideoName function
	int video_id;
	// Indicates that a simulation in progress
	bool simulation_in_progress;
	// Old video recording path
	string old_path;
	// New video recording path
	string new_path;

	// Use this function to track the connection state to VREP. If the client is not connected to VREP, the program ends. If exist temporary disconections in-between, a warning message is emitted
	void trackConnection();	

	// The vector of all joints that are controlled by this Vrep Simulator.
	std::vector < Joint * > jointVector;
	// The vector of all objects that are controlled by this Vrep Simulator.
	std::vector < Object * > objectVector;
	// The vector of all vision sensors that are controlled by this Vrep Simulator.
	std::vector < VisionSensor * > visionSensorVector;
	// The vector of all collisionable objects that are controlled by this Vrep Simulator.
	std::vector < CollisionObject * > collisionObjectVector;
	// the handler of all object for VrepApi
	std::vector < int * > VrepHandlerVector;
	// the mapping from JointUniqueId to Vrep Handler Vector corresponding position.
	std::map < int , int  > jointIdToVrepHandler_map;
	std::map < int , int  > objectIdToVrepHandler_map;
	std::map < int , int  > visionSensorIdToVrepHandler_map;
	std::map < int , int  > collisionObjectIdToVrepHandler_map;
public:

	int getObjectHandler(Object * object);
	
	/**
	 * \brief Sets the configuration for video recording mode
	 * \param active Set true if video recording is needed. False as default.
	 */
	// 
	void setVideoRecordingMode(bool active = true);
	/**
	 * \brief Constructor with parameters. Starts a communication thread with VREP through specific ip address and port.
	 * \param video_recording set true if video recording is needed. False as default.
	 * \param port The port number where to connect. PORTNB as default.
	 * \param ip The ip address where VREP is located. "127.0.0.1" as default.
	 */
	RobotVREP(bool video_recording = false, int port = PORTNB, const char * ip = "127.0.0.1");

	/**
	 * \brief Destructor. Ends the communication thread.
	 */
	~RobotVREP();
	
	/**
	 * \brief Returns the ID of the current connection. Use this function to track the connection state to VREP. 
	 * \return a connection ID, or -1 if the client is not connected to VREP. Different connection IDs indicate temporary disconections in-between.
	 */
	int getConnectionId();

	/**
	 * \brief Allows to temporarily halt the communication thread from sending data. This can be useful if you need to send several values to V-REP that should be received and evaluated at the same time.
	 * \param action Whether the communication thread should pause or run normally. Use 1 for pause and 0 for run normally.
	 */
	void pauseCommunication(int action);

	/**
	 * \brief Retrieves the simulation time of the last fetched command (i.e. when the last fetched command was processed on the server side). The function can be used to verify how "fresh" a command reply is, or whether a command reply was recently updated.
	 * \return The simulation time in milliseconds when the command reply was generated, or 0 if simulation was not running.
	*/
	int getLastCmdTime();

	/**
	 * \brief Requests a start of a simulation (or a resume of a paused simulation).
	 * \param operationMode A remote API function operation mode. Default operation mode for this function is simx_opmode_oneshot if the argument is not passed.
	 */
	void startSimulation(simxInt operationMode = /*simx_opmode_oneshot_wait*/ 0x010000);

	/**
	 * \brief Requests a stop of the running simulation.
	 * \param operationMode A remote API function operation mode. Default operation mode for this function is simx_opmode_oneshot if the argument is not passed.
	 */
	void stopSimulation(simxInt operationMode = /*simx_opmode_oneshot_wait*/ 0x010000);

	/**
	 * \brief Requests a pause of a simulation.
	 * \param operationMode A remote API function operation mode. Default operation mode for this function is simx_opmode_oneshot if the argument is not passed.
	*/
	void pauseSimulation(simxInt operationMode = /*simx_opmode_oneshot*/ 0x000000);

	/**
	 * \brief Retrieves an object handle based on its name.
	 * \param name Name of the object.
	 * \param operationMode A remote API function operation mode. Default operation mode for this function is simx_opmode_oneshot if the argument is not passed.
	 * \return Object handle.
	 */
	int getObjectHandle(char name[], simxInt operationMode = /*simx_opmode_oneshot*/ 0x000000);

	/**
	 * \brief Retrieves a collision object handle based on its name.
	 * \param name Name of the collision object.
	 * \param collisionHandle Pointer to a value that will receive the handle.
	 * \param operationMode A remote API function operation mode. Default operation mode for this function is simx_opmode_oneshot_wait if the argument is not passed.
	*/
	int getCollisionHandle(char name[], simxInt operationMode = /*simx_opmode_oneshot_wait*/ 0x010000);

	/**
	 * \brief Sets the position of an object.
	 * \param object Object of Object class.
	 * \param relativeTo Indicates relative to which reference frame the position is specified. Specify -1 to set the absolute position, sim_handle_parent to set the position relative to the object's parent, or an object handle relative to whose reference frame the position is specified.
	 * \param position the position values (x, y and z).
	 * \param operationMode A remote API function operation mode. Default operation mode for this function is simx_opmode_oneshot if the argument is not passed.
	*/
	void setObjectPosition(Object * object, vector < double > position, int relativeTo = -1, simxInt operationMode = /*simx_opmode_oneshot*/ 0x000000);

	 /**
	 * \brief Sets the orientation of an object.
	 * \param object Object of Object class.
	 * \param relativeTo Indicates relative to which reference frame the orientation is specified. Specify -1 to set the absolute orientation, sim_handle_parent to set the orientation relative to the object's parent, or an object handle relative to whose reference frame the orientation is specified.
	 * \param orientation Euler angles (alpha, beta and gamma).
	 * \param operationMode A remote API function operation mode. Default operation mode for this function is simx_opmode_oneshot if the argument is not passed.
	*/
	void setObjectOrientation(Object * object, vector < double > orientation, int relativeTo = -1, simxInt operationMode = /*simx_opmode_oneshot*/ 0x000000);

	/**
	 * \brief Retrieves the position of an object.
	 * \param object Object of Object class.
	 * \param relativeTo Indicates relative to which reference frame we want the position. Specify -1 (sets as default) to retrieve the absolute position, sim_handle_parent to retrieve the position relative to the object's parent, or an object handle relative to whose reference frame you want the position.
	 * \param operationMode A remote API function operation mode. Recommended operation modes for this function are simx_opmode_streaming (the first call) and simx_opmode_buffer (the following calls, assigned as default).
	 * \return Vector that contains the object position.
	 */
	vector < double > getObjectPosition(Object * object, int relativeTo = -1, simxInt operationMode = /*simx_opmode_buffer*/ 0x060000);

	/**
	 * \brief Retrieves the velocity of an object.
	 * \param object Object of Object class.
	 * \param velocity Retrieves the linear and angular velocity of an object.
	 * \param operationMode A remote API function operation mode. Recommended operation modes for this function are simx_opmode_streaming (the first call) and simx_opmode_buffer (the following calls, assigned as default).
	 * \return Vector that contains the object linear and angular position.
	 */
	vector < double > getObjectVelocity(Object * object, simxInt operationMode = /*simx_opmode_buffer*/ 0x060000);
	
	/**
	 * \brief Retrieves the orientation (Euler angles) of an object.
	 * \param object Object of Object class.
	 * \param relativeTo Indicates relative to which reference frame we want the orientation. Specify -1 to retrieve the absolute orientation, sim_handle_parent to retrieve the orientation relative to the object's parent, or an object handle relative to whose reference frame you want the orientation.
	 * \param operationMode A remote API function operation mode. Recommended operation modes for this function are simx_opmode_streaming (the first call) and simx_opmode_buffer (the following calls, assigned as default).
	 * \return Vector that contains the object orientation in Euler angles (alpha, beta and gamma).
	 */
	vector < double > getObjectOrientation(Object * object, int relativeTo, simxInt operationMode = /*simx_opmode_buffer*/ 0x060000);

	/**
	 * \brief Retrieves the intrinsic position of a joint.
	 * \param joint Object of Joint class.
	 * \param operationMode A remote API function operation mode. Recommended operation modes for this function are simx_opmode_streaming (the first call) and simx_opmode_buffer (the following calls, assigned as default).
	 * \return The intrinsic position of the joint. This is a one-dimensional value: if the joint is revolute, the rotation angle is returned, if the joint is prismatic, the translation amount is returned, etc.
	 */
	double getJointPosition(Joint * joint, simxInt operationMode = /*simx_opmode_buffer*/ 0x060000);

	/**
	 * \brief Sets the target position of a joint if the joint is in torque/force mode (also make sure that the joint's motor and position control are enabled).
	 * \param joint Object of Joint class.
	 * \param joint_pos Target position of the joint (angular or linear value depending on the joint type).
	 * \param operationMode A remote API function operation mode. Recommended operation modes for this function are simx_opmode_oneshot (default) or simx_opmode_streaming.
	 */
	void setJointTargetPosition(Joint * joint, double joint_pos, simxInt operationMode = /*simx_opmode_oneshot*/ 0x000000);

	/**
	 * \brief Sets the intrinsic target velocity of a non-spherical joint.
	 * \param joint Object of Joint class.
	 * \param joint_vel Target velocity of the joint (linear or angular velocity depending on the joint-type).
	 * \param operationMode A remote API function operation mode. Recommended operation modes for this function are simx_opmode_oneshot (default) or simx_opmode_streaming.
	*/
	void setJointTargetVelocity(Joint * joint, double joint_vel, simxInt operationMode = /*simx_opmode_oneshot*/ 0x000000);

	/**
	 * \brief Retrieves the force or torque applied to a joint along/about its active axis. This function retrieves meaningful information only if the joint is prismatic or revolute, and is dynamically enabled. With the Bullet engine, this function returns the force or torque applied to the joint motor (torques from joint limits are not taken into account). With the ODE or Vortex engine, this function returns the total force or torque applied to a joint along/about its z-axis.
	 * \param joint Object of Joint class.
	 * \param operationMode A remote API function operation mode. Recommended operation modes for this function are simx_opmode_streaming (the first call) and simx_opmode_buffer (the following calls, assigned as default).
	 * \return The force or the torque applied to the joint along/about its z-axis.
	 */
	double getJointForce(Joint * joint, simxInt operationMode = /*simx_opmode_buffer*/ 0x060000);

	/**
	 * \brief Adds a message to the status bar.
	 * \param message The message to display.
	 * \param operationMode A remote API function operation mode. Recommended operation modes for this function are simx_opmode_oneshot or simx_opmode_streaming.
	 */
	void addStatusbarMessage(char * message, simxInt operationMode = /*simx_opmode_oneshot*/ 0x000000);

	/**
	 * \brief Reads the collision state of a registered collision object.
	 * \param collisionHandle Handle of the collision object.
	 * \param collisionState A pointer to a value receiving the collision state (0: not colliding).
	 * \param operationMode A remote API function operation mode. Recommended operation modes for this function are simx_opmode_streaming (the first call) and simx_opmode_buffer (the following calls, assigned as default).
	 */
	bool readCollision(CollisionObject * collisionObject, simxInt operationMode = /*simx_opmode_buffer*/ 0x060000);	

	/**
	 * \brief Retrieves the resolution of the image of a vision sensor.
	 * \param visionSensor Object of VisionSensor class.
	 * \param operationMode  A remote API function operation mode. Recommended operation modes for this function are simx_opmode_streaming (the first call) and simx_opmode_buffer (the following calls, assigned as default).
	 * \return Vector of two int values with the image resolution.
	 */
	vector < int > getVisionSensorResolution(VisionSensor * visionSensor, simxInt operationMode = /*simx_opmode_buffer*/ 0x060000);

	/**
	 * \brief Retrieves the image of a vision sensor.
	 * \param visionSensor Object of VisionSensor class.
	 * \param operationMode  A remote API function operation mode. Recommended operation modes for this function are simx_opmode_streaming (the first call) and simx_opmode_buffer (the following calls, assigned as default).
	 * \return A unsigned char pointer with the image data.
	 */
	unsigned char * getVisionSensorImage(VisionSensor * visionSensor, simxInt operationMode = /*simx_opmode_buffer*/ 0x060000);

	/**
	 * \brief Change the path where the screen recording is saved VREP. This function can only be used if the simulator is stopped.
	 * \param path Screen recording path.
	 * \param operationMode A remote API function operation mode. Default operation mode for this function is simx_opmode_oneshot if the argument is not passed.
	 */
	void changeVideoPath(char path[], simxInt operationMode = /*simx_opmode_oneshot_wait*/ 0x010000);


	void changeVideoName(char name[], simxInt operationMode = /*simx_opmode_oneshot_wait*/ 0x010000);

	/** 

	*/
	void addJoint ( Joint * joint );

	/**

	*/
	void addObject( Object * object );

	/**

	*/
	void addVisionSensor( VisionSensor * visionSensor );

	/**

	*/
	void addCollisionObject( CollisionObject * collisionObject );

	/**

	*/
	void moveJoints();

	/**

	*/
	void moveJointsToInitialPosition();

	/**

	*/
	bool checkAllCollisions();



};


#endif
