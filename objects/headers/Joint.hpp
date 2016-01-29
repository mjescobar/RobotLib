#ifndef JOINT_HPP
#define JOINT_HPP

#include "Object.hpp"

#include <cstring>
#include <cmath>

#define RAD_CONV (double)(M_PI/180.0)
#define PRECISION 6
#define TOLERANCE (double)(0.1*RAD_CONV)

//AX motor property	 ----------------------------------------------------------------------------
#define AX_CENTER_ANGLE 150.0
#define AX_VEL_MOT_DEF 900
#define AX_MAX_ANGLE (300.0*M_PI/180)
#define AX_RESOLUTION_ANGLE 1023.0
#define AX_MAX_ANGULAR_VELOCITY 11.938051
#define AX_VELOCITY_RESOLUTION 1023.0
#define AX_VELOCITY_TO_NUMBER(X) (X*(AX_VELOCITY_RESOLUTION/AX_MAX_ANGULAR_VELOCITY))
#define AX_NUMBER_TO_VELOCITY(X) (X*(AX_MAX_ANGULAR_VELOCITY/AX_VELOCITY_RESOLUTION))
#define AX_RAD_TO_NUMBER(X) ((X + AX_CENTER_ANGLE*M_PI/180.0)*(AX_RESOLUTION_ANGLE/AX_MAX_ANGLE))
#define AX_NUMBER_TO_RAD(X) (X*(AX_MAX_ANGLE/AX_RESOLUTION_ANGLE) - AX_CENTER_ANGLE*M_PI/180.0)
//-----------------------------------------------------------------------------------------------

//Filter constants ------------------------------------------------------------------------------
#define W_CONST (double)(100.0+1.32*2.0*M_PI)
#define K_CONST 200.0
#define H_CONST (double)(1.0/160.0)
//-----------------------------------------------------------------------------------------------

/**
 * \namespace ANN_USM
 * \brief Dedicated to artificial intelligence development in Santa María University.
 */
namespace ANN_USM
{	
	/**
	 * \class Joint
	 * \brief The joint class is inherited from the Object class and is used to interact with motors in an virtual environment of simulation named VREP or in real robots, more easily and transparently for the user.
	 */
	class Joint : public Object
	{
		// contein the current position of the motor and other two pass positions
		double * positions;
		// contain the last current position
		double current_position;
		// contain the last force readed of the motor
		double force;
		// contain the first position asigned to the motor 
		double initial_position;
		// Maximum value of the motor position
		double max_value;
		// Minimum value of the motor position
		double min_value;
		// Slope sign of the motor position wave in the previous time step
		int pass_slope_sign;
		// Slope sign of the motor position wave in the current time step
		int next_slope_sign;
		// Variable that indicates whether a change of direction of the motor position wave
		bool joint_change_direction;
		// Pointer to a function used for receive data from the user unit to radians.
		double (Joint::*convertToRadFrom)(double);
		// Pointer to a function used for give data from radians to the user unit.
		double (Joint::*convertFromRadTo)(double);
		// Pointer to a function used for give data from joint object to VREP or real motor.
		double (Joint::*robotInput)(double);
		// Pointer to a function used for receive data from VREP or real robot to joint object.
		double (Joint::*robotOutput)(double);
		// Pointer to velocity value of the respective motor in real robot.
		int * motor_velocity;
		// Trunc the number from decimal digits of some value.
		double truncValue(double value, int precision);
		//convert value from radians to radians.
		double RADtoRAD(double value);
		//convert value from degrees to radians.
		double DEGtoRAD(double value);
		//convert scale value between -1 and 1 to radians.
		double SCALEtoRAD(double value);
		//convert value from Ax motor scale to radians.
		double AXtoRAD(double value);
		//convert value from radians to degrees.
		double RADtoDEG(double value);
		//convert value from radians to scale value between -1 and 1.
		double RADtoSCALE(double value);
		//convert value from radians to Ax motor scale value.
		double RADtoAX(double value);

		double Filter(double value);

		double * filtered_positions;

	public:

		/**
		 * \brief Constructor with parameters.
		 * \param simulator Object of type Robotsimulator used to interact with VREP.
		 * \param name Name of the Joint object.
		 * \param max_value Maximum value of the motor position.
		 * \param min_value Minimum value of the motor position.
		 * \param unit Unit used in the user workspace.
		 * \param position Initial position asigned to the motor.
		 */
		Joint(RobotSimulator * simulator, char name[], double max_value, double min_value, const char * unit, double position);

		/**
		 * \brief Constructor with parameters.
		 * \param simulator Object of type Robotsimulator used to interact with VREP.
		 * \param name Name of the Joint object.
		 * \param max_value Maximum value of the motor position.
		 * \param min_value Minimum value of the motor position.
		 * \param unit Unit used in the user workspace.
		 */
		Joint(RobotSimulator * simulator, char name[], double max_value, double min_value, const char * unit);

		/**
		 * \brief Constructor with parameters.
		 * \param cm700 Object of type CM700 used to interact with real motors.
		 * \param name Name of the Joint object.
		 * \param id Identificator corresponding to a dynamixel real motor.
		 * \param max_value Maximum value of the motor position.
		 * \param min_value Minimum value of the motor position.
		 * \param unit Unit used in the user workspace.
		 * \param position Initial position asigned to the motor.
		 */
		Joint(CM700 * cm700, char name[], int id, double max_value, double min_value, const char * unit, double position);

		/**
		 * \brief Destructor.
		 */
		~Joint();

		/**
		 * \brief Asign a position to the motor.
		 * \param Position Position asigned to motor.
		 */
		void setJointPosition(double position);

		/**
		 * \brief Restart motor to initial values.
		 */
		void setJointInitialPosition();

		/**
		 * \brief Retrieve the joint current position from simulator.
		 * \return Joint current position.
		 */
		double getJointCurrentPosition();

		/**
		 * \brief Retrieve the joint current position from simulator.
		 * \param operationMode The remote API function operation mode. Recommended operation modes for this function are simx_opmode_streaming (the first call) and simx_opmode_buffer (the following calls).
		 * \return Joint current position.
		 */
		double getJointCurrentPosition(simxInt operationMode);

		/**
		 * \brief Retrieves the force or torque applied to a joint along/about its active axis.
		 * \return The force or the torque applied to the joint along/about its z-axis.
		 */
		double getJointForce();

		/**
		 * \brief Retrieves the force or torque applied to a joint along/about its active axis.
		 * \param operationMode The remote API function operation mode. Recommended operation modes for this function are simx_opmode_streaming (the first call) and simx_opmode_buffer (the following calls).
		 * \return The force or the torque applied to the joint along/about its z-axis.
		 */
		double getJointForce(simxInt operationMode);

		/**
		 * \brief Retrieve the joint last position assigned with the function setJointPosition().
		 * \return Joint current position.
		 */
		double getJointGoalPosition();

		/**
		 * \brief Retrieve the joint last filter (filtered with Filter() function) position assigned with the function setJointPosition().
		 * \return Joint current position.
		 */
		double getJointFilteredPosition();

		/**
		 * \brief Retrieve the joint position of the last call of function getJointCurrentPosition().
		 * \return Joint current position.
		 */
		double getJointLastCurrentPosition();

		/**
		 * \brief Retrieve the joint force of the last call of function getJointForce().
		 * \return Joint force.
		 */
		double getJointLastForce();

		/**
		 * \brief Retrieve the initial position of the joint.
		 * \return Initial joit position.
		 */
		double getJointInitialPosition();

		/**
		 * \brief Retrieve the maximum angle value of the joint.
		 * \return Maximum angle value.
		 */
		double getMaxAngle();

		/**
		 * \brief Retrieve the minimum angle value of the joint.
		 * \return Minimum angle value.
		 */
		double getMinAngle();

		/**
		 * \brief Retrieve if exist some joint change direction in the present time step.
		 * \return True if exist a joint change direction.
		 */
		bool getJointChangeDirection();

		/**
		 * \brief Updates the values of all motors.
		 */
		void refreshValues();
	};
}

#endif