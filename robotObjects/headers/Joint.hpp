#ifndef JOINT_HPP
#define JOINT_HPP

#include <cstring>
#include <cmath>
#include <stdlib.h>
#include "Object.hpp"

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

using namespace std;
	
/**
 * \class Joint
 * \brief The joint class is inherited from the Object class and is used to interact with motors in an virtual environment of simulation named VREP or in real robots, more easily and transparently for the user.
 */
class Joint : public Object
{
	// contein the current position of the motor and other two pass positions
	double * positions;
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

public:
	/**
	 * \brief Constructor with parameters.
	 * \param name Name of the Joint object.
	 * \param max_value Maximum value of the motor position.
	 * \param min_value Minimum value of the motor position.
	 * \param unit Unit used in the user workspace.
	 * \param position Initial position asigned to the motor.
	 */
	Joint(const char * unit = "RAD", const char name[] = "unknown", double max_value = M_PI, double min_value = -M_PI, double initial_position = 0.0);

	/**
	 * \brief Destructor.
	 */
	~Joint();
	/**

	 */
	void setInitialValues();

	/**
	 * \brief Asign a position to the motor.
	 * \param Position Position asigned to motor.
	 */
	void setJointNextPosition(double position);

	/**
	 * \brief Retrieve if exist some joint change direction in the present time step.
	 * \return True if exist a joint change direction.
	 */
	bool getJointChangeDirection();
	
	/**
	 * \brief Retrieve the joint last position assigned with the function setJointPosition().
	 * \return Joint current position.
	 */
	double getJointGoalPosition();

	/**
	 * \brief Retrieve the initial position of the joint in radian.
	 * \return Initial joit position.
	 */
	double getJointInitialPositionRad();

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
	* \brief A controlles is added to manipulate the Joint, for example in a simulator like VREP or in a real dynamixel motor, or even in both.
	**/
	double getNextPositionRad();

};

// Trunc the number from decimal digits of some value.
double truncValue(double value, int precision);

#endif