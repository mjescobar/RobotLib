#ifndef CM700_CPP
#define CM700_CPP

#include "cm700.h"

//#define _DEBUG

/*
	remember: BaudRate = 2.000.000/(baudNum + 1) 
*/
CM700::CM700(string serialPort, int baudNum = 1) 
{
	// Se comienza la comunicacion serial con los motores.
	if (dxl_initialize(serialPort.c_str(), baudNum) == 0) 
	{
		cout << "Failed to open USB2Dynamixel!" << endl;
		cout << "Press Enter key to terminate..." << endl;
		cin.get();
		return;
	} 
	else 
	{
		cout << "USB2Dynamixel inicializado!" << endl;
	}

}

CM700::~CM700( ) 
{
	motors.clear();
}

//We try to obtain the most information posible to create the motor throught the motor model number, if is not in our registers then is necesary to use the other constructor.
void CM700::addMotor(int id)
{
	//Verifing that the id is unique
	if( idToMotorsVectorPosition.find( id ) == idToMotorsVectorPosition.end() )
	{
		//The Id already exist
		std::cerr << "ERROR::CM700::addMotor::The id already exist" << std::endl;
		exit (EXIT_FAILURE);
	}

	Motor motor;
	motor.id=id;
	// Obtaining information about the motor throught consulting the dynamixel motor connected with the corresponding id.

	//Obtaining the model number.
	int model = 1;

	//Verify if the model is in our register, in case that is not in our registers then a error es sent.
	if ( !verifyModel( model ) )
	{
		std::cerr << "ERROR::void CM700::addMotor(int id)::Your dynamixel model is not in our register, in that case you have to use an alternative addMotor method" << std::endl;
		exit(EXIT_FAILURE);
	}

	//Settings motors values from model.
	setMotorParametersFromModel( model, motor);

	// Adding the motor
	motors.push_back( motor );
	idToMotorsVectorPosition.insert ( std::pair<int,int> ( id, motors.size() -1 ) );
}



//We try to obtain the most information posible to create the motor throught the motor model number, if is not in our registers then is necesary to use the other constructor.
void CM700::addMotor(int id, int angleResolution, bool hasCurrentSensor, int velocityResolution, double angleRangeDeg)
{
	//Verifing that the id is unique
	if( idToMotorsVectorPosition.find( id ) == idToMotorsVectorPosition.end() )
	{
		//The Id already exist
		std::cerr << "ERROR::CM700::addMotor::The id already exist" << std::endl;
		exit (EXIT_FAILURE);
	}

	Motor motor;
	motor.id = id;
	// Obtaining information about the motor throught consulting the dynamixel motor connected with the corresponding id.

	//Obtaining the model number.
	int model = 1;

	//Verify if the model is in our register, in case that is not in our registers then a error es sent.
	if ( !verifyModel( model ) )
	{
		std::cerr << "ERROR::void CM700::addMotor(int id)::Your dynamixel model is not in our register, in that case you have to use an alternative addMotor method" << std::endl;
		exit(EXIT_FAILURE);
	}

	//Settings motors values from model.
	motor.angleResolution = angleResolution;
	motor.hasCurrentSensor = hasCurrentSensor;
	motor.velocityResolution = velocityResolution;
	motor.angleRangeDeg = angleRangeDeg;

	// Adding the motor
	motors.push_back( motor );
	idToMotorsVectorPosition.insert ( std::pair<int,int> ( id, motors.size() -1 ) );
}



void CM700::setMotorPosition(int id, double angle_RAD, double normalizedVelocity)
{
	if( idToMotorsVectorPosition.find( id ) == idToMotorsVectorPosition.end() )
	{
		//The Id already exist
		std::cerr << "ERROR::CM700::setMotorPosition::The id is not in the current memory, you have to add the Id before to call this method." << std::endl;
		exit (EXIT_FAILURE);
	}

	int motorVectorPosition = idToMotorsVectorPosition.at( id );

	// if angle_RAD is no in 0-2Pi then is important to convert that.
	while(angle_RAD >= M_PI || angle_RAD <= -M_PI)
	{
		if( angle_RAD < -M_PI )
		{
			angle_RAD += 2*M_PI;
		}
		else
		{
			angle_RAD -= 2*M_PI;
		}
	}

	//Cheking inpuit are coherent.
	if(normalizedVelocity > 1 || normalizedVelocity < 0 )
	{
		std::cerr << "ERROR::CM700::setMotorPosition::normalizedVelocity must be in [0,1]" <<  std::endl;
		exit(EXIT_FAILURE);
	}
	if ( fabs(angle_RAD) >= motors.at( motorVectorPosition ).angleRangeDeg*(M_PI/180.0) ) 
	{
		std::cerr << "ERROR::CM700::setMotorPosition::angle_RAD is not in the phisical angle range of the dynamixel motor" <<  std::endl;
		exit(EXIT_FAILURE);
	}


	// EACH MODEL COULD HAVE DIFERENT AMOUNT OF ANGLE RESOLUTION, THAT MEANS THAT A ANDLE IN RAD WILL BE A DIFERENT GOALPOSITION VALUE.

	// the goal position have to be transformed taking in account that the value 0 is not in the mitdle but in one extreme (the midle is in the angleResolution/2) and that the dynamizel could have a range of degress and not always 360 degrees
	int goalPosition = (int)( motors.at( motorVectorPosition ).angleResolution 
		+ ( motors.at( motorVectorPosition ).angleResolution/( motors.at( motorVectorPosition ).angleRangeDeg*(M_PI/180.0) ) ) 
		* (angle_RAD + motors.at( motorVectorPosition ).angleRangeDeg*(M_PI/180.0)/2.0 ) );
	// the speed is scaled because for convention is normalized then 0 is the minimum and 1 is the max value.
	int speed = (int)( normalizedVelocity * motors.at(motorVectorPosition).velocityResolution);
	motors.at( motorVectorPosition ).tposition = goalPosition;
	motors.at( motorVectorPosition ).tspeed = speed;
}

int CM700::getMotorPosition(int id)
{
	if( idToMotorsVectorPosition.find( id ) == idToMotorsVectorPosition.end() )
	{
		//The Id already exist
		std::cerr << "ERROR::CM700::getMotorPosition::The id is not in the current memory, you have to add the Id before to call this method." << std::endl;
		exit (EXIT_FAILURE);
	}
	auto motorVectorPosition = idToMotorsVectorPosition.at( id );
	return motors.at(motorVectorPosition).cposition;
} 

void CM700::refreshAll()
{
	for (unsigned int i = 0; i < motors.size(); i++) 
	{
		dxl_set_txpacket_id 			(motors.at( i ).id);
		dxl_set_txpacket_length 		(4);
		dxl_set_txpacket_instruction 	(INST_READ); 
		dxl_set_txpacket_parameter 		(0, P_PRESENT_POSITION_L); 
		dxl_set_txpacket_parameter 		(1, 11); 
		dxl_txrx_packet 				();

		if ( dxl_get_result() != COMM_RXSUCCESS ) 
		{
			continue;
		}
	 
		motors.at(i).cposition 	= _MW(dxl_get_rxpacket_parameter(0), dxl_get_rxpacket_parameter(1));
		motors.at(i).cspeed 		= _MW(dxl_get_rxpacket_parameter(2), dxl_get_rxpacket_parameter(3)) & 0x3FF;
		motors.at(i).load 			= _MW(dxl_get_rxpacket_parameter(4), dxl_get_rxpacket_parameter(5));
		motors.at(i).volt 			= dxl_get_rxpacket_parameter(6);
		motors.at(i).temperature 	= dxl_get_rxpacket_parameter(7);
		motors.at(i).current = 0;

		// If the motor have current sensor.
		if ( motors.at(i).hasCurrentSensor )
		{
			dxl_set_txpacket_id 			( motors.at( i ).id );
			dxl_set_txpacket_length 		(4);
			dxl_set_txpacket_instruction 	( INST_READ ); 
			dxl_set_txpacket_parameter 		(0, P_CURRENT_LOW); 
			dxl_set_txpacket_parameter 		(1, 2); 
			dxl_txrx_packet 				();

			if ( dxl_get_result() != COMM_RXSUCCESS ) 
			{
				continue;
			}
			motors.at(i).current = _MW(dxl_get_rxpacket_parameter(0), dxl_get_rxpacket_parameter(1));
		}
		
	}
}

void CM700::moveAll()
{
	int param_per_actuator = 4;
	

	dxl_set_txpacket_id( BROADCAST_ID);
	dxl_set_txpacket_length( (param_per_actuator + 1) * ((int)motors.size()) + 4);
	dxl_set_txpacket_instruction( INST_SYNC_WRITE );
	dxl_set_txpacket_parameter( 0, P_GOAL_POSITION_L );
	dxl_set_txpacket_parameter( 1, param_per_actuator );

	for (unsigned int i = 0; i < motors.size(); i++) {
		dxl_set_txpacket_parameter(2 + 5 * i, motors.at(i).id);
		dxl_set_txpacket_parameter(2 + 5 * i + 1, _L16(motors.at(i).tposition));
		dxl_set_txpacket_parameter(2 + 5 * i + 2, _H16(motors.at(i).tposition));
		dxl_set_txpacket_parameter(2 + 5 * i + 3, _L16(motors.at(i).tspeed));
		dxl_set_txpacket_parameter(2 + 5 * i + 4, _H16(motors.at(i).tspeed));
	}

	dxl_tx_packet();
	int amountOfAttempts = 4;
	int attempsCount = 0;
	while ( dxl_get_result() != COMM_TXSUCCESS )
	{
		
		std::clog << "Warning::data losses tring to move dynamixel motors" << std::endl;
		//try again
		dxl_tx_packet();
		amountOfAttempts++;
		if(attempsCount == amountOfAttempts)
		{
			std::clog << "Warning::Amount of attemps to move dynamixel motors exceeded." << std::endl;
			break;
		}
	}
}

// void CM700::setTorque(bool enable)
// {
// 	int value = 0;
// 	if (enable == true) {
// 		this->moveAll();
// 		value = 1;
// 	} else {
// 		value = 0;
// 	}

// #ifdef USE_USB2DXL
// 	dxl_set_txpacket_id(BROADCAST_ID);
// 	dxl_set_txpacket_length(2 * num_actuadores + 4);
// 	dxl_set_txpacket_instruction(INST_SYNC_WRITE);
// 	dxl_set_txpacket_parameter(0, P_TORQUE_ENABLE);
// 	dxl_set_txpacket_parameter(1, 1);

// 	for (int i = 0; i < num_actuadores; i++) {
// 		dxl_set_txpacket_parameter(2 + 2 * i, actuadores[i].id);
// 		dxl_set_txpacket_parameter(3 + 2 * i, value);
// 	}

// 	dxl_txrx_packet();
// #else
// 	buffer_out[0] = 0xFF;
// 	buffer_out[1] = SETTORQUE;
// 	buffer_out[2] = num_actuadores;
// 	buffer_out[3] = value;

// 	serial_io_flush(fd);
// 	serial_send(fd, buffer_out, 4);
// #endif
// }


void CM700::printValues() {
	for (unsigned int i = 0; i < motors.size(); i++) {
		printf("[%03u] [CP: %4u|TP: %4u] [CV: %4u|TV: %4u]\n",
			motors.at(i).id,
			motors.at(i).cposition, motors.at(i).tposition,
			motors.at(i).cspeed, motors.at(i).tspeed);
	}
}

bool CM700::verifyModel ( int model )
{
	if(model == AX12)
	{
		return true;
	}
	if(model == MX64)
	{
		return true;
	}
	return false;
} 

void CM700::setMotorParametersFromModel (int model, Motor & motor)
{	
	if(model == AX12)
	{
		motor.angleResolution = 1023;
		motor.hasCurrentSensor = false;
		motor.velocityResolution = 2047;
		motor.angleRangeDeg = 300;
	}

	if(model == MX64)
	{
		motor.angleResolution = 4095;
		motor.hasCurrentSensor = false;
		motor.velocityResolution = 2047;
		motor.angleRangeDeg = 360;
	}
}

#endif
