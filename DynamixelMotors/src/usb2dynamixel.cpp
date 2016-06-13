
#include "usb2dynamixel.h"

//#define _DEBUG

/*
	remember: BaudRate = 2.000.000/(baudNum + 1) 
*/
USB2Dynamixel::USB2Dynamixel(string serialPort, int baudNum = 1) 
{
	//Se comienza la comunicacion serial con los motores.
	if (dxl_initialize( std::stoi( serialPort ), baudNum) == 0) // Modificar para que acepte un string dado que es mas convencional
	{
		cerr << "Failed to open USB2Dynamixel!" << endl;
		exit(EXIT_FAILURE);
	} 
	else 
	{
		cout << "USB2Dynamixel inicializado!" << endl;
	}
}

USB2Dynamixel::~USB2Dynamixel( ) 
{
	motors.clear();
}

//We try to obtain the most information posible to create the motor throught the motor model number, if is not in our registers then is necesary to use the other constructor.
void USB2Dynamixel::addMotor(Joint * joint, int id)
{

	//Verifing that the id is unique
	if( idToMotorsVectorPosition_map.find( id ) != idToMotorsVectorPosition_map.end() )
	{
		//The Id already exist
		std::cerr << "ERROR::USB2Dynamixel::addMotor::The id already exist:: " << std::endl;
		exit (EXIT_FAILURE);
	}
	this->jointVector.push_back(joint);

	Motor motor;
	motor.id = id;
	motor.jointId =  joint->getUniqueObjectId();

	//identify model
	int model = identifyModel(id);

	//Settings motors values from model.
	setParametersFromModel(model, motor);

	// Adding the motor
	motors.push_back( motor );
	idToMotorsVectorPosition_map.insert ( std::pair<int,int> ( id, motors.size() -1 ) );
	jointIdToId_map.insert( std::pair<int,int> ( motor.jointId , motor.id  ) );


}



//We try to obtain the most information posible to create the motor throught the motor model number, if is not in our registers then is necesary to use the other constructor.
void USB2Dynamixel::addMotor(Joint * joint, int id, int angleResolution, bool hasCurrentSensor, int velocityResolution, double angleRangeDeg)
{
	//Verifing that the id is unique
	if( idToMotorsVectorPosition_map.find( id ) != idToMotorsVectorPosition_map.end() )
	{
		//The Id already exist
		std::cerr << "ERROR::USB2Dynamixel::addMotor::The id already exist:: " << std::endl;
		exit (EXIT_FAILURE);
	}
	this->jointVector.push_back(joint);

	Motor motor;
	motor.id = id;
	motor.jointId =  joint->getUniqueObjectId();

	//Settings motors values from model.
	motor.angleResolution = angleResolution;
	motor.hasCurrentSensor = hasCurrentSensor;
	motor.velocityResolution = velocityResolution;
	motor.angleRangeDeg = angleRangeDeg;

	// Adding the motor
	motors.push_back( motor );
	idToMotorsVectorPosition_map.insert ( std::pair<int,int> ( id, motors.size() -1 ) );
	jointIdToId_map.insert( std::pair<int,int> ( motor.jointId , motor.id  ) );
}

double USB2Dynamixel::getAngle( int  jointId ) 
{
	int id = jointIdToId_map.at( jointId );

	if( idToMotorsVectorPosition_map.find( id ) == idToMotorsVectorPosition_map.end() )
	{
		//The Id already exist
		std::cerr << "ERROR::USB2Dynamixel::getMotorPosition::The id is not in the current memory, you have to add the Id before to call this method." << std::endl;
		exit (EXIT_FAILURE);
	}
	auto motorVectorPosition = idToMotorsVectorPosition_map.at( id );
	int position = motors.at(motorVectorPosition).cposition;
	double resolution =  motors.at( motorVectorPosition ).angleResolution ;
	double range = 	motors.at( motorVectorPosition ).angleRangeDeg*(M_PI/180.0);
	return ( position - resolution/2.0 )*(range/resolution) ;// position = resolution/2 -> angle = 0;; position = 0 -> angle = -range/2 (creating the line is obtained the equation)
}

void USB2Dynamixel::refreshAll()
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

void USB2Dynamixel::move()
{
	int param_per_actuator = 4;
	

	dxl_set_txpacket_id( BROADCAST_ID );
	dxl_set_txpacket_length( (param_per_actuator + 1) * ((int)jointVector.size()) + 4);
	dxl_set_txpacket_instruction( INST_SYNC_WRITE );
	dxl_set_txpacket_parameter( 0, P_GOAL_POSITION_L );
	dxl_set_txpacket_parameter( 1, param_per_actuator );

	for (unsigned int i = 0; i < jointVector.size(); i++) 
	{
		int motorId = jointVector.at(i)->getUniqueObjectId();
		int motorVectorPosition = idToMotorsVectorPosition_map.at( jointIdToId_map.at( motorId ) );
		double angle_RAD = jointVector.at(i)->getNextPositionRad();

		dxl_set_txpacket_parameter(2 + 5 * i, motors.at(motorVectorPosition).id);

		// the goal position have to be transformed taking in account that the value 0 is not in the mitdle but in one extreme (the midle is in the angleResolution/2) and that the dynamizel could have a range of degress and not always 360 degrees
		double resolution =  motors.at( motorVectorPosition ).angleResolution ;
		double range = 	motors.at( motorVectorPosition ).angleRangeDeg*(M_PI/180.0);
		int goalPosition = (int)(resolution/2.0 + (resolution*angle_RAD)/range); // position = resolution/2 -> angle = 0;; position = 0 -> angle = -range/2 (creating the line is obtained the equation)


		dxl_set_txpacket_parameter(2 + 5 * i + 1, _L16(goalPosition) );
		dxl_set_txpacket_parameter(2 + 5 * i + 2, _H16(goalPosition) );
		// dxl_set_txpacket_parameter(2 + 5 * i + 3, _L16(motors.at(idPositionInMotors ).tspeed));
		// dxl_set_txpacket_parameter(2 + 5 * i + 4, _H16(motors.at(idPositionInMotors ).tspeed));
		dxl_set_txpacket_parameter(2 + 5 * i + 3, _L16(512) ); // I think that does not do anything
		dxl_set_txpacket_parameter(2 + 5 * i + 4, _H16(512) );
	}

	dxl_txrx_packet();
	int amountOfAttempts = 4;
	int attempsCount = 0;
	while ( dxl_get_result() != COMM_TXSUCCESS )
	{
		//std::clog << "Warning::data losses tring to move dynamixel motors" << std::endl;
		//try again
		dxl_tx_packet();
		attempsCount++;
		if(attempsCount >= amountOfAttempts)
		{
			std::clog << "Warning::Amount of attemps to move dynamixel motors exceeded." << std::endl;
			break;
		}
	}
}


void USB2Dynamixel::printValues() {
	for (unsigned int i = 0; i < motors.size(); i++) {
		printf("[%03u] [CP: %4u|TP: %4u] [CV: %4u|TV: %4u]\n",
			motors.at(i).id,
			motors.at(i).cposition, motors.at(i).tposition,
			motors.at(i).cspeed, motors.at(i).tspeed);
	}
}

bool USB2Dynamixel::verifyModel ( int model )
{
	if(model == AX12 || model == AX18)
	{
		return true;
	}
	if(model == MX64)
	{
		return true;
	}
	return false;
} 

void USB2Dynamixel::setParametersFromModel (int model, Motor & motor)
{	
	if(model == AX12 || model == AX18 )
	{
		motor.angleResolution = 1023;
		motor.hasCurrentSensor = false;
		motor.velocityResolution = 2047;
		motor.angleRangeDeg = 300;
	}

	else if(model == MX64)
	{
		motor.angleResolution = 4095;
		motor.hasCurrentSensor = false;
		motor.velocityResolution = 2047;
		motor.angleRangeDeg = 360;
	}
}



int USB2Dynamixel::identifyModel( int id )
{
	int attemps = 3;
	int model;
	
	int i;
	for (i = 0; i < attemps; ++i)
	{

		dxl_set_txpacket_id 			( id );
		dxl_set_txpacket_length 		( 4 );
		dxl_set_txpacket_instruction 	( INST_READ ); 
		dxl_set_txpacket_parameter 		( 0, 0 ); 
		dxl_set_txpacket_parameter 		( 1, 2 ); 
		dxl_txrx_packet 				();

		model	= _MW(dxl_get_rxpacket_parameter(0), dxl_get_rxpacket_parameter(1));

		if(model != 0)
		{
			break;
		}
	}
	if(model == 0)
	{
		cerr << "Can not identify the dynamixel model with id " << id << endl;
		exit(1);
	}

	cerr << "model: " << model << endl;//borrar

	return model;
}
