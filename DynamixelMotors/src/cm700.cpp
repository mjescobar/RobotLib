#include "cm700.h"

CM700::CM700(string serialPort, speed_t baudRate) 
{
	memset(buffer_in, 0, sizeof(uint8_t));
	memset(buffer_out, 0, sizeof(char));
	serial_name = strdup( serialPort.c_str() );
	fd = serial_open(serial_name, baudRate);
}

CM700::~CM700()
{
	serial_close(fd);
	free(serial_name);
} 

void CM700::addMotor(int id)
{

	//Verifing that the id is unique
	if( idToMotorsVectorPosition_map.find( id ) != idToMotorsVectorPosition_map.end() )
	{
		//The Id already exist
		std::cerr << "ERROR::CM700::addMotor(int id)::The id already exist" << std::endl;
		exit (EXIT_FAILURE);
	}

	Motor motor;
	motor.id=id;
	// Obtaining information about the motor throught consulting the dynamixel motor connected with the corresponding id.

	//Obtaining the model number.
	int model = AX12; // to do... right now is only for AX, but may be simply.

	//Verify if the model is in our register, in case that is not in our registers then a error es sent.
	if ( !verifyModel( model ) )
	{
		std::cerr << "ERROR::void CM700(int id)::addMotor(int id)::Your dynamixel model is not in our register, in that case you have to use an alternative addMotor method" << std::endl;
		exit(EXIT_FAILURE);
	}

	//Settings motors values from model.
	setMotorParametersFromModel( model, motor);

	// Adding the motor
	motors.push_back( motor );
	idToMotorsVectorPosition_map.insert ( std::pair<int,int> ( id, motors.size() -1 ) );
}

//We try to obtain the most information posible to create the motor throught the motor model number, if is not in our registers then is necesary to use the other constructor.
void CM700::addMotor(int id, int angleResolution, bool hasCurrentSensor, int velocityResolution, double angleRangeDeg)
{
	//Verifing that the id is unique
	if( idToMotorsVectorPosition_map.find( id ) != idToMotorsVectorPosition_map.end() )
	{
		//The Id already exist
		std::cerr << "ERROR::CM700::addMotor(int id, int angleResolution, bool hasCurrentSensor, int velocityResolution, double angleRangeDeg)::The id already exist:: " << std::endl;
		exit (EXIT_FAILURE);
	}

	Motor motor;
	motor.id = id;

	//Settings motors values from model.
	motor.angleResolution = angleResolution;
	motor.hasCurrentSensor = hasCurrentSensor;
	motor.velocityResolution = velocityResolution;
	motor.angleRangeDeg = angleRangeDeg;

	// Adding the motor
	motors.push_back( motor );
	idToMotorsVectorPosition_map.insert ( std::pair<int,int> ( id, motors.size() -1 ) );
}

void USB2Dynamixel::setNextMotorAngle(int id, double angle_RAD, double normalizedVelocity)
{
	if( idToMotorsVectorPosition_map.find( id ) == idToMotorsVectorPosition_map.end() )
	{
		//The Id already exist
		std::cerr << "ERROR::USB2Dynamixel::setMotorPosition::The id is not in the current memory, you have to add the Id before to call this method." << std::endl;
		exit (EXIT_FAILURE);
	}

	int motorVectorPosition = idToMotorsVectorPosition_map.at( id );

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
		std::cerr << "ERROR::USB2Dynamixel::setMotorPosition::normalizedVelocity must be in [0,1]" <<  std::endl;
		exit(EXIT_FAILURE);
	}
	if ( fabs(angle_RAD) >= motors.at( motorVectorPosition ).angleRangeDeg*(M_PI/180.0) ) 
	{
		std::cerr << "ERROR::USB2Dynamixel::setMotorPosition::angle_RAD is not in the phisical angle range of the dynamixel motor" <<  std::endl;
		exit(EXIT_FAILURE);
	}


	// EACH MODEL COULD HAVE DIFERENT AMOUNT OF ANGLE RESOLUTION, THAT MEANS THAT A ANGLE IN RAD WILL BE A DIFERENT GOALPOSITION VALUE.

	// the goal position have to be transformed taking in account that the value 0 is not in the mitdle but in one extreme (the midle is in the angleResolution/2) and that the dynamizel could have a range of degress and not always 360 degrees
	double resolution =  motors.at( motorVectorPosition ).angleResolution ;
	double range = 	motors.at( motorVectorPosition ).angleRangeDeg*(M_PI/180.0);
	int goalPosition = (int)(resolution/2.0 + (resolution*angle_RAD)/range); // position = resolution/2 -> angle = 0;; position = 0 -> angle = -range/2 (creating the line is obtained the equation)
	// the speed is scaled because for convention is normalized then 0 is the minimum and 1 is the max value.
	int speed = (int)( normalizedVelocity * motors.at(motorVectorPosition).velocityResolution);
	motors.at( motorVectorPosition ).tposition = goalPosition;
	motors.at( motorVectorPosition ).tspeed = speed;


	// Now it is placed within a vector of motor that will be moved in le next move() call.
	// take in account that if is in that vector then is no necesary put it aggain
	std::vector<int>::iterator it;
	it = find (idMotorsWithNewPosition_Vect.begin(), idMotorsWithNewPosition_Vect.end(), id);
	if (it == idMotorsWithNewPosition_Vect.end())
	{
		idMotorsWithNewPosition_Vect.push_back( id ); // Only if is not in the vector this line occur
	}
}


// right now the position is not in rad.
double CM700::getMotorAngle( int id ) 
{
	if( idToMotorsVectorPosition_map.find( id ) == idToMotorsVectorPosition_map.end() )
	{
		//The Id already exist
		std::cerr << "ERROR::CM700::getMotorPosition::The id is not in the current memory, you have to add the Id before to call this method." << std::endl;
		exit (EXIT_FAILURE);
	}
	auto motorVectorPosition = idToMotorsVectorPosition_map.at( id );
	int position = motors.at(motorVectorPosition).cposition;
	double resolution =  motors.at( motorVectorPosition ).angleResolution ;
	double range = 	motors.at( motorVectorPosition ).angleRangeDeg*(M_PI/180.0);
	return ( position - resolution/2.0 )*(range/resolution) ;// position = resolution/2 -> angle = 0;; position = 0 -> angle = -range/2 (creating the line is obtained the equation)
}


void CM700::refreshAll()
{
	int valid_actuators = 0;

	buffer_out[0] = 0xFF;
	buffer_out[1] = ASKPOSNDVEL;
	buffer_out[2] = motors.size();

	serial_io_flush(fd);
	serial_send(fd, buffer_out, 3);
	serial_read(fd, buffer_in, 3, DEF_TIMEOUT);

	if (buffer_in[0] != 0xFF || buffer_in[1] != RPYPOSNVEL) {
		serial_io_flush(fd);
		return;
	}

	valid_actuators = buffer_in[2];
	serial_read(fd, buffer_in, 9 * valid_actuators, DEF_TIMEOUT);
	serial_io_flush(fd);

	for (int i = 0; i < valid_actuators; i++) 
	{
		for (unsigned int j = 0; j < motors.size(); j++) 
		{
			if (motors.at(j).id == buffer_in[0 + 9 * i]) 
			{
				motors.at(j).cposition = _MW(buffer_in[1 + 9 * i], buffer_in[2 + 9 * i]);
				motors.at(j).cspeed = _MW(buffer_in[3 + 9 * i], buffer_in[4 + 9 * i]) & 0x3FF;
				motors.at(j).load = _MW(buffer_in[5 + 9 * i], buffer_in[6 + 9 * i]);
				motors.at(j).volt = buffer_in[7 + 9 * i];
				motors.at(j).temperature = buffer_in[8 + 9 * i];				
			}
		}
	}
}

void CM700::printValues()
{
	for (unsigned int i = 0; i < motors.size(); i++)
	{
		cout << "ID::"<< motors.at(i).id << endl;
		cout << "cposition: " <<  motors.at(i).cposition << endl;
		cout << "cspeed: " <<  motors.at(i).cspeed << endl;
		cout << "load: " <<  motors.at(i).load << endl;
		cout << "volt: " <<  motors.at(i).volt << endl;
		cout << "temperature: " <<  motors.at(i).temperature << endl;
	} 
}


void CM700::move()
{
	buffer_out[0] = 0xFF;
	buffer_out[1] = SETPOSNDVEL;
	buffer_out[2] = motors.size();

	for (unsigned int i = 0; i < motors.size(); i++) {
		buffer_out[5 * i + 3] = 254 - motors.at(i).id;
		buffer_out[5 * i + 4] = _L16(motors.at(i).tposition);
		buffer_out[5 * i + 5] = _H16(motors.at(i).tposition);
		buffer_out[5 * i + 6] = _L16(motors.at(i).tspeed);
		buffer_out[5 * i + 7] = _H16(motors.at(i).tspeed);
	}

	serial_io_flush(fd);
	serial_send(fd, buffer_out, 5 * motors.size() + 3);

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