#include "dynamixel.h"
#include "serial.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>    // std::find
#include "usb2dynamixel.h"
#include "Joint.hpp"


using namespace std;

int main(int argc, char * argv[])
{
	// int attemps = 3;
	// int model;
	// cout << "ID: " << atoi(argv[1]) << endl;
	// int i;
	// for (i = 0; i < attemps; ++i)
	// {
	// 	if (dxl_initialize( 0, 1) == 0) 
	// 	{
	// 		cout << "Failed to open USB2Dynamixel!" << endl;
	// 		cout << "Press Enter key to terminate..." << endl;
	// 		cin.get();
	// 		return 1;
	// 	} 
	// 	else 
	// 	{
	// 		cout << "USB2Dynamixel inicializado!" << endl;
	// 	}

	// 	dxl_set_txpacket_id 			( atoi( argv[ 1 ] ) );
	// 	dxl_set_txpacket_length 		( 4 );
	// 	dxl_set_txpacket_instruction 	( INST_READ ); 
	// 	dxl_set_txpacket_parameter 		( 0, 0 ); 
	// 	dxl_set_txpacket_parameter 		( 1, 2 ); 
	// 	dxl_txrx_packet 				();

	// 	model	= _MW(dxl_get_rxpacket_parameter(0), dxl_get_rxpacket_parameter(1));

	// 	if(model != 0)
	// 	{
	// 		break;
	// 	}
	// }
	
	// cout << "model: " << model  << "   i: " << i<< endl;

	USB2Dynamixel * usb2dyn = new USB2Dynamixel("/dev/ttyUSB0", 1); 
	int numMotores = 4;
	vector <Joint *> jointVector;
	for (int i = 0; i < numMotores; ++i)
	{
		Joint * joint = new Joint ( M_PI/4.0, M_PI/4.0, (char *)"RAD" );
		usb2dyn->addMotor(joint, i+1);
		jointVector.push_back(joint);
	}
	
	for (int i = 0; i < numMotores; ++i)
	{
		jointVector.at(i)->setJointNextPosition(0);
	}
	
	usb2dyn->move();

}