//DONT use this test!, will change the id of all dynamixel to 99.
//Seting id 99 to the dynamixel.
#include "dynamixel.h"
#include <iostream>
using namespace std;

int main(int argc, char * argv[])
{

	if (dxl_initialize(argv[1], 1) == 0) 
	{
		cout << "Failed to open USB2Dynamixel!" << endl;
		cout << "Press Enter key to terminate..." << endl;
		cin.get();
		return 0;
	} 
	else 
	{
		cout << "USB2Dynamixel inicializado!" << endl;
	}


	
	dxl_set_txpacket_id( BROADCAST_ID );
	dxl_set_txpacket_length(4);
	dxl_set_txpacket_instruction( INST_WRITE );
	dxl_set_txpacket_parameter( 0, 99 );
	dxl_tx_packet();
	if ( dxl_get_result() != COMM_TXSUCCESS )
	{
		std::cout << "not fine" << std::endl;
	}
		std::cout << "all fine" << std::endl;

	return 0;

}