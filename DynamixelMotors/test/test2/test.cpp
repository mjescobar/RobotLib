
#include "dynamixel.h"
#include "cm700.h"
#include <iostream>
#include <unistd.h>
using namespace std;

int main(int argc, char * argv[])
{
	CM700 * cm700 = new CM700("/dev/ttyUSB1", 1);

	cm700->addMotor(1);
	cm700->setMotorPosition(1, 1.0, 0.5);
	cm700->move();

	sleep(2);
	cm700->setMotorPosition(1, 1.5, 0.5);
	cm700->move();
	sleep(1);

	return 0;
}