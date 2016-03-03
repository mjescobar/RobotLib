CC = g++ -O3 -std=c++11
VPATH = .:../src:../headers:../objects
CFLAGS = -g -Wall -fPIC -I../src -I../headers -I../../RobotVREP/headers -I../../RobotVREP/remoteApi -I../../RobotVREP/include -I../objects -I../../robotObjects/headers -I../../robotObjects/objects -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255
LDFLAGS = -lpthread

getInfoOfMotor: getInfoOfMotor.cpp 
	@$(CC) $(CFLAGS)  getInfoOfMotor.cpp ../../robotObjects/objects/Joint.o ../objects/usb2dynamixel.o ../objects/dynamixel.o ../objects/dxl_hal.o -o getInfoOfMotorApp

run:
	./getInfoOfMotorApp
  