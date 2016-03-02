CC = g++ -O3 -std=c++11
VPATH = ./src ./headers ../robotObjects/headers ../robotObjects/src ../RobotVREP/remoteApi
CFLAGS = -g -Wall -fPIC -I./src -I./headers -I./objects -I../robotObjects/headers -I../RobotVREP/headers -I../RobotVREP/remoteApi -I../RobotVREP/include -I../RobotVREP/objects -I../robotObjects/objects -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255
LDFLAGS = -lpthread

all: usb2dynamixel.o cm700.o serial.o dxl_hal.o dynamixel.o 

usb2dynamixel.o: usb2dynamixel.cpp dynamixel.o
	@mkdir -p objects
	@echo "Compiling usb2dynamixel.cpp to usb2dynamixel.o"
	@$(CC) $(CFLAGS) -c ./src/usb2dynamixel.cpp  -o ./objects/usb2dynamixel.o

cm700.o: cm700.cpp dynamixel.o
	@mkdir -p objects
	@echo "Compiling cm700.cpp to cm700.o"
	$(CC) $(CFLAGS) -c ./src/cm700.cpp  -o ./objects/cm700.o

serial.o: serial.cpp
	@mkdir -p objects
	@echo "Compiling serial.cpp to serial.o"
	@$(CC) $(CFLAGS) -c ./src/serial.cpp -o ./objects/serial.o

dxl_hal.o: dxl_hal.c
	@mkdir -p objects
	@echo "Compiling dxl_hal.c to dxl_hal.o"
	@$(CC) $(CFLAGS) -c ./src/dxl_hal.c -o ./objects/dxl_hal.o

dynamixel.o: dynamixel.c dxl_hal.o
	@mkdir -p objects
	@echo "Compiling dynamixel.c to dynamixel.o"
	@$(CC) $(CFLAGS) -c ./src/dynamixel.c  -o ./objects/dynamixel.o


clean:
	@rm -f -R ./objects