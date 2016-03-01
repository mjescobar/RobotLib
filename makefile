LDFLAGS = -lpthread


OBJS = ./remoteApi/extApi.o ./remoteApi/extApiPlatform.o

EXT_OBJS = ./robotObjects/objects/Joint.o ./robotObjects/objects/Dummy.o ./robotObjects/objects/CollisionObject.o ./robotObjects/objects/Object.o ./RobotVREP/objects/RobotVREP.o ./DynamixelMotors/objects/cm700.o ./DynamixelMotors/objects/serial.o ./DynamixelMotors/objects/dynamixel.o ./DynamixelMotors/objects/dxl_hal.o



OODIR=objects/objects
_OOBJS = Joint.o CollisionObject.o Object.o 
OOBJS = $(patsubst %,$(OODIR)/%,$(_OOBJS))

RCODIR=DynamixelMotors/objects
_RCOBJS = cm700.o serial.o dynamixel.o dxl_hal.o
RCOBJS = $(patsubst %,$(RCODIR)/%,$(_RCOBJS))

all: 
	@echo "Compiling RobotVREP"
	@cd ./RobotVREP; make
	@echo "Compiling DynamixelMotors"
	@cd ./DynamixelMotors; make
	@echo "Compiling robotObjects"
	@cd ./robotObjects; make

clean:
		@rm -f $(OBJS)
		@cd ./RobotVREP; make clean		
		@cd ./DynamixelMotors; make clean
		@cd ./robotObjects; make clean


cleanall:
		@rm -f $(OBJS)
		@cd ./RobotVREP; make clean		
		@cd ./DynamixelMotors; make clean
		@cd ./objects; make clean


cleandocs:
	@rm -f -R ./doc

install:
	@g++ -shared -Wl,-soname,librobotlib.so.1 -o librobotlib.so.1.0 $(OOBJS) $(RCOBJS) $(RVOBJS) $(LDFLAGS)
	@ln -sf librobotlib.so.1.0 librobotlib.so
	@ln -sf librobotlib.so.1.0 librobotlib.so.1
	@mv librobotlib.so.1.0 librobotlib.so librobotlib.so.1 /usr/lib
	@mkdir -p /usr/include/ROBOTLIB_headers/
	@cp ./robotObjects/headers/* /usr/include/ROBOTLIB_headers/
	@cp ./RobotVREP/include/*.h /usr/include/ROBOTLIB_headers/
	@cp ./RobotVREP/remoteApi/*.h /usr/include/ROBOTLIB_headers/
	@cp ./RobotVREP/headers/* /usr/include/ROBOTLIB_headers/
	@cp ./DynamixelMotors/headers/* /usr/include/ROBOTLIB_headers/
	@cp ROBOTLIB /usr/include
	@chmod go+r /usr/include/ROBOTLIB_headers/*
	@chmod go+r /usr/include/ROBOTLIB

docs:
	@mkdir -p doc
	@doxygen ROBOTLIB_doxyfile 