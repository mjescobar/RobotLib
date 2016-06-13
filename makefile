LDFLAGS = -lpthread

EXT_OBJS = ./robotObjects/objects/VisionSensor.o ./robotObjects/objects/Joint.o ./robotObjects/objects/CollisionObject.o ./robotObjects/objects/Object.o ./RobotVREP/objects/RobotVREP.o ./DynamixelMotors/objects/cm700.o ./DynamixelMotors/objects/serial.o ./DynamixelMotors/objects/dynamixel.o . /DynamixelMotors/objects/dxl_hal.o /DynamixelMotors/objects/usb2dynamixel.o



ROBOTOBJECTS_ODIR=robotObjects/objects
_ROBOTOBJECTS_OBJS = VisionSensor.o Joint.o CollisionObject.o Object.o 
ROBOTOBJECTS_OBJS = $(patsubst %,$(ROBOTOBJECTS_ODIR)/%,$(_ROBOTOBJECTS_OBJS))

ROBOTVREP_ODIR=RobotVREP/objects
_ROBOTVREP_OBJS = RobotVREP.o extApi.o extApiPlatform.o
ROBOTVREP_OBJS = $(patsubst %,$(ROBOTVREP_ODIR)/%,$(_ROBOTVREP_OBJS))

DYNAMIXELMOTORS_ODIR=DynamixelMotors/objects
_DYNAMIXELMOTORS_OBJS = cm700.o serial.o dynamixel.o dxl_hal.o usb2dynamixel.o
DYNAMIXELMOTORS_OBJS = $(patsubst %,$(DYNAMIXELMOTORS_ODIR)/%,$(_DYNAMIXELMOTORS_OBJS))

all: 
	@echo "Compiling RobotVREP"
	@cd ./RobotVREP; make
	@echo "Compiling DynamixelMotors"
	@cd ./DynamixelMotors; make
	@echo "Compiling robotObjects"
	@cd ./robotObjects; make

clean:
	@cd ./RobotVREP; make clean		
	@cd ./DynamixelMotors; make clean
	@cd ./robotObjects; make clean

cleandocs:
	@rm -f -R ./doc

install:
	@g++ -shared -Wl,-soname,librobotlib.so.1 -o librobotlib.so.1.0 $(ROBOTOBJECTS_OBJS) $(DYNAMIXELMOTORS_OBJS) $(ROBOTVREP_OBJS) $(LDFLAGS)
	@ln -sf librobotlib.so.1.0 librobotlib.so
	@ln -sf librobotlib.so.1.0 librobotlib.so.1
	@mv librobotlib.so.1.0 librobotlib.so librobotlib.so.1 /usr/lib
	@mkdir -p /usr/include/ROBOTLIB_headers/
	@rm -f /usr/include/ROBOTLIB_headers/*
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