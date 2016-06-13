CC = g++ -O3
VPATH = ./src ./headers 
CFLAGS = -g -Wall -fPIC -I../RobotVREP/remoteApi -I../RobotVREP/include -I./src -I./headers -I./objects -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255
EXTFLAGS = -I../RobotVREP/headers -I../DynamixelMotors/headers

LDFLAGS = -lpthread

all: VisionSensor.o Joint.o CollisionObject.o Object.o

VisionSensor.o: VisionSensor.cpp 
	@mkdir -p objects 
	@echo "Compiling VisionSensor.cpp to VisionSensor.o"
	@$(CC) $(CFLAGS) $(EXTFLAGS) -c ./src/VisionSensor.cpp -o ./objects/VisionSensor.o

Joint.o: Joint.cpp 
	@mkdir -p objects 
	@echo "Compiling Joint.cpp to Joint.o"
	@$(CC) $(CFLAGS) $(EXTFLAGS) -c ./src/Joint.cpp -o ./objects/Joint.o

CollisionObject.o: CollisionObject.cpp 
	@mkdir -p objects 
	@echo "Compiling CollisionObject.cpp to CollisionObject.o"
	@$(CC) $(CFLAGS) $(EXTFLAGS) -c ./src/CollisionObject.cpp -o ./objects/CollisionObject.o

Object.o: Object.cpp
	@mkdir -p objects 
	@echo "Compiling Object.cpp to Object.o"
	@$(CC) $(CFLAGS) $(EXTFLAGS) -c ./src/Object.cpp -o ./objects/Object.o

clean:
	@rm -f -R ./objects