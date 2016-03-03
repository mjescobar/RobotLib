CC = g++ -O3
VPATH = ./src ./headers 
CFLAGS = -g -Wall -fPIC -I../RobotVREP/remoteApi -I../RobotVREP/include -I./src -I./headers -I./objects -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255
EXTFLAGS = -I../RobotVREP/headers -I../DynamixelMotors/headers

LDFLAGS = -lpthread

all: Joint.o CollisionObject.o RobotObject.o


Joint.o: Joint.cpp 
	@mkdir -p objects 
	@echo "Compiling Joint.cpp to Joint.o"
	@$(CC) $(CFLAGS) $(EXTFLAGS) -c ./src/Joint.cpp -o ./objects/Joint.o

CollisionObject.o: CollisionObject.cpp 
	@mkdir -p objects 
	@echo "Compiling CollisionObject.cpp to CollisionObject.o"
	@$(CC) $(CFLAGS) $(EXTFLAGS) -c ./src/CollisionObject.cpp -o ./objects/CollisionObject.o

RobotObject.o: robotObject.cpp
	@mkdir -p objects 
	@echo "Compiling robotObject.cpp to robotObject.o"
	@$(CC) $(CFLAGS) $(EXTFLAGS) -c ./src/robotObject.cpp -o ./objects/robotObject.o

clean:
	@rm -f -R ./objects