CC = g++ -O3 -std=c++11
VPATH = ./src ./headers ../robotObjects/headers ../robotObjects/src ../RobotVREP/remoteApi ../RobotVREP/include ../RobotVREP/headers
CFLAGS = -g -Wall -fPIC -I./src -I./headers -I./objects -I../robotObjects/headers -I../RobotVREP/headers -I../RobotVREP/remoteApi -I../RobotVREP/include -I../RobotVREP/objects -I../robotObjects/objects -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255
LDFLAGS = -lpthread

    
all: prueba

prueba: prueba.cpp
	@$(CC) $(CFLAGS) ./prueba.cpp ../robotObjects/objects/Joint.o ../robotObjects/objects/Object.o ../robotObjects/objects/CollisionObject.o ../RobotVREP/objects/RobotVREP.o ../RobotVREP/objects/extApi.o ../RobotVREP/objects/extApiPlatform.o  -o pruebaApp -lpthread

prueba_lib: prueba_lib.cpp
	@$(CC) -g -Wall -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 ./prueba_lib.cpp -o prueba_libApp -L. -lrobotlib
