CC = gcc
CXX = g++
VPATH = ./src ./headers ./include ./remoteApi ../robotObjects/src ../robotObjects/headers ../DynamixelMotors/src ../DynamixelMotors/headers 
CFLAGS = -fPIC -I./remoteApi -I./include -I./headers -I../robotObjects/headers -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255
CFLAGS_EXT = -g -Wall -I./src -I./headers 
CFLAGS_EXT_2 = -I../robotObjects/src -I../robotObjects/headers -I../DynamixelMotors/src -I../DynamixelMotors/headers
LDFLAGS = -lpthread

OS = $(shell uname -s)
ifeq ($(OS), Linux)
	CFLAGS += -D__linux
else
	CFLAGS += -D__APPLE__
endif

ODIR=objects
_OBJS = RobotVREP.o extApi.o extApiPlatform.o
OBJS = $(patsubst %,$(ODIR)/%,$(_OBJS))

IDIR1=headers
_DEPS1 = RobotVREP.hpp 
DEPS1 = $(patsubst %,$(IDIR1)/%,$(_DEPS1))

IDIR2=include
_DEPS2 = v_repTypes.h v_repLib.h v_repConst.h extApiCustomConst.h 
DEPS2 = $(patsubst %,$(IDIR2)/%,$(_DEPS2))

IDIR3=remoteApi
_DEPS3 = extApi.h extApiCustom.h extApiInternal.h extApiPlatform.h 
DEPS3 = $(patsubst %,$(IDIR3)/%,$(_DEPS3))

SDIR1=src
SDIR2=remoteApi

all: RobotVREP.o extApi.o extApiPlatform.o

# $(ODIR)/%.o: $(SDIR1)/%.cpp $(DEPS1) $(DEPS2) $(DEPS3)
# 	@mkdir -p objects 
# 	@echo "Compiling $< to $@"
# 	$(CXX) $(CFLAGS_EXT) $(CFLAGS) -c $< -o $@

RobotVREP.o: RobotVREP.hpp RobotVREP.cpp 
	@mkdir -p objects
	$(CXX) $(CFLAGS_EXT) $(CFLAGS) -c ./src/RobotVREP.cpp -o ./objects/RobotVREP.o

# $(ODIR)/%.o: $(SDIR2)/%.c $(DEPS2) $(DEPS3)
# 	@mkdir -p objects 
# 	@echo "Compiling $< to $@" 
# 	$(CC) $(CFLAGS) -c $< -o $@

extApiPlatform.o: extApiPlatform.c extApiPlatform.h
	@mkdir -p objects
	$(CC) $(CFLAGS) -c ./remoteApi/extApiPlatform.c -o ./objects/extApiPlatform.o

extApi.o: extApi.c extApi.h
	@mkdir -p objects
	$(CC) $(CFLAGS) -c ./remoteApi/extApi.c -o ./objects/extApi.o

clean:
	@rm -f -R ./objects

prueba: ./src/prueba.cpp
	$(CXX) $(CFLAGS_EXT_2) $(CFLAGS_EXT) $(CFLAGS) ./src/prueba.cpp ./objects/RobotVREP.o ./objects/extApi.o ./objects/extApiPlatform.o ../robotObjects/objects/Object.o ../robotObjects/objects/Joint.o ../DynamixelMotors/objects/cm700.o ../DynamixelMotors/objects/serial.o ../DynamixelMotors/objects/dynamixel.o ../DynamixelMotors/objects/dxl_hal.o -o prueba $(LDFLAGS)

cleanprueba:
	@rm -f prueba