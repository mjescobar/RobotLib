CC = gcc
CXX = g++
VPATH = ./src ./headers ./include ./remoteApi ../objects/src ../objects/headers ../robotCM700/src ../robotCM700/headers
CFLAGS = -fPIC -I./remoteApi -I./include -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255
CFLAGS_EXT = -g -Wall -I./src -I./headers 
CFLAGS_EXT_2 = -I../objects/src -I../objects/headers -I../robotCM700/src -I../robotCM700/headers
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


all: $(OBJS)

$(ODIR)/%.o: $(SDIR1)/%.cpp $(DEPS1) $(DEPS2) $(DEPS3)
	@mkdir -p objects 
	@echo "Compiling $< to $@"
	$(CXX) $(CFLAGS_EXT) $(CFLAGS) -c $< -o $@

$(ODIR)/%.o: $(SDIR2)/%.c $(DEPS2) $(DEPS3)
	@mkdir -p objects 
	@echo "Compiling $< to $@" 
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	@rm -f -R ./objects

prueba: ./src/prueba.cpp
	$(CXX) $(CFLAGS_EXT_2) $(CFLAGS_EXT) $(CFLAGS) ./src/prueba.cpp ./objects/RobotVREP.o ./objects/extApi.o ./objects/extApiPlatform.o ../objects/objects/Object.o ../objects/objects/Joint.o ../robotCM700/objects/cm700.o ../robotCM700/objects/serial.o ../robotCM700/objects/dynamixel.o ../robotCM700/objects/dxl_hal.o -o prueba $(LDFLAGS)

cleanprueba:
	@rm -f prueba