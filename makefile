CFLAGS = -fPIC -I./remoteApi -I./include -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255
LDFLAGS = -lpthread

OBJS = ./remoteApi/extApi.o ./remoteApi/extApiPlatform.o
EXT_OBJS = ./objects/objects/Joint.o ./objects/objects/Dummy.o ./objects/objects/CollisionObject.o ./objects/objects/Object.o ./robotSimulator/objects/RobotSimulator.o ./robotCM700/objects/cm700.o ./robotCM700/objects/serial.o ./robotCM700/objects/dynamixel.o ./robotCM700/objects/dxl_hal.o

OS = $(shell uname -s)
ECHO = @

ifeq ($(OS), Linux)
	CFLAGS += -D__linux
else
	CFLAGS += -D__APPLE__
endif

all: $(OBJS)
	@echo "Compiling RobotSimulator"
	@cd ./robotSimulator; make
	@echo "Compiling robotCM700"
	@cd ./robotCM700; make
	@echo "Compiling Objects"
	@cd ./objects; make


%.o: %.cpp
		@echo "Compiling $< to $@"
		$(ECHO)$(CXX) $(CFLAGS) -c $< -o $@

%.o: %.c
		@echo "Compiling $< to $@" 
		$(ECHO)$(CC) $(CFLAGS) -c $< -o $@

clean:
		@rm -f $(OBJS)
		@cd ./robotSimulator; make clean		
		@cd ./robotCM700; make clean
		@cd ./objects; make clean
		@cd ./example; make clean

cleandocs:
	@rm -f -R ./doc

install:
	@g++ -shared -Wl,-soname,librobotlib.so.1 -o librobotlib.so.1.0 $(EXT_OBJS) $(OBJS) $(LDFLAGS)
	@ln -sf librobotlib.so.1.0 librobotlib.so
	@ln -sf librobotlib.so.1.0 librobotlib.so.1
	@mv librobotlib.so.1.0 librobotlib.so librobotlib.so.1 /usr/lib
	@mkdir -p /usr/include/ROBOTLIB_headers/
	@cp ./include/*.h /usr/include/ROBOTLIB_headers/
	@cp ./remoteApi/*.h /usr/include/ROBOTLIB_headers/
	@cp ./objects/headers/* /usr/include/ROBOTLIB_headers/
	@cp ./robotCM700/headers/* /usr/include/ROBOTLIB_headers/
	@cp ./robotSimulator/headers/* /usr/include/ROBOTLIB_headers/
	@cp ROBOTLIB /usr/include
	@chmod go+r /usr/include/ROBOTLIB_headers/*
	@chmod go+r /usr/include/ROBOTLIB

docs:
	@mkdir -p doc
	@doxygen ROBOTLIB_doxyfile 