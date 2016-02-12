LDFLAGS = -lpthread

RVODIR=RobotVREP/objects
_RVOBJS = RobotVREP.o extApi.o extApiPlatform.o
RVOBJS = $(patsubst %,$(RVODIR)/%,$(_RVOBJS))

OODIR=objects/objects
_OOBJS = Joint.o CollisionObject.o Object.o 
OOBJS = $(patsubst %,$(OODIR)/%,$(_OOBJS))

RCODIR=robotCM700/objects
_RCOBJS = cm700.o serial.o dynamixel.o dxl_hal.o
RCOBJS = $(patsubst %,$(RCODIR)/%,$(_RCOBJS))

all: 
	@echo "Compiling RobotVREP"
	@cd ./RobotVREP; make
	@echo "Compiling robotCM700"
	@cd ./robotCM700; make
	@echo "Compiling Objects"
	@cd ./objects; make

clean:
		@rm -f $(OBJS)
		@cd ./RobotVREP; make clean		
		@cd ./robotCM700; make clean
		@cd ./objects; make clean

cleanall:
		@rm -f $(OBJS)
		@cd ./RobotVREP; make clean		
		@cd ./robotCM700; make clean
		@cd ./objects; make clean
		@cd ./example; make clean

cleandocs:
	@rm -f -R ./doc

install:
	@g++ -shared -Wl,-soname,librobotlib.so.1 -o librobotlib.so.1.0 $(OOBJS) $(RCOBJS) $(RVOBJS) $(LDFLAGS)
	@ln -sf librobotlib.so.1.0 librobotlib.so
	@ln -sf librobotlib.so.1.0 librobotlib.so.1
	@mv librobotlib.so.1.0 librobotlib.so librobotlib.so.1 /usr/lib
	@mkdir -p /usr/include/ROBOTLIB_headers/
	@cp ./RobotVREP/include/*.h /usr/include/ROBOTLIB_headers/
	@cp ./RobotVREP/remoteApi/*.h /usr/include/ROBOTLIB_headers/
	@cp ./RobotVREP/headers/* /usr/include/ROBOTLIB_headers/
	@cp ./objects/headers/* /usr/include/ROBOTLIB_headers/
	@cp ./robotCM700/headers/* /usr/include/ROBOTLIB_headers/
	@cp ROBOTLIB /usr/include
	@chmod go+r /usr/include/ROBOTLIB_headers/*
	@chmod go+r /usr/include/ROBOTLIB

docs:
	@mkdir -p doc
	@doxygen ROBOTLIB_doxyfile 