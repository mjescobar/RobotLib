CC = g++ -O3
VPATH = ./src ./headers 
CFLAGS = -g -Wall -fPIC -I../remoteApi -I../include -I./src -I./headers -I./objects -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255
LDFLAGS = -lpthread

all: RobotVREP.o 

RobotVREP.o: RobotVREP.cpp 
	@mkdir -p objects 
	@echo "Compiling RobotVREP.cpp to RobotVREP.o"
	@$(CC) $(CFLAGS) -c ./src/RobotVREP.cpp -o ./objects/RobotVREP.o

clean:
	@rm -f -R ./objects