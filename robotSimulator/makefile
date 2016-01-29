CC = g++ -O3
VPATH = ./src ./headers 
CFLAGS = -g -Wall -fPIC -I../remoteApi -I../include -I./src -I./headers -I./objects -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255
LDFLAGS = -lpthread

all: RobotSimulator.o 

RobotSimulator.o: RobotSimulator.cpp 
	@mkdir -p objects 
	@echo "Compiling RobotSimulator.cpp to RobotSimulator.o"
	@$(CC) $(CFLAGS) -c ./src/RobotSimulator.cpp -o ./objects/RobotSimulator.o

clean:
	@rm -f -R ./objects