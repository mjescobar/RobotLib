CC = g++ -O3
VPATH = ./src ./headers 
CFLAGS = -g -Wall -fPIC -I../remoteApi -I../include -I./src -I./headers -I./objects -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255
EXTFLAGS = -I../robotSimulator/headers -I../robotCM700/headers
LDFLAGS = -lpthread

all: Object.o Joint.o Dummy.o CollisionObject.o 

Object.o: Object.cpp 
	@mkdir -p objects 
	@echo "Compiling Object.cpp to Object.o"
	@$(CC) $(CFLAGS) $(EXTFLAGS) -c ./src/Object.cpp -o ./objects/Object.o

Joint.o: Joint.cpp 
	@mkdir -p objects 
	@echo "Compiling Joint.cpp to Joint.o"
	@$(CC) $(CFLAGS) $(EXTFLAGS) -c ./src/Joint.cpp -o ./objects/Joint.o

Dummy.o: Dummy.cpp 
	@mkdir -p objects 
	@echo "Compiling Dummy.cpp to Dummy.o"
	@$(CC) $(CFLAGS) $(EXTFLAGS) -c ./src/Dummy.cpp -o ./objects/Dummy.o

CollisionObject.o: CollisionObject.cpp 
	@mkdir -p objects 
	@echo "Compiling CollisionObject.cpp to CollisionObject.o"
	@$(CC) $(CFLAGS) $(EXTFLAGS) -c ./src/CollisionObject.cpp -o ./objects/CollisionObject.o

clean:
	@rm -f -R ./objects