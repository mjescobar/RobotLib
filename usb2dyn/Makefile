CC = g++ -O3
VPATH = ./src ./headers 
CFLAGS = -g -Wall -fPIC -I./src -I./headers -I./objects -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255
LDFLAGS = -lpthread

all: usb2dyn.o 

usb2dyn.o: 
	@mkdir -p objects
	@echo "Compiling usb2dyn.cpp to usb2dyn.o"
	@$(CC) $(CFLAGS) -c ./src/usb2dyn.cpp -o ./objects/usb2dyn.o


clean:
	@rm -f -R ./objects