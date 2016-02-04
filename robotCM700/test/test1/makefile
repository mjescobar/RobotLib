CC = g++ -O3 -std=c++11
VPATH = .:../../src:../../headers:../../objects
CFLAGS = -g -Wall -fPIC -I../../src -I../../headers -I../../objects -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255
LDFLAGS = -lpthread

test: test.cpp 
	@$(CC) $(CFLAGS)  test.cpp ../../objects/cm700.o ../../objects/dynamixel.o ../../objects/dxl_hal.o -o testApp

run:
	./testApp
  