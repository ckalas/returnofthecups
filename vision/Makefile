all: main

OS := $(shell uname)
ifeq ($(OS), Darwin)
	CFLAGS=-fPIC -g -Wall  $(shell pkg-config --cflags opencv) -I /usr/local/include/libusb-1.0/ 
	LIBS = $(shell pkg-config --libs opencv)
	INCLUDE = -I /usr/local/include/libfreenect
	FREE_LIBS = -L/usr/local/lib -lfreenect
else
	CFLAGS=-fPIC -g -Wall  $(shell pkg-config --cflags opencv libusb)  -I /usr/local/include/libusb-1.0/ 
	LIBS = $(shell pkg-config --libs opencv)
	INCLUDE = $(shell pkg-config --clfags libfreenect )
	FREE_LIBS = $(shell pkg-config --libs libfreenect)
endif


main: device.cpp cups.cpp main.cpp 
	$(CXX) $(INCLUDE) $(CFLAGS) $? -o $@  $(LIBS) $(FREE_LIBS) 

%.o: %.cpp
	$(CXX) -c $(CFLAGS) $< -o $@

clean:
	rm -rf *.o main
