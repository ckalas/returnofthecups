all: main

run: main
	sudo ./main

OS := $(shell uname)
ifeq ($(OS), Darwin)
	CFLAGS= -std=c++11 -fPIC -g -Wall  `pkg-config --cflags opencv` -I /usr/local/include/libusb-1.0/ 
	LIBS = `pkg-config --libs opencv`
	INCLUDE = -I /usr/local/include/libfreenect
	FREE_LIBS = -L/usr/local/lib -lfreenect
else
	CFLAGS= -std=c++11 -fPIC -g -Wall  $(shell pkg-config --cflags opencv libusb-1.0)  -I/usr/local/include/libusb-1.0/ 
	LIBS = $(shell pkg-config --libs opencv)
	INCLUDE = $(shell pkg-config --cflags libfreenect )
	FREE_LIBS = $(shell pkg-config --libs libfreenect)
endif


main: fiducial.o device.o cups.o main.o
	$(CXX) $(INCLUDE) $(CFLAGS) $+ -o $@ $(LIBS) $(FREE_LIBS)

test: trackTest.o
	$(CXX) $(INCLUDE) $(CFLAGS) $+ -o $@ $(LIBS) $(FREE_LIBS)

%.o: %.cpp
	$(CXX) -c $(CFLAGS) $< -o $@

clean:
	rm -rf *.o main
