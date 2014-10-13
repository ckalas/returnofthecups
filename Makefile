all: main

CFLAGS=-fPIC -g -Wall  $(shell pkg-config --cflags opencv libusb) 
LIBS = $(shell pkg-config --libs opencv)

# INCLUDE = -I /usr/local/include/libfreenect
# FREE_LIBS = -L/usr/local/lib -lfreenect

INCLUDE = $(shell pkg-config --clfags libfreenect )
FREE_LIBS = $(shell pkg-config --libs libfreenect)


main: device.cpp cups.cpp main.cpp 
	$(CXX) $(INCLUDE) $(CFLAGS) $? -o $@  $(LIBS) $(FREE_LIBS) 

%.o: %.cpp
	$(CXX) -c $(CFLAGS) $< -o $@

clean:
	rm -rf *.o main
