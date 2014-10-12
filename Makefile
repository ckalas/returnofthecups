all: main

CFLAGS=-fPIC -g -Wall `pkg-config --cflags opencv` -I /usr/local/include/libusb-1.0/
LIBS = `pkg-config --libs opencv`
INCLUDE = -I /usr/local/include/libfreenect
FREE_LIBS = -L/usr/local/lib -lfreenect

main: device.cpp cups.cpp main.cpp 
	$(CXX) $(INCLUDE) $(CFLAGS) $? -o $@  $(LIBS) $(FREE_LIBS) 

%.o: %.cpp
	$(CXX) -c $(CFLAGS) $< -o $@

clean:
	rm -rf *.o main