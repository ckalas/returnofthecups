CFLAGS = --std=c++11 -fPIC -g 
CFLAGS += $(shell pkg-config --cflags opencv libfreenect libusb-1.0)
LIBS = --std=c++11 -fPIC -g  $(shell pkg-config --libs opencv libfreenect libusb-1.0)

all: main
	./main

main: main.o device.o calibration.o detect_markers.o
	$(CXX) $+ -o $@ $(LIBS) -g
%.o: %.cpp
	$(CXX) -c $(CFLAGS) $< -o $@

clean:
	rm -rf *.o main *# *~
