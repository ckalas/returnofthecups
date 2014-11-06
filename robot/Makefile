CFLAGS = $(shell pkg-config --cflags opencv)
LIBS = $(shell pkg-config --libs opencv)

all: main

run:
	sudo ./main

main: main.o multi_motor.o dynamixel.o dxl_hal.o ikine.o fkine.o control_and_input.o interpolate.o
	g++ $+ -o main $(LIBS)

%.o: %.cpp
	$(CXX) -std=c++11 $(CFLAGS) -c $<

%.o: %.c
	$(CXX) $(CFLAGS) -c $<

clean:
	rm main *.o *~
	git rm -r --cached ../.metadata/*
