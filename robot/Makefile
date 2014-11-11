CFLAGS = $(shell pkg-config --cflags opencv)
LIBS = $(shell pkg-config --libs opencv)

all: main

run: main
	sudo ./robot

main: main.o multi_motor.o dynamixel.o dxl_hal.o ikine.o fkine.o control_and_input.o interpolate.o
	g++ $+ -o robot

%.o: %.cpp
	$(CXX) -std=c++11 -c $<

%.o: %.c
	$(CXX) -c $<

clean:
	rm main *.o ~*
	git rm -r --cached ../.metadata/*
