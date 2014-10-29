CFLAGS = $(shell pkg-config --cflags opencv) -g

all: main
	sudo ./main

main: main.o multi_motor.o dynamixel.o dxl_hal.o ikine.o
	g++ main.o multi_motor.o dynamixel.o dxl_hal.o ikine.o -o main $(LIBS)

main.o: main.cpp
	g++ $(CFLAGS) -c main.cpp

multi_motor.o: multi_motor.cpp
	g++ -c multi_motor.cpp

dynamixel.o: dynamixel.c
	g++ -c dynamixel.c

dxl_hal.o: dxl_hal.c
	g++ -c dxl_hal.c

ikine.o: ikine.cpp
	g++ -c ikine.cpp

clean:
	rm main *.o *~ *#
