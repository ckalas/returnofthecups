CFLAGS = $(shell pkg-config --cflags opencv)
LIBS = $(shell pkg-config --libs opencv)

all: main
	sudo ./main

main: main.o multi_motor.o dynamixel.o dxl_hal.o
	g++ main.o multi_motor.o dynamixel.o dxl_hal.o -o main $(LIBS)

main.o: main.cpp
	g++ $(CFLAGS) -c main.cpp

multi_motor.o: multi_motor.cpp
	g++ -c multi_motor.cpp

dynamixel.o: dynamixel.c
	g++ -c dynamixel.c

dxl_hal.o: dxl_hal.c
	g++ -c dxl_hal.c

clean:
	rm main *.o
