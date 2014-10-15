all: main
	sudo ./main

main: main.o multi_dnmx_motor.o dynamixel.o dxl_hal.o
	g++ main.o multi_dnmx_motor.o dynamixel.o dxl_hal.o -o main 

main.o: main.cpp
	g++ -c main.cpp

multi_dnmx_motor.o: multi_dnmx_motor.cpp
	g++ -c multi_dnmx_motor.cpp

dynamixel.o: dynamixel.c
	g++ -c dynamixel.c

dxl_hal.o: dxl_hal.c
	g++ -c dxl_hal.c

clean:
	rm main *.o
