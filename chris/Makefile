all: main

main: main.o multi_dnmx_motor.o dynamixel.o dxl_hal.o
	$(CXX) main.o multi_dnmx_motor.o dynamixel.o dxl_hal.o -o main 

main.o: main.cpp
	$(CXX) -c main.cpp

multi_dnmx_motor.o: multi_dnmx_motor.cpp
	$(CXX) -c multi_dnmx_motor.cpp

dynamixel.o: dynamixel.c
	$(CC) -c dynamixel.c

dxl_hal.o: dxl_hal.c
	$(CC) -c dxl_hal.c

clean:
	rm main *.o
