LIBS = -L../lib/libdxl
INCLUDE = -I../include

all: main
	./main

main: main.o
	g++ main.o -o main $(LIBS)

main.o: main.cpp
	g++ -c $(INCLUDE) main.cpp

clean:
	rm main *.o
