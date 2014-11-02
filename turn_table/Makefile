all: maestro

run:
	sudo ./maestro
	
maestro: maestro.o
	gcc maestro.o -o maestro
maestro.o: maestro.c
	gcc -c maestro.c

clean:
	rm maestro *.o
