all: main

main: one two

one: one.o
	g++ -std=c++11 one.o -o one -g
two: two.o
	g++ -std=c++11 two.o -o two -g

%o:%cpp
