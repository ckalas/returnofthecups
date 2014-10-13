CFLAGS = -c 
CFLAGS +=  $(shell pkg-config --cflags opencv)
LIBS = $(shell pkg-config --libs opencv)

all: vid_cap	

vid_cap: vid_cap.cpp
	g++ vid_cap.cpp -o vid_cap $(shell pkg-config --cflags --libs opencv)

clean:
	rm vid_cap 
