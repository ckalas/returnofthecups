#ifndef RECTANGLE_H
#define RECTANGLE_H

class Rectangle {
	int width, height;

public:
	void set_values (int, int);
	int area() { return width * height; }
};


#endif