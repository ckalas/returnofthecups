#ifndef CONTROL_AND_INPUT_H
#define CONTROL_AND_INPUT_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <vector>

using namespace std;

#define DIST_MOVE 5

int getch(void);
void game_control(vector<double> *coords);
bool input_coords(vector<double> *coords);


#endif
