#ifndef CONTROL_AND_INPUT_H
#define CONTROL_AND_INPUT_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <vector>
#include "multi_motor.h"

using namespace std;

#define DIST_MOVE 5

int getch(void);
void game_control(vector<double> *coords);
bool input_coords(vector<double> *coords);
void set_goals(vector<int> *goal, vector<double> angles);
bool get_motor_angles(vector<int> *motor_bit_angle,CMulti_DNMX_Motor *Motors);

#endif
