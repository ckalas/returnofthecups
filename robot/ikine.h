#ifndef IKINE_H
#define IKINE_H

#include <math.h>
#include <iostream>
#include <vector>

// The following are in mm units:
#define L1			150 //156
#define L2			175
#define L3			170 // sum = 94 + 165 = 259
//#define L4			10

#define THETA_A_OFFSET	0
#define THETA_B_OFFSET	0
#define THETA_C_OFFSET 	0

#define GRIP_ANGLE 82

#define OPEN 1
#define CLOSED 0

using namespace std;

//void ikine(vector<double> coords);
//void ikine(void);
bool ikine(vector<double> *coords, vector<double> * angles, int grip);
void print_values(vector<double> *values);
bool check_angle_range(vector<double> *angles);

#endif
