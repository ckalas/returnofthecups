#ifndef IKINE_H
#define IKINE_H

#include <math.h>
#include <iostream>
#include <vector>

// The following are in mm units:
#define L1			156
#define L2			95  //80
#define L3			115 //80
//#define L4			10

#define THETA_A_OFFSET	0
#define THETA_B_OFFSET	0
#define THETA_C_OFFSET 	0

using namespace std;

//void ikine(vector<double> coords);
//void ikine(void);
void ikine( vector<double> coords, vector<double> * angles );
void print_values( vector<double>* values );

#endif
