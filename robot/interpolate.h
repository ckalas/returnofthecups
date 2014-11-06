#ifndef INTERPOLATE_H
#define INTERPOLATE_H


#include <iostream>
#include <vector>
#include <cmath>

#define SAMPLES 10
#define HEIGHT 250 // temporary fixed height
#define WRIST_OFFSET 30.0
#define VELOCITY 5 // mm per second
#define UPDATE_INTERVAL 10 // 10 ms

using namespace std;

int calculate_samples(vector<double> start, vector<double> end);

vector<double> linspace(double a, double b, int n);
void print_vector (vector<double> *print);
void print_vector( vector< vector<double> > *print);
void print_vector( vector< vector<int> > *print);

void interpolate( vector< vector<int> > *pathGen, vector< vector<double> > *mainPos , int samples );
void pick_up_cup( vector< vector<int> > *interpolatedPath, vector<double> *coords, vector<int> *curr_coords );

void rad2bits( vector<double> *radian, vector<double> *bits );

#endif
