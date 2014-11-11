#ifndef INTERPOLATE_H
#define INTERPOLATE_H


#include <iostream>
#include <vector>
#include <cmath>

#define SAMPLES 100 // samples bewteen points
//#define HEIGHT 250 // temporary fixed height
#define WRIST_OFFSET 30.0
#define VELOCITY 5 // mm per second
//#define UPDATE_INTERVAL 100000 // 1/SAMPLES as micro seconds
#define UPDATE_INTERVAL 10000 / SAMPLES
using namespace std;

int calculate_samples(vector<double> start, vector<double> end);

vector<double> linspace(double a, double b, int n);
void print_vector (vector<double> *print);
void print_vector(vector<vector<double>> *print);
void print_vector(vector<vector<int>> *print);

void interpolate( vector< vector<int> > *pathGen, vector< vector<double> > *mainPos);
bool generate_path(vector<vector<int>> *interpolatedPath, vector<double> *coords, vector<int> *curr_coords, int grip);

void rad2bits(vector<double> *radian, vector<double> *bits);

#endif
