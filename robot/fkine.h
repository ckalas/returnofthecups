#ifndef FKINE_H
#define FKINE_H


#include <iostream>
#include <vector>

#define sind(x) (sin(fmod((x),360) * M_PI / 180))
#define cosd(x) (cos(fmod((x),360) * M_PI / 180))
#define tand(x) (tan(fmod((x),360) * M_PI / 180))

using namespace std;

void fkine(vector<double> *angles, vector<double> *coords);
void print_angle(vector<int> *fkine_vector);
void print_angle(vector<double> *fkine_vector);

void bits_to_degree(vector<double> *degrees, vector<int> *bits);


#endif
