#include "fkine.h"
#include "ikine.h"

/**
 * input angles in degrees NOT RADIANS
 */
void fkine(vector<double> *angles, vector<double> *coords) {
	double x, y, z;

	double theta0, theta1, theta2;
	theta0 = angles->at(0);
	theta1 = angles->at(1); // 40 is constant angle offset
	theta2 = angles->at(2);

	theta1 += 90; //M_PI / 4;
	theta2 -= 60; //M_PI / 3;

	cerr << "theta0: " << theta0 << endl
	     << "theta1: " << theta1 << endl
	     << "theta2: " << theta2 << endl;


	y = L2 * cosd(theta1) + L3 * cosd(theta1 + theta2);
	z = L2 * sind(theta1) + L3 * sind(theta1 + theta2);
	// x need to be calculated last, DO NOT CHANGE
	x = y * tand(angles->at(0));

	coords->at(0) = x;
	coords->at(1) = y;
	coords->at(2) = z + L1;
}

void print_angle(vector<int> *fkine_vector) {
	cerr << fkine_vector->at(0) << ", "
			<< fkine_vector->at(1) << ", "
			<< fkine_vector->at(2) << ", "
			<< fkine_vector->at(3) << endl;
}

void print_angle(vector<double> *fkine_vector) {
	cerr << fkine_vector->at(0) << ", "
			<< fkine_vector->at(1) << ", "
			<< fkine_vector->at(2) << ", "
			<< fkine_vector->at(3) << endl;
}

void bits_to_degree(vector<double> *bits, vector<double> *degrees) {
	degrees->at(0) = (bits->at(0) - 2096.0)  /  2096.0 * 180.0 * -1; // just inverse value
	degrees->at(1) = (bits->at(1) - 512.0) / 512 * 150.0 * -1;
	degrees->at(2) = (bits->at(2) - 512.0) / 512 * 150.0 * -1;
	degrees->at(3) = (bits->at(3) - 512.0) / 512 * 150.0;
}
