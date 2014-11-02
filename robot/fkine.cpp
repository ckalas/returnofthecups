#include "fkine.h"
#include "ikine.h"

void fkine(vector<double> *angles) {
	double x, y, z;

	y = L2 * cos(angles->at(1)) + L3 * cos(angles->at(1) + angles->at(2));
	z = L2 * sin(angles->at(1)) + L3 * sin(angles->at(1) + angles->at(2));
	// x need to be calculated last, DO NOT CHANGE
	x = y * tand(angles->at(0));
}

void print_angle(vector<int> *fkine_vector) {
	cout << fkine_vector->at(0) << ", "
			<< fkine_vector->at(1) << ", "
			<< fkine_vector->at(2) << ", "
			<< fkine_vector->at(3) << endl;
}

void print_angle(vector<double> *fkine_vector) {
	cout << fkine_vector->at(0) << ", "
			<< fkine_vector->at(1) << ", "
			<< fkine_vector->at(2) << ", "
			<< fkine_vector->at(3) << endl;
}

void bits_to_degree(vector<double> *degrees, vector<int> *bits) {
	degrees->at(0) = (bits->at(0) - 2096.0)  /  2096.0 * 180.0 * -1; // just inverse value
	degrees->at(1) = (bits->at(1) - 512.0) / 512 * 150.0 * -1;
	degrees->at(2) = (bits->at(2) - 512.0) / 512 * 150.0 * -1;
	degrees->at(3) = (bits->at(3) - 512.0) / 512 * 150.0;
}
