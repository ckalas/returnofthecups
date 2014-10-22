#include "ikine.h"

using namespace std;

void ikine(void) {

	vector<double> coords;
	vector<double>::const_iterator i;

	coords.push_back(3.14);
	coords.push_back(1);

	cout << endl << "Ikine says, hello world" << endl << endl;

	cout << "Vector: " << coords[0] << endl;

	/*
	for(i = coords.begin(); i!= coords.end(); i++) {
		cout << (*i) << endl;
	}
	*/
}