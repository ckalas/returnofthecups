#include "ikine.h"
#include "fkine.h"

// Input x, y ,z and the angle vector returns the values to input to the  motors
bool ikine(vector<double> coords, vector<double>* angles) {
	//cout << endl << "Andy Ikine says, hello world" << endl << endl;

	double x = coords[0];
	double y = coords[1];
	double z = coords[2] - L1;

	// checks if the given xyz coords is out of range
	
//	if ( sqrt(pow(y,2) + pow(z,2)) > (L2 + L3) ||
//		 sqrt(pow(y,2) + pow(x,2)) > (L2 + L3) ||
//		 sqrt(pow(x,2) + pow(z,2)) > (L2 + L3) ||

	// calculate the closes reach
	double reach_limit = L1*cosd(90) + L2*cosd(90 + (asin((-L1*sind(90))/L2) * 180 / M_PI) );
	cout << "Min reach: " << reach_limit << endl;

	// Note that the dynamixel and rotate 150(degree) from origin
	if ( sqrt( pow(x,2) + pow(y,2) + pow(z,2) ) < (L2 + L3) ||
		 reach_limit > y || z < 10) {

		cout << "Out of reach" << endl;
		
//		angles->at(0) = 0;
//		angles->at(1) = 0;
//		angles->at(2) = 0;

		return false;
	}
	
	// (horizontal, vertical) theta A
	angles->at(0) = atan2(x,y); 

	// converted hypotenuse as the new y value 
	y = y / cos(angles->at(0)); 

	// equation from the online source
	double temp1 = (pow(y,2) + pow(z,2) - pow(L2,2) - pow(L3,2)) / (2 * L2 * L3);
	double temp2 = -sqrt( 1 - pow(temp1, 2) );
	
	// theta C
	angles->at(2) = atan2( temp2, temp1);


	double k1 = L2 + L3 * cos(angles->at(2));
	double k2 = L3 * sin(angles->at(2));
	
	// theta B
	angles->at(1) = atan2( z, y ) - atan2( k2, k1 );

	//print_values(angles);
	return true;
}

void print_values( vector<double>* values) {
	cout << "Print value: " << endl;

	for( int i = 0; i < values->size(); i++ ) {
		cout << values->at(i) / M_PI * 180 << ", ";
	}
	cout << endl << endl;
}
