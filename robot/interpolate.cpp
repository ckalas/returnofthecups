#include "interpolate.h"
#include "ikine.h"
#include "fkine.h"
#include "multi_motor.h"  // used to find the bits conversion


vector<double> linspace(double a, double b, int n) {
    vector<double> array;
    double step = (b-a) / (n-1);

    for(int i=0; i<n; i++) {
	array.push_back(a);
	a += step;
    }

    return array;
}

int calculate_samples (vector<double> start, vector<double> end) {
    vector<double> coords_start (4);
    vector<double> coords_end (4);

    bits_to_degree( &start, &coords_start);
    bits_to_degree( &end, &coords_end );

    double mag_start = sqrt( pow(coords_start.at(0), 2) +
			     pow(coords_start.at(1), 2) +
			     pow(coords_start.at(2), 2) );

    double mag_end = sqrt( pow(coords_end.at(0), 2) +
			  pow(coords_end.at(1), 2) +
			  pow(coords_end.at(2), 2) );
    
    /*
    cout << "mag_start = " << mag_start << endl
	 << "mag_end = " << mag_end << endl;
    */

    return int( abs(mag_start - mag_end) / VELOCITY * 1000 / UPDATE_INTERVAL );
    //return 0;
}	      

void interpolate(vector<vector<int>> *pathGen, vector< vector<double>> *mainPos) {
    
    vector<double> x,y,z,g;
    vector<int> temp (4);

    for(int i = 0 ; i<mainPos->size()-1; i++) {
    
	x = linspace(mainPos->at(i).at(0), mainPos->at(i+1).at(0), SAMPLES);
	y = linspace(mainPos->at(i).at(1), mainPos->at(i+1).at(1), SAMPLES);
	z = linspace(mainPos->at(i).at(2), mainPos->at(i+1).at(2), SAMPLES);
	g = linspace(mainPos->at(i).at(3), mainPos->at(i+1).at(3), SAMPLES);

	for (int j = 0; j<SAMPLES; j++) {

	    temp = { int(x.at(j)), int(y.at(j)), int(z.at(j)), int(g.at(j)) };
	    pathGen->push_back(temp);
	}

	//cout << endl;
    }
}

bool generate_path( vector<vector<int>> *interpolatedPath, vector<double> *coords,vector<int> *curr_coords, int grip) {
    vector< vector<double> > generalPath;
    vector<double> temp (4);
    vector<double> angles (4);

    // clear the current list
    interpolatedPath->clear();

    temp = { double(curr_coords->at(0)), double(curr_coords->at(1)),
	     double(curr_coords->at(2)), double(curr_coords->at(3)) };
    generalPath.push_back( temp );
        
    temp = {coords->at(0), coords->at(1), coords->at(2), coords->at(3)};
    if (!ikine(&temp, &temp, grip)) {
	   cerr << "Ikine not happy" << endl;
	   //cout << 1 << endl;
       return false;
    }
    rad2bits(&temp, &temp);
    generalPath.push_back( temp );
    
    print_vector(&generalPath);
    
    interpolate(interpolatedPath, &generalPath);
    return true;
}

void rad2bits( vector<double> *radian, vector<double> *bits) {
    //radian->at(2) -= WRIST_OFFSET / 180 * M_PI;
    bits->at(0) = mx12w_angle2bits(radian->at(0));
    bits->at(1) = ax12a_angle2bits(radian->at(1));
    bits->at(2) = ax12a_angle2bits(radian->at(2));
    bits->at(3) = ax12a_angle2bits(radian->at(3));
}

void print_vector (vector<double> *print) {
    for (vector<double>::const_iterator i = print->begin(); i != print->end(); ++i) {
    cerr << *i << ", ";    
    }
    cerr << endl << endl;
}

void print_vector( vector< vector<double> > *print) {
    for(vector< vector<double> >::const_iterator i = print->begin(); i!= print->end(); ++i) {
    cerr<< "[ ";
    for(int j = 0; j<4; j++) {
        cerr << (*i).at(j) << " ";
    }
    cerr << "]" << endl;
    }
}

void print_vector( vector< vector<int> > *print) {
    for(vector< vector<int> >::const_iterator i = print->begin(); i!= print->end(); ++i) {
    cerr << "[ ";
    for(int j = 0; j<4; j++) {
        cerr << (*i).at(j) << " ";
    }
    cerr << "]" << endl;
    }
}
