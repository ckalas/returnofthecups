#include <iostream>
#include <vector>

#define SAMPLES 10

using namespace std;

vector<double> linspace(double a, double b, int n) {
    vector<double> array;
    double step = (b-a) / (n-1);

    for(int i=0; i<n; i++) {
	array.push_back(a);
	a += step;
    }

    /*
    if (array.size() > n) 
	array.pop_back();
    */

    return array;
}

void print_vector (vector<double> print) {
    for (vector<double>::const_iterator i = print.begin(); i != print.end(); ++i) {

	cerr << *i << ", ";
    }
    cerr << endl;
}

void print_vector( vector< vector<double> > *print) {
    for(vector< vector<double> >::const_iterator i = print->begin(); i!= print->end(); ++i) {
	cerr << "[ ";
	for(int j = 0; j<4; j++) {
	    cerr << (*i).at(j) << " ";
	}
	cerr << "]" << endl;
    }
}

void interpolate( vector< vector<double> > *pathGen, vector< vector<double> > *mainPos ) {
    cerr<< "In interpolate" << endl;

    vector<double> x,y,z,g, temp(4);

    for(int i = 0 ; i<mainPos->size()-1; i++) {

	cerr << "loop: " << i << endl;
	cerr<< "start x: " << mainPos->at(i).at(0) << endl;	
	cerr << "end x : " << mainPos->at(i+1).at(0) << endl;

	x = linspace(mainPos->at(i).at(0), mainPos->at(i+1).at(0), SAMPLES);
	y = linspace(mainPos->at(i).at(1), mainPos->at(i+1).at(1), SAMPLES);
	z = linspace(mainPos->at(i).at(2), mainPos->at(i+1).at(2), SAMPLES);
	g = linspace(mainPos->at(i).at(3), mainPos->at(i+1).at(3), SAMPLES);
	
	for (int j = 0; j<SAMPLES; j++) {

	    temp = { x.at(j), y.at(j), z.at(j), g.at(j) };
	    pathGen->push_back(temp);
	}

	cerr << endl;
    }
}

int main (int argc, char **argv) {

    vector< vector<double> > pathGen;
    vector< vector<double> > generalPath;
    vector<double> temp (4);

    temp = {0.0, 0.0, 0.0, 0.0}; // current position
    generalPath.push_back( temp );
    
    temp = {100.0, 20.0, 10.0, 1.0}; // move to 1    
    generalPath.push_back( temp );

    temp = {-100.0, 200.0, 100.0, 0.0};
    generalPath.push_back( temp );

    cout << generalPath.size() << endl;

    interpolate(&pathGen, &generalPath);
    // to pass pathGen and get the bit positions

    cout << endl; 

    print_vector(&pathGen);

    return 0;
}
