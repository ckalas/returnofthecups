#include "optimal_cup_range.h"



#define DIST_MAX sqrt( pow((L2+L3),2) - pow(HEIGHT,2) )
#define DIST_MIN 180
#define DIST_MED (DIST_MAX+DIST_MIN)/2.0

#define LARGE_CUP_R 45 // mm
#define MEDIUM_CUP_R 39 // mm

/**
 * @param: The input coordinates
 * @param: The return values
 * @param: If 1 cup is Large, if 0 cup is Medium
 */
bool get_optimal_pos(vector<double> *input, vector<double> *output, int cupSize) {
    double x = input->at(0); // cup x coords
    double y = input->at(1); // cup y coords
    double theta = atan2(y, x); // may need to switch parameters around
    
    double x_lower, x_upper, y_lower, y_upper, resultant_lower, resultant_upper, dif_lower, dif_upper;
    bool upperBound = false, lowerBound = false;

    if (cupSize) {
	x_lower = x - (LARGE_CUP_R * sin(theta));
	y_lower = y - (LRAGE_CUP_R * cos(theta)); 
	x_upper = x + (LARGE_CUP_R * sin(theta));
	y_upper = y + (LARGE_CUP_R * cos(theta));
    }
    else {
	x_lower = x - (MEDIUM_CUP_R * sin(theta));
	y_lower = y - (MEDIUM_CUP_R * cos(theta));
	x_upper = x + (MEDIUM_CUP_R * sin(theta));
	y_upper = y + (MEDIUM_CUP_R * cos(theta));
    }

    resultant_lower = sqrt( pow(x_lower,2) + pow(y_lower,2) );
    resultant_upper = sqrt( pow(x_upper,2) + pow(y_upper,2) );

    if ( resultant < DIST_MAX && resultant > DIST_MIN ) {
	// set true, upper radius in within bounds
	upperBound = true;
    }

    if ( resultant_lower < DIST_MAX && resultant_lower > DIST_MIN ) {
	// set true, lower radius in within bounds
	lowerBound = true;
    }

    // check which bound is closer to the median range
    if ( lowerBound && upperBound) {
	dif_lower = abs(resultant_lower - DIST_MED);
	dif_upper = abs(resultant_upper - DIST_MED);

	if (dif_lower < dif_upper) 
	    upperBound = false;
	else
	    lowerBound = false;
    }

    if ( upperBound ) {
	// set the upper bound coords
	output->at(0) = input->at(0) + x_upper;
	output->at(1) = input->at(1) + y_upper;
	output->at(2) = input->at(2);
	output->at(3) = input->at(3);
	return true;
    }
    else if ( lowerBound ) {
	// set the lower bound coords
	output->at(0) = input->at(0) + x_lower;
	output->at(1) = input->at(1) + y_lower;
	output->at(2) = input->at(2);
	output->at(3) = input->at(3);
	return true;
    }
    else {
	return false;
    }
}
	
    
