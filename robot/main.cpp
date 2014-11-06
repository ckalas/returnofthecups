#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <math.h>
#include "multi_motor.h"
#include "ikine.h"
#include "fkine.h"
#include "control_and_input.h"
#include "interpolate.h"
#include <stdio.h>

#define GRIP_ANGLE 65
#define WRIST_OFFSET 30.0
#define ELBOW_OFFSET 0 //2.0

using namespace std;
using namespace cv;

void print_coords(vector<double> *coords) {
    cout << "x: " << coords->at(0) << ", y: "
	 << coords->at(1) << ", z: " << coords->at(2)
	 << ", grip: " << coords->at(3) << endl;    
}

int main( int argc, char *argv[] ) 
{
    cout << "hello world" << endl;

    //cout << "sqrt: " << sqrt(9) << endl;
    vector<int> fkine_vector (4);
    vector<double> angles (4);
    vector<double> coords (4);
    vector<double> curr_coords (4);
    vector<int> motor_bit_angle (4);
    coords.at(0) = 0;
    coords.at(1) = (L2 + L3)/2;
    coords.at(2) = L1;
    coords.at(3) = 0;

    // path generation variables
    vector< vector<int> > pathGen; // used to storage interpolated values of bits

	
    CMulti_DNMX_Motor Motors;

    int a = 512;
    vector<int> goal_pos(4);
    goal_pos.at(0)= 2096;
    goal_pos.at(1) = a; //205;
    goal_pos.at(2)= a; //650;
    goal_pos.at(3) = 800;

    int curr_pos[4] ={0,0,0,0};

    // init with baud 1Mbps57k, refer to bauds.txt for mapping

    Motors.initialization(1);
    Motors.set_torque(1023);
    Motors.set_speed(100);
    Motors.read_speed();

    Motors.move_to_goal_pos(&goal_pos, curr_pos);





    /**
     * From your view:
     *      left vs right is the x direction
     *      closer vs further is the y direction
     *      up vs down is the z direction
     */
    while (1) {

	/** 
	 * Must note that the first generated path will slightly vary
	 * due to it going above the cup
	 */  

	/* INCOMPLETE
	input_coords(&coords);
	motor_bit_angle = goal_pos;
	//Motors.read_motor_angles(&motor_bit_angle);
	pick_up_cup( &pathGen, &coords, &motor_bit_angle);
	//print_vector( &pathGen );
	cout << pathGen.size() << endl;
	
	for (int i = 0; i<pathGen.size(); i++) {
	    //cout << "moving loop " << endl;
	    Motors.move_to_goal_pos( &pathGen.at(i), curr_pos );
	    usleep(UPDATE_INTERVAL*1000); // micro second updating
	} 
	*/


    	/**
    	 * This section is for the inverse kinematics
    	 */
	// uncomment either input_coords or game_control

	input_coords(&coords);
	//game_control(&coords);

	if ( !(ikine(&coords, &angles)) ) 
	    continue;

	goal_pos[0] = mx12w_angle2bits(angles[0]);
	goal_pos[1] = ax12a_angle2bits(angles[1]);
	goal_pos[2] = ax12a_angle2bits(angles[2]);
	goal_pos[3] = ax12a_angle2bits(angles[3]);
	
	print_angle(&angles);
	print_angle(&coords);
	print_angle(&goal_pos);
	Motors.move_to_goal_pos( &goal_pos, curr_pos);

    	/**
    	 * This section is for the foward kinematics
    	 */
	//Motors.no_torque_generate();

	/*
	if ( 1 ) {
	    cout << "Reading fkine" << endl;
	    //Motors.read_motor_angles(&fkine_vector);

	    //bits_to_degree(&fkine_vector, &coords);

	    //print_angle(&angles);
	    fkine(&angles, &coords);
	    print_coords(&coords);
	    cout << endl;
	}
	*/
	
    }

    Motors.no_torque_generate();
    Motors.set_torque(0);

    dxl_terminate();
    cout << "dxl_terminate" << endl;

    return 0;
}
