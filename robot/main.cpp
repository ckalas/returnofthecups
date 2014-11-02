#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <math.h>
#include "multi_motor.h"
#include "ikine.h"
#include "fkine.h"
#include "control_and_input.h"
#include <stdio.h>

#define GRIP_ANGLE 45

using namespace std;
using namespace cv;

int main( int argc, char *argv[] ) 
{
    cout << "hello world" << endl;

    //cout << "sqrt: " << sqrt(9) << endl;
    vector<int> fkine_vector (4);
    vector<double> angles (4);
    vector<double> coords (4);
    coords.at(0) = 0;
    coords.at(1) = (L2 + L3)/2;
    coords.at(2) = L1;
	
    print_values(&coords);

    CMulti_DNMX_Motor Motors;

    int a = 512;
    int goal_pos[4] = {2096, a, a, a};//(a + (1024*(12/150.0))), a};
    int curr_pos[4] ={0,0,0,0};

    cout << goal_pos << endl;

    // init with baud 57k, refer to bauds.txt for mapping
    Motors.initialization(34);
    Motors.set_torque(1023);    
    Motors.set_speed(80);
    Motors.read_speed();
    
    //ikine(coords, &angles);

    Motors.move_to_goal_pos(goal_pos, curr_pos);
        
    /**
     * From your view:
     *      left vs right is the x direction
     *      closer vs further is the y direction
     *      up vs down is the z direction
     */
    while (1) {

    	/**
    	 * This section is for the inverse kinematics
    	 */


		// uncomment either input_coords or game_control
		//input_coords(&coords);
		game_control(&coords);

		cout << "x: " << coords.at(0) << ", y: "
			 << coords.at(1) << ", z: " << coords.at(2)
			 << ", grip: " << coords.at(3) << endl;

		ikine(coords, &angles);

		if(coords.at(3) == 1) {
			angles[3] = -GRIP_ANGLE / 180.0 * M_PI;
		}
		else {
			angles[3] = GRIP_ANGLE / 180.0 * M_PI;
		}

		angles[2] -= 22.0 / 180 * M_PI;

		goal_pos[0] = Motors.mx12w_angle2bits(angles[0]);
		goal_pos[1] = Motors.ax12a_angle2bits(angles[1]);
		goal_pos[2] = Motors.ax12a_angle2bits(angles[2]);
		goal_pos[3] = Motors.ax12a_angle2bits(angles[3]);

		Motors.move_to_goal_pos( goal_pos, curr_pos);

    	/**
    	 * This section is for the foward kinematics
    	 */

//    	Motors.no_torque_generate();
//    	Motors.set_torque(0);
//    	Motors.read_motor_angles(&fkine_vector);
//    	bits_to_degree(&angles, &fkine_vector);
//    	//print_angle(&fkine_vector);
//    	//print_angle(&angles);
//    	fkine(&angles);
    }
    
	Motors.no_torque_generate();
    Motors.set_torque(0);

    dxl_terminate();
    cout << "dxl_terminate" << endl;
	
    return 0;
}
