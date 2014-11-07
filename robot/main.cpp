#include <iostream>
#include <math.h>
#include "ikine.h"
#include "fkine.h"
#include "control_and_input.h"
#include "interpolate.h"


#define GRIP_ANGLE 65
#define WRIST_OFFSET 30.0
#define ELBOW_OFFSET 0 //2.0

using namespace std;

enum state_t {GO_TO_CUP, GRIP, MOVE_AWAY, DROP, RESET};


void print_coords(vector<double> *coords) {
    cout << "x: " << coords->at(0) << ", y: "
	 << coords->at(1) << ", z: " << coords->at(2)
	 << ", grip: " << coords->at(3) << endl;    
}

int main( int argc, char *argv[] ) 
{

    vector<int> fkine_vector (4);
    vector<double> angles (4);
    vector<double> coords (4);
    vector<double> curr_coords (4);
    vector<int> motor_bit_angle (4);
    coords.at(0) = 0;
    coords.at(1) = (L2 + L3)/2;
    coords.at(2) = L1;
    coords.at(3) = 0;
    bool validRead;

    // path generation variables
    vector< vector<int> > pathGen;
    vector< vector<double> > generalPath; // used to storage interpolated values of bits

    CMulti_DNMX_Motor Motors;

    vector<int> goal_pos(4);
    goal_pos.at(0)= 2096;
    goal_pos.at(1) = 512; //205;
    goal_pos.at(2)= 512; //650;
    goal_pos.at(3) = 800;

    int curr_pos[4] ={0,0,0,0};

    // init with baud 1Mbps, refer to bauds.txt for mapping

    Motors.initialization(1);
    Motors.set_torque(1023);
    Motors.set_speed(100);

    Motors.move_to_goal_pos(&goal_pos, curr_pos);

    state_t state = GO_TO_CUP;
    bool finished = false;

    usleep(1000000);


    // Main program loop

    // NOTES TO THE READER: curr_pos is not used at all, probably get rid of it.

    while (!finished) {

		switch (state) {
			// Wait for input and move to cup with open gripper
			case GO_TO_CUP:
				validRead = false;
				if(input_coords(&coords)){
					// Ensure the current motor position is a valid result
					while(!get_motor_angles(&motor_bit_angle, &Motors));
					if(!point_to_point(&pathGen, &coords, &motor_bit_angle, OPEN)) {
						break;
					}
					// Perform the interpolation
					for (size_t i = 0; i<pathGen.size(); i++) {
					    Motors.move_to_goal_pos( &pathGen.at(i), curr_pos );
					    usleep(UPDATE_INTERVAL);
					}
					// Go to next state
					state = GRIP;
				}
				else {
					finished = true;
				}
				break;
			// Actuate the gripper to hold the cup
			case GRIP:
				ikine(&coords, &angles, CLOSED);
				set_goals(&goal_pos, angles);
				Motors.move_to_goal_pos(&goal_pos, curr_pos);
				state = MOVE_AWAY;
				break;
			// Wait for input and move the cup there
			case MOVE_AWAY:
				validRead = false;
				if(input_coords(&coords)){
					// Ensure the current motor position is a valid result
					while(!get_motor_angles(&motor_bit_angle, &Motors));
					if(!point_to_point(&pathGen, &coords, &motor_bit_angle, CLOSED)) {
						break;
					}
					// Perform the interpolation
					for (size_t i = 0; i<pathGen.size(); i++) {
					    Motors.move_to_goal_pos( &pathGen.at(i), curr_pos );
					    usleep(UPDATE_INTERVAL);
					}
					// Go to next state
					state = DROP;
				}
				else {
					finished = true;
				}
				break;
			// Release cup
			case DROP:
				ikine(&coords, &angles, OPEN);
				set_goals(&goal_pos, angles);
				Motors.move_to_goal_pos(&goal_pos, curr_pos);
				state = RESET;
				break;

			case RESET:
				usleep(2000000);
				finished = true;
				break;


		}
	
    }

    Motors.no_torque_generate();
    Motors.set_torque(0);
    dxl_terminate();
    cout << "Successfully exited program" << endl;

    return 0;
}
