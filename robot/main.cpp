#include <iostream>
#include <math.h>
#include "ikine.h"
#include "fkine.h"
#include "control_and_input.h"
#include "interpolate.h"


#define WRIST_OFFSET 30.0
#define ELBOW_OFFSET 0 //2.0

using namespace std;

enum state_t {INIT, GO_TO_CUP, GRIP, UP, MOVE_AUTO, DROP, RESET};


void print_coords(vector<double> *coords) {
    cout << "x: " << coords->at(0) << ", y: "
	 << coords->at(1) << ", z: " << coords->at(2)
	 << ", grip: " << coords->at(3) << endl;    
}

int main(int argc, char **argv) {

    vector<int> fkine_vector (4);
    vector<double> angles (4);
    vector<double> coords (4);
    vector<double> autofill(4);
    vector<double> coaster(4);
    vector<double> curr_coords (4);
    vector<int> motor_bit_angle (4);
    coords.at(0) = 0;
    coords.at(1) = (L2 + L3)/2;
    coords.at(2) = L1;
    coords.at(3) = 0;
    bool validRead;

    // Path generation variables
    vector<vector<int>> pathGen;
    vector<vector<double>> generalPath; // used to storage interpolated values of bits

    CMulti_DNMX_Motor Motors;

    vector<int> goal_pos(4);
    goal_pos.at(0)= 2096;
    goal_pos.at(1) = 512; //205;
    goal_pos.at(2)= 512; //650;
    goal_pos.at(3) = 280;

    int curr_pos[4] ={0,0,0,0};

    // Init with baud 1Mbps, refer to bauds.txt for mapping

    Motors.initialization(1);
    Motors.set_torque(1023);
    Motors.set_speed(100);

    Motors.move_to_goal_pos(&goal_pos, curr_pos);

    

    state_t state = INIT;
    bool finished = false;

    usleep(1000000);


    // Main program loop

    // NOTES : curr_pos is not used at all, probably get rid of it.

    while (!finished) {

    	cout << "Current state: " << state << endl;

		switch (state) {
			// Init state - get location of various markers
			case INIT:
				cin >> autofill.at(0); //x
				autofill.at(0) = autofill.at(0) * 10; //scaling main is cm
				cin >> autofill.at(1); //y
				autofill.at(1) = autofill.at(1) * 10; //scaling main is cm
				cin >> autofill.at(2); //z
				autofill.at(2) = 90; //hardcord autofill height
				cerr << "x: "<< autofill.at(0) << endl; 
				cerr << "y: " << autofill.at(1) << endl;
				cerr << "z: "<< autofill.at(2) << endl;
				state = GO_TO_CUP;
				break;
			// Wait for input and move to cup with open gripper
			case GO_TO_CUP:
				validRead = false;
				// Blocks here for input from human or other program
				if(input_coords(&coords)){
					// Ensure the current motor position is a valid result
					while(!get_motor_angles(&motor_bit_angle, &Motors));

					if(!generate_path(&pathGen, &coords, &motor_bit_angle, OPEN)) {
						break;
					}
					// Perform the interpolation
					for (size_t i = 0; i<pathGen.size(); i++) {
					    Motors.move_to_goal_pos( &pathGen.at(i), curr_pos );
					    usleep(UPDATE_INTERVAL);
					}

					/*
					ikine(&coords, &angles, OPEN);
					set_goals(&goal_pos, angles);
					Motors.move_to_goal_pos(&goal_pos, curr_pos);
					usleep(1000000);
					*/

					//sleep(2);
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
				sleep(2);
				state = UP;
				break;
			// Wait for input and move the cup there
			case UP:
			    cout << "UP" << endl;
			    validRead = false;
			    coords.at(2) += 180; //move the cup directly up 120
			    // Ensure the current motor position is a valid result
			    while(!get_motor_angles(&motor_bit_angle, &Motors));

			    if(!generate_path(&pathGen, &coords, &motor_bit_angle, CLOSED)) {
				    break;
			    }
			    // Perform the interpolation
			    for (size_t i = 0; i<pathGen.size(); i++) {
				Motors.move_to_goal_pos( &pathGen.at(i), curr_pos );
				usleep(UPDATE_INTERVAL);
			    }
			    sleep(2);

			    /*
			    ikine(&coords, &angles, CLOSED);
			    set_goals(&goal_pos, angles);
			    Motors.move_to_goal_pos( &goal_pos, curr_pos );
			    sleep(3);
			    */
			    // Go to next state
			    state = MOVE_AUTO;
			    break;
			    
			case MOVE_AUTO:
				validRead = false;
				// Ensure the current motor position is a valid result
				while(!get_motor_angles(&motor_bit_angle, &Motors));
				if(!generate_path(&pathGen, &autofill, &motor_bit_angle, CLOSED)) {
					break;
				}

				// Perform the interpolation
				for (size_t i = 0; i<pathGen.size(); i++) {
				    Motors.move_to_goal_pos( &pathGen.at(i), curr_pos );
				    usleep(UPDATE_INTERVAL);
				}
				sleep(1);

				/*
				ikine(&coords, &angles, CLOSED);
				set_goals(&goal_pos, angles);
				Motors.move_to_goal_pos( &goal_pos, curr_pos );
				sleep(5);
				*/

				// Go to next state
				state = DROP;
				break;
			// Release cup
			case DROP:
				ikine(&autofill, &angles, OPEN);
				set_goals(&goal_pos, angles);
				Motors.move_to_goal_pos(&goal_pos, curr_pos);
				sleep(2);
				state = RESET;
				break;
			// Go back to starting position
			case RESET:
				coords.at(0) = 0.0;
				coords.at(1) = 150.0;
				coords.at(2) = 300.0;
				ikine(&coords, &angles, OPEN);
				set_goals(&goal_pos, angles);
				Motors.move_to_goal_pos(&goal_pos, curr_pos);
				state = GO_TO_CUP;
				get_motor_angles(&motor_bit_angle, &Motors);
				// Signal to parent for new cup location
				cout << "1" << endl;
				sleep(2);
				break;


		}
	
    }

    Motors.no_torque_generate();
    Motors.set_torque(0);
    dxl_terminate();
    cout << "Successfully exited program" << endl;

    return 0;
}
