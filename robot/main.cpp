#include <iostream>
#include <math.h>
#include "ikine.h"
#include "fkine.h"
#include "control_and_input.h"
#include "interpolate.h"


#define WRIST_OFFSET 30.0
#define ELBOW_OFFSET 0 //2.0
#define HEIGHT       260
#define DROP_HEIGHT  120

using namespace std;

// States for the state machine - could use some renaming
enum state_t {INIT, GO_TO_CUP, GRIP, UP, MOVE_ACROSS, MOVE_DOWN, MOVE_UP_TWO, COASTER, COASTER_DOWN, DROP, RESET};


void print_coords(vector<double> *coords) {
    cerr << "x: " << coords->at(0) << ", y: "
	 << coords->at(1) << ", z: " << coords->at(2)
	 << ", grip: " << coords->at(3) << endl;    
}

int main(int argc, char **argv) {

	// init some vectors
    vector<int> fkine_vector (4);
    vector<double> angles (4);
    vector<double> coords (4);
    vector<double> autofill(4);
    vector<double> coaster(4);
    vector<double> tmp(4);
    vector<double> curr_coords (4);
    vector<int> motor_bit_angle (4);
    // put some starting values in
    coords.at(0) = 0;
    coords.at(1) = (L2 + L3)/2;
    coords.at(2) = L1;
    coords.at(3) = 0;
    bool validRead;

    // Path generation variables
    vector<vector<int>> pathGen;
    vector<vector<double>> generalPath; // used to storage interpolated values of bits

    // Create a multi dynamixel motor object
    CMulti_DNMX_Motor Motors;

    vector<int> goal_pos(4);
    goal_pos.at(0)= 2096;
    goal_pos.at(1) = 512; //205;
    goal_pos.at(2)= 512; //650;
    goal_pos.at(3) = 280;

    int curr_pos[4] ={0,0,0,0};

    // Init with baud 1Mbps, refer to bauds.txt for mappings

    Motors.initialization(1);
    Motors.set_compliance();
    Motors.set_torque(1023);
    Motors.set_speed(750);

    //Motors.test_registers();

    Motors.move_to_goal_pos(&goal_pos, curr_pos);

    state_t state = INIT;//INIT;
    bool finished = false;

    sleep(1);

    // Main program loop

    // NOTES : curr_pos is not used at all, probably get rid of it or actuall use it
    //         in move_to_goal().
    // TODO: write function to do the repetivive moving procedure that has literally
    //       just been copy pasted.

    while (!finished) {

		switch (state) {
			// Init state - get location of various markers
			case INIT:
				//autofill read in
				cin >> autofill.at(0); //x
				autofill.at(0) = autofill.at(0) * 10; //scaling main is cm
				cin >> autofill.at(1); //y
				autofill.at(1) = autofill.at(1) * 10; //scaling main is cm
				cin >> autofill.at(2); //z
				autofill.at(2) = DROP_HEIGHT; //hardcord autofill height

				//coaster read in
				cin >> coaster.at(0);
				coaster.at(0) = coaster.at(0) * 10;
				cin >> coaster.at(1);
				coaster.at(1) = coaster.at(1) * 10;
				cin >> coaster.at(2);
				coaster.at(2) = DROP_HEIGHT;

				cerr << "Autofill location at: " << endl;
				cerr << "x: "<< autofill.at(0) << endl; 
				cerr << "y: "<< autofill.at(1) << endl;
				cerr << "z: "<< autofill.at(2) << endl;

				cerr << "Coaster location at: " << endl;
				cerr << "x: " << coaster.at(0) << endl;
				cerr << "y: " << coaster.at(1) << endl;
				cerr << "z: " << coaster.at(2) << endl;

				fflush(stdin);
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
				Motors.stillMoving();
				state = UP;
				break;
			// Wait for input and move the cup there
		case UP:
			    validRead = false;

			    // Move to reset
			    coords.at(0) = 0.0;
			    coords.at(1) = 150.0;
			    coords.at(2) = 300.0;
			    ikine(&coords, &angles, CLOSED);
			    set_goals(&goal_pos, angles);
			    Motors.move_to_goal_pos(&goal_pos, curr_pos);
			    Motors.stillMoving();


			    coords.at(2) = HEIGHT; //+= 180; //move the cup directly up 120
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
			    Motors.stillMoving();

			    // Go to next state
			    state = MOVE_ACROSS;
			    break;
			    
			case MOVE_ACROSS:
				validRead = false;
				tmp.at(0) = autofill.at(0);
				tmp.at(1) = autofill.at(1);
				tmp.at(2) = coords.at(2);
				//print_vector(&autofill);
				//print_vector(&tmp);
			
				// Ensure the current motor position is a valid result
				while(!get_motor_angles(&motor_bit_angle, &Motors));
				if(!generate_path(&pathGen, &tmp, &motor_bit_angle, CLOSED)) {
					break;
				}

				// Perform the interpolation
				for (size_t i = 0; i<pathGen.size(); i++) {
				    Motors.move_to_goal_pos( &pathGen.at(i), curr_pos );
				    usleep(UPDATE_INTERVAL);
				}
				Motors.stillMoving();

				// Go to next state
				state = MOVE_DOWN;
				break;
				
			case MOVE_DOWN:
				validRead = false;
				
				//print_vector(&autofill);
				
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
				Motors.stillMoving();

				// Go to next state
				state = MOVE_UP_TWO;
				cerr << endl << "Please dispense" << endl;
				sleep(3);
				break;
			case MOVE_UP_TWO:
				validRead = false;
				
				tmp.at(0) = autofill.at(0);
				tmp.at(1) = autofill.at(1);
				tmp.at(2) = HEIGHT - 10;

				//print_vector(&tmp);

				// Ensure the current motor position is a valid result
				while(!get_motor_angles(&motor_bit_angle, &Motors));
				//move to tmp as per location of move accross
				if(!generate_path(&pathGen, &tmp, &motor_bit_angle, CLOSED)) {
					break;
				}

				// Perform the interpolation
				for (size_t i = 0; i<pathGen.size(); i++) {
				    Motors.move_to_goal_pos( &pathGen.at(i), curr_pos );
				    usleep(UPDATE_INTERVAL);
				}
				Motors.stillMoving();

				coords.at(0) = 0.0;
				coords.at(1) = 150.0;
				coords.at(2) = 300.0;
				ikine(&coords, &angles, CLOSED);
				set_goals(&goal_pos, angles);
				Motors.move_to_goal_pos(&goal_pos, curr_pos);
				Motors.stillMoving();

				// Go to next state
				state = COASTER;
				break;
			case COASTER:
				tmp.at(0) = coaster.at(0);
				tmp.at(1) = coaster.at(1);
				tmp.at(2) = HEIGHT;
				//tmp.at(2) = tmp.at(2); //leave at the same height as the previous state
				validRead = false;
				// Ensure the current motor position is a valid result
				while(!get_motor_angles(&motor_bit_angle, &Motors));
				//move to tmp as per location of move across
				if(!generate_path(&pathGen, &tmp, &motor_bit_angle, CLOSED)) {
					break;
				}

				// Perform the interpolation
				for (size_t i = 0; i<pathGen.size(); i++) {
				    Motors.move_to_goal_pos( &pathGen.at(i), curr_pos );
				    usleep(UPDATE_INTERVAL);
				}
				Motors.stillMoving();

				// Go to next state
				state = COASTER_DOWN;
				break;
			case COASTER_DOWN:
				validRead = false;
				// Ensure the current motor position is a valid result
				while(!get_motor_angles(&motor_bit_angle, &Motors));
				//move to tmp as per location of move accross
				if(!generate_path(&pathGen, &coaster, &motor_bit_angle, CLOSED)) {
					break;
				}

				// Perform the interpolation
				for (size_t i = 0; i<pathGen.size(); i++) {
				    Motors.move_to_goal_pos( &pathGen.at(i), curr_pos );
				    usleep(UPDATE_INTERVAL);
				}
				Motors.stillMoving();

				// Go to next state
				state = DROP;
				//sleep(5);
				break;

			case DROP:
			    //autofill.at(2) = DROP_HEIGHT;
			    tmp.at(0) = coaster.at(0);
			    tmp.at(1) = coaster.at(1);
			    tmp.at(2) = DROP_HEIGHT;
				ikine(&tmp, &angles, OPEN);
				set_goals(&goal_pos, angles);
				Motors.move_to_goal_pos(&goal_pos, curr_pos);
				Motors.stillMoving();
				cerr << endl << "Cup of coffee or tea served" << endl;
				state = RESET;
				break;
			// Go back to starting position
			case RESET:
			    // moves back up and then back to origin
			    coaster.at(2) = HEIGHT;
			    ikine(&coaster, &angles, OPEN);
			    set_goals(&goal_pos, angles);
			    Motors.move_to_goal_pos(&goal_pos, curr_pos);
			    Motors.stillMoving();



				coords.at(0) = 0.0;
				coords.at(1) = 150.0;
				coords.at(2) = 300.0;
				ikine(&coords, &angles, OPEN);
				set_goals(&goal_pos, angles);
				Motors.move_to_goal_pos(&goal_pos, curr_pos);
				Motors.stillMoving();
				state = GO_TO_CUP;
				get_motor_angles(&motor_bit_angle, &Motors);
				// Signal to parent for new cup location
				cout << "1";
				Motors.stillMoving();
				break;


		}
	
    }

    Motors.no_torque_generate();
    Motors.set_torque(0);
    dxl_terminate();
    //cout << "Successfully exited program" << endl;

    return 0;
}
