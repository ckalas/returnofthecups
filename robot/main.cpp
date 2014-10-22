#include <iostream>
#include "multi_motor.h"
#include <cv.h>
#include <highgui.h>
#include <math.h>

#include "ikine.h"

using namespace std;
using namespace cv;

int main( int argc, char *argv[] ) 
{
    cout << "hello world" << endl;

    //cout << "sqrt: " << sqrt(9) << endl;
    vector<double> angles (3);
    vector<double> coords (3);
    coords.at(0) = 0;
    coords.at(1) = 95;
    coords.at(2) = 150;
    
    ikine(coords, &angles);
	
    CMulti_DNMX_Motor Motors;

    int a = 512;
    int goal_pos[4] = {2096, a, a, a};
    int curr_pos[4] ={0,0,0,0};


    // init with baud 20k, rgefer to bauds.txt for mapping
    /*
    Motors.initialization(34);
    Motors.set_torque(1023);    
    Motors.set_speed(80);
    Motors.read_speed();
    
    Motors.move_to_goal_pos(goal_pos, curr_pos);
    */
    
    /**
     * From you view:
     *      left vs right is the x direction
     *      closer vs further is the y direction
     *      up vs down is the z direction
     */
    while (1) {
        cout << "x: ";
        cin >> coords.at(0);
        cout << "y: ";
        cin >> coords.at(1);
        cout << "z: ";
        cin >> coords.at(2);

        ikine(coords, &angles);

        /*
        goal_pos[0] = Motors.mx12w_angle2bits(angles[0]);
        goal_pos[1] = Motors.ax12a_angle2bits(angles[1]);
        goal_pos[2] = Motors.ax12a_angle2bits(angles[2]);
        Motors.move_to_goal_pos( goal_pos, curr_pos);
        */
    }

    //char k;

    /****************
     * Need to get the motors to move at small increment to allow smooth movement
     * and also a function to check if the motor has reach +- 5 or 10 of the goal pos
     * vs the curr_pos. 
     * Also you need to set an angle limit to prevent the motor from getting pass
     * certain limit which might cause tangling of wires.
     *
     * The MX-12W works the current code
     */

    /*  
    while (1) {
	
	cin.get();

	goal_pos[0] -= 5;

	//for (int j=0; j<20; j++) {
	Motors.move_to_goal_pos(goal_pos,curr_pos);

	for(int i=0;i<3;i++){
	    cout << i << ": " << curr_pos[i] << endl;
	}
    }
    */
    
    Motors.no_torque_generate();
    Motors.set_torque(0);

    dxl_terminate();
    cout << "dxl_terminate" << endl;
	
    return 0;
}
