#include <iostream>
#include "multi_motor.h"
#include <cv.h>
#include <highgui.h>

using namespace std;
using namespace cv;

int main( int argc, char *argv[] ) 
{
    cout << "hello world" << endl;
	
    CMulti_DNMX_Motor Motors;

    int a = 512;
    int goal_pos[3] = {100, a, a};
    int curr_pos[3] ={0,0,0};
    //int temp;

    // init with baud 20k, rgefer to bauds.txt for mapping
    Motors.initialization(34);
    Motors.set_torque(1023);    
    Motors.set_speed(80);
    Motors.read_speed();
    
    goal_pos[0] = 2048;
    goal_pos[1] = 512;
    goal_pos[2] = 512;

    Motors.move_to_goal_pos(goal_pos, curr_pos);
    sleep(3);

    goal_pos[0] = 100;
    goal_pos[1] += 100;
    goal_pos[2] += 300;
    Motors.move_to_goal_pos( goal_pos, curr_pos);
    sleep(3);

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
