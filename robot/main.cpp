#include <iostream>
#include "multi_dnmx_motor.h"

using namespace std;

int main( int argc, char *argv[] ) 
{
    cout << "hello world" << endl;
	
    CMulti_DNMX_Motor Motors;

    int goal_pos[3] ={256,256,256};
    int curr_pos[3] ={0,0,0};
    //int temp;

    // init with baud 20k, refer to bauds.txt for mapping
    Motors.initialization(34);


    for (int j=0; j<5; j++) {
	Motors.move_to_goal_pos(goal_pos,curr_pos);

	for(int i=0;i<3;i++){
	    cout << i << ": " << curr_pos[i] << endl;
	}
    }

    Motors.no_torque_generate();

    dxl_terminate();
    cout << "dxl_terminate" << endl;
	
    return 0;
}
