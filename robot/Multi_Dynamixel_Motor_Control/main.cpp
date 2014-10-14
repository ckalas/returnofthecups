//////////////////////////////////////////////////////////////
//                                                          //
//      Mult Dynamixel Control Program                      //
//                Youngmok Yun (yunyoungmok@gmail.com)      //
//                                                          //
//////////////////////////////////////////////////////////////

#include <QtCore/QCoreApplication>
#include <iostream>

#include "multi_dnmx_motor.h"

using namespace std;

int main(int argc, char *argv[])
{

    CMulti_DNMX_Motor Motors;

    int goal_pos[4] ={500,500,500,500};
    int curr_pos[4] ={0,0,0,0};
    int temp;

    // init with baud 20k, refer to bauds.txt for mapping
    Motors.initialization(9);
    Motors.move_to_goal_pos(goal_pos,curr_pos);

    for(int j=0;j<5;j++){
        for(int i=0;i<4;i++){
            cout << i << ": " << curr_pos[i] << endl;
        }
    }

    Motors.no_torque_generate();

    dxl_terminate();
    cout << "dxl_terminate" << endl;


    return 1;
}

