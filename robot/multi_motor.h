//////////////////////////////////////////////////////////////
//                                                          //
//      Mult Dynamixel Control Program                      //
//                Youngmok Yun (yunyoungmok@gmail.com)      //
//                                                          //
//////////////////////////////////////////////////////////////

#ifndef MULTI_DNMX_MOTOR_H
#define MULTI_DNMX_MOTOR_H

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include "dynamixel.h"

/**
 * Note, only the lower byte might by need to 
 * read or write values.
 * Verify this is the case
 */

// Control table address
#define P_GOAL_POSITION_L	      30
#define P_GOAL_POSITION_H	      31
#define P_PRESENT_POSITION_L	  36
#define P_PRESENT_POSITION_H	  37

#define P_MOVING_SPEED_L          32
#define P_MOVING_SPEED_H          33
#define P_PRESENT_SPEED_L         38
#define P_PRESENT_SPEED_H         39
#define P_MOVING		          46

#define P_TORQUE_ENABLE		      24
#define P_TORQUE_LIMIT_L          34
#define P_TORQUE_LIMIT_H          35

// User setting
#define BAUD_NUM                  34      // 1: 1Mbps 34:57142bps
#define NUM_OF_MOTORS             4       // Number of motors

#define MOTOR_ID_1                4 // Motor 1 ID
#define MOTOR_ID_2                2 // Motor 2 ID
#define MOTOR_ID_3                3 // Motor 3 ID
#define MOTOR_ID_4                1 // Motor 4 ID


void PrintCommStatus(int CommStatus);
void PrintErrorCode(void);

class CMulti_DNMX_Motor
{
private:
    int baudnum;
    int deviceIndex;

    int CommStatus;
    int Motor_ID[NUM_OF_MOTORS];

    int PresentPos[NUM_OF_MOTORS];

    void PrintCommStatus(int CommStatus);
    void PrintErrorCode();


public:
    CMulti_DNMX_Motor();
    ~CMulti_DNMX_Motor();
    bool initialization(int baudnum);
    int check_com_status(void);
    void move_to_goal_pos (int GoalPos[], int PresentPos[]);
    void set_torque(int torque);
    void set_speed(int speed);
    void read_speed(void);
    void no_torque_generate();

};

#endif // MULTI_DNMX_MOTOR_H