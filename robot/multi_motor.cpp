//////////////////////////////////////////////////////////////
//                                                          //
//      Mult Dynamixel Control Program                      //
//                Youngmok Yun (yunyoungmok@gmail.com)      //
//                                                          //
//////////////////////////////////////////////////////////////

#include "multi_motor.h"

CMulti_DNMX_Motor::CMulti_DNMX_Motor()
{
}

CMulti_DNMX_Motor::~CMulti_DNMX_Motor()
{
    no_torque_generate();
    dxl_terminate();
}


bool CMulti_DNMX_Motor::initialization(int baudnum){
    
    // make variable array lengths

    //int GoalPos[2] = {0, 4095}; // for Ex series
    deviceIndex = 0;
    PresentPos[0] = 0;
    PresentPos[1] = 0;
    PresentPos[2] = 0;
    PresentPos[3] = 0;
    
    /**
     * Motor 1 is the base
     * Motor 2 is the elbow joint
     * Motor 3 is the wrist joint
     */
    Motor_ID[0] = MOTOR_ID_1;
    Motor_ID[1] = MOTOR_ID_2;
    Motor_ID[2] = MOTOR_ID_3;
    Motor_ID[3] = MOTOR_ID_4;

    printf( "\n\n Motor initialization \n\n" );

    ///////// Open USB2Dynamixel ////////////
    if( dxl_initialize(deviceIndex, baudnum) == 0 )
	{
            printf( "Failed to open USB2Dynamixel!\n" );
            printf( "Press Enter key to terminate...\n" );
            getchar();
            return 0;
	}
    else
	printf( "Succeed to open USB2Dynamixel!\n" );

}


void CMulti_DNMX_Motor::move_to_goal_pos(int GoalPos[], int PresentPos[]){

    // add do while loop if it is still moving or has not at least reached it's 
    // goal position (need function to check if +- 5-->10 bit )

    // add acceleration depending on diff of goal and curr pos

    for(int i=0;i<NUM_OF_MOTORS ;i++){

        // Write goal position
        dxl_write_word( Motor_ID[i], P_GOAL_POSITION_L, GoalPos[i] );

        // Read present position
        PresentPos[i] = dxl_read_word( Motor_ID[i], P_PRESENT_POSITION_L );

        if (check_com_status() != 0)
            break;
    }
}

int CMulti_DNMX_Motor::check_com_status(void) {
    CommStatus = dxl_get_result();

    if( CommStatus == COMM_RXSUCCESS ) {
        // printf( "%d   %d\n",GoalPos[i], PresentPos[i] );
        PrintErrorCode();
    } else {
	    PrintCommStatus(CommStatus);
	    return 1;
    }

    return 0;
}

void CMulti_DNMX_Motor::set_torque(int torque){
    for (int i=0; i<NUM_OF_MOTORS; i++) {
        int ret = dxl_read_word( Motor_ID[i], P_TORQUE_ENABLE);
        printf("Motor No. %d, Torque enabled: %d\n", i, ret);

        dxl_write_word( Motor_ID[i], P_TORQUE_LIMIT_L, torque);

        ret = dxl_read_word( Motor_ID[i], P_TORQUE_LIMIT_L);
        printf("Motor No. %d, Torque applied: %d\n", i, ret);

        if (check_com_status() != 0)
            break;
    }
}

void CMulti_DNMX_Motor::set_speed(int speed) {
    for (int i=0; i<NUM_OF_MOTORS; i++) {
        dxl_write_word( Motor_ID[i], P_MOVING_SPEED_L, speed);

        if (check_com_status() != 0)
            break;
    }
}

void CMulti_DNMX_Motor::read_speed(void) {
    for (int i=0; i < NUM_OF_MOTORS; i++) {
        int ret = dxl_read_word( Motor_ID[i], P_MOVING_SPEED_L);
        printf("Motor No. %d, Speed value: %d\n", i, ret);
    }
}

// Print communication result
void CMulti_DNMX_Motor::PrintCommStatus(int CommStatus)
{
    switch(CommStatus)
        {
        case COMM_TXFAIL:
	    printf("COMM_TXFAIL: Failed transmit instruction packet!\n");
	    break;

        case COMM_TXERROR:
	    printf("COMM_TXERROR: Incorrect instruction packet!\n");
	    break;

        case COMM_RXFAIL:
	    printf("COMM_RXFAIL: Failed get status packet from device!\n");
	    break;

        case COMM_RXWAITING:
	    printf("COMM_RXWAITING: Now recieving status packet!\n");
	    break;

        case COMM_RXTIMEOUT:
	    printf("COMM_RXTIMEOUT: There is no status packet!\n");
	    break;

        case COMM_RXCORRUPT:
	    printf("COMM_RXCORRUPT: Incorrect status packet!\n");
	    break;

        default:
	    printf("This is unknown error code!\n");
	    break;
        }
}

// Print error bit of status packet
void CMulti_DNMX_Motor::PrintErrorCode()
{
    if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
	printf("Input voltage error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
	printf("Angle limit error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
	printf("Overheat error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
	printf("Out of range error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
	printf("Checksum error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
	printf("Overload error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
	printf("Instruction code error!\n");
}


void CMulti_DNMX_Motor::no_torque_generate(){
    for (int i=0;i<NUM_OF_MOTORS;i++){
        dxl_write_word( Motor_ID[i], P_TORQUE_ENABLE, 0 );
    }

}
