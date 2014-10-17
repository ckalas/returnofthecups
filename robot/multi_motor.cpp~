//////////////////////////////////////////////////////////////
//                                                          //
//      Mult Dynamixel Control Program                      //
//                Youngmok Yun (yunyoungmok@gmail.com)      //
//                                                          //
//////////////////////////////////////////////////////////////

#include "multi_dnmx_motor.h"

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
    

    Motor_ID[0] = MOTOR_ID_1;
    Motor_ID[1] = MOTOR_ID_2;
    Motor_ID[2] = MOTOR_ID_3;

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


    for(int i=0;i<NUM_OF_MOTORS ;i++){

        // Write goal position
        dxl_write_word( Motor_ID[i], P_GOAL_POSITION_L, GoalPos[i] );

        // Read present position
        PresentPos[i] = dxl_read_word( Motor_ID[i], P_PRESENT_POSITION_L );
        CommStatus = dxl_get_result();

        if( CommStatus == COMM_RXSUCCESS )
        {
//                printf( "%d   %d\n",GoalPos[i], PresentPos[i] );
                PrintErrorCode();
        }
        else
        {
                PrintCommStatus(CommStatus);
                break;
        }

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
