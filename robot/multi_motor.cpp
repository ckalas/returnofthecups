//////////////////////////////////////////////////////////////
//                                                          //
//      This work was derived from (below):                 //
//                                                          //
//      Mult Dynamixel Control Program                      //
//                Youngmok Yun (yunyoungmok@gmail.com)      //
//                                                          //
//////////////////////////////////////////////////////////////

#include "multi_motor.h"
#include <cmath>

#define COMPLIANCE 128
#define VELOCITY 100 // 100 millimetre per second

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
     * Motor 4 is the gripper
     */
    Motor_ID[0] = MOTOR_ID_1;
    Motor_ID[1] = MOTOR_ID_2;
    Motor_ID[2] = MOTOR_ID_3;
    Motor_ID[3] = MOTOR_ID_4;



   

     cerr << "Motor initialization" << endl; 

     ///////// Open USB2Dynamixel ////////////
     if( dxl_initialize(deviceIndex, baudnum) == 0 ) {
	 cerr <<  "Failed to open USB2Dynamixel!" << endl;
	 cerr <<  "Press key to terminate..." << endl;
	 getchar();
	 return 0;
     }
     else
	 cerr << "Succeed to open USB2Dynamixel!" << endl;
 }


 void CMulti_DNMX_Motor::move_to_goal_pos(vector<int> *GoalPos, int PresentPos[]){

     // add do while loop if it is still moving or has not at least reached it's 
     // goal position (need function to check if +- 5-->10 bit )

     // add acceleration depending on diff of goal and curr pos
     bool finished = false;
     int i;
     while (!finished) {
	 for(i=0;i<NUM_OF_MOTORS ;i++){

	     // Write goal position
	     dxl_write_word( Motor_ID[i], P_GOAL_POSITION_L, GoalPos->at(i) );
	     // If error, try again
	     if (check_com_status() != 0) {
		 break;
	     }
	 }

	 if (i == NUM_OF_MOTORS) {
	     finished = true;
	 }
     }
 }

 void CMulti_DNMX_Motor::read_motor_angles(vector<int> *PresentPos) {
	 for(int i=0; i<NUM_OF_MOTORS; i++) {
		 PresentPos->at(i) = dxl_read_word( Motor_ID[i], P_PRESENT_POSITION_L );

		 if (check_com_status() != 0)
			 break;
	 }
 }

 int CMulti_DNMX_Motor::check_com_status(void) {
     CommStatus = dxl_get_result();

     if( CommStatus == COMM_RXSUCCESS ) {
	 PrintErrorCode();
     } else {
	     PrintCommStatus(CommStatus);
	     return 1;
     }

     return 0;
 }

 void CMulti_DNMX_Motor::set_speed(int speed) {
     int setSpeed;
     for (int i=0; i<NUM_OF_MOTORS; i++) {
	 switch (i) {
	     case 0:
		 setSpeed = speed;//(int)speed/7.9661;
		 break;
	     case 1:
		 setSpeed = speed;
		 break;
	     case 2:
		 setSpeed = speed;
		 break;
	     case 3:
		 setSpeed = speed;
		 break;
	 }

	 dxl_write_word( Motor_ID[i], P_MOVING_SPEED_L, setSpeed);
	 cerr << "motor " << i << " speed: " << setSpeed << endl;
     }
 }

 void CMulti_DNMX_Motor::read_speed(void) {
     for (int i=0; i < NUM_OF_MOTORS; i++) {
	 int ret = dxl_read_word( Motor_ID[i], P_MOVING_SPEED_L);
	 //printf("Motor No. %d, Speed value: %d\n", i, ret);
     }
 }

bool CMulti_DNMX_Motor::read_compliance(void) {
    int ret, ret2;
    for(int i = 1; i < 3; i++ ) {
	ret = dxl_read_byte( Motor_ID[i], CW_COMPLIANCE_SLOPE );
	ret2 = dxl_read_byte( Motor_ID[i], CCW_COMPLIANCE_SLOPE );
	cerr << "MotorID: " << i << ", CW slope: " << ret 
	     << ", CCW slope: " << ret2 << endl;
    }

    if (ret != 128 && ret2 != 128)
	return false;
    else 
	return true;
}
	
void CMulti_DNMX_Motor::stillMoving(void) {
    while ( 1 ) {
	int temp = 0;

	for( int i = 0; i < NUM_OF_MOTORS; i++ ) {
	    temp |= dxl_read_word( Motor_ID[i], P_MOVING );
	}	

	if ( !temp ) break;
 
	usleep(100000); // 1 ms
    }
}

// Print communication result
void CMulti_DNMX_Motor::PrintCommStatus(int CommStatus) {
    switch(CommStatus)
        {
        case COMM_TXFAIL:
	    cerr << "COMM_TXFAIL: Failed transmit instruction packet!" << endl;
	    break;

        case COMM_TXERROR:
	    cerr << "COMM_TXERROR: Incorrect instruction packet!" << endl;
	    break;

        case COMM_RXFAIL:
	    cerr << "COMM_RXFAIL: Failed get status packet from device!" << endl;
	    break;

        case COMM_RXWAITING:
	    cerr << "COMM_RXWAITING: Now recieving status packet!" << endl;
	    break;

        case COMM_RXTIMEOUT:
	    cerr << "COMM_RXTIMEOUT: There is no status packet!" << endl;
	    break;

        case COMM_RXCORRUPT:
	    cerr << "COMM_RXCORRUPT: Incorrect status packet!" << endl;
	    break;

        default:
	    cerr << "This is unknown error code!" << endl;
	    break;
        }
}

// Print error bit of status packet
void CMulti_DNMX_Motor::PrintErrorCode() {
    if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
	cerr <<"Input voltage error!" << endl;

    if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
	cerr << "Angle limit error!" << endl;

    if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
	cerr << "Overheat error!" << endl;

    if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
	cerr << "Out of range error!" << endl;

    if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
	cerr << "Checksum error!" << endl;

    if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
	cerr << "Overload error!" << endl;

    if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
	cerr << "Instruction code error!" << endl;
}


void CMulti_DNMX_Motor::set_torque(int torque){
    for (int i=0; i<NUM_OF_MOTORS; i++) {
        int ret = dxl_read_word( Motor_ID[i], P_TORQUE_ENABLE);
        //printf("Motor No. %d, Torque enabled: %d\n", i, ret);

        dxl_write_word( Motor_ID[i], P_TORQUE_LIMIT_L, torque);

        ret = dxl_read_word( Motor_ID[i], P_TORQUE_LIMIT_L);
        //printf("Motor No. %d, Torque applied: %d\n", i, ret);

        if (check_com_status() != 0)
            break;
    }
}

void CMulti_DNMX_Motor::no_torque_generate(void){
    for (int i=0;i<NUM_OF_MOTORS;i++){
        dxl_write_word( Motor_ID[i], P_TORQUE_ENABLE, 0 );
    }

}


int mx12w_angle2bits( double radians ) {
    double actual_angle = MX_12W_ANGLE / 2 - (radians / M_PI * 180);
    return int( actual_angle / MX_12W_ANGLE * MX_12W_BITS);
}

int ax12a_angle2bits( double radians ) {
    double actual_angle = AX_12A_ANGLE / 2.0 - (radians / M_PI * 180);
    //cout << "in angle2bits ax12a: " << actual_angle << ", " << radians << " PI: " << M_PI << endl;
    return int( actual_angle / AX_12A_ANGLE * AX_12A_BITS);
}

int ax12a_angle2bits_elbow( double radians) {
    double actual_angle = AX_12A_ANGLE / 2.0 - (radians / M_PI * 180);
    actual_angle -= M_PI / 4;
    //cout << "in angle2bits ax12a: " << actual_angle << ", " << radians << " PI: " << M_PI << endl;
    return int( actual_angle / AX_12A_ANGLE * AX_12A_BITS);
}

void CMulti_DNMX_Motor::test_registers(void) {
    int register_ret;
    for (int j = 1; j < 3; j++) {
    for (int i = 0; i < 50; i++) {
	register_ret = dxl_read_word( Motor_ID[j], i );
	cerr << "Register : " << i << " --> value = " << register_ret << endl;
    }
    }
}

void CMulti_DNMX_Motor::set_compliance(void) {
    //while ( !read_compliance() ) {
    //   for (int j = 0; j < 5; j++) {
	// Set the Compliance Slope
	for (int i = 1; i < 3; i++) {
	    dxl_write_byte( Motor_ID[i], CW_COMPLIANCE_SLOPE, COMPLIANCE); // clockwise
	    dxl_write_byte( Motor_ID[i], CCW_COMPLIANCE_SLOPE, COMPLIANCE); // counter-clockwise
	    check_com_status();
	}

	// making the register has been written
	//	usleep(1000);
	//    }

    read_compliance();
}
