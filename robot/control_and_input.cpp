#include "control_and_input.h"
#include "ikine.h"

int getch(void)
{
    int ch;
    struct termios oldt;
    struct termios newt;
    tcgetattr(STDIN_FILENO, &oldt); /*store old settings */
    newt = oldt; /* copy old settings to new settings */
    newt.c_lflag &= ~(ICANON | ECHO); /* make one change to old settings in new settings */
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); /*apply the new settings immediatly */
    ch = getchar(); /* standard getchar call */
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); /*reapply the old settings */
    return ch; /*return received char */
}


void game_control(vector<double> *coords) {

    char k = getch();

    if (k == 'w') {
    	//cout << "forwards" << endl;
    	coords->at(1) += DIST_MOVE;
    }
    if (k == 's') {
    	//cout << "backwards" << endl;
    	coords->at(1) -= DIST_MOVE;
    }
    if (k == 'a') {
    	//cout << "left" << endl;
    	coords->at(0) -= DIST_MOVE;
    }
    if (k == 'd') {
    	//cout << "right" << endl;
    	coords->at(0) += DIST_MOVE;
    }
    if (k == 'u') {
    	//cout << "up" << endl;
    	coords->at(2) += DIST_MOVE;
    }
    if (k == 'j') {
    	//cout << "down" << endl;
    	coords->at(2) -= DIST_MOVE;
    }
    if (k == 'g') {
    	cout << "gripper" << endl;
    	if (coords->at(3) == 0)
    		coords->at(3) = 1;
    	else
    		coords->at(3) = 0;
    	cout << coords->at(3) << endl;
    }
    if (k == 'o') {
		coords->at(0) = 0;
		coords->at(1) = (L2 + L3) / 2;
		coords->at(2) = L1;
		coords->at(3) = 0;
    }
    if (k == 'q')
    	exit(0);
}


bool input_coords(vector<double> *coords) {
    
    cerr << "x: ";
    cin >> coords->at(0);
    coords->at(0) = coords->at(0) * 10; //scaling, from main is in cm
    
    cerr << "y: ";
    cin >> coords->at(1);
    coords->at(1) = coords->at(1) * 10; //scaling, from main is in cm
    cerr << "z: ";
    cin >> coords->at(2);
    coords->at(2) = 140;

    /*
    cerr << "xc: "<< coords->at(0) << endl; 
    cerr << "yc: " << coords->at(1) << endl;
    cerr << "zc: "<< coords->at(2) << endl;
    */
    fflush(stdin);

    return true;
}



void set_goals(vector<int> *goal_pos, vector<double> angles) {
	goal_pos->at(0) = mx12w_angle2bits(angles[0]);
	goal_pos->at(1) = ax12a_angle2bits(angles[1]);
	goal_pos->at(2) = ax12a_angle2bits(angles[2]);
	goal_pos->at(3) = ax12a_angle2bits(angles[3]);

	//print_vector(goal_pos);
}

bool get_motor_angles(vector<int> *motor_bit_angle, CMulti_DNMX_Motor *Motors) {
	Motors->read_motor_angles(motor_bit_angle);
	if(!((*motor_bit_angle)[0] % 256) || (*motor_bit_angle)[1] == 0 || (*motor_bit_angle)[2] == 0) {
		return false;
	}
	return true;
}
