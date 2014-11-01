#include <iostream>
#include "multi_motor.h"
#include <cv.h>
#include <highgui.h>
#include <math.h>
#include "ikine.h"
#include <stdio.h>

#include <unistd.h>
#include <termios.h>

using namespace std;
using namespace cv;

#define DIST_MOVE 2

void input_coords(vector<double> *coords) {
    cout << "x: ";
    cin >> coords->at(0);
    cout << "y: ";
    cin >> coords->at(1);
    cout << "z: ";
    cin >> coords->at(2);
    cout << "grip: (1 on, 0 off) ";
    cin >> coords->at(3);
}

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


    //char k = getch();

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





int main( int argc, char *argv[] ) 
{
    cout << "hello world" << endl;

    //cout << "sqrt: " << sqrt(9) << endl;
    vector<double> angles (4);
    vector<double> coords (4);
    coords.at(0) = 0;
    coords.at(1) = (L2 + L3)/2;
    coords.at(2) = L1;
	
    CMulti_DNMX_Motor Motors;

    int a = 512;
    int goal_pos[4] = {2096, a, a, a};//(a + (1024*(12/150.0))), a};
    int curr_pos[4] ={0,0,0,0};

    cout << goal_pos << endl;

    // init with baud 57k, refer to bauds.txt for mapping
    Motors.initialization(34);
    Motors.set_torque(1023);    
    Motors.set_speed(100);
    Motors.read_speed();
    
    ikine(coords, &angles);

    Motors.move_to_goal_pos(goal_pos, curr_pos);
        
    /**
     * From your view:
     *      left vs right is the x direction
     *      closer vs further is the y direction
     *      up vs down is the z direction
     */
    while (1) {

	//input_coords(&coords);

	game_control(&coords);
	
	cout << "x: " << coords.at(0) << ", y: "
	     << coords.at(1) << ", z: " << coords.at(2)
	     << ", grip: " << coords.at(3) << endl;

	ikine(coords, &angles);

	/*
	angles[0] = coords.at(0) / 180 * M_PI ;
	angles[1] = coords.at(1) / 180 * M_PI;
	angles[2] = coords.at(2) / 180 * M_PI -  22.00 / 180 * M_PI; // angle offset -22(degree)

	cout << "checking wrist angle: " << angles[2] << endl;
	*/
	if(coords.at(3) == 1) {
	    angles[3] = -30.0 / 180 * M_PI;
	}
	else {
	    angles[3] = 30.0 / 180 * M_PI;
	}

	angles[2] -= 22.0 / 180 * M_PI;

        goal_pos[0] = Motors.mx12w_angle2bits(angles[0]);
        goal_pos[1] = Motors.ax12a_angle2bits(angles[1]);
        goal_pos[2] = Motors.ax12a_angle2bits(angles[2]);
	goal_pos[3] = Motors.ax12a_angle2bits(angles[3]);

	/*
	cout << "base: " << goal_pos[0] << endl
	     << "elbow: " << goal_pos[1] << endl
	     << "wrist: " << goal_pos[2] << endl
	     << "grip: " << goal_pos[3] << endl;
	*/
        Motors.move_to_goal_pos( goal_pos, curr_pos);        
    }
    
    Motors.no_torque_generate();
    Motors.set_torque(0);

    dxl_terminate();
    cout << "dxl_terminate" << endl;
	
    return 0;
}
