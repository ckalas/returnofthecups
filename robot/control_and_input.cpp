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
