#include "cup_planning.h"
#include <cv.h>

#define _USE_MATH_DEFINES 


//x_init - initial x distance from base to cup  
//y_init - initial y distance from base to cup
Point2f cup_prediction(float t, Point2f p_0, Point2f* p_c) {
    //float y_t = 0; //distance between arm and centre of table
    float RPM = 2;

    //offset the coord with table at the center
    Point2f offSet; //offset from fiducial to center of turn table
    offSet.x = 0;
    offSet.y = 0;
    
    Point2f pointOnTable;
    pointOnTable.x = p_0.x + offSet.x;
    pointOnTable.y = p_0.y + offSet.y;

    Point2f moveArm;

    //theta = 0 corresponds to the +x_axis in rectangular coords
	moveArm.x = (sqrt(pow(pointOnTable.x, 2) + pow(pointOnTable.y, 2)) * 
		cos(atan(pointOnTable.y / pointOnTable.x) + (M_PI / 30) * t * RPM)) - offSet.x;
    moveArm.y = sqrt(pow(pointOnTable.x, 2) + pow(pointOnTable.y, 2)) * 
    	sin(atan(pointOnTable.y / pointOnTable.x) + (M_PI / 30) * t * RPM) - offSet.y;
    return moveArm;
}
