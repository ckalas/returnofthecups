#include "cup_planning.h"

#define _USE_MATH_DEFINES 


//x_init - initial x distance from base to cup  
//y_init - initial y distance from base to cup
void cup_prediction(float t, Point2f p_0, Point2f* p_c) {
    float y_t = 0; //distance between arm and centre of table
    float RPM = 2;

    //theta = 0 corresponds to the +x_axis in rectangular coords
    *p_c.x = sqrt(pow(p_0.x, 2) + pow(p_0.y, 2)) * cos(atan(p_0.y / p_0.x) + (M_PI / 30) * t * RPM);
    *p_c.y = sqrt(pow(p_0.x, 2) + pow(p_0.y, 2)) * sin(atan(p_0.y / p_0.x) + (M_PI / 30) * t * RPM) + y_t;
}
