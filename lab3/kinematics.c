/*
 * kinematics.c
 *
 *  Created on: Feb 1, 2017
 *      Author: nick
 */

#include <RBELib/RBELib.h>
#include "kinematics.h"

void calculate_forward_kinematics(float theta1, float theta2, float *x, float *y)
{
	theta1 = (M_PI*theta1)/180.0;
	theta2 = (M_PI*theta2)/180.0;

	*x = -LOWLINKLENGTH*sin(theta1) - HIGHLINKLENGTH*sin(theta1+theta2);
	*y = BASELENGTH + LOWLINKLENGTH*cos(theta1) + HIGHLINKLENGTH*cos(theta1+theta2);
}


void calculate_inverse_kinematics(float *theta1, float *theta2, float x, float y)
{
	y = y - BASELENGTH;

	float px2 = x*x;
	float py2 = y*y;
	float a1_2 = LOWLINKLENGTH*LOWLINKLENGTH;
	float a2_2 = HIGHLINKLENGTH*HIGHLINKLENGTH;

	float beta = atan2(y,x);
	float gamma = acos((px2 + py2 + a1_2 - a2_2)/(2*LOWLINKLENGTH*sqrt(px2+py2)));

	float theta1_1 = (beta + gamma)*180.0/M_PI - 90.0;
	float theta1_2 = (beta - gamma)*180.0/M_PI - 90.0;

	*theta2 = acos(((px2 + py2) - (a1_2 + a2_2))/(2*LOWLINKLENGTH*HIGHLINKLENGTH)) * 180.0/M_PI - 90.0;

	printf("%f, %f, %f\r\n",theta1_1,theta1_2,*theta2);
}
