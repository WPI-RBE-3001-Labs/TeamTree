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
