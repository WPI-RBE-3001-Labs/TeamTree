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

	*x = -LOW_LINK_LENGTH*sin(theta1) - HIGH_LINK_LENGTH*sin(theta1+theta2);
	*y = BASELENGTH + LOW_LINK_LENGTH*cos(theta1) + HIGH_LINK_LENGTH*cos(theta1+theta2);
}
