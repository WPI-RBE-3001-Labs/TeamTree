/*
 * kinematics.c
 *
 *  Created on: Feb 1, 2017
 *      Author: nick
 */

#include <RBELib/RBELib.h>
#include "main.h"
#include "kinematics.h"

void calculate_forward_kinematics(float theta1, float theta2, float *x, float *y)
{
	theta1 = (M_PI*theta1)/180.0;
	theta2 = (M_PI*theta2)/180.0;

	*x = -LOW_LINK_LENGTH*sin(theta1) - HIGH_LINK_LENGTH*sin(theta1+theta2);
	*y = BASELENGTH + LOW_LINK_LENGTH*cos(theta1) + HIGH_LINK_LENGTH*cos(theta1+theta2);
}

//theta1_1 goes with theta2,  theta1_2 goes with -theta
void calculate_inverse_kinematics(float *theta1, float *theta2, float x, float y)
{
	y = y - BASELENGTH;

	float px2 = x*x;
	float py2 = y*y;
	float a1_2 = LOW_LINK_LENGTH*LOW_LINK_LENGTH;
	float a2_2 = HIGH_LINK_LENGTH*HIGH_LINK_LENGTH;

	float beta = atan2(y,x);
	float gamma = acos((px2 + py2 + a1_2 - a2_2)/(2.0*LOW_LINK_LENGTH*sqrt(px2+py2)));

	float theta1_1 = (beta + gamma)*180.0/M_PI - 90.0;
	float theta1_2 = (beta - gamma)*180.0/M_PI - 90.0;

	*theta2 = -acos(((px2 + py2) - (a1_2 + a2_2))/(2.0*LOW_LINK_LENGTH*HIGH_LINK_LENGTH)) * 180.0/M_PI;
//	*theta2 = ((px2 + py2) - (a1_2 + a2_2))/(2.0*LOW_LINK_LENGTH*HIGH_LINK_LENGTH);

	//printf("%f, %f, %f, %f\r\n",theta1_1,theta1_2,*theta2, -*theta2);

	if(x > 0)
	{
		if(theta2 > 0)
		{
			*theta1 = theta1_1;
		}
		else
		{
			*theta1 = theta1_2;
			*theta2 = -*theta2;
		}
	}
	else
	{
		if(theta2 < 0)
		{
			*theta1 = theta1_1;
		}
		else
		{
			*theta1 = theta1_2;
			*theta2 = -*theta2;
		}
	}
}


void limit_angles(float *angle, char link)
{
	if(link == LOWLINK)
	{
		if(*angle > LOW_LINK_MAX_ANGLE)
			*angle = LOW_LINK_MAX_ANGLE;
		if(*angle < LOW_LINK_MIN_ANGLE)
			*angle = LOW_LINK_MIN_ANGLE;
	}
	if(link == HIGHLINK)
	{
		if(*angle > HIGH_LINK_MAX_ANGLE)
			*angle = HIGH_LINK_MAX_ANGLE;
		if(*angle < HIGH_LINK_MIN_ANGLE)
			*angle = HIGH_LINK_MIN_ANGLE;
	}
}
