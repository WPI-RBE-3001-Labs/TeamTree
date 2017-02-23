/*
 * kinematics.h
 *
 *  Created on: Feb 1, 2017
 *      Author: nick
 */

#pragma once

#define BASELENGTH 142
#define LOW_LINK_LENGTH 151
#define HIGH_LINK_LENGTH 155

#define LOW_LINK_MAX_ANGLE 90
#define LOW_LINK_MIN_ANGLE -108
#define HIGH_LINK_MAX_ANGLE	82
#define HIGH_LINK_MIN_ANGLE -102


#define TOOL_WAIT_POS_X 200
#define TOOL_WAIT_POS_Y 360

#define TOOL_TEST_POS_X 220
#define TOOL_TEST_POS_Y 150

#define TOOL_READY_POS_Y 145
#define TOOL_PICKUP_POS_Y 110


void calculate_forward_kinematics(float theta1, float theta2, float *x, float *y);
void calculate_inverse_kinematics(float *theta1, float *theta2, float x, float y);
void limit_angles(float *angle, char link);
