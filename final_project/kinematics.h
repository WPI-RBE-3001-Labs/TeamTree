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

#define TOOL_READY_POS_Y 155
#define TOOL_PICKUP_POS_Y 115

#define TOOL_LIGHT_POS_X -240
#define TOOL_LIGHT_POS_Y 280

#define TOOL_HEAVY_POS_X 300
#define TOOL_HEAVY_POS_Y 140


void calculate_forward_kinematics(float theta1, float theta2, float *x, float *y);
void calculate_inverse_kinematics(float *theta1, float *theta2, float x, float y);
void limit_angles(float *angle, char link);
