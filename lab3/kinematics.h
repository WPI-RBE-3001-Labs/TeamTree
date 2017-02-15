/*
 * kinematics.h
 *
 *  Created on: Feb 1, 2017
 *      Author: nick
 */

#pragma once

#define BASELENGTH 143
#define LOWLINKLENGTH 152
#define HIGHLINKLENGTH 112

void calculate_forward_kinematics(float theta1, float theta2, float *x, float *y);
void calculate_inverse_kinematics(float *theta1, float *theta2, float x, float y);
