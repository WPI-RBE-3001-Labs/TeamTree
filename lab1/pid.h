/*
 * pid.h
 *
 *  Created on: Jan 30, 2017
 *      Author: nick
 */

#pragma once

struct PIDconst;

typedef struct {
	float kP;
	float kI;
	float kD;
	float last_adc;
	float integral;
	float int_cap;
	float setpoint;
	float sensor;
	float error;
}PIDconst;

extern PIDconst base_pid, arm_pid;

float get_pid_error(char link);
void init_pid();
float calculate_pid_output(float sensor, float setpoint, char link);
