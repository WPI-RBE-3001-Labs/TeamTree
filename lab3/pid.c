/*
 * pid.c
 *
 *  Created on: Jan 30, 2017
 *      Author: nick
 */

#include <RBELib/RBELib.h>
#include "pid.h"

PIDconst base_pid, arm_pid, base_pid_follow, arm_pid_follow;

void init_pid() {
	base_pid.kP = 0.03;
	base_pid.kI = 0.0044; //.003
	base_pid.kD = 0;
	base_pid.int_cap = 20;
	base_pid.integral = 0;
	base_pid.last_adc = 0;

	arm_pid.kP = 0.025;
	arm_pid.kI = 0.0045;
	arm_pid.kD = 0.05;
	arm_pid.int_cap = 20;
	arm_pid.integral = 0;
	arm_pid.last_adc = 0;
}

float calculate_pid_output(float sensor, float setpoint, char link) {
	PIDconst *pid_stuff;
	if (link == 0) {
		pid_stuff = &base_pid;
	} else if (link == 1) {
		pid_stuff = &arm_pid;
	} else {
		printf("stop being stupid, pid not happy");
		return 0;
	}

	float error = sensor - setpoint;
	pid_stuff->error = error;

	pid_stuff->integral += error;
	if (pid_stuff->integral > pid_stuff->int_cap) {
		pid_stuff->integral = pid_stuff->int_cap;
	}
	if (pid_stuff->integral < -pid_stuff->int_cap) {
		pid_stuff->integral = -pid_stuff->int_cap;
	}

	float derivative = sensor - pid_stuff->last_adc;
	float pid_output = pid_stuff->kP * error
			+ pid_stuff->kI * pid_stuff->integral + pid_stuff->kD * derivative;
	//printf("p %f i%f d %f\r\n",pid_stuff->kP,pid_stuff->kI,pid_stuff->kD);
	//printf("int %f intCap %f last %f\r\n",pid_stuff->integral,pid_stuff->int_cap,pid_stuff->last_adc);
	//printf("%f, %f, %f ,%f\r\n", setpoint, sensor, pid_output,error);
	pid_stuff->last_adc = sensor;
	return pid_output;
}

float get_pid_error(char link) {
	PIDconst *pid_stuff;
	if (link == 0) {
		pid_stuff = &base_pid;
	} else if (link == 1) {
		pid_stuff = &arm_pid;
	} else {
		printf("stop being stupid, pid not happy");
		return 0;
	}

	return pid_stuff->error;
}
