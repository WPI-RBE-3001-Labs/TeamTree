/*
 * accelerometer.h
 *
 *  Created on: Feb 11, 2017
 *      Author: nick
 */

#pragma once


#define ACC_SS_LOW PORTD &= ~(1<<PORTD7)
#define ACC_SS_HIGH PORTD |= (1<<PORTD7)


int get_accelerometer_axis(unsigned char axis);
void init_accelerometer();
float get_accelerometer_vref();
float get_accelerometer_axis_g(unsigned char channel);
