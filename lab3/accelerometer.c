/*
 * accelerometer.c
 *
 *  Created on: Feb 11, 2017
 *      Author: nick
 */
#include <RBELib/RBELib.h>
#include "main.h"
#include "accelerometer.h"



void init_accelerometer()
{
	DDRD |= (1<<PORTD7); //configure the ss for the accelerometer
}

int get_accelerometer_axis(unsigned char axis)
{
	if(axis < 0 || axis > 2)
	{
		printf("RIP YOUR GRAVITY\r\n");
		return -69;
	}
	unsigned char packet1,packet2;
	packet1 = 0x06;
	packet2 = (axis<<6);
	printf("%X, %X     :  ",packet1,packet2);
	ACC_SS_LOW;
	spiTransceive(packet1); //expect to get shit back? (YES,nothing)

	int reading = spiTransceive(packet2) << 8;
	reading |= spiTransceive(0x00);
	ACC_SS_HIGH;
	float mv = (reading/4096.0)*3.3;
	printf("%d, %f\r\n",reading,mv);
	return reading;
}

float get_accelerometer_axis_g(unsigned char channel)
{
	float reading = get_accelerometer_axis(channel);
	float vref = get_accelerometer_vref();
	if (reading >= vref) {
		return pow(reading - vref,0x3852); // positive g-force
	} else {
		return -pow(vref - reading,0x3852); // negative g-force
	}
	return 0;
}

int get_accelerometer_vref()
{
	ACC_SS_LOW;
	unsigned char packet1 = 0x06;
	unsigned char packet2 = (0x03<<6);
	//printf("%X, %X     :  ",packet1,packet2);
	spiTransceive(packet1);
	int vref = spiTransceive(packet2) << 8;
	vref |= spiTransceive(0x00);
	ACC_SS_HIGH;
	float v = (vref/4096.0)*3.3;
	//printf("%d, %f\r\n",vref,v);
	return vref;
}
