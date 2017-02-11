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

	ACC_SS_LOW;
	spiTransceive(packet1); //expect to get shit back? (YES,nothing)

	unsigned char H = spiTransceive(packet2);
	unsigned char L = spiTransceive(0x00);
	ACC_SS_HIGH;
	int reading = ((H&0x0F)<<8) | L;
	float mv = (reading/4096.0)*3.3;
	printf("%X, %X, %d, %f\r\n",H&0x0F,L,reading,mv);
	return reading;
}

float get_accelerometer_axis_g(unsigned char channel)
{
	float reading = get_accelerometer_axis(channel);
	reading = (reading/4095.0)*3.3;
	return (reading - get_accelerometer_vref())/(0.333);
}

float get_accelerometer_vref()
{
	ACC_SS_LOW;
	unsigned char packet1 = 0x06;
	unsigned char packet2 = (0x03<<6);
	spiTransceive(packet1);
	unsigned char H = spiTransceive(packet2);
	unsigned char L = spiTransceive(0x00);
	ACC_SS_HIGH;
	float vref = ((H&0x0F)<<8) | L;
	float v = (vref/4096.0)*3.3;
	//printf("%f, %f\r\n",vref,v);
	return v;
}
