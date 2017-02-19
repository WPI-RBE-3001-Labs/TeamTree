/*
 * accelerometer.c
 *
 *  Created on: Feb 11, 2017
 *      Author: nick
 */
#include <RBELib/RBELib.h>
#include "main.h"
#include "accelerometer.h"
#include <stdlib.h>
#include "spi.h"


void init_accelerometer()
{
	DDRD |= (1<<PORTD7); //configure the ss for the accelerometer
}

int get_accelerometer_axis(unsigned char axis)
{
	init_spi_master(spi_bps288000); //slow the fucking spi down so it works
	//with this shitty bi-directional spi hack.....
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

	unsigned char H = (spiTransceive(packet2))&0x1f;
	unsigned char L = spiTransceive(0x00);
	char buffer[10];
	char buffer1[10];
	itoa (H,buffer,2);
	itoa (L,buffer1,2);
	//printf("%s, %s     :  ",buffer,buffer1);
	ACC_SS_HIGH;
	int reading = (H<<8) | L;
	float mv = (reading/4095.0)*3.3;
	//printf("%d, %f\r\n",reading,mv);
	return reading;
}

float get_accelerometer_axis_g(unsigned char channel)
{
	float reading = get_accelerometer_axis(channel);
	float vref = get_accelerometer_vref();
	//reading = (reading/4095.0)*3.3;
	//vref = (vref/4095.0)*3.3;
	return (reading - vref)*0.0022;
}

int get_accelerometer_vref()
{
	init_spi_master(spi_bps288000); //slow the fucking spi down so it works
	//with this shitty bi-directional spi hack.....
	ACC_SS_LOW;
	unsigned char packet1,packet2;
	packet1 = 0x06;
	packet2 = (3<<6);

	ACC_SS_LOW;
	spiTransceive(packet1); //expect to get shit back? (YES,nothing)

	unsigned char H = (spiTransceive(packet2))&0x1f;
	unsigned char L = spiTransceive(0x00);
	ACC_SS_HIGH;
	int vref = (H<<8) | L;
	float v = (vref/4095.0)*3.3;
	//printf("%d, %f\r\n",vref,v);
	return vref;
}
