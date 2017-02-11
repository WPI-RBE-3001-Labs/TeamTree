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

int get_accelerometer_axis(int axis)
{
	if(axis < 0 || axis > 3)
	{
		printf("RIP YOUR GRAVITY\r\n");
		return -69;
	}
	unsigned char packet;
	packet = (1<<8)|(1<<7)|(axis<<4);

	ACC_SS_LOW;
	spiTransceive(packet); //expect to get shit back?

	unsigned char H = spiTransceive(0x00);
	unsigned char L = spiTransceive(0x00);
	ACC_SS_HIGH;

	printf("%X, %X\r\n",H,L);
	return 0;

}
