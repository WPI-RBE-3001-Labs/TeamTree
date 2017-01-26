/*
 * dac.c
 *
 *  Created on: Jan 26, 2017
 *      Author: nick
 */

#include <RBELib/RBELib.h>
#include "spi.h"

void set_dac(unsigned char dac, int val) {
	if (val < 0 || val > 4095) {
		printf("RIP YOUR DAC");
		return;
	}
	char addr = dac & 0x0f;
	char cmd = 0x03 << 4; //write to and update DAC Reg
	char data[3];
	data[0] = cmd | addr;
	data[1] = (unsigned char) (val >> 2);
	data[2] = (val & 0x03) << 6;

	for(int i = 0; i < 3; i++)
	{
		spi_send_byte(data[i]);
		printf("%d : %u\n\r",i,data[i]);
	}
}
