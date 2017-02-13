/*
 * spi.c
 *
 *  Created on: Jan 23, 2017
 *      Author: nick
 */

#include <RBELib/RBELib.h>
#include "spi.h"

void init_spi_master(int speed)
{
	DDRBbits._P5 = OUTPUT; //MOSI set as output
	DDRBbits._P6 = INPUT; //MISO as input
	DDRBbits._P7 = OUTPUT; //SCK as output
	DDRBbits._P4 = OUTPUT; //SS as output

	//enable spi, set as master
	//MSB sent first, F_CPU/4 speed
	//leading edge rising, latch on leading edge.
	//accelerometer will not FUCKING work at this speed
	if(speed == spi_bps2304000)
	{
		SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0);
		SPSR = (1<<SPI2X);
	}
	else if(speed == spi_bps288000)
	{
		SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR1);
		SPSR = 0;
	}
	else
	{
		printf("rip spi\r\n");
	}
	PORTBbits._P4 = 1; //set the CS pin low?
}

unsigned char spiTransceive(BYTE data)
{
	SPDR = data; //rip
	while(!(SPSR & (1<<SPIF)))//make sure that shit isn't full
		;
	return SPDR;
}
