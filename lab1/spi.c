/*
 * spi.c
 *
 *  Created on: Jan 23, 2017
 *      Author: nick
 */

#include <RBELib/RBELib.h>

void init_spi_master(int speed)
{
	DDRBbits._P5 = OUTPUT; //MOSI set as output
	DDRBbits._P6 = INPUT; //MISO as input
	DDRBbits._P7 = OUTPUT; //SCK as output
	DDRBbits._P4 = OUTPUT; //SS as output

	//enable spi, set as master
	//MSB sent first, F_CPU/4 speed
	//leading edge rising, latch on leading edge.
	SPCR = (1<<SPE) | (1<<MSTR);
	PORTBbits._P4 = 0; //set the CS pin low?
}

void spi_send_byte(char data)
{
	SPDR = data; //rip
	while(!(SPSR & (1<<SPIF)))//make sure that shit isn't full
		;
}
