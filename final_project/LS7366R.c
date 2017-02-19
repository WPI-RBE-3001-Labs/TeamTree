/*
 * LS7366R.c

 *
 *  Created on: Feb 9, 2017
 *      Author: nick
 */
#include <RBELib/RBELib.h>
#include "main.h"
#include "spi.h"
#include "LS7366R.h"



int EncoderCounts(int channel) {
	init_spi_master(spi_bps2304000);
	if(channel == 0)
	{
		SS_EN0_LOW;
	}
	else
	{
		SS_EN1_LOW;
	}

	spiTransceive(READ_CNTR); //ask for shit
	//do this four times since we configured for four byte counter
	unsigned char H = spiTransceive(0xff);
	unsigned char L = spiTransceive(0xff);
	SS_EN0_HIGH;
	SS_EN1_HIGH;
	//printf("%X, %X\r\n", H, L);
	return ((H<<8) | L);
}

void init_encoders() {
	init_spi_master(spi_bps2304000);
	DDRCbits._P4 = OUTPUT;
	DDRCbits._P5 = OUTPUT;

	SS_EN0_HIGH;
	SS_EN1_HIGH;

	SS_EN0_LOW;
	spiTransceive(CLR_STR);
	SS_EN0_HIGH;

	SS_EN1_LOW;
	spiTransceive(CLR_STR);
	SS_EN1_HIGH;

	SS_EN0_LOW;
	spiTransceive(CLR_CNTR);
	SS_EN0_HIGH;

	SS_EN1_LOW;
	spiTransceive(CLR_CNTR);
	SS_EN1_HIGH;

	write_encoder_reg(0,WRITE_MDR0,QUADRX1|FREE_RUN|DISABLE_INDX|ASYNCH_INDX|FILTER_1);
	write_encoder_reg(1,WRITE_MDR0,QUADRX1|FREE_RUN|DISABLE_INDX|ASYNCH_INDX|FILTER_1);
	write_encoder_reg(0,WRITE_MDR1,NO_FLAGS|BYTE_2|EN_CNTR);
	write_encoder_reg(1,WRITE_MDR1,NO_FLAGS|BYTE_2|EN_CNTR);
}

void write_encoder_reg(int channel, unsigned char op, unsigned char data_s) {
	init_spi_master(spi_bps2304000);
	if (channel == 0) {
		SS_EN0_LOW
		;
	} else if (channel == 1) {
		SS_EN1_LOW
		;
	}
	else
	{
		printf("RiP ur ENdcoderee");
		return;
	}
	spiTransceive(op);
	spiTransceive(data_s);
	SS_EN1_HIGH
	;
	SS_EN0_HIGH
	;
}

void reset_encoder_count(int channel)
{
	init_spi_master(spi_bps2304000);
	if(channel == 0)
	{
		SS_EN0_LOW;
	}
	else
	{
		SS_EN1_LOW;
	}

	spiTransceive(CLR_CNTR);

	SS_EN0_HIGH;
	SS_EN1_HIGH;
}


float get_encoder_degrees(int channel)
{
	float ticks = EncoderCounts(channel);
	if(channel == 0)
	{
		return ticks/(48.0*172.0/(360.0*2.0));
	}
	else
	{
		return ticks/10.0;
	}
	return 0;
}
