/*
 * adc.c
 *
 *  Created on: Jan 19, 2017
 *      Author: nick
 */

#include <RBELib/RBELib.h>
#include "Global.h"
#include "main.h"
#include "adc.h"

int read_adc(unsigned int channel) {

	//set the direction of the port to an input
	DDRA &= ~(1 << channel);
	DIDR0 = (1 << channel); //disable the digital part for power saving yo?

	//select the channel in the MUX
	ADMUX &= 0b11100000;
	ADMUX |= channel;

	//Start an ADC conversion
	ADCSRA |= (1 << ADSC);

	//wait for the converion to complete
	while (ADCSRA & (1 << ADSC))
		;
	//read the low and then the high register
	unsigned char L = ADCL;
	unsigned char H = ADCH;
	return ((0x03 & H) << 8) | L;
}

void init_adc_port(int channel) {
	//set the direction of the port to an input
	DDRA &= ~(1 << channel);
	DIDR0 = (1 << channel); //disable the digital part for power saving yo?

	//select the channel in the MUX
	ADMUX &= 0b11100000;
	ADMUX |= channel;
}

void init_adc() {
	//enable the adc
	PRR0 = 0; //just to be sure
	ADMUX = (1 << REFS0); //set the avcc voltage ref
	//ADC enable, trigger enable, interrupt enable, 128 prescaler
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	//ADCSRA |= (1 << ADIE); //enable interrupt
	//sei();
	//enable global interrupts
}

void init_adc_trigger_timer() {
	PRR0 = 0; //just to be sure
	ADMUX = (1 << REFS0); //set the avcc voltage ref
	//ADC enable, trigger enable, interrupt enable, 128 prescaler
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADATE) | (1 << ADPS2)
			| (1 << ADPS0);
	ADCSRB = (1 << ADTS1) | (1 << ADTS0);	//compare A timer0
	init_adc_port(2);
}

//adc interrupt
ISR(ADC_vect) {
	//adc conversion is complete
	TCNT0 = 0;
	PORTBbits._P4 = ~PORTBbits._P4;
	unsigned char L = ADCL;
	unsigned char H = ADCH;
	int adcReading = ((0x03 & H) << 8) | L;
	//ADCSRA &= ~(1 << ADIF); //clear the adc interrupt flag
	printf("%1.2f %d\n\r", ((float) currTime) / 1000.0, adcReading);
}


float map_pot_angle(int channel, char link)
{
	float value = read_adc(channel);
	if(link == LOWLINK)
	{
		return fmap(value, HORIZONTALPOTBASE, VERTICALPOTBASE,
							0, 90) - 90.0;
	}
	else if(link == HIGHLINK)
	{
		return fmap(value, HORIZONTALPOTARM, VERTICALPOTARM,
				-90, 0);
	}
	return -69.9;
}
