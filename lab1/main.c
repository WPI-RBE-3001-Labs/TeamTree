/*
 * main.c
 *
 *  Created on: Jan 18, 2017
 *      Author: peter
 */

#include <RBELib/RBELib.h>
#include "main.h"
#include "adc.h"

int state = 0;
unsigned int adcReading;

int main(int argv, char* argc[]) {
	initRBELib();
	init_serial(bps115200);
	init_led();
	init_timer0();
	init_adc();
	char welcome[] = "\n\ryo Dawg\n\r";
	transmit(welcome, 11);

	while (1) {
		adcReading = read_adc(1);
		//adcReading = 6969;
		char data[6];
		data[4] = '\n';
		data[5] = '\r';
		adcString(adcReading,data);
		transmit(data, 6);
		_delay_ms(200);
	}

	return 0;
}

ISR(TIMER0_OVF_vect) {
	//PORTBbits._P4 = 0;//state;
	//state = !state;
	//char data[] = "Hello!\r\n";
	//transmit(data, 8);
	//ADCSRA |= (1<<ADSC);
}

void init_timer0() {
	TIMSK0 = (1 << TOIE0);
	// set timer0 counter initial value to 0
	TCNT0 = 0x00;
	TIFR0 = 1 << TOV0;
	// start timer0 with /1024 prescaler
	TCCR0B = (1 << CS02) | (1 << CS00);
}

void init_led() {
	DDRBbits._P4 = OUTPUT;
	PORTBbits._P4 = 1;
}

void init_serial(unsigned int baudrate_coded) {
	UBRR0H = (unsigned char) (baudrate_coded >> 8);
	UBRR0L = (unsigned char) baudrate_coded;
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (3 << UCSZ00);
}

void transmit(char *data, unsigned int datalen) {
	for (int i = 0; i < datalen; i++) {
		while (!(UCSR0A & (1 << UDRE0)))
			;
		UDR0 = data[i];
	}
}

char receive_byte() {
	while (!(UCSR0A & (1 << RXC0)))
		;
	return UDR0;
}

void recieve(char *outdata, unsigned int bytes_to_read) {
	for (int i = 0; i < bytes_to_read; i++) {
		outdata[i] = receive_byte();
	}
}

void adcString(int adcVal, char* string) {
	string[0] = adcVal / 1000 + 0x30; //1000th place
	string[1] = (adcVal % 1000) / 100 + 0x30; //100th place
	string[2] = ((adcVal % 1000)%100) / 10 + 0x30; //10th place
	string[3] = adcVal % 10 + 0x30;
}

void button_led() {
	DDRBbits._P3 = INPUT;
	DDRBbits._P4 = OUTPUT;
	while (1) {
		PORTBbits._P4 = PINBbits._P3;
		_delay_ms(20);
	}
}

void putCharDebug(char byteToSend) {
}
