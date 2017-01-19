/*
 * main.c
 *
 *  Created on: Jan 18, 2017
 *      Author: peter
 */

#include <RBELib/RBELib.h>
#include "main.h"

int main(int argv, char* argc[]) {
	initRBELib();
	init_serial(bps115200);

	while (1) {
		char data[4];
		recieve(data, 4);
		transmit(data, 4);
		_delay_ms(200);
	}

	return 0;
}

void init_serial(unsigned int baudrate_coded) {
	UBRR0H = (unsigned char)(baudrate_coded >> 8);
	UBRR0L = (unsigned char)baudrate_coded;
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (3 << UCSZ00);
}

void transmit(char *data, unsigned int datalen) {
	for (int i = 0; i < datalen; i++) {
		while (!(UCSR0A & (1<<UDRE0)))
			;
		UDR0 = data[i];
	}
}

char receive_byte(){
	while (!(UCSR0A & (1 << RXC0)))
		;
	return UDR0;
}

void recieve(char *outdata, unsigned int bytes_to_read) {
	for (int i = 0; i < bytes_to_read; i++) {
		outdata[i] = receive_byte();
	}
}

void button_led() {
	DDRBbits._P3 = INPUT;
	DDRBbits._P4 = OUTPUT;
	while (1) {
		PORTBbits._P4 = PINBbits._P3;
		_delay_ms(20);
	}
}


void putCharDebug(char byteToSend) {}
