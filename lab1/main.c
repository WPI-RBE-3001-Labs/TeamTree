/*
 * main.c
 *
 *  Created on: Jan 18, 2017
 *      Author: peter
 */

#include <RBELib/RBELib.h>
#include "main.h"
#include "adc.h"
#include "Global.h"
unsigned int adcReading;
volatile unsigned long currTime = 0;

BOOL dank = false;
unsigned long tempTime;
int max_val = 1;
int prev_val = 1;
int freq = 1;

int main(int argv, char* argc[]) {
	initRBELib();
	init_serial(bps500000);
	init_led();
	init_timer0();
	init_timer1();
	init_timer2();
	init_adc();
	//init_adc_trigger_timer();
	sei();
	printf("Yo Dawg\n\r");

	DDRBbits._P2 = OUTPUT;
	PORTBbits._P2 = 1;

	DDRBbits._P0 = OUTPUT;
	DDRBbits._P1 = OUTPUT;
	DDRBbits._P2 = OUTPUT;

	PORTBbits._P0 = 1;
	PORTBbits._P1 = 1;
	PORTBbits._P2 = 1;

	while (1) {
		/*adcReading = read_adc(2);
		 float voltage = adcReading / 1023.0 * 5000.0;
		 float angle = adcReading / 1023.0 * 180.0;
		 int duty = map(adcReading,0,1023,0,255);
		 //OCR0A = duty;
		 printf("%f,Adc: %d,mV: %1.4f,Angle: %f, DUTY: %d\n\r", ((float)currTime)/1000.0, adcReading,
		 voltage, angle,duty);
		 _delay_ms(200);*/
		/*if (!PINBbits._P2 && !dank) {
		 TCCR0B = (1 << CS00) | (1 << CS02);
		 dank = true;
		 tempTime = currTime;
		 }

		 if (dank && (currTime - tempTime > 1000)) {
		 TCCR0B = 0; //turn off the timer
		 dank = false;
		 }*/
		adcReading = read_adc(2);
		int val = map(adcReading, 0, 1023, 0, max_val);
		int duty = map(adcReading, 0, 1023, 0, 100);
		if (prev_val != val) {
			OCR1A = val;
			prev_val = val;

		}
		if (!PINBbits._P0) {
			ICR1 = 36000; //set timer for 1Hz
			TCNT1 = 0;
			max_val = 36000;
			freq = 1;

		} else if (!PINBbits._P1) {
			ICR1 = 36000 / 20; //set timer for 1Hz
			max_val = 36000 / 20;
			TCNT1 = 0;
			freq = 20;
		} else if (!PINBbits._P2) {
			ICR1 = 36000 / 100; //set timer for 1Hz
			max_val = 36000 / 100;
			TCNT1 = 0;
			freq = 100;
		}
		_delay_ms(25);

		printf("%d,%d,%d,%1.2f\n\r", duty, freq,
				PINDbits._P5, ((float)adcReading)/1023.0*5.0);

	}

	return 0;
}

ISR(TIMER0_OVF_vect) {
	//PORTBbits._P4 = ~ PORTBbits._P4;
}

ISR(TIMER0_COMPA_vect) {
	//PORTBbits._P4 = ~ PORTBbits._P4;
}

unsigned long getTime() {
	return currTime;
}

//make PWM yo
//using port PB3 - OC0A
void init_timer0() {

	DDRBbits._P3 = OUTPUT; //need to enable output on this pin

	TIMSK0 = (1 << TOIE0) | (1 << OCIE0A); //enable interrupts on overflow!

	//enable fast-pwm, set the OC0A pin to be toggled on trigger
	TCCR0A = (1 << COM0A1) | (1 << WGM00) | (1 << WGM01);

	//select a 1024 for clock prescaler
	//TCCR0B = (1 << CS00) | (1 << CS02);

	//set the value that compare register triggers at
	OCR0A = 76;
}

//using this timer to keep track of time?
void init_timer2() {
	//don't do any waveform generation
	TCCR2A = (1 << WGM21);  //setup for CTC mode
	TCCR2B = (1 << CS22) | (1 << CS20); //prescaler;
	OCR2A = 143;  //set the timer to count up
	TIMSK2 = (1 << OCIE2A); //enable interrupt on timercnt = OCR2A
}

void init_timer1() {
	DDRDbits._P5 = OUTPUT;

	TCCR1A = (1 << COM1A1);
	TCCR1B = (1 << CS12) | (1 << WGM13);
	ICR1 = 36000;
	OCR1A = 25200;

}

ISR(TIMER2_COMPA_vect) {
//TCNT2 = 0; //reset the timer2 count ;)
	cli();
	currTime++;
	sei();
//PORTBbits._P4 = ~PORTBbits._P4;
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
//UCSR0A = (1 << U2X0);
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
	string[2] = ((adcVal % 1000) % 100) / 10 + 0x30; //10th place
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
	char byte[1];
	byte[0] = byteToSend;
	transmit(byte, 1);
}

int map(int val, int in_min, int in_max, int out_min, int out_max) {
	return ((float) val) / ((float) (in_max - in_min))
			* ((float) (out_max - out_min)) + out_min;
}
