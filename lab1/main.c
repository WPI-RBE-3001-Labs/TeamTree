/*
 * main.c
 *
 *  Created on: Jan 18, 2017
 *      Author: peter
 */

#include <RBELib/RBELib.h>
#include <stdlib.h>
#include "main.h"
#include "spi.h"
#include "adc.h"
#include "Global.h"
volatile unsigned long currTime = 0;
volatile bool pid_ready = 0;

unsigned long tempTime = false;
int max_val = 1;
int prev_val = 1;
int freq = 1;
float base_setpoint;

int main(int argv, char* argc[]) {
	initRBELib();
	init_serial(uart_bps230400);
	init_led();
	init_timer0();
//	init_timer1();
	init_timer2();
	init_adc();
	init_spi_master(spi_bps230400);
	init_pid();

	DDRBbits._P0 = INPUT;
	DDRBbits._P1 = INPUT;
	DDRBbits._P2 = INPUT;
	DDRBbits._P3 = INPUT;


	//init_adc_trigger_timer();
	set_motor(1,0);
	set_motor(0,0);
	sei();

	while (1) {
		unsigned int adcReading = read_adc(2);
		float angle = map(adcReading, HORIZONTALPOT, VERTICALPOT, 0, 90);
		float current = get_current(0);
		if(pid_ready)
		{
			set_motor(0,calculate_pid_output(angle, base_setpoint, 0));
			pid_ready = false;
		}

		if (!PINBbits._P0) {
			base_setpoint = 0;
		}
		else if (!PINBbits._P1) {
			base_setpoint = 30;
		}
		else if (!PINBbits._P2) {
			base_setpoint = 60;
		}
		else if (!PINBbits._P3) {
			base_setpoint = 90;
		}


	}

	return 0;
}

/** channel is 0 or 1 **/
float get_current(int channel) {
	int raw = read_adc(channel);
	float current = fmap(raw, 0, 1024, 0, 5) - CURRENT_BIAS;
	if (fabs(current) < 0.01) {
		return 0.0;
	}
	return -current;
}

/** motor_id is 0 for first link, and 1 for second link.
 * velocity is -1 to 1. Positive is RHR going outward along motor shaft */
void set_motor(int motor_id, float velocity) {
	if (velocity > 1) { velocity = 1;}
	if (velocity < -1) { velocity = -1;}
	unsigned char dac_chan1 = 0;
	unsigned char dac_chan2 = 1;

	if (motor_id == 0) {
		dac_chan1 = 0;
		dac_chan2 = 1;
	}
	else if (motor_id == 1){
		dac_chan1 = 2;
		dac_chan2 = 3;
	}

	int dac_out = fmap(velocity, -1, 1, -4096, 4096);
	if (dac_out >= 0) {
		setDAC(dac_chan1, 0);
		setDAC(dac_chan2, abs(dac_out));
	}
	else {
		setDAC(dac_chan1, abs(dac_out));
		setDAC(dac_chan2, 0);
	}

}


ISR(TIMER0_OVF_vect) {
}

ISR(TIMER0_COMPA_vect) {
	// PID
	pid_ready = true;
}

unsigned long getTime() {
	return currTime;
}

//make PWM yo
//using port PB3 - OC0A
void init_timer0() {
	TIMSK0 = (1 << OCIE0A); //enable interrupts on overflow!

	//enable CTC mode
	TCCR0A = (1 << WGM01);

	//select a 1024 for clock prescaler
	TCCR0B = (1 << CS00) | (1 << CS02);

	//set the value that compare register triggers at
	OCR0A = 180;
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

float fmap(float val, float in_min, float in_max, float out_min, float out_max) {
	return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int map(int val, int in_min, int in_max, int out_min, int out_max) {
	return (float)(val - in_min) / ((float) (in_max - in_min))
			* ((float) (out_max - out_min)) + out_min;
}
