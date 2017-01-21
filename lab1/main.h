/*
 * main.h
 *
 *  Created on: Jan 18, 2017
 *      Author: peter
 */

#ifndef MAIN_H_
#define MAIN_H_

#define bps115200 9
#define bps9600 119
#define bps250000 4
#define bps500000 4
#define false 0
#define true 1

void button_led();
void init_led();
void init_timer0();
void init_timer2();
void echo_serial();
void init_serial(unsigned int baudrate);
void transmit(char *data, unsigned int datalen);
void recieve(char *outdata, unsigned int bytes_to_read);
char receive_byte();
int map(int val, int in_min, int in_max, int out_min, int out_max);

void adcString(int adcVal, char* string);

#endif /* MAIN_H_ */
