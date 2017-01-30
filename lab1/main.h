/*
 * main.h
 *
 *  Created on: Jan 18, 2017
 *      Author: peter
 */

#pragma once

#include <stdbool.h>

#define uart_bps115200 9
#define uart_bps9600 119
#define uart_bps250000 4
#define uart_bps230400 4
#define uart_bps500000 4

#define HORIZONTALPOT 237
#define VERTICALPOT 600
#define CURRENT_BIAS 2.73

void button_led();
void init_led();
void init_timer0();
void init_timer1();
void init_timer2();
void echo_serial();
void init_serial(unsigned int baudrate);
void transmit(char *data, unsigned int datalen);
void recieve(char *outdata, unsigned int bytes_to_read);
char receive_byte();
float get_current(int channel);
void set_motor(int motor_id, float velocity);
float fmap(float val, float in_min, float in_max, float out_min, float out_max);
int map(int val, int in_min, int in_max, int out_min, int out_max);

void adcString(int adcVal, char* string);
