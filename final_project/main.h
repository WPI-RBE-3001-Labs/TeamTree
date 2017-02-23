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

#define HORIZONTALPOTBASE 229
#define VERTICALPOTBASE 588

#define HORIZONTALPOTARM 179
#define VERTICALPOTARM 579

#define CURRENT_BIAS 2.73

#define LOWLINK 0
#define HIGHLINK 1
#define LOWLINKMG 0.15

#define PID_TEST_THROTTLE .30

#define HEAVY_OBJECT_THRES 207 //mC or something like that lol


#define SNAPSHOT 1
#define MANUAL 2
#define SAMPLESHIT 3
#define INVERSEDEBUG 4
#define FINALPROJECT 69

#define SUB_IDLE 0
#define SUB_SEE 1
#define SUB_READY 2
#define SUB_SAW 3
#define SUB_GRAB 4
#define SUB_PICKUP 5
#define SUB_CURRENT 6
#define SUB_DROP 7
#define SUB_GRIP 8
#define SUB_PICKUP_CURRENT 9
#define SUB_SORT_LIGHT 10
#define SUB_SORT_HEAVY 11

void pid_persiodic_follow();
void stop_motors();
float get_arm_angle(char link);
void pid_periodic();
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
