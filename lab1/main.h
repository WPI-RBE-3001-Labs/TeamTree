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

void button_led();
void echo_serial();
void init_serial(unsigned int baudrate);
void transmit(char *data, unsigned int datalen);
void recieve(char *outdata, unsigned int bytes_to_read);
char receive_byte();

#endif /* MAIN_H_ */
