/*
 * main.c
 *
 *  Created on: Jan 18, 2017
 *      Author: peter
 */

#include <RBELib/RBELib.h>

int main(int argv, char* argc[]) {
	DDRBbits._P4 = OUTPUT; //Set Port B Pin 4 to output
	while(1){
	PINBbits._P4 = 0; //Sets Port B Pin 4 to low
	_delay_ms(100); //Delay .1 sec
	PINBbits._P4 = 1; //Sets Port B Pin 4 to high
	_delay_ms(100); //Delay .1 sec
	}
	return 0;
}

