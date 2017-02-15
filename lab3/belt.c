/*
 * belt.c
 *
 *  Created on: Feb 14, 2017
 *      Author: nick
 */

#include <RBELib/RBELib.h>
#include "belt.h"
void init_belt()
{
	initAltCom(115200);
	setServo(BELTPORT,BELT_STOPPED);
}

void belt_stop()
{
	setServo(BELTPORT,BELT_STOPPED);
}

void belt_forward()
{
	setServo(BELTPORT,FULLFORWARD);
}

void belt_backwards()
{
	setServo(BELTPORT,FULLBACKWARDS);
}
