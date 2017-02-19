/*
 * gripper.c
 *
 *  Created on: Feb 18, 2017
 *      Author: nick
 */

#include <RBELib/RBELib.h>
#include "gripper.h"

void init_gripper()
{
	initAltCom(115200);
	setServo(GRIPPERPORT,GRIPPEROPEN);
}


void open_gripper()
{
	setServo(GRIPPERPORT,GRIPPEROPEN);
}

void close_gripper()
{
	setServo(GRIPPERPORT,GRIPPERCLOSE);
}
