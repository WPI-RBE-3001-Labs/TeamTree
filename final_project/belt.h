/*
 * belt.h
 *
 *  Created on: Feb 14, 2017
 *      Author: nick
 */

#pragma once

#define BELT_STOPPED 150
#define FULLFORWARD 0
#define FULLBACKWARDS 300

#define BELTPORT 0


#define DIST_IR_ARM 10.25 //cm

void belt_backwards();
void init_belt();
void belt_stop();
void belt_forward();
