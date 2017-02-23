/*
 * gripper.h
 *
 *  Created on: Feb 18, 2017
 *      Author: nick
 */

#pragma once

#define GRIPPERPORT 1

#define GRIPPEROPEN 0
#define GRIPPERCLOSE 190


void init_gripper();
void open_gripper();
void close_gripper();
