/*
 * ir.h
 *
 *  Created on: Feb 18, 2017
 *      Author: nick
 */

#pragma once

#define IR_A_PORT 4
#define IR_B_PORT 5

#define A_A 29.922527472046
#define A_B 0.997844385648

#define B_A 30.933523095153
#define B_B .9949967762932

#define BASE_TO_SIDEPLATE 16.3
#define IR_TO_SIDEPLATE 20
#define WIDTH_BLOCK 2.5

#define FUDGE_FACTOR 0


#define DIST_BETWEEN_IR 3.5 //cm

float get_ir_cm(char sensor);
float get_ir_cm_base(char sensor);
