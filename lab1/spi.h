/*
 * spi.h
 *
 *  Created on: Jan 23, 2017
 *      Author: nick
 */

#pragma once

#define spi_bps230400 5
#define spi_bps460800 0
#define spi_bps921600 4
void init_spi_master(int speed);
