/*
 * spi.h
 *
 *  Created on: Jan 23, 2017
 *      Author: nick
 */

#ifndef SPI_H_
#define SPI_H_

#define bps460800 0
#define bps921600 4
#define bps230400 5

void init_spi_master(int speed);

void spi_send_byte(char data);

#endif /* SPI_H_ */
