/*
 * adc.h
 *
 *  Created on: Jan 19, 2017
 *      Author: nick
 */

#ifndef ADC_H_
#define ADC_H_

void init_adc();
int read_adc(unsigned int channel);
void init_adc_trigger_timer();

#endif /* ADC_H_ */
