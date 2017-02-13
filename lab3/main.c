/*
 * main.c
 *
 *  Created on: Jan 18, 2017
 *      Author: peter
 */

#include <RBELib/RBELib.h>
#include <stdlib.h>
#include "main.h"
#include "accelerometer.h"
#include "spi.h"
#include "pid.h"
#include "adc.h"
#include "LS7366R.h"
#include "Global.h"
#include "kinematics.h"


volatile unsigned long currTime = 0;
volatile bool pid_ready = 0;

unsigned long tempTime = false;
int max_val = 1;
int prev_val = 1;
int freq = 1;
float base_setpoint;
float arm_setpoint = -45;

float snapshot[2][30];
float predefined[2][20];
int snapshot_counter = 0;
int snapshot_cursor = 0;
unsigned long snapshot_time = 0;
unsigned long pid_settle_time = 0;
unsigned long encoder_time = 0;
bool pid_done = false;

bool snap_debounce = false;

bool snapmode = true;
bool print_encoder = false;
char prev_state = 0;

int run_mode = MANUAL;

int main(int argv, char* argc[]) {
	initRBELib();
	init_serial(uart_bps230400);
	init_led();
	init_timer0();
	//init_timer1();
	init_timer2();
	init_adc();
	init_spi_master(spi_bps2304000);
	init_pid();
	init_encoders();
	init_accelerometer();

	DDRBbits._P0 = INPUT;
	DDRBbits._P1 = INPUT;
	DDRBbits._P2 = INPUT;
	DDRBbits._P3 = INPUT;
	DDRDbits._P0 = INPUT;
	DDRDbits._P5 = OUTPUT;
	PORTDbits._P5 = 1;

	predefined[0][0] = -44;
	predefined[1][0] = -68;
	predefined[0][1] = -50;
	predefined[1][1] = -57;
	predefined[0][2] = -56;
	predefined[1][2] = -52;
	predefined[0][3] = -62;
	predefined[1][3] = -50;
	predefined[0][4] = -66;
	predefined[1][4] = -51;
	predefined[0][5] = -66;
	predefined[1][5] = -62;
	predefined[0][6] = -65;
	predefined[1][6] = -75;
	predefined[0][7] = -62;
	predefined[1][7] = -85;
	predefined[0][8] = -56;
	predefined[1][8] = -98;
	predefined[0][9] = -51;
	predefined[1][9] = -108;
	predefined[0][10] = -44;
	predefined[1][10] = -116;
	predefined[0][11] = -36;
	predefined[1][11] = -120;
	predefined[0][12] = -29;
	predefined[1][12] = -117;
	predefined[0][13] = -28;
	predefined[1][13] = -107;
	predefined[0][14] = -28;
	predefined[1][14] = -98;
	predefined[0][15] = -33;
	predefined[1][15] = -86;

	if (!snapmode) {
		snapshot_counter = 16;
	}

	//init_adc_trigger_timer();
	//set_motor(1, 0);
	//set_motor(0, 0);
	sei();
	//printf("X,Y,Theta1,Theta2\r\n");
	EncoderCounts(0);
	while (1) {

		switch (run_mode) {

		case MANUAL: {
			if (!PINBbits._P0) {
				set_motor(0, 0);
				reset_encoder_count(0);
			} else if (!PINBbits._P1) {
				set_motor(0, 0.19214);
				reset_encoder_count(0);
			} else if (!PINBbits._P2) {
				set_motor(0, 0.19214 * 2);
				reset_encoder_count(0);
			} else if (!PINBbits._P3) {
				set_motor(0, -0.19214);
				reset_encoder_count(0);
			}
			if(currTime - encoder_time >= 100)
			{
				//printf("%f\r\n",get_encoder_degrees(0));
				encoder_time = currTime;
				float gz = get_accelerometer_axis_g(2);
				printf("%f\r\n",gz);
				//get_accelerometer_axis(1);
				//get_accelerometer_vref();

			}
			break;
		}

		case SNAPSHOT: {
			//pid_periodic();
			float x, y;
			calculate_forward_kinematics(get_arm_angle(LOWLINK),
					get_arm_angle(HIGHLINK), &x, &y);
			printf("%f, %f, %f, %f\r\n", x, y, get_arm_angle(LOWLINK),
					get_arm_angle(HIGHLINK));

			if (!PINBbits._P2) {
				snapshot_counter = 0;
				snapshot_cursor = 0;
			}

			if (!PINDbits._P0) //snapshot cap mode
			{
				if (prev_state == 1) {
					snapshot_cursor = 0;
				}
				prev_state = 0;
				unsigned int low = read_adc(4);
				unsigned int high = read_adc(5);
				float low_angle = map(low, HORIZONTALPOTBASE, VERTICALPOTBASE,
						0, 90) - 90.0;
				float high_angle = map(high, HORIZONTALPOTARM, VERTICALPOTARM,
						-90, 0);

				base_setpoint = low_angle;
				arm_setpoint = high_angle;
				if (!PINBbits._P3 && !snap_debounce && snapshot_counter <= 30) //button has been pressed
						{
					snap_debounce = true;
					snapshot[0][snapshot_counter] = get_arm_angle(LOWLINK);
					snapshot[1][snapshot_counter] = get_arm_angle(HIGHLINK);
					//calculate_forward_kinematics(get_arm_angle(LOWLINK),get_arm_angle(HIGHLINK),&x,&y);
					//printf("recorded: LOW: %f HIGH: %f move# %d x: %f y: %f\r\n",snapshot[0][snapshot_counter],snapshot[1][snapshot_counter],snapshot_counter,x,y);
					snapshot_counter++;
					snapshot_time = currTime;
					PORTDbits._P5 = 0;
					if (snapshot_counter == 1) {
						base_setpoint = get_arm_angle(LOWLINK);
						arm_setpoint = get_arm_angle(HIGHLINK);
					}
				} else if (PINBbits._P3 && snap_debounce) {
					if (currTime - snapshot_time > 500) //give 1/2 sec chill time
							{
						snap_debounce = false;
						PORTDbits._P5 = 1;
					}
				}
			} else {
				if (prev_state == 0) {
					base_setpoint = snapshot[0][0];
					arm_setpoint = snapshot[1][0];
					prev_state = 1;
				}
				//printf("Theta2: %f Theta3: %f Set1: %f Set2: %f Cursor %d\r\n",get_arm_angle(LOWLINK),get_arm_angle(HIGHLINK),base_setpoint,arm_setpoint,snapshot_cursor);
				if (pid_done) {
					snapshot_cursor =
							snapshot_cursor + 1 < snapshot_counter ?
									snapshot_cursor + 1 : 0;
					if (snapmode) {
						base_setpoint = snapshot[0][snapshot_cursor];
						arm_setpoint = snapshot[1][snapshot_cursor];
					} else {
						base_setpoint = predefined[0][snapshot_cursor];
						arm_setpoint = predefined[1][snapshot_cursor];
					}
					//printf("MOVE# %d Done\r\n",snapshot_cursor - 1 >= 0 ? snapshot_cursor - 1: snapshot_counter);
					//printf("NEW ANGLES LOW: %f HIGH: %f\r\n",snapshot[0][snapshot_cursor],snapshot[1][snapshot_cursor]);
					pid_done = false;
					pid_settle_time = currTime;
				}

			}
			pid_periodic();
			break;
		}
		}
	}

	return 0;
}

void stop_motors() {
	set_motor(0, 0);
	set_motor(1, 0);
}

void pid_periodic() {
	float current_base = get_current(0);
	float current_arm = get_current(1);
	if (pid_ready) {
		float base_val = calculate_pid_output(get_arm_angle(LOWLINK),
				base_setpoint, 0);
		set_motor(0,
				base_val
						+ sin((M_PI * get_arm_angle(LOWLINK)) / 180.0)
								* LOWLINKMG);
		set_motor(1,
				calculate_pid_output(get_arm_angle(HIGHLINK), arm_setpoint, 1));
		pid_ready = false;
		//printf("%f\r\n",get_arm_angle(HIGHLINK));
		//printf("pot: %d angle %f\r\n",adcReading_arm,angle_arm);
		//printf("%f ,%f ,%f ,%f\r\n",base_setpoint,get_arm_angle(LOWLINK),base_val,current_base);
		if (fabs(get_pid_error(LOWLINK)) < 4
				&& fabs(get_pid_error(HIGHLINK) < 2)) {
			if (currTime - pid_settle_time > 100) {
				pid_done = true;
			}
		} else {
			pid_settle_time = currTime;
			//printf("Not There LOW %f HIGH %f\r\n",get_pid_error(LOWLINK),get_pid_error(HIGHLINK));
		}
	}
}

float get_arm_angle(char link) {
	if (link == LOWLINK) {
		unsigned int adcReading_base = read_adc(2);
		return map(adcReading_base, HORIZONTALPOTBASE, VERTICALPOTBASE, 0, 90)
				- 90.0;
	} else if (link == HIGHLINK) {
		unsigned int adcReading_arm = read_adc(3);
		return map(adcReading_arm, HORIZONTALPOTARM, VERTICALPOTARM, -90, 0);
	}
	return 0;
}

/** channel is 0 or 1 **/
float get_current(int channel) {
	int raw = read_adc(channel);
	float current = fmap(raw, 0, 1024, 0, 5) - CURRENT_BIAS;
	if (fabs(current) < 0.01) {
		return 0.0;
	}
	return -current;
}

/** motor_id is 0 for first link, and 1 for second link.
 * velocity is -1 to 1. Positive is RHR going outward along motor shaft */
void set_motor(int motor_id, float velocity) {
	if (velocity > 1) {
		velocity = 1;
	}
	if (velocity < -1) {
		velocity = -1;
	}
	unsigned char dac_chan1 = 0;
	unsigned char dac_chan2 = 1;

	if (motor_id == 0) {
		dac_chan1 = 0;
		dac_chan2 = 1;
	} else if (motor_id == 1) {
		dac_chan1 = 3;
		dac_chan2 = 2;
	}

	int dac_out = fmap(velocity, -1, 1, -4096, 4096);
	if (dac_out >= 0) {
		setDAC(dac_chan1, 0);
		setDAC(dac_chan2, abs(dac_out));
	} else {
		setDAC(dac_chan1, abs(dac_out));
		setDAC(dac_chan2, 0);
	}

}

ISR( TIMER0_OVF_vect) {
}

ISR( TIMER0_COMPA_vect) {
	// PID
	pid_ready = true;
}

unsigned long getTime() {
	return currTime;
}

//make PWM yo
//using port PB3 - OC0A
void init_timer0() {
	TIMSK0 = (1 << OCIE0A); //enable interrupts on overflow!

	//enable CTC mode
	TCCR0A = (1 << WGM01);

	//select a 1024 for clock prescaler
	TCCR0B = (1 << CS00) | (1 << CS02);

	//set the value that compare register triggers at
	OCR0A = 180;
}

//using this timer to keep track of time?
void init_timer2() {
	//don't do any waveform generation
	TCCR2A = (1 << WGM21);  //setup for CTC mode
	TCCR2B = (1 << CS22) | (1 << CS20); //prescaler;
	OCR2A = 143;  //set the timer to count up
	TIMSK2 = (1 << OCIE2A); //enable interrupt on timercnt = OCR2A
}

void init_timer1() {
	DDRDbits._P5 = OUTPUT;

	TCCR1A = (1 << COM1A1);
	TCCR1B = (1 << CS12) | (1 << WGM13);
	ICR1 = 36000;
	OCR1A = 25200;

}

ISR( TIMER2_COMPA_vect) {
//TCNT2 = 0; //reset the timer2 count ;)
	cli();
	currTime++;
	sei();
//PORTBbits._P4 = ~PORTBbits._P4;
}

void init_led() {
	DDRBbits._P4 = OUTPUT;
	PORTBbits._P4 = 1;
}

void init_serial(unsigned int baudrate_coded) {
	UBRR0H = (unsigned char) (baudrate_coded >> 8);
	UBRR0L = (unsigned char) baudrate_coded;
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (3 << UCSZ00);
//UCSR0A = (1 << U2X0);
}

void transmit(char *data, unsigned int datalen) {
	for (int i = 0; i < datalen; i++) {
		while (!(UCSR0A & (1 << UDRE0)))
			;
		UDR0 = data[i];
	}
}

char receive_byte() {
	while (!(UCSR0A & (1 << RXC0)))
		;
	return UDR0;
}

void recieve(char *outdata, unsigned int bytes_to_read) {
	for (int i = 0; i < bytes_to_read; i++) {
		outdata[i] = receive_byte();
	}
}

void adcString(int adcVal, char* string) {
	string[0] = adcVal / 1000 + 0x30; //1000th place
	string[1] = (adcVal % 1000) / 100 + 0x30; //100th place
	string[2] = ((adcVal % 1000) % 100) / 10 + 0x30; //10th place
	string[3] = adcVal % 10 + 0x30;
}

void button_led() {
	DDRBbits._P3 = INPUT;
	DDRBbits._P4 = OUTPUT;
	while (1) {
		PORTBbits._P4 = PINBbits._P3;
		_delay_ms(20);
	}
}

void putCharDebug(char byteToSend) {
	char byte[1];
	byte[0] = byteToSend;
	transmit(byte, 1);
}

float fmap(float val, float in_min, float in_max, float out_min, float out_max) {
	return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int map(int val, int in_min, int in_max, int out_min, int out_max) {
	return (float) (val - in_min) / ((float) (in_max - in_min))
			* ((float) (out_max - out_min)) + out_min;
}
