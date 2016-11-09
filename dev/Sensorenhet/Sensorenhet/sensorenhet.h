/*
 * sensorenhet.h
 *
 * Created: 11/9/2016 10:49:32 AM
 *  Author: emino969
 */ 

#ifndef SENSORENHET_H_
#define SENSORENHET_H_

#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <avr/interrupt.h>

#ifndef PACKET_SIZE
#define PACKET_SIZE 8   // define packet size
#endif

typedef enum {IR_LEFT_BACK, IR_LEFT_FRONT, IR_RIGHT_BACK, IR_RIGHT_FRONT, IR_BACK, LIDAR} sensor_t;

/*
 *	Start a AD conversion
 */
void start_ir_read(uint8_t channel);

/*
 * Initiate the analog/digital conversion
 */
void init_ad();

/*
 *	Convert the current value at ADC to corresponding voltage
 */
double get_ir_voltage();

/*
 * Convert voltage given by getIrVoltage() to distance in cm.
 */
double ir_output_to_centimeters();

double lidar_output_to_centimeters();

/*
 *	Wait until there is no current conversions left
 */
void wait_for_ir_read();

void init_lidar();

/*
 * Return the distance recorded by a given sensor.
 */
double read_sensor(sensor_t s);

#endif /* SENSORENHET_H_ */