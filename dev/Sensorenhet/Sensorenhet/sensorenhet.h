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
void startADConversion(uint8_t channel);

/*
 * Initiate the analog/digital conversion
 */
void initAdc();

/*
 *	Convert the current value at ADC to corresponding voltage
 */
double getAdcVoltage();

/*
 * Convert voltage given by getIrVoltage() to distance in cm.
 */
double irOutputToCentimeters();

double lidarOutputToCentimeters();

/*
 *	Wait until there is no current conversions left
 */
void waitForADConversion();

void initLidar();

/*
 * Return the distance recorded by a given sensor.
 */
double readSensor(sensor_t s);

#endif /* SENSORENHET_H_ */