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
#define PACKET_SIZE (1 << 7)   // define packet size
#endif

#define SENSOR_ADRESS 128

  /**
   @brief   A enum type with all the distance sensor types taht can be read and they are in the format sensor_t
*/
typedef enum {IR_LEFT_BACK, IR_LEFT_FRONT, IR_RIGHT_BACK, IR_RIGHT_FRONT, IR_BACK, LIDAR} sensor_t;

 /**
   @brief   Starts AD conversion on a specific chanel
   @param   chanel is a uint8_t number of a chanel and the value can never be greater than 7
   @return  none
*/
void startADConversion(uint8_t channel);

  /**
   @brief   Initilize the analog/digital conversion
   @param   none
   @return  none
*/
void initAdc();

  /**
   @brief   Converts the current value from A/D conversion to corresponding voltage
   @param   none
   @return  A double that is a voltage value
*/
double getAdcVoltage();

  /**
   @brief   Uses getAdcVoltage for IR sensors and converts the value to a distance in centimeters
   @param   none
   @return  A double value that is the IR distance in centimeters
*/
double getIrDistance();

  /**
   @brief   Uses getAdcVoltage for LIDAR and converts the value to a distance in centimeters
   @param   none
   @return  A double value that is the LIDAR distance in centimeters
*/
double getLidarDistance();

   /**
   @brief   Waits for the current A/D conversion to finnish
   @param   none
   @return  none
*/
void waitForADConversion();

  /**
   @brief   Initilize LIDAR
   @param   none
   @return  none
*/
void initLidar();

/*
 * Return the distance recorded by a given sensor.
 */
   /**
   @brief   Reads a given sensor and returns the distance
   @param   s is the sensor that will be read in hte format sensor_t
   @return  A double value that is distance in centimeters
*/
double readSensor(sensor_t s);

#endif /* SENSORENHET_H_ */