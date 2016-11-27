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
#include "../../uart/Communication.h"
#include "i2c/i2cmaster.h"


#ifndef PACKET_SIZE
#define PACKET_SIZE (1 << 7)
#endif

#ifndef SENSOR_ADRESS
#define SENSOR_ADRESS 1
#endif

#ifndef MAX_IR_DISTANCE_CM
#define MAX_IR_DISTANCE_CM 30.0
#endif




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

/*
 * Uses getAdcVoltage for IR sensors and converts the value to a distance in centimeters
 * 
 * _returns_
 * (double): the IR distance in centimeters, or MAX_IR_DISTANCE_CM, whichever is smallest.
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

/*
 * Get the corresponding sensor channel of a given message type.
 * 
 * _parameters_
 * (t_msgType) mst: the message type to convert
 *
 * _returns_
 * (sensor_t) its corresponding channel, or INV if the message type is not a valid sensor.
 */
sensor_t msgTypeToSensor(t_msgType mst);

/*
 * Transmits a DONE without payload over UART.
 */
void sendError();

/*
 * Transmits a SENSOR_DATA message with a payload consisting of a 2-byte number
 *  as two single-byte numbers, starting with the MSBits.
 *
 * _parameters_
 * (uint16_t val): the payload
 */
void sendReply(uint16_t val);

/*
 * Returns the 8 least significant bits of a given number.
 *
 * _parameters_
 * (unsigned int) n: the number to extract from
 * 
 * _returns_
 * (uint16_t): the 8 least significant bits
 */
uint8_t lowestByte(unsigned int n);

/*
 * Convert int to string and send it over UART for debugging porpuses.
 *
 * _parameters_
 * (int) n: the number to send
 */
void sendInt(int n);

/*
 * Writes val to reg on MPU-6050
 *
 * _parameters_
 * (uint8_t) reg: the register to write on
 * (uint8_t) val: the value to write on reg
 */
void MPU6050_writereg(uint8_t reg, uint8_t val);

/*
 * Reads from reg on MPU-6050
 *
 * _parameters_
 * (uint8_t) reg: the register to read from (0x47 = gyro on z-axis)
 * _returns_
 * (uint16_t): the register value
 */
uint16_t MPU6050_readreg(uint8_t reg);

/*
 * Sets up the MPU-6050
 * 
 * See function and datasheet for setup values
 */
void Init_MPU6050();



#endif /* SENSORENHET_H_ */