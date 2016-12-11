#ifndef SENSORENHET_H_
#define SENSORENHET_H_

/**
 * @brief 8MHz clock speed.
 */
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
#include <avr/wdt.h>

/**
 * @brief Size of a single UART packet in bits.
 */
#ifndef PACKET_SIZE
#define PACKET_SIZE (1 << 7)
#endif

/**
 * @brief Sensor unit UART address.
 */
#ifndef SENSOR_ADRESS
#define SENSOR_ADRESS 1
#endif

/**
 * @brief The maximum detectable distance (in cm) by the IR sensors.
 */
#ifndef MAX_IR_DISTANCE_CM
#define MAX_IR_DISTANCE_CM 30.0
#endif

/**
  * @brief All available sensors.
  */
typedef enum {IR_LEFT_BACK, IR_LEFT_FRONT, IR_RIGHT_BACK, IR_RIGHT_FRONT, IR_BACK, LIDAR, REFLEX_LEFT, REFLEX_RIGHT} sensor_t;

/**
 * @brief Starts A/D conversion on the specified channel.
 *
 * @param   channel The channel to inititate A/D conversion. Must not be greater than 7.
 */
void startADConversion(uint8_t channel);

/**
 * @brief Initialize the A/D converter.
 */
void initAdc();

/**
 * @brief Converts the current value from A/D conversion to corresponding voltage.
 *
 * @return  Voltage value.
 */
double getAdcVoltage();

/**
 * @brief Uses the A/D converter for IR sensors and converts the value to a distance in centimeters.
 *
 * @return  The sensor's distance in centimeters or MAX_IR_DISTANCE_CM, whichever is smallest.
 */
double getIrDistance();

/**
 * @brief Uses the A/D converter for LIDAR and converts the value to a distance in centimeters
 *
 * @return  The LIDAR distance in centimeters.
 */
double getLidarDistance();

/**
 * @brief Waits for the current A/D conversion to finish.
 */
void waitForADConversion();

/**
 * @brief Initilize LIDAR.
 */
void initLidar();

/**
 * @brief Reads a given IR sensor and returns the distance.
 *
 * @param   s The sensor to read.
 * @return  Given sensor's distance in centimeters.
*/
double readSensor(sensor_t s);

/**
 * @brief Get the corresponding sensor channel of a given message type.
 *
 * @param   mst The message type to convert.
 * @return  The message type's corresponding channel, or INV if the message type is not a valid sensor.
 */
sensor_t msgTypeToSensor(t_msgType mst);

/**
 * @brief Transmits a DONE without payload over UART.
 */
void sendError();

/**
 * @brief Transmits a SENSOR_DATA message with a payload consisting of a 2-byte number as two single-byte numbers, starting with the MSBits.
 *
 * @param val   The payload to transmit.
 */
void sendReply(uint16_t val);

/**
 * @brief Returns the 8 least significant bits of a given integer.
 *
 * @param   n The integer to extract from.
 * @return  The 8 least significant bits.
 */
uint8_t lowestByte(unsigned int n);

/**
 * @brief Convert int to string and send it over UART for debugging purposes.
 *
 * @param n The integer to send.
 */
void sendInt(int n);

/**
 * @brief Returns the average gyro output over a number of iterations.
 *
 * Intended to be used in standstill to acquire the bias for gyroOutputToAngularRate().
 *
 * @return The bias value.
 */
long calculateBias();

/**
 * @brief Writes val to reg on MPU-6050.
 *
 * @param   reg The register to write to.
 * @param   val The value to write to reg.
 */
void MPU6050_writereg(uint8_t reg, uint8_t val);

/**
 * @brief Reads from reg on MPU-6050,
 *
 * @param   reg The register to read from (0x47 = gyro on z-axis).
 * @return  The register value.
 */
uint16_t MPU6050_readreg(uint8_t reg);

/**
 * @brief Sets up the MPU-6050
 * 
 * See function and datasheet for setup values.
 */
void Init_MPU6050();

/**
 * @brief Initiates the reflex sensors on both left and right side.
 */
void initReflex();

/**
 * @brief Returns the voltage for a given reflex sensor.
 *
 * @param s The sensor type, must be REFLEX_LEFT or REFLEX_RIGHT.
 * @return  The voltage from chosen reflex sensor.
 */
double getReflexVoltage(sensor_t s);

#endif /* SENSORENHET_H_ */