#ifndef STYRENHET_H_
#define STYRENHET_H_
#include "../../uart/Communication.h"

/**
 * @brief Motors available to target.
 */
typedef enum{LEFT_SIDE, RIGHT_SIDE} side_t;

/**
 * @brief Motor directions.
 */
typedef enum {NONE, FORWARD, BACKWARD, TOGGLE} direction_t;

/**
 * @brief Turn directions.
 */
typedef enum {LEFT_TURN, RIGHT_TURN} turn_t;

/**
 * @brief Initiates all the PWM connections                                    
 */
void initPWM();

/**
 * @brief Transforms a number between 0-100 to the actual representation that is needed to set that speed.
 *
 * @param speedPercentage   The speed percentage between 0-100 to get the corresponding PWM setting for.
 * @return                  The PWM setting that corresponds to the input speed percentage. 
 */
uint8_t getTransformSpeed(uint8_t speedPercentage);

/**
 * @brief Sleep for the given milliseconds.
 *
 * @param ms    Sleep duration in ms.
 */
void delay_ms(uint16_t ms);

/**
 * @brief Sets the given pin under the given port to the value 0/1 given in direction_t.
 *
 * @param port      The port which the pin is located on.
 * @param portnr    The pin number.
 * @param direction The direction to set the pin towards.
 */
void setPinValue(volatile uint8_t *port, uint8_t portnr, direction_t direction);

/**
 * @brief Stops all the motors.
 */
void stopMotors();

/**
 * @brief Sets speed and direction for the given motor side.
 *
 * @param side              The robot side to control.
 * @param direction         The direction which the motor-side will drive towards.
 * @param speedPercentage   The speed at which the side will drive (0-100).
 */
void setSpeed(side_t side, direction_t direction, uint8_t speedPercentage);

/**
 * @brief Turns the robot in the given direction at the given speed.
 *
 * @param turn              The direction in which the robot will turn.
 * @param speedPercentage   The speed at which the turn will be made (0-100).
 */
void turnDirection(turn_t turn, uint8_t speedPercentage);

/**
 * @brief Moves the robot in the towards the given direction and the given speed.
 *
 * @param direction         The direction which the robot will drive towards.
 * @param speedPercentage   The speed at which the driving will be made (0-100).
 */
void move(direction_t direction, uint8_t speedPercentage);

/**
 * @brief Set the wanted servo angle. 
 * 
 * @param angle     The servo angle to go to (0-180).
 */
void setServoAngle(uint8_t angle);

/**
 * @brief Moves the robot in the towards the given direction and the given speed for sleepTime milliseconds.
 *
 * @param direction         The direction which the robot will drive towards.
 * @param speedPercentage   The speed at which the driving will be made (0-100).
 * @param sleepTime         The time in ms that the robot will move.
 */
void moveMS(direction_t direction, uint8_t speedPercentage, uint16_t sleepTime);

/**
 * @brief Turns the robot in the given direction at the given speed for sleepTime milliseconds.
 *
 * @param turn              The direction in which the robot will turn.
 * @param speedPercentage   The speed at which the turn will be made (0-100).
 * @param sleepTime         The time in ms that the robot will turn.
 */
void turnDirectionMS(turn_t turn, uint8_t speedPercentage, uint16_t sleepTime);

/**
 * @brief Translates payload values and then moves the robot in the towards the given direction and the given speed.
 *
 * @param[out] payload   Move arguments given in byte form.
 */
void moveMSPL(char* payload);

/**
 * @brief Translates payload values and then moves the robot in the towards the given direction and the given speed.
 *
 * @param[out] payload:  Turn arguments given in byte form.
 */
void turnDirectionMSPL(char* payload);

/**
 * @brief Translates payload values and sets speeds and direction for both motor sides.
 *
 * @param[out] payload   Speed and direction for both motor sides given in byte form.
 */
void setSpeedsPL(char* payload);

/**
 * @brief Translates payload values and sets the given servo angle.
 * 
 * @param[out] payload   Servo angle given in byte form.
 */
void setServoAnglePL(char* payload);

/**
 * @brief Send back diagnostic information to the control unit (meaning it should and must listen to the output from the controller).
 *
 * If it doesn't there's a risk that the control unit will get incorrect data or get stuck in an indef. loop.
 *
 * TODO: Make this approach more robust
 */
void sendDiag();


/**
 * @brief Transmit and ACK meta-packet over UART without payload.
 */
void transmitAcknowledge();

/**
 * @brief Handles execution of controller functions by passing the payload to the correct payload function, given by the t_msgType.
 *
 * @param function  The enum corresponding to the wanted function and payload.
 * @param payload   The payload with arguments for the t_msgType function.
 */
void executeFunction(t_msgType function, char* payload);
#endif