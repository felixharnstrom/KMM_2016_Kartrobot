/*
 * Styrenhet.h
 *
 * Author: danma344
 */
#ifndef STYRENHET_H_
#define STYRENHET_H_
#include "../../uart/Communication.h"

typedef enum{LEFT_SIDE, RIGHT_SIDE} side_t; //What side of motors are we targeting?
typedef enum {NONE, FORWARD, BACKWARD, TOGGLE} direction_t; //What direction are the motors running in?
typedef enum {LEFT_TURN, RIGHT_TURN} turn_t; //What way are we turning?

/*
 * Initiates all the PWM connections                                    
 */
void initPWM();

/*
 * Transforms a number between 0-100 to the actual representation that is needed to set that speed.
 *
 * _Parameters_
 * (uint8_t) speedPercentage: The speed percentage between 0-100 to get the corresponding PWM setting for.
 *
 * _Returns_
 * (uint8_t) The PWM setting that corresponds to the input speed percentage. 
 */
uint8_t getTransformSpeed(uint8_t speedPercentage);
/*
 * Sleep for the given milliseconds.
 *
 * _Parameters_
 * (uint32_t) ms: the milliseconds to sleep
 */
void delay_ms(uint16_t ms);

/*
 * Sets the given pin under the given port to the value 0/1 given in direction_t.
 *
 * _Parameters_
 * (volatile uint8_t) *port: The port which the pin is located on.
 * (uint8_t) portnr: The pin number.
 * (direction_t) direction: The direction to set the pin towards.
 */
void setPinValue(volatile uint8_t *port, uint8_t portnr, direction_t direction);

/*
 * Stops all the motors.
 */
void stopMotors();

/*
 * Sets speed and direction for the given motor side.
 *
 * _Parameters_
 * (side_t) side: The robot side to control.
 * (direction_t) direction: The direction which the motor-side will drive towards.
 * (uint8_t) speedPercentage: The speed at which the side will drive. (0-100)
 */
void setSpeed(side_t side, direction_t direction, uint8_t speedPercentage);

/*
 * Turns the robot in the given direction at the given speed.
 *
 * _Parameters_
 * (turn_t) turn: The direction in which the robot will turn.
 * (uint8_t) speedPercentage: The speed at which the turn will be made. (0-100)
 */
void turnDirection(turn_t turn, uint8_t speedPercentage);

/*
 * Moves the robot in the towards the given direction and the given speed.
 *
 * _Parameters_
 * (direction_t) direction: The direction which the robot will drive towards.
 * (uint8_t) speedPercentage: The speed at which the driving will be made. (0-100)
 */
void move(direction_t direction, uint8_t speedPercentage);

/*
 * Set the wanted servo angle. 
 * 
 * _Parameters_
 * (uint8_t) angle: The servo angle to go to. (0 - 180)
 */
void setServoAngle(uint8_t angle);

/*
 * Moves the robot in the towards the given direction and the given speed for sleepTime milliseconds.
 *
 * _Parameters_
 * (direction_t) direction: The direction which the robot will drive towards.
 * (uint8_t) speedPercentage: The speed at which the driving will be made. (0-100)
 * (uint32_t) sleepTime: The time in ms that the robot will move.
 */
void moveMS(direction_t direction, uint8_t speedPercentage, uint16_t sleepTime);

/*
 * Turns the robot in the given direction at the given speed for sleepTime milliseconds.
 *
 * _Parameters_
 * (turn_t) turn: The direction in which the robot will turn.
 * (uint8_t) speedPercentage: The speed at which the turn will be made. (0-100)
 * (uint32_t) sleepTime: The time in ms that the robot will turn.
 */
void turnDirectionMS(turn_t turn, uint8_t speedPercentage, uint16_t sleepTime);

/*
 * Translates payload values and then moves the robot in the towards the given direction and the given speed.
 *
 * _Parameters_
 * (char*) payload: Move arguments given in byte form.
 */
void moveMSPL(char* payload);

/*
 * Translates payload values and then moves the robot in the towards the given direction and the given speed.
 *
 * _Parameters_
 * (char*) payload: Turn arguments given in byte form.
 */
void turnDirectionMSPL(char* payload);

/*
 * Translates payload values and sets speeds and direction for both motor sides.
 *
 * _Parameters_
 * (char*) payload: Speed and direction for both motor sides given in byte form.
 */
void setSpeedsPL(char* payload);

/*
 * Translates payload values and sets the given servo angle.
 * 
 * _Parameters_
 * (char*) payload: Servo angle given in byte form.
 */
void setServoAnglePL(char* payload);

/*
 * Send back diagnostic information to the control unit (meaning it should and must listen to the output from the controller)
 * If it doesn't there's a risk that the control unit will get incorrect data or get stuck in an indef. loop.
 * TODO: Make this approach more robust
 */
void sendDiag();


/*
 * Transmit and ACK meta-packet over UART without payload.
 */
void transmitAcknowledge();

/*
 * Handles execution of controller functions by passing the payload to the correct payload function, given by the t_msgType.
 *
 * _Parameters_
 * (t_msgType) function: The enum corresponding to the wanted function and payload.
 * (char*) payload: The payload with arguments for the t_msgType function.
 */
void executeFunction(t_msgType function, char* payload);
#endif