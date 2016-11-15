/*
 * Styrenhet.h
 *
 * Author: danma344
 */
#ifndef STYRENHET_H_
#define STYRENHET_H_
#include "../../uart/Communication.h"

typedef enum{LEFT_SIDE, RIGHT_SIDE} side_t; //What side of motors are we targeting?
typedef enum {NONE, FORWARD, BACKWARD} direction_t; //What direction are the motors running in?
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
void delay_ms(uint32_t ms);
void setPinValue(volatile uint8_t *port, uint8_t portnr, direction_t direction);

void stopMotors();
void setSpeed(side_t side, direction_t direction, uint8_t speedPercentage);
void turnDirection(turn_t turn, uint8_t speedPercentage);
void move(direction_t direction, uint8_t speedPercentage);
/*
 * Set the wanted servo angle. (0 is left, 90 middle and 180 right side)
 */
void setServoAngle(uint8_t angle);

void moveMS(direction_t direction, uint8_t speedPercentage, uint32_t sleepTime);
void turnDirectionMS(turn_t turn, uint8_t speedPercentage, uint32_t sleepTime);

void moveMSPL(char* payload);
void turnDirectionMSPL(char* payload);
void setSpeedPL(char* payload);
void setServoAnglePL(char* payload);
/*Send back diagnostic information to the control unit
(meaning it should and must listen to the output from the controller)
If it doesn't there's a risk that the control unit will get incorrect data or get stuck in an indef. loop.
TODO: Make this approach more robust
*/
void getDiag();

void executeFunction(t_msgType function, char* payload);
#endif