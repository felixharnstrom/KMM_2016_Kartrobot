/*
* Sensorenhet.c
*
* Created: 11/4/2016 4:08:27 PM
*  Author: emino969
*/
#define F_CPU 8000000UL	//8MHz

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include "UART.h"

typedef enum{
    LEFT_SIDE,
    RIGHT_SIDE
} side_t;
typedef enum {
    NONE = -1,
    FORWARD = 1,
    BACKWARD = 0
} direction_t; //Other possible solution is to handle case for negative, none and positive speed, eg -100 <= speed <= 100
typedef enum {
    LEFT_TURN,
    RIGHT_TURN
} turn_t; //Could perhaps be modified into a single enum


/************************************************************************/
/* Initiates all the PWM connections                                     */
/************************************************************************/
void initPWM(){
    //PWM for LIDAR tower servo
    TCCR3A |= (1 << WGM30) | (1 << WGM31) | (1 << COM3B1); //Com3B0 = 0 for inverted
    TCCR3B |= (1 << WGM32) | (1 << WGM33) | (1 << CS31); //WGM32 = 0 should yield Timer 0 -> Max and then reset (1 << WGM32)
    OCR3A = 20000;	//Corresponds to 50Hz
    OCR3B = 700; //The start value for the duty cycle
    
    //Robot left side
    TCCR0A |= (1 << WGM00) | (1 << WGM01) | (1 << COM0B1) | (1 << COM0A1); //Com0B0 = 0 for inverted
    TCCR0B |= (1 << CS01) | (1 << CS00);
    OCR0A = 0; //Speed is 0-255 (But since we need atleast a pulse for the speed to work we will use 0 -> 240) OCR0A is right side Pin 6 is direction
    OCR0B = 0; //OCR0B is left side Pin 7 is direction
}

/************************************************************************/
/* Transforms a number between 0-100 to the actual representation that is needed to set that speed */
/************************************************************************/
uint8_t getTransformSpeed(uint8_t speedPercentage){
    return (int)(speedPercentage*2.52);
}

/************************************************************************/
/* Delays for the given ms                                                                     */
/************************************************************************/
void delay_ms(uint32_t ms){
    while(ms > 0){
        _delay_ms(1);
        --ms;
    }
}

void setPinValue(volatile uint8_t *port, uint8_t portnr, direction_t direction){
    switch(direction){
        case FORWARD:
            *port |= (1 << portnr);
        break;
        case BACKWARD:
            *port &= ~(1 << portnr);
        break;
        default:
            //Do nothing
        break;
    }
}

void setSpeed(side_t side, direction_t direction, uint8_t speedPercentage){
    switch(side){
        case RIGHT_SIDE:
            OCR0A = getTransformSpeed(speedPercentage);
            setPinValue(&PORTB, PORTB5, direction);
        break;
        case LEFT_SIDE:
            OCR0B = getTransformSpeed(speedPercentage);
            setPinValue(&PORTB, PORTB6, direction);
        break;
    }
}

void turnDirection(turn_t turn, uint8_t speedPercentage){
    switch(turn){
        case RIGHT_TURN:
            setSpeed(RIGHT_SIDE, FORWARD, speedPercentage);
            setSpeed(LEFT_SIDE, BACKWARD, speedPercentage);
        break;
        case LEFT_TURN:
            setSpeed(RIGHT_SIDE, BACKWARD, speedPercentage);
            setSpeed(LEFT_SIDE, FORWARD, speedPercentage);
        break;
    }
}

void stopMotors(){
    setSpeed(RIGHT_SIDE, NONE, 0);
    setSpeed(LEFT_SIDE, NONE, 0);
}

void turnDirectionMS(turn_t turn, uint8_t speedPercentage, uint32_t sleepTime){
    turnDirection(turn, speedPercentage);
    delay_ms(sleepTime);
    stopMotors();
}

void turnDirectionAngle(turn_t turn, uint8_t angle){
    //TODO: We got to calculate the time to do this turning. Weight/speed/wheels affect it.
    
}

//Turns x*90degrees, eg. x squares using a constant speed for precision
void turnSquares(turn_t turn, uint8_t times){
    
}

//payload[0]
void turnSquaresPL(char* payload){
    uint8_t direction = (uint8_t)payload[0];
    uint8_t turns = (uint8_t)payload[1];
    switch(direction){
        case 0:
            turnSquares(LEFT_TURN,turns);
            break;
        case 1:
            turnSquares(RIGHT_TURN, turns);
            break;
        default:
            //Should not happen
            break;
    }    
}


void move(direction_t direction, uint8_t speedPercentage){
    setSpeed(RIGHT_SIDE, direction, speedPercentage);
    setSpeed(LEFT_SIDE, direction, speedPercentage);
}

void moveMS(direction_t direction, uint8_t speedPercentage, uint32_t sleepTime){
    move(direction, speedPercentage);
    delay_ms(sleepTime);
    stopMotors();
}

void moveSquares(direction_t direction, uint8_t squares){
    //TODO: We got to calculate the time to do this move. Weight/speed/wheels affect it.
    moveMS(direction,40,2000*squares);
}

//For outside use
void moveSquaresPL(char* payload){
    // uint8_t directionValue = (uint8_t)payload[0];
    uint8_t squareMask = 0x03;
    uint8_t directionMask = 0x01;
    
    
    uint8_t directionValue = ((uint8_t)payload[0]) & directionMask; // Direction number mask 0000 0001
    uint8_t squares = ((uint8_t)payload[1]) & squareMask;
    //((uint8_t)payload[1])&0x03;
    switch(directionValue){
        case 0:
        moveSquares(BACKWARD, squares);
        break;
        case 1:
        moveSquares(FORWARD, squares);
        break;
        default:
        //Should not happen if kontrollenhet is correctly using the protocol
        break;
    }
}


/*
 * Set the wanted servo angle. (0 is left, 90 middle and 180 right side)
 */
void setServoAngle(uint8_t angle)
{
    if(angle > 180){
        angle = 180; //Otherwise we might hurt the servo
    }        
    OCR3B = 710 + (int)(8.33 * angle);
}

void setServoAnglePL(char* payload){
    uint8_t angle = (uint8_t)payload[0];
    if(angle > 180)
        angle = 180;
    setServoAngle(angle);
}



//object* ?
void executeFunction(t_msgType function, char* payload){
    int adr = 0;
    int size = 2;
    switch(function){
        case ECHO :
            uart_msg_transmit(&adr, &size, &function, payload);
            break;
        case MOVE_SQUARES :
            moveSquaresPL(payload);
            break;
        case TURN_DEGREES :
            //TODO
            break;
        case TURN_SQUARES :
            turnSquaresPL(payload);
            break;
        case SET_SERVO_ANGLE :
            setServoAnglePL(payload);
            break;       
    }
}

int main(void)
{
    DDRB = 0xFF;	//All pins on port A as output
    DDRA = 0xFF;
    
    initPWM();
    uart_init();
    _delay_ms(10000);
    while(1){
        int adr;            //Not needed
        int size;           //Size of payload
        char payload[7];    //Payload data 0->6
        t_msgType funcEnum; //Corresponding function to be used with
        PORTA |= (0 << PORTA1);
        uart_msg_receive(&adr, &size, &funcEnum, payload);   
        PORTA |= (1 << PORTA1);
        //Not currently caring about size 
        executeFunction(funcEnum, payload);
    }
    
    
    /*moveMS(FORWARD,40, 2000);
    moveMS(BACKWARD,40, 2000);
    turnDirectionMS(RIGHT_TURN, 50, 1300);
    turnDirectionMS(LEFT_TURN, 50, 1300); */
    /*
        char c;
        int packet[8];
        //DDRA = (1<<DDA7)|(1<<DDA6)|(1<<DDA5)|(1<<DDA4)|(1<<DDA3)|(1<<DDA2)|(1<<DDA1)|(1<<DDA0);	//All pins on port A as output
        uart_init();
        while(1)
        {
            c = uart_receive();
            if(c != 0){
                PORTA |= (1 << PORTA0);
                for (int i = 0; i < 8; ++i) {
                    packet[i] = (c >> i) & 1;
                }
                for (int i = 7; i >= 0; --i) {
                    if(packet[i] == 1){
                        uart_transmit('1');
                        }else{
                        uart_transmit('0');
                    }
                }
            }
        }
        */
}