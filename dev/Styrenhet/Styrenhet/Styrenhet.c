/*
* Styrenhet.c
*
*  Author: danma344
*/
#define F_CPU 8000000UL    //8MHz

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "Styrenhet.h"

void initPWM(){
    //PWM for LIDAR tower servo
    TCCR3A |= (1 << WGM30) | (1 << WGM31) | (1 << COM3B1); //Com3B0 = 0 for inverted
    TCCR3B |= (1 << WGM32) | (1 << WGM33) | (1 << CS31); //WGM32 = 0 should yield Timer 0 -> Max and then reset (1 << WGM32)
    OCR3A = 20000;    //Corresponds to 50Hz
    OCR3B = 1556; //The start value for the duty cycle (770, 0 deg | 1553, 90 deg)
    
    //Robot left side
    TCCR0A |= (1 << WGM00) | (1 << WGM01) | (1 << COM0B1) | (1 << COM0A1); //Com0B0 = 0 for inverted
    TCCR0B |= (1 << CS01) | (1 << CS00);
    OCR0A = 0; //Speed is 0-255 (But since we need atleast a pulse for the speed to work we will use 0 -> 240) OCR0B is right side Pin 6 is direction
    OCR0B = 0; //OCR0A is left side Pin 7 is direction
}

uint8_t getTransformSpeed(uint8_t speedPercentage){
    return (int)(speedPercentage*2.52);
}


void delay_ms(uint16_t ms){
    while(ms > 0){
        _delay_ms(1);
        --ms;
    }
}

void setPinValue(volatile uint8_t *port, uint8_t portnr, direction_t direction){
    switch(direction){
        case BACKWARD:
            *port &= ~(1 << portnr);
        break;
        case FORWARD:
            *port |= (1 << portnr);
        break;
        default:
            //Do nothing (Also do this for value NONE)
        break;
    }
}

void stopMotors(){
    setSpeed(RIGHT_SIDE, NONE, 0);
    setSpeed(LEFT_SIDE, NONE, 0);
}

void setSpeed(side_t side, direction_t direction, uint8_t speedPercentage){
    switch(side){
        case RIGHT_SIDE:
        OCR0B = getTransformSpeed(speedPercentage);
        setPinValue(&PORTB, PORTB6, direction);
        break;
        case LEFT_SIDE:
        OCR0A = getTransformSpeed(speedPercentage);
        setPinValue(&PORTB, PORTB5, direction);
        break;
    }
}

void turnDirection(turn_t turn, uint8_t speedPercentage){
    switch(turn){
        case LEFT_TURN:
        setSpeed(RIGHT_SIDE, FORWARD, speedPercentage);
        setSpeed(LEFT_SIDE, BACKWARD, speedPercentage);
        break;
        case RIGHT_TURN:
        setSpeed(RIGHT_SIDE, BACKWARD, speedPercentage);
        setSpeed(LEFT_SIDE, FORWARD, speedPercentage);
        break;
    }
}

void move(direction_t direction, uint8_t speedPercentage){
    setSpeed(RIGHT_SIDE, direction, speedPercentage);
    setSpeed(LEFT_SIDE, direction, speedPercentage);
}

void setServoAngle(uint8_t angle){
    if(angle > 180){
        angle = 180; //Otherwise we might hurt the servo
    }
    OCR3B = 773 + (uint16_t)(8.72 * angle);
}

void moveMS(direction_t direction, uint8_t speedPercentage, uint16_t sleepTime){
    move(direction, speedPercentage);
    delay_ms(sleepTime);
    stopMotors();
}

void turnDirectionMS(turn_t turn, uint8_t speedPercentage, uint16_t sleepTime){
    turnDirection(turn, speedPercentage);
    delay_ms(sleepTime);
    stopMotors();
}

void moveMSPL(char* payload){
    uint8_t directionMask = 0x01;
    
    uint8_t directionValue = ((uint8_t)payload[0]) & directionMask; // Direction number mask 0000 0001
    uint8_t speed= ((uint8_t)payload[1]);
    //Otherwise motors go cray-cray
    if(speed > 100)
        speed = 100;
    uint16_t ms= (((uint16_t)payload[2]) << 8);
    ms += (uint16_t)payload[3];
    side_t direction;
    
    switch(directionValue){
        case 0:
            direction = BACKWARD;
            break;
        case 1:
            direction = FORWARD;
            break;
        default:
            return;
            //Should not happen if kontrollenhet is correctly using the protocol
            break;
    }
    if(ms != 0){
        moveMS(direction, speed, ms);
    }
    else{
        move(direction, speed);
    }
}

void turnDirectionMSPL(char* payload){
    uint8_t turnMask = 0x01;
    
    uint8_t turnValue = ((uint8_t)payload[0]) & turnMask; // Direction number mask 0000 0001
    uint8_t speed = ((uint8_t)payload[1]);
    //Otherwise motors go cray-cray
    if(speed > 100)
     speed = 100;
    uint16_t ms= (((uint16_t)payload[2]) << 8);
    ms += (uint16_t)payload[3];
    
    turn_t turn;
    
    switch(turnValue){
        case 0:
            turn = LEFT_TURN;
            break;
        case 1:
            turn = RIGHT_TURN;
            break;
        default:
            return;
            //Should not happen if kontrollenhet is correctly using the protocol
        break;
    }
    
    if(ms != 0){
        turnDirectionMS(turn, speed, ms);
    }
    else{
        turnDirection(turn, speed);
    }
}

void setSpeedsPL(char* payload){
    uint8_t directionMask = 0x01;
    
    uint8_t leftDirectionValue = ((uint8_t)payload[0]) & directionMask; // Direction number mask 0000 0001
    uint8_t leftSpeed = ((uint8_t)payload[1]);
    
    uint8_t rightDirectionValue = ((uint8_t)payload[2]) & directionMask; // Direction number mask 0000 0001
    uint8_t rightSpeed = ((uint8_t)payload[3]);
    
    //Otherwise motors go cray-cray
    if(leftSpeed > 100)
       leftSpeed = 100;
      
    if(rightSpeed > 100)
       rightSpeed = 100;

    direction_t leftDirection; //0 left
    direction_t rightDirection; //1 right
    
    switch(leftDirectionValue){
        case 0:
            leftDirection = BACKWARD;
            break;
        case 1:
            leftDirection = FORWARD;
            break;
        default:
            return;
            //Should not happen if kontrollenhet is correctly using the protocol
    }    

    switch(rightDirectionValue){
        case 0:
            rightDirection = BACKWARD;
            break;
        case 1:
            rightDirection = FORWARD;
            break;
        default:
            return;
            //Should not happen if kontrollenhet is correctly using the protocol
    }

    setSpeed(LEFT_SIDE,leftDirection,leftSpeed);
    setSpeed(RIGHT_SIDE,rightDirection,rightSpeed);    
}

void setServoAnglePL(char* payload){
    uint8_t angle = (uint8_t)payload[0];
    setServoAngle(angle);
}

void sendDiag(){
    uint16_t servoMsbMask = 0xFF00;
    uint16_t servoLsbMask = 0x00FF;
    int payloadLength = 6;
    int adress = 0;
    t_msgType send_diag_type = MOTOR_GET_DIAG;
    
    /*
    Packet 0, 1 contains left side information
    Packet 2, 3 contains right side information
    0,2 contains motor direction, 0=backward, 1=forward
    1,3 contains motor PWM(M) (M/2.5 for speed percentage)
    4,5 contains servo PWM (S) 4 being MSB, 5 LSB eg S=[4]*2^8+[5] ((S-708)/8.45 for gyro angle)
    */
    uint8_t rightSideDirection = (PORTB & (1 << PINB6)) != 0;
    uint8_t rightSidePWM = OCR0B;
    
    uint8_t leftSideDirection = (PORTB & (1 << PINB5)) != 0;
    uint8_t leftSidePWM = OCR0A;
    
    uint16_t servoPWM = OCR3B; //This might need to be casted
    uint8_t servoMsbPWM = (uint8_t)((servoPWM & servoMsbMask) >> 8);
    uint8_t servoLsbPWM = (uint8_t)(servoPWM & servoLsbMask);
    
    char payload[6] = {leftSideDirection, leftSidePWM, rightSideDirection, rightSidePWM, servoMsbPWM, servoLsbPWM};
    uart_msg_transmit(&adress, &payloadLength, &send_diag_type, payload); //TODO: Different msgType as this is a response?
    //Just receive ack so we don't mess something up
    int adr = 0;
    int size = 0;
    t_msgType msg_type;
    char *payload_dummy;
    uart_msg_receive(&adr,&size, &msg_type,payload_dummy); //ack dummy
}

void transmitAcknowledge() {
    int adr = 0;
    int size = 0;
    t_msgType type = ACK;
    char payload;
    uart_msg_transmit(&adr, &size, &type, &payload);
}

void executeFunction(t_msgType function, char* payload){
    int adr = 0;
    int size = 2;
    switch(function){
        case ECHO :
            uart_msg_transmit(&adr, &size, &function, payload);
            break;
        case MOTOR_MOVE_MS :
            moveMSPL(payload);
            break;
        case MOTOR_TURN_MS :
            turnDirectionMSPL(payload);
            break;  
        case MOTOR_SET_SIDE_SPEED:
            setSpeedsPL(payload);
            break;
        case MOTOR_SET_SERVO_ANGLE :
            setServoAnglePL(payload);
            break;       
        case MOTOR_STOP_MOTORS :
            stopMotors();
            break;
        case MOTOR_GET_DIAG :
            sendDiag(); //Sends diag. data over uart
            break;
        default:
            //Do nothing
            break;
    }
}

int main(void)
{
    DDRA = 0xFF;    //All pins on port A as output
    DDRB = 0xFF;    //All pins on port B as output
    initPWM();
    comm_init();
    //Set interrupts enabled
    sei();
    _delay_ms(500);
    PORTA |= (1 << PORTA0) | (1 << PORTA1);
    while(1){
        int adr;            //Not needed
        int size;           //Size of payload
        char payload[7];    //Payload data 0->6
        t_msgType funcEnum; //Corresponding function to be used with
        PORTA |= ~(1 << PORTA1); //Set segment PORTA1 to 0 while waiting for new uart data
        uart_msg_receive(&adr, &size, &funcEnum, payload);  
        PORTA |= (1 << PORTA1); //Set segment PORTA1 to 1 while executing received command
        //transmitAcknowledge();
        executeFunction(funcEnum, payload);
    }
}