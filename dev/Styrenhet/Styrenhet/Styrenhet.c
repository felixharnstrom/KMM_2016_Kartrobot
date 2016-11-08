/*
* Sensorenhet.c
*
* Created: 11/4/2016 4:08:27 PM
*  Author: emino969
*/
#define F_CPU 8000000UL	//8MHz
#define BAUD 9600                                   // define baud
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)            // set baud rate value for UBRR


#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

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

// function to initialize UART
void uart_init (void)
{
    UBRR0H = (BAUDRATE>>8);                      // shift the register right by 8 bits
    UBRR0L = BAUDRATE;                           // set baud rate
    UCSR0B|= (1<<TXEN0)|(1<<RXEN0);                // enable receiver and transmitter
    UCSR0C|= (1<<USBS0)|(3<<UCSZ00);			// 8bit data format
}

// function to send data
void uart_transmit (unsigned char data)
{
    loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
    UDR0 = data;
}

// function to receive data
unsigned char uart_receive (void)
{	if(RXC0 == 1){
        //loop_until_bit_is_set(UCSR0A, RXC0); /* Wait until data exists. */
    return (unsigned)UDR0;
}
else{
    return 0;
}
}

void receive_packet (char* buffer){
    unsigned char a;
    int i = 0;
    while(i < 8){
        a = uart_receive();
        buffer[i] = a;
        i++;
    }
}

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
    return (int)(speedPercentage*2.5);
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

void moveSquare(direction_t direction, uint8_t speedPercentage){
    //TODO: We got to calculate the time to do this move. Weight/speed/wheels affect it.
    
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

/*
 * Set the wanted servo angle. (0 is left, 90 middle and 180 right side)
 */
void setAngle(uint8_t angle)
{
    OCR3B = 710 + (int)(8.33 * angle);
}


int main(void)
{
    
    
    DDRB = 0xFF;	//All pins on port A as output
    DDRA = 0xFF;
    DDRD |= (1 << DDD0);

    initPWM();
    uart_init();
    _delay_ms(1000);
    PORTA |= (1 << PORTA1);
    moveMS(FORWARD,40, 2000);
    moveMS(BACKWARD,40, 2000);
    turnDirectionMS(RIGHT_TURN, 50, 1300);
    turnDirectionMS(LEFT_TURN, 50, 1300);
    
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
    /*
        setAngle(0);
        _delay_ms(1000);
        setAngle(90);
        _delay_ms(1000);
        setAngle(180);
        _delay_ms(2000);
        */

    
}