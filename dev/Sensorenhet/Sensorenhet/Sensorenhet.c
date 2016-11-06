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

void initPWM(){
	TCCR3A |= (1 << WGM30) | (1 << WGM31) | (1 << COM3B1) | (1 << COM3A1); //Com3B0 = 0 for inverted
	TCCR3B |= (1 << WGM32) | (1 << WGM33) | (1 << CS31); //WGM32 = 0 should yield Timer 0 -> Max and then reset (1 << WGM32)
	OCR3A = 20000;	//Corresponds to 50Hz
	OCR3B = 700; //The start value for the duty cycle
}

void setPWM(double dutyCycle)
{
	OCR3B = 1024 * dutyCycle;
}

/*
 * Set the wanted servo angle.
 */
void setAngle(uint32_t angle)
{
	OCR3B = 710 + (int)(8.33 * angle);
}


int main(void)
{
	DDRB = (1<<DDB7);	//All pins on port A as output
	initPWM();
	
    while(1)
    {
		setAngle(0);
		_delay_ms(2000);
		setAngle(45);
		_delay_ms(2000);
		setAngle(160);
		_delay_ms(2000);
    }
}