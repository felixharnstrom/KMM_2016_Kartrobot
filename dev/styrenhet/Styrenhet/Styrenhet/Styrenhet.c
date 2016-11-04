/*
 * Test.c
 *
 * Created: 11/3/2016 10:57:58 AM
 *  Author: felha423
 */ 

#define F_CPU 8000000UL	//8MHz
#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
	DDRA = (1<<DDA7)|(1<<DDA6)|(1<<DDA5)|(1<<DDA4)|(1<<DDA3)|(1<<DDA2)|(1<<DDA1)|(1<<DDA0);	//All pins on port A as output
    while(1)
    {
        PORTA |= (1<<PORTA5);    //Turn 6th bit on PORTA (i.e. PA5) to 1 => on
        _delay_ms(1000);        //Delay for 1000ms => 1 sec
        PORTA &= ~(1<<PORTA5);    //Turn 6th bit on PORTA (i.e. PA5) to 0 => off
        _delay_ms(1000);        //Delay for 1000ms => 1 sec
    }
	
}