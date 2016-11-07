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

/* TODO should be a sub-program that reads all sensors and returns one int value per sensor
void readSensor(int left1, int left2, int right1, int right2, int back)
{
	
}
*/

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

/*
 *	Start a conversion
 */
void startConversion(uint8_t channel) {
    //The channel value can never be greater than 7
    channel &= 0x07;
    
    //zero the MUX values (the 3 first bits)
    ADMUX = (ADMUX & 0xF8) | channel;
    
    //ADSC: Starts a conversion
    ADCSRA |= (1<<ADSC);
}

/*
 * Initiate the analog/digital conversion
 */
void initAD() {
    //ADEN: Enables ADC
    //ADPS[2:0]: Changes the clock divider 
    ADCSRA |= (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
    
    //REFS0: Selects AVCC as reference voltage (+5V)
    ADMUX |= (1<<REFS0);
}

/*
 *	Convert the current value at ADC to corresponding voltage
 */
double getVoltage() {
    double const AVCC = 5;
    return (double)(ADC * AVCC) / 1024.0;
}

/*
 *	Wait until there is no current conversions left
 */
void waitForConversion() {
    while(ADCSRA & (1<<ADSC));
}

int main(void) {
    DDRB |= (1<<DDB0);
    initAD();

    while(1) {
        //Loop over all channels
        for (uint8_t i = 0; i < 4; i++) {
            startConversion(i);
            waitForConversion();
            
            if (getVoltage() > 0.4) {
                PORTB |= (1 << PORTB0);
            } else {
                PORTB &= (0 << PORTB0);
            }
        }
    }
}

/*
Janis Version:

int main(void)
{
	DDRB = (1<<DDB7);	//All pins on port A as output
	DDRA = 0x20;
	
	initPWM();
	
	int analogIrSignal;
	
	while(1)
    {
		analogIrSignal = ADC0D;
		
		if(analogIrSignal < 500) {
			PINA = 0x20;
		} else {
			PINA = 0x00;
		}
    }
}

*/