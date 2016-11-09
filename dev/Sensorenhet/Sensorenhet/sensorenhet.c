/*
 * Sensorenhet.c
 *
 * Created: 11/4/2016 4:08:27 PM
 *  Author: emino969
 */ 

#include "sensorenhet.h"

//global variable that keeps track of the LIDAR counter
volatile uint8_t lidarCounter;

void startIrRead(uint8_t channel) {
    //The channel value can never be greater than 7
    channel &= 0x07;
    
    //zero the MUX values (the 3 first bits)
    ADMUX = (ADMUX & 0xF8) | channel;
    
    //ADSC: Starts a conversion
    ADCSRA |= (1<<ADSC);
}

void initAD() {
    //ADEN: Enables ADC
    //ADPS[2:0]: Changes the clock divider 
    ADCSRA |= (1<<ADEN) | (0<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);
    
    //REFS0: Selects AVCC as reference voltage (+5V)
    ADMUX |= (1<<REFS0);
}


double getIrVoltage() {
    double const AVCC = 5;
    return (double)(ADC * AVCC) / 1024.0;
}


double irOutputToCentimeters(double voltage) {
	// Formula approximated to the inverse of the following sets of data points (x, y):
	/* 5 1.495
	   7.5 1.420
	   10 1.250
	   15 0.845
	   20 0.620
	   25 0.500
	   30 0.445 */
	static const double A = 4.5770;
	static const double B = -0.6340;
	return pow(voltage / A, 1 / B);
}

double lidarOutputToCm(double lidarOutput) {
    return lidarOutput; // TODO
}


void waitForIrRead() {
    while(ADCSRA & (1<<ADSC));
}

/*
 *  Interrupt vector that's triggered when LIDAR monitor value is changed
 */
ISR (INT2_vect)
{
    //If the LIDAR monitor input is 1
    if ((PINB & 0b00000100) == 0b00000100)
    {
        //Start counter 0
        TCNT0 = 0;
        TCCR0B |= (1<<CS02);
    }
    else
    {
        //Update the LIDAR counter
        lidarCounter = TCNT0;
        TCCR0B &= (0<<CS02);
    }
}

void initLidar()
{
    //Enable interrupts
    sei();
    
    //PortB2: LIDAR input
    //PortB3: LIDAR trigger
    DDRB = (1<<DDB3) | (0<<DDB2) | (1<<DDB0);
    
    //Trigger = 0 ==> LIDAR sends data 
    PORTB |= (0<<PORTB3);
    
    //React on any logical change
    EICRA |= (0<<ISC21) | (1<<ISC20);
    
    //Choose INT2 (PB3) as trigger for the interrupt
    EIMSK |= (1<<INT2);
    EIFR |= (1<<INTF2);
}

double readSensor(sensor_t s)
{
    switch(s) {
        case LIDAR: return lidarOutputToCm(lidarCounter);
        case IR_LEFT_BACK: startIrRead(0); break;
        case IR_RIGHT_BACK: startIrRead(1); break;
        case IR_LEFT_FRONT: startIrRead(2); break;
        case IR_RIGHT_FRONT: startIrRead(3); break;
        case IR_BACK: startIrRead(4); break;
    }
    waitForIrRead();
    return irOutputToCentimeters(getIrVoltage());
}

int main(void)
{   
    initLidar();

    while(1)
    {
        if (lidarCounter > 100)
            PORTB |= (1<<PORTB0);
        else
            PORTB &= (0<<PORTB0);
    }
}

/*
Hannes and Janis Debugging session.

int main(void) {
    DDRB |= (1<<DDB0);
    initAD();
	uart_init();

    while(1) {
        //Loop over all channels
        for (uint8_t i = 4; i < 5; i++) {
            startIrRead((i);
            waitForIrRead();
            
            if (getIrVoltage() > 0.4) {
                PORTB |= (1 << PORTB0);
            } else {
                PORTB &= (0 << PORTB0);
            }
			
			int vint = irOutputToCentimeters(getIrVoltage());
            
			//Get a string without any trash
			int charCount = 32;
			char vstr[charCount];
			for (int i = 0; i < charCount; ++i)
				vstr[i] = ' '; 
			// int to str
			sprintf(vstr, "%d", vint); 
			// Send the string via UART
			for (int i = 0; i < charCount; ++i)
				if (vstr[i] != 0)
					uart_transmit(vstr[i]);
			uart_transmit('\n');
        }
    }
}

*/