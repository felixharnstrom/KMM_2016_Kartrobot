/*
 * Sensorenhet.c
 *
 * Created: 11/4/2016 4:08:27 PM
 *  Author: emino969
 */ 

#include "sensorenhet.h"
#include "UART.c" //gick inte att hitta funktionerna med UART.h av någon anledning?

//global variable that keeps track of the LIDAR counter
volatile uint8_t lidarCounter;

void start_ir_read(uint8_t channel) {
    //The channel value can never be greater than 7
    channel &= 0x07;
    
    //zero the MUX values (the 3 first bits)
    ADMUX = (ADMUX & 0xF8) | channel;
    
    //ADSC: Starts a conversion
    ADCSRA |= (1<<ADSC);
}

void init_ad() {
    //ADEN: Enables ADC
    //ADPS[2:0]: Changes the clock divider 
    ADCSRA |= (1<<ADEN) | (0<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);
    
    //REFS0: Selects AVCC as reference voltage (+5V)
    ADMUX |= (1<<REFS0);
}


double get_ir_voltage() {
    double const AVCC = 5;
    return (double)(ADC * AVCC) / 1024.0;
}


double ir_output_to_centimeters() {
    double voltage = get_ir_voltage();
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

double lidar_output_to_centimeters() {
    //Counter * 256 / CLK = 10^-5 * Length [cm] <==>
    //Length [cm] = Counter * 256 * 10^5 / (8 * 10 ^ 6) = 3.2
    const double c = 3.2;
    return lidarCounter * c;
}


void wait_for_ir_read() {
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
        //TCNT0 is the counter value
        TCNT0 = 0;
        //Use the CLK / 256 clock to avoid counter overflow
        TCCR0B |= (1<<CS02);
    }
    else
    {
        //Update the LIDAR value with the counter
        lidarCounter = TCNT0;
        //Turn off the timer
        TCCR0B &= (0<<CS02);
    }
}

void init_lidar()
{
    //Enable interrupts
    sei();
    
    //PortB2: LIDAR input
    //PortB3: LIDAR trigger
    DDRB = (1<<DDB3) | (0<<DDB2) | (1<<DDB0);
    
    //PORTB3 = 0 ==> LIDAR sends data 
    PORTB |= (0<<PORTB3);
    
    //React on any logical change
    EICRA |= (0<<ISC21) | (1<<ISC20);
    
    //Choose INT2 (PB3) as trigger for the interrupt
    EIMSK |= (1<<INT2);
    EIFR |= (1<<INTF2);
}

double read_sensor(sensor_t s)
{
    switch(s) {
        case LIDAR: return lidar_output_to_centimeters();
        case IR_LEFT_BACK: start_ir_read(0); break;
        case IR_RIGHT_BACK: start_ir_read(1); break;
        case IR_LEFT_FRONT: start_ir_read(2); break;
        case IR_RIGHT_FRONT: start_ir_read(3); break;
        case IR_BACK: start_ir_read(4); break;
    }
    wait_for_ir_read();
    return ir_output_to_centimeters();
}

void sendInt(int n) {
	int charCount = 7;
	//Get a string without any trash
	char vstr[charCount];
	
	for (int i = 0; i < charCount; ++i)
	vstr[i] = ' ';
	// int to str
	sprintf(vstr, "%d", n);
	// Send the string via UART
	for (int i = 0; i < charCount; ++i)
	if (vstr[i] != 0)
	uart_transmit(vstr[i]);
	uart_transmit('\n');
}

double readGyro() {
	start_ir_read(5);
	wait_for_ir_read();
	return get_ir_voltage();
}

double gyroOutputToAngularRate(double gyroOutput) {
	static const double BIAS = 2.5;
	static const double GAIN = 1;
	return (gyroOutput - BIAS) / GAIN;
}

int main(void)
{
	uart_init();
	init_lidar();
	init_ad();

	while(1)
	{
		//int vint = lidar_output_to_centimeters();
		//int vint = readGyro();
		
		sendInt(1337);
		//sendInt(gyroOutputToAngularRate(vint) * 1000);
		
	}
}