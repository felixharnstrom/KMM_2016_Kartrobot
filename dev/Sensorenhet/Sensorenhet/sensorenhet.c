/*
 * Sensorenhet.c
 *
 * Created: 11/4/2016 4:08:27 PM
 *  Author: emino969
 */ 

#include "sensorenhet.h"
#include "UART.h"

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

double gyroOutputToAngularRate(double gyroOutput, double bias) {
	static const double GAIN = 1.0 / 17.0; //TODO: may require further adjustment
	return (gyroOutput - bias) / GAIN;
}

/*
 * Returns the average gyro output over a number of iterations.
 * Intended to be used in standstill to acquire bias.
 */
double calculateBias() {
	static const int ITERATIONS = 1000;
	double sum = 0;
	for (int i = 0; i < ITERATIONS; ++i)
		sum += readGyro();
	return sum / ((double) ITERATIONS);
}

/*
 * Continually transmits the current angle over UART.
 */
void calibrationTest() {
	//Set up Timer1
	TCNT1 = 0;  //set timer to zero, may not be necessary. This register will count up over time.
	TCCR1A = 0;	//normal counting up - output compare pins not used, initially zero so not necessary
	TCCR1B |= ((1 << CS10) | (1 << CS12)); // start the timer at 8MHz/1024

	double clockRate = 8000000.0 / 1024.0;
	double timeToMax = 65535 / clockRate;
	double clocksPerSec = 65535 / timeToMax;
	
	double bias = calculateBias();
	
	double angle;
	
	while(1)
	{
		//int vint = lidar_output_to_centimeters();
		//int vint = readGyro();
		int time = TCNT1;
		TCNT1 = 0;
		
		double av = gyroOutputToAngularRate(readGyro(), bias);
		double dif = av * (time / clocksPerSec);
		angle += dif;
		
		//if (dif > 0.01)
		//sendInt(1000*dif);
		sendInt(100*angle);
		//sendInt(gyroOutputToAngularRate(readGyro()) * 100);
		
		while(TCNT1 < clocksPerSec / 1000);
	}
}

/*
 * Return the byte yielded from the 8 least significant bits.
 */
uint8_t lowestByte(unsigned int n) {
	return (n & 0xFF);
}

/*
 * Send a 2-byte number as two single-byte numbers, starting with the MSB.
 */
void sendReply(uint16_t val) {
	uint8_t lowest = lowestByte(val);
	uint8_t highest = lowestByte(val >> 8);
	uart_transmit(highest);
	uart_transmit(lowest);
	//sendInt(highest); sendInt(lowest);
}

int main(void)
{
	uart_init();
	init_lidar();
	init_ad();
	
	//calibrationTest();
	//while(1) sendReply(0x201);
	
	double bias = calculateBias();
	
	while(1)
	{
		uint8_t msg = uart_receive();
		//uint8_t msg = 5;
		switch (msg) {
			case 0: //IR 0
			case 1: //IR 1
			case 2: //IR 2
			case 3: //IR 3
			case 4: //IR 4
			case 5:;//LIDAR
				double sensorOutput = read_sensor(msg);
				uint16_t mmRounded = sensorOutput * 10;
				sendReply(mmRounded);
				break;
			case 6:;//Gyro
				double gyroOutput = gyroOutputToAngularRate(readGyro(), bias);
				int16_t perHektoSecond = gyroOutput * 100;
				uint16_t usigned = *(uint16_t*)&perHektoSecond; // Interpret the same bit pattern as uint16
				sendReply(usigned);
				break;
			default: 
				break;
		}
	}
}
