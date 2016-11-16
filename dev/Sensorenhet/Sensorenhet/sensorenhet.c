/*
 * Sensorenhet.c
 *
 * Created: 11/4/2016 4:08:27 PM
 *  Author: emino969
 */ 

#include "sensorenhet.h"


/*
 * Global variable that keeps track of the LIDAR counter.
 * Read-only.
 */
volatile uint16_t lidarCounter;

void startADConversion(uint8_t channel) {
    //The channel value can never be greater than 7
    channel &= 0x07;
    
    //zero the MUX values (the 3 first bits)
    ADMUX = (ADMUX & 0xF8) | channel;
    
    //ADSC: Starts a conversion
    ADCSRA |= (1<<ADSC);
}

void initAdc() {
    //ADEN: Enables ADC
    //ADPS[2:0]: Changes the clock divider 
    ADCSRA |= (1<<ADEN) | (0<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);
    
    //REFS0: Selects AVCC as reference voltage (+5V)
    ADMUX |= (1<<REFS0);
}


double getAdcVoltage() {
    double const AVCC = 5;
    return (double)(ADC * AVCC) / 1024.0;
}


double getIrDistance() {
    double voltage = getAdcVoltage();
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
	double dist = pow(voltage / A, 1 / B);
	return (dist < MAX_IR_DISTANCE_CM) ? dist : MAX_IR_DISTANCE_CM;
}

double getLidarDistance() {
    //Counter * 256 / CLK = 10^-5 * Length [cm] <==>
    //Length [cm] = Counter * 256 * 10^5 / (8 * 10 ^ 6) = 3.2
	//New lidarCounter runs 256 times as fast
    const double c = 3.2 / 256.0;
    return lidarCounter * c;
}

void waitForADConversion() {
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
        //TCNT1 is the counter value
		//Start counting
        TCNT1 = 0;
    }
    else
    {
        //Update the LIDAR value with the counter
        lidarCounter = TCNT1;
    }
}

void initLidar()
{
	//Set up Timer1
	TCNT1 = 0;  //set timer to zero, may not be necessary. This register will count up over time.
	TCCR1A = 0;	//normal counting up - output compare pins not used, initially zero so not necessary
	TCCR1B |= ((1 << CS10) | (0 << CS11) | (0 << CS12)); // start the timer with no prescaler
	
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

double readSensor(sensor_t s)
{
    switch(s) {
        case LIDAR: return getLidarDistance();
        case IR_LEFT_BACK: startADConversion(0); break;
        case IR_RIGHT_BACK: startADConversion(1); break;
        case IR_LEFT_FRONT: startADConversion(2); break;
        case IR_RIGHT_FRONT: startADConversion(3); break;
        case IR_BACK: startADConversion(4); break;
    }
    waitForADConversion();
    return getIrDistance();
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
	startADConversion(5);
	waitForADConversion();
	return getAdcVoltage();
}

double gyroOutputToAngularRate(double gyroOutput, double bias) {
	static const double GAIN = 1.0 / 17.0; //TODO: may require further adjustment
	return (gyroOutput - bias) / GAIN;
}

double calculateBias() {
	static const int ITERATIONS = 1000;
	double sum = 0;
	for (int i = 0; i < ITERATIONS; ++i)
		sum += readGyro();
	return sum / ((double) ITERATIONS);
}


void calibrationTest() {
	//Set up Timer1
	//Note that the LIDAR is using the same timer
	TCNT1 = 0;  //set timer to zero, may not be necessary. This register will count up over time.
	TCCR1A = 0;	//normal counting up - output compare pins not used, initially zero so not necessary
	TCCR1B |= ((1 << CS10) | (0 << CS11) | (1 << CS12)); // start the timer at 8MHz/1024
		
	double clockRate = 8000000.0 / 1024.0;
	double timeToMax = 65535 / clockRate;
	double clocksPerSec = 65535 / timeToMax;
	
	double bias = calculateBias();
	
	double angle = 0;
	
	while(1) {
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


uint8_t lowestByte(unsigned int n) {
	return (n & 0xFF);
}

void sendReply(uint16_t val) {
	uint8_t lowest = lowestByte(val);
	uint8_t highest = lowestByte(val >> 8);
	
	int address = SENSOR_ADRESS;
	int payloadSize = 2;
	t_msgType msgType= SENSOR_DATA;
	char payload[2];
	payload[0] = highest;
	payload[1] = lowest;
	uart_msg_transmit(&address, &payloadSize, &msgType, payload);
}

void sendError() {
	int address = SENSOR_ADRESS;
	int payloadSize = 0;
	t_msgType msgType= DONE;
	uart_msg_transmit(&address, &payloadSize, &msgType, NULL);
}

sensor_t msgTypeToSensor(t_msgType mst) {
	switch (mst) {
		case SENSOR_READ_IR_LEFT_FRONT:
			return IR_LEFT_FRONT;
		case SENSOR_READ_IR_LEFT_BACK:
			return IR_LEFT_BACK;
		case SENSOR_READ_IR_RIGHT_FRONT:
			return IR_RIGHT_FRONT;
		case SENSOR_READ_IR_RIGHT_BACK:
			return IR_RIGHT_BACK;
		case SENSOR_READ_IR_BACK:
			return IR_BACK;
		case SENSOR_READ_LIDAR:
			return LIDAR;
		default:
			return -1;		
	}
}

int main(void)
{
	comm_init();
	sei();
	initLidar();
	initAdc();
	
	double bias = calculateBias();
	
	while(1) {
		/* Read client request */
		int dontCare1;
		int dontCare2;
		char dontCare3;
		t_msgType msg;
		uart_msg_receive(&dontCare1, &dontCare2, &msg, &dontCare3);
		
		/* Acknowledge client */
		int tAddress = SENSOR_ADRESS;
		int payloadSize = 0;
		t_msgType msgType = ACK;
		uart_msg_transmit(&tAddress, &payloadSize, &msgType, NULL);
		
		/* Process and reply */
		switch (msg) {
			case SENSOR_READ_IR_LEFT_FRONT:
			case SENSOR_READ_IR_LEFT_BACK:
			case SENSOR_READ_IR_RIGHT_FRONT:
			case SENSOR_READ_IR_RIGHT_BACK:
			case SENSOR_READ_IR_BACK:
			case SENSOR_READ_LIDAR:;
				double sensorOutput = readSensor(msgTypeToSensor(msg));
				uint16_t mmRounded = sensorOutput * 10;
				sendReply(mmRounded);
				break;
				
			case SENSOR_READ_GYRO:;
				double gyroOutput = gyroOutputToAngularRate(readGyro(), bias);
				int16_t perHektoSecond = gyroOutput * 100;
				uint16_t usigned = *(uint16_t*)&perHektoSecond; // Interpret the same bit pattern as uint16
				sendReply((uint16_t) perHektoSecond);
				break;
				
			default: 
				sendError();
				break;
		}
	}
}
