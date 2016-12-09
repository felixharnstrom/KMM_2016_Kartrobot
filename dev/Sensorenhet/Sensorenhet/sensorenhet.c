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

/*
 * Global variable that counts every time the wheels have
 * turned 1/4 of a circle.
 */
volatile uint16_t reflexCounterLeft;
volatile uint16_t reflexCounterRight;

/*
 * Global toggle variables that keeps
 * track on the current state for the reflexes.
 * Is 1 if reflex > 3.0, 0 if reflex < 1.0
 */
volatile uint8_t reflexHighLeft;
volatile uint8_t reflexHighRight;

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

/*
 *  Interrupt issued by watchdog to read reflex sensors on
 *  both sides.
 */
ISR (WDT_vect)
{
    double reflexLeft = getReflexVoltage(REFLEX_LEFT);
    //reflexCounterLeft = (uint16_t)(getReflexVoltage(REFLEX_LEFT));
    double reflexRight = getReflexVoltage(REFLEX_RIGHT);
    
    if (reflexLeft > 2.0 && reflexHighLeft == 0)
    {
        reflexCounterLeft++;
        reflexHighLeft = 1;
    } 
    else if (reflexLeft < 1.5 && reflexHighLeft == 1)
    {
        reflexCounterLeft++;
        reflexHighLeft = 0;
    }
    else if (reflexRight > 2.0 && reflexHighRight == 0)
    {
        reflexCounterRight++;
        reflexHighRight = 1;
    }
    else if (reflexRight < 1.5 && reflexHighRight == 1)
    {
        reflexCounterRight++;
        reflexHighRight = 0;
    }
}

/*
 *  Initiate watchdog interrupt for reflex sensors
 */
void initReflex()
{
    //Set the reflex counters to zero
    reflexCounterLeft = 0;
    reflexCounterRight = 0;
    
    //Initiate start values for reflex togglers
    reflexHighLeft = 0;
    reflexHighRight = 0;
    cli();
    wdt_reset();
    
    //Enable watchdog interrupts and reset everytime
    //the counter overflows.
    WDTCSR |= (1<<WDCE) |(0<<WDE) | (1<<WDIE);
    
    //Set the oscillator prescaler so that the interrupt
    //is issued every 32 ms.
    WDTCSR |= (1<<WDP0) | (0<<WDP1) | (0<<WDP2);
    sei();
}

void initLidar()
{
    //Set up Timer1
    TCNT1 = 0;  //set timer to zero, may not be necessary. This register will count up over time.
    TCCR1A = 0;    //normal counting up - output compare pins not used, initially zero so not necessary
    TCCR1B |= ((1 << CS10) | (0 << CS11) | (0 << CS12)); // start the timer with no prescaler
    
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

//Read the given reflex sensor voltage
double getReflexVoltage(sensor_t s)
{
    if (s == REFLEX_LEFT)
    {
        startADConversion(6);
    }
    else if (s == REFLEX_RIGHT)
    {
        startADConversion(7);
    }
    waitForADConversion();
    return getAdcVoltage();
}

double readSensor(sensor_t s)
{
    switch(s) {
        case LIDAR: return getLidarDistance(); break;
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
        case SENSOR_READ_REFLEX_LEFT:
            return REFLEX_LEFT;
        case SENSOR_READ_REFLEX_RIGHT:
            return REFLEX_RIGHT;
        case SENSOR_READ_IR_BACK:
            return IR_BACK;
        case SENSOR_READ_LIDAR:
            return LIDAR;
        default:
            return -1;        
    }
}

// Source http://www.avrfreaks.net/forum/mpu6050-and-atmega328p-peter-fleury-implementation-problem
#define MPU6050  0xD0     // (0x68 << 1) I2C slave address
unsigned char ret;        // return value
uint16_t raw;             // raw sensor value
int16_t gyroZValue;          // x axis acceleration raw value

long calculateBias() {
    static const int ITERATIONS = 10000;
    long sum = 0;
    for (long i = 0; i < ITERATIONS; ++i)
        sum += MPU6050_readreg(0x43);
    return sum / ITERATIONS;
}

void MPU6050_writereg(uint8_t reg, uint8_t val) {
    i2c_start(MPU6050+I2C_WRITE);
    i2c_write(reg);  // go to register e.g. 106 user control
    i2c_write(val);  // set value e.g. to 0100 0000 FIFO enable
    i2c_stop();      // set stop condition = release bus
}


uint16_t MPU6050_readreg(uint8_t reg)
{
    i2c_start_wait(MPU6050+I2C_WRITE);  // set device address and write mode
    i2c_write(reg);                     // ACCEL_XOUT
    i2c_rep_start(MPU6050+I2C_READ);    // set device address and read mode
    raw = i2c_readAck();                // read one intermediate byte
    raw = (raw<<8) | i2c_readNak();     // read last byte
    i2c_stop();
    return raw;
} 


void Init_MPU6050()
{
    i2c_init();     // init I2C interface
    _delay_ms(200);  // Wait for 200 ms.

    ret = i2c_start(MPU6050+I2C_WRITE);       // set device address and write mode
    if ( ret ) {
         /* failed to issue start condition, possibly no device found */
         i2c_stop();
         while(1) {;;}  // lock program here as sensor init failed
    }
    else {
        /* issuing start condition ok, device accessible */
        MPU6050_writereg(0x6B, 0x00); // go to register 107 set value to 0000 0000 and wake up sensor
        MPU6050_writereg(0x19, 0x08); // go to register 25 sample rate divider set value to 0000 1000 for 1000 Hz
        MPU6050_writereg(0x1C, 0x08); // go to register 28 acceleration configuration set value to 0000 1000 for 4g, normal line tension is 2,7g
        MPU6050_writereg(0x23, 0xF8); // go to register 35 FIFO enable set value to 1111 1000 for all sensors not slave
        MPU6050_writereg(0x37, 0x10); // go to register 55 interrupt configuration set value to 0001 0000 for logic level high and read clear
        MPU6050_writereg(0x38, 0x01); // go to register 56 interrupt enable set value to 0000 0001 data ready creates interrupt
        MPU6050_writereg(0x6A, 0x40); // go to register 106 user control set value to 0100 0000 FIFO enable
	}
}

int main(void)
{
    comm_init();
    //Initiate interrupts
    sei();
    initLidar();
    initAdc();
    initReflex();
    
	gyroZValue = 0;        // initial gyro value (z-axis)

    Init_MPU6050();    // MPU-6050 init
	int bias = (int) calculateBias();
    _delay_ms(500);
    
    uint16_t reflex;

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
            case SENSOR_READ_IR_BACK:;
				double sum = 0;
				for (int i = 0; i<5; i++){
					double sensorOutput = readSensor(msgTypeToSensor(msg));
					uint16_t mmRounded = sensorOutput * 10;
					sum = sum + mmRounded;
				}
				sendReply(sum/5);
				break;
            case SENSOR_READ_LIDAR:;
				double sensorOutput = readSensor(msgTypeToSensor(msg));
				uint16_t mmRounded = sensorOutput * 10;
				sendReply(mmRounded);
				break;
                
            case SENSOR_READ_GYRO:;
				gyroZValue = MPU6050_readreg(0x43)-bias;   // read raw X acceleration from fifo
                sendReply(gyroZValue/1.3);
                break;
                
            case SENSOR_READ_REFLEX_LEFT:;
                reflex = (uint16_t)(3.14 * 32.5 * reflexCounterLeft / 2);
                sendReply(reflex);
                break;
                
            case SENSOR_READ_REFLEX_RIGHT:;
                reflex = (uint16_t)(3.14 * 32.5 * reflexCounterRight / 2);
                sendReply(reflex);
                break;
                
            default: 
                sendError();
                break;
        }
    }
}