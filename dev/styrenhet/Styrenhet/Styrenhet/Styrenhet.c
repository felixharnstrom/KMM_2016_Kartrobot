/*
 * Styrenhet.c
 *
 * Created: 11/3/2016 10:57:58 AM
 *  Author: felha423
 */ 

#define F_CPU 8000000UL	//8MHz
#define BAUD 9600                                   // define baud
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)            // set baud rate value for UBRR

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

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
{
	loop_until_bit_is_set(UCSR0A, RXC0); /* Wait until data exists. */
    return UDR0;
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

int main(void)
{
	char c;
	int packet[8];
	//DDRA = (1<<DDA7)|(1<<DDA6)|(1<<DDA5)|(1<<DDA4)|(1<<DDA3)|(1<<DDA2)|(1<<DDA1)|(1<<DDA0);	//All pins on port A as output
	uart_init();
    while(1)
    {
		c = uart_receive();
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
			//loop_until_bit_is_set(UCSR0A, UDRE0);
		
		
		
		
		//if(a == 'a'){
			//PORTA |= (1<<PORTA5);    //Turn 6th bit on PORTA (i.e. PA5) to 1 => on
			////_delay_ms(1000);        //Delay for 1000ms => 1 sec
		//}else{
			//PORTA &= ~(1<<PORTA5);    //Turn 6th bit on PORTA (i.e. PA5) to 0 => off
			////_delay_ms(1000);        //Delay for 1000ms => 1 sec
		//}
		//uart_transmit('a');
    }
	
}