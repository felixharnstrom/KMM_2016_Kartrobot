/*
 * UART.c
 *
 * Created: 11/5/2016 11:04:13 AM
 *  Author: felha423
 */

#include <avr/io.h>
#include <stdio.h>

#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#ifndef BAUD
#define BAUD 9600   // define baud
#endif

#ifndef BAUDRATE
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)    // set baud rate value for UBRR
#endif

void uart_init (void){
	UBRR0H = (BAUDRATE>>8);             // shift the register right by 8 bits
	UBRR0L = BAUDRATE;                  // set baud rate
	UCSR0B|= (1<<TXEN0)|(1<<RXEN0);     // enable receiver and transmitter
	UCSR0C|= (1<<USBS0)|(3<<UCSZ00);    // 8bit data format
}

void uart_transmit (unsigned char c){
	loop_until_bit_is_set(UCSR0A, UDRE0);   // wait until data register empty
	UDR0 = c;
}

unsigned char uart_receive (void){
	loop_until_bit_is_set(UCSR0A, RXC0);    // Wait until data exists
	return UDR0;
}

void uart_packet_receive (int size, int* packet){
	char c;
	c = uart_receive();
	for(int i = 0; i < size; ++i){
		packet[i] = (c >> i) & 1;
	}
}
