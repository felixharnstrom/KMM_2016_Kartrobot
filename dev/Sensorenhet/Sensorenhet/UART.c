﻿
/*
 * UART.c
 *
 * Created: 11/5/2016 11:04:13 AM
 *  Author: felha423
 */

#include <avr/io.h>
#include <stdio.h>

#include "UART.h"

#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#ifndef BAUD
#define BAUD 9600   // define baud
#endif

#ifndef BAUDRATE
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)    // set baud rate value for UBRR
#endif

void uart_init(void){
	UBRR0H = (BAUDRATE>>8);             // shift the register right by 8 bits
	UBRR0L = BAUDRATE;                  // set baud rate
	UCSR0B|= (1<<TXEN0)|(1<<RXEN0);     // enable receiver and transmitter
	UCSR0C|= (1<<USBS0)|(3<<UCSZ00);    // 8bit data format
}

void uart_transmit(unsigned char c){
	loop_until_bit_is_set(UCSR0A, UDRE0);   // wait until data register empty
	UDR0 = c;
}

unsigned char uart_receive(void){
	loop_until_bit_is_set(UCSR0A, RXC0);    // Wait until data exists
	return UDR0;
}

void uart_packet_receive(int size, int* packet){
	char c;
	c = uart_receive();
	for(int i = 0; i < size; ++i){
		packet[i] = (c >> i) & 1;
	}
}

void uart_msg_transmit(int address, int payloadSize, t_msgType msgType, char* payload){
    /* Construct meta packet */
    int meta_packet;
    int type;
    switch(msgType){
    case SENSOR :
        type = 0;
    	break;
    case MOTOR :
        type = 1;
        break;
    default:
        return;
    }
    meta_packet = (address * 128) + (payloadSize * 16) + type;

    /* Transmit meta packet */
    uart_transmit(meta_packet);

    /* Transmit payload */
    for(int i = 0; i < payloadSize; ++i){
        uart_transmit(payload[i]);
    }
}

void uart_msg_receive(int* address, int* payloadSize, t_msgType* msgType, char* payload){
    uint8_t adrMask = 0x80;   // 10000000b
    uint8_t sizeMask = 0x70;   // 01110000b
    uint8_t typeMask = 0x0F;   // 00001111b

    char c;
    c = uart_receive();     // receive meta packet
    *address = (c & adrMask) >> 7;
    *payloadSize = (c & sizeMask) >> 4;
    int type = (c & typeMask);
    switch(type){
        case 0 :
            *msgType = SENSOR;
            break;
        case 1 :
            *msgType = MOTOR;
            break;
        default:
            return;
    }

    for(int i = 0; i < *payloadSize; ++i){
        payload[i] = uart_receive();
    }
}