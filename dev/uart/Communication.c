﻿/*
 * communication.c
 *
 * Created: 11/5/2016 11:04:13 AM
 *  Author: felha423
 */

 #ifndef F_CPU
 #define F_CPU 8000000UL
 #endif

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>

#include "Communication.h"
#include "UART.h"

#ifndef BAUD
#define BAUD 9600   // define baud
#endif

void comm_init(void){
    uart_init(UART_BAUD_SELECT(BAUD,F_CPU));
}

void uart_transmit(unsigned char c){
	loop_until_bit_is_set(UCSR0A, UDRE0);   // wait until data register empty
	UDR0 = c;
    loop_until_bit_is_set(UCSR0A, TXC0);  // Wait until transmission ready.
}

unsigned char uart_receive(void){
	loop_until_bit_is_set(UCSR0A, RXC0);    // wait until data exists
	return UDR0;
}

int uart_msg_transmit(int* address, int* payloadSize, t_msgType* msgType, char* payload){
    /* Construct meta packet */
    int meta_packet;
    int type;
    type = msgTypeEncode(msgType);

    /* Return -1 if invalid type */
    if(type == -1){
        return -1;
    }

    meta_packet = (*address * 128) + (*payloadSize * 16) + type;

    /* Transmit meta packet */
    uart_putc((unsigned char)meta_packet);

    /* Transmit payload */
    for(int i = 0; i < *payloadSize; ++i){
        uart_putc((unsigned char)payload[i]);
    }

    return 0;
}

int uart_msg_receive(int* address, int* payloadSize, t_msgType* msgType, char* payload){
    uint8_t adrMask = 0x80;     // 10000000b
    uint8_t sizeMask = 0x70;    // 01110000b
    uint8_t typeMask = 0x0F;    // 00001111b

    /* Receive meta packet */
    unsigned int c;
    int b = 1;
        while(b == 1){
            c = uart_getc();
            if ( c & UART_NO_DATA ){
                /*
                 * no data available from UART
                 */
            }else{
                b = 0;
            }
        }

    PORTA &= ~(1 << PORTA1);

    /* Extract meta information */
    *address = (c & adrMask) >> 7;
    *payloadSize = (c & sizeMask) >> 4;
    int type = (c & typeMask);
    *msgType = msgTypeDecode(type);

    /* Return -1 if invalid type */
    if(*msgType == INV){
        return -1;
    }

    /* Receive payload */
    for(int i = 0; i < *payloadSize; ++i){
        int a = 1;
        while(a == 1){
            c = uart_getc();
            if ( c & UART_NO_DATA ){
                /*
                 * no data available from UART
                 */
            }else{
                a = 0;
                payload[i] = c;
            }
        }
    }
    return 0;
}

int msgTypeEncode(t_msgType* msgType){
    switch(*msgType){
        case ACK :
            return 0;
            break;
        case MOVE_MS :
            return 1;
            break;
        case TURN_MS :
            return 2;
            break;
        case SET_SIDE_SPEED:
            return 3;
            break;
        case SET_SERVO_ANGLE :
            return 4;
            break;
        case STOP_MOTORS :
            return 5;
            break;
        case DONE :
            return 15;
            break;
        default:
            return -1;
    }
}

t_msgType msgTypeDecode(int msgType){
    switch(msgType){
        case 0 :
            return ACK;
            break;
        case 1 :
            return MOVE_MS;
            break;
        case 2 :
            return TURN_MS;
            break;
        case 3 :
            return SET_SIDE_SPEED;
            break;
        case 4 :
            return SET_SERVO_ANGLE;
            break;
        case 5 :
            return STOP_MOTORS;
            break;
        case 15 :
            return DONE;
            break;
        default:
            //That's impossible!
            return INV;
            break;
     }
}