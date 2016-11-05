/*
 * Styrenhet.c
 *
 * Created: 11/3/2016 10:57:58 AM
 *  Author: felha423
 */

#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

#ifndef PACKET_SIZE
#define PACKET_SIZE 8   // define packet size
#endif

#include "uart.h"

int main(void){
	uart_init();
    while(1){
        uart_receive();
        uart_msg_transmit(0, 6, MOTOR, "bcdefg");   // sends "abcdefg"
    }
}