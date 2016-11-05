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
	int packet[8];
	uart_init();
    while(1){
	    uart_packet_receive(PACKET_SIZE, packet);   // receive a packet
        // reply with the bit pattern of the packet
	    for(int i = 7; i >= 0; --i){
		    if(packet[i] == 1){
			    uart_transmit('1');
			    }else{
			    uart_transmit('0');
		    }
	    }
    }
}