﻿/*
 * UART.h
 *
 * Created: 11/5/2016 11:04:23 AM
 *  Author: felha423
 */ 


#ifndef UART_H_
#define UART_H_

/*
 * Initialize UART 
 */
void uart_init (void);

/* 
 * Send a single character (i.e. 8 bits) over UART. Waits until data is sent.
 *
 * _Parameters_
 * (unsigned char) c: the character to transmit
 */
void uart_transmit (unsigned char data);

/* 
 * Receive a single character (i.e. 8 bits) over UART. Waits until data is received.
 */
unsigned char uart_receive (void);

/* 
 * Receive a single UART packet. The packet is stored in the given array, MSB at index 0.
 *
 * _Parameters_
 * (int) size: packet size in bits
 * (int*) packet: array to hold the resulting bit pattern
 */
void uart_packet_receive (int size, int* packet);


#endif /* UART_H_ */