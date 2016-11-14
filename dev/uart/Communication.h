/*
 * Communication.h
 *
 * Created: 11/5/2016 11:04:23 AM
 *  Author: felha423
 */


#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

/*
 * Initialize UART
 */
void comm_init (void);

/*
 * DEPRECATED: Use uart_putc() instead
 * Send a single character (i.e. 8 bits) over UART. Waits until data is sent.
 *
 * _Parameters_
 * (unsigned char) c: the character to transmit
 */
void uart_transmit (unsigned char data);

/*
 * DEPRECATED: Use uart_getc() instead
 * Receive a single character (i.e. 8 bits) over UART. Waits until data is received.
 *
 * _Returns_
 * (unsigned char) A single 8-bit packet. May not correspond to ASCII character.
 */
unsigned char uart_receive (void);

/*
 * Constructs a meta packet from the given parameters, and transmits the meta packet with the payload.
 *
 * _Parameters_
 * (int) address: sender address
 * (int) payloadSize: number of payload characters
 * (t_msgType) msgType: the type of message to transmit
 * (char*) payload: the payload to transmit
 *
 * _Returns_
 * (int) 0 if well-formed message, else -1.
 */
int uart_msg_transmit(int* address, int* payloadSize, t_msgType* msgType, char* payload);

/*
 * Receives a meta packet and the following payload.
 *
 * _Parameters_
 * (int*) address: sender address
 * (int*) payloadSize: number of payload characters
 * (t_msgType*) msgType: received messsage type
 * (char*) payload: received payload
 *
 * _Returns_
 * (int) 0 if well-formed message, else -1.
 */
int uart_msg_receive(int* address, int* payloadSize, t_msgType* msgType, char* payload);

#endif /* COMMUNICATION_H_ */