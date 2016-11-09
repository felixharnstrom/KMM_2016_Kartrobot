/*
 * UART.h
 *
 * Created: 11/5/2016 11:04:23 AM
 *  Author: felha423
 */


#ifndef UART_H_
#define UART_H_

typedef enum {ACK, ECHO, INV} t_msgType;    //INV refers to an invalid type, useful for error handling

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
 *
 * _Returns_
 * (unsigned char) A single 8-bit packet. May not correspond to ASCII character.
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

/*
 * Returns the integer encoding for the given message type.
 *
 * _Parameters_
 * (t_msgType*) msgType: the message type to be encoded
 *
 * _Returns_
 * (int) The integer encoding for the given message type. Returns -1 if no encoding exist.
 */
int msgTypeEncode(t_msgType* msgType);

/*
 * Returns the message type for the given integer encoding.
 *
 * _Parameters_
 * (int) msgType: the integer to be decoded
 *
 * _Returns_
 * (int) The message type corresponding to the given integer encoding. Returns ERR if given integer is not a valid encoding.
 */
t_msgType msgTypeDecode(int msgType);

#endif /* UART_H_ */