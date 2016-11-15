/*
 * UART.h
 *
 * Created: 11/5/2016 11:04:23 AM
 *  Author: felha423
 */


#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

typedef enum {
		// General
		ACK,
		// Motor-specific
		MOTOR_MOVE_MS, MOTOR_TURN_MS, MOTOR_SET_SERVO_ANGLE, MOTOR_SET_SIDE_SPEED, MOTOR_STOP_MOTORS, MOTOR_GET_DIAG,
		// Sensor-specific
		SENSOR_READ_IR_LEFT_FRONT, SENSOR_READ_IR_LEFT_BACK, SENSOR_READ_IR_RIGHT_FRONT, SENSOR_READ_IR_RIGHT_BACK,
		SENSOR_READ_IR_BACK, SENSOR_READ_LIDAR, SENSOR_READ_GYRO,
		// More general
		INV, ECHO, DONE
	} t_msgType;

typedef enum {MOTOR, SENSOR} t_unitType;

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
 * (t_unitType) unitType: the type of unit to decode to
 *
 * _Returns_
 * (int) The message type corresponding to the given integer encoding. Returns ERR if given integer is not a valid encoding.
 */
t_msgType msgTypeDecode(int msgType, t_unitType unitType);

#endif /* COMMUNICATION_H_ */