#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

/**
 * @brief Message types.
 */
typedef enum {
        // General
        ACK,
        // Motor-specific
        MOTOR_MOVE_MS, MOTOR_TURN_MS, MOTOR_SET_SERVO_ANGLE, MOTOR_SET_SIDE_SPEED, MOTOR_STOP_MOTORS, MOTOR_GET_DIAG,
        // Sensor-specific
        SENSOR_READ_IR_LEFT_FRONT, SENSOR_READ_IR_LEFT_BACK, SENSOR_READ_IR_RIGHT_FRONT, SENSOR_READ_IR_RIGHT_BACK,
        SENSOR_READ_IR_BACK, SENSOR_READ_LIDAR, SENSOR_READ_GYRO, SENSOR_DATA, SENSOR_READ_REFLEX_LEFT, SENSOR_READ_REFLEX_RIGHT,
        // More general
        INV, ECHO, DONE
    } t_msgType;

/**
 * @brief Unit types in the system.
 */
typedef enum {MOTOR, SENSOR} t_unitType;

/**
 * @brief Initialize UART.
 */
void comm_init (void);

/**
 * @brief Send a single character (i.e. 8 bits) over UART. Waits until data is sent.
 *
 * @deprecated  Use uart_putc() instead.
 * @param data  The character to transmit.
 */
void uart_transmit (unsigned char data);

/**
 * @brief Receive a single character (i.e. 8 bits) over UART. Waits until data is received.
 *
 * @deprecated  Use uart_getc() instead
 * @return      A single 8-bit packet. May not correspond to ASCII character.
 */
unsigned char uart_receive (void);

/**
 * @brief Constructs a meta packet from the given parameters, and transmits the meta packet with the payload.
 *
 * @param address       Sender address.
 * @param payloadSize   Number of payload characters.
 * @param msgType       The type of message to transmit.
 * @param payload       The payload to transmit.
 * @return              0 if well-formed message, else -1.
 */
int uart_msg_transmit(int* address, int* payloadSize, t_msgType* msgType, char* payload);

/**
 * @brief Receives a meta packet and the following payload.
 *
 * @param[out] address      Sender address.
 * @param[out] payloadSize  Number of payload characters.
 * @param[out] msgType      Received messsage type.
 * @param[out] payload      Received payload.
 * @return 0 if well-formed message, else -1.
 */
int uart_msg_receive(int* address, int* payloadSize, t_msgType* msgType, char* payload);

/**
 * @brief Returns the integer encoding for the given message type.
 *
 * @param msgType   The message type to be encoded
 * @return          The integer encoding for the given message type. Returns -1 if no encoding exist.
 */
int msgTypeEncode(t_msgType* msgType);

/**
 * @brief Returns the message type for the given integer encoding.
 *
 * @param msgType   The integer to be decoded.
 * @param unitType  The type of unit to decode to.
 * @return          The message type corresponding to the given integer encoding. Returns ERR if given integer is not a valid encoding.
 */
t_msgType msgTypeDecode(int msgType, t_unitType unitType);

#endif /* COMMUNICATION_H_ */