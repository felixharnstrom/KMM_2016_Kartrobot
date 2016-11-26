/*
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
#define BAUD 38400   // define baud
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
	// If the first bit of address is 0, then we have a motor
	if (*address == 0) {
		*msgType = msgTypeDecode(type, MOTOR);
	} else { // Sensor
		*msgType = msgTypeDecode(type, SENSOR);
	}

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
		/* motor-specific */
        case MOTOR_MOVE_MS :
            return 1;
            break;
        case MOTOR_TURN_MS :
            return 2;
            break;
        case MOTOR_SET_SIDE_SPEED:
            return 3;
            break;
        case MOTOR_SET_SERVO_ANGLE :
            return 4;
            break;
        case MOTOR_STOP_MOTORS :
            return 5;
            break;
        case MOTOR_GET_DIAG :
            return 6;
            break;
		/* sensor-specific */
		case SENSOR_READ_IR_LEFT_FRONT :
			return 1;
			break;
		case SENSOR_READ_IR_LEFT_BACK :
			return 2;
			break;
		case SENSOR_READ_IR_RIGHT_FRONT:
			return 3;
			break;
		case SENSOR_READ_IR_RIGHT_BACK :
			return 4;
			break;
		case SENSOR_READ_IR_BACK :
			return 5;
			break;
		case SENSOR_READ_LIDAR :
			return 6;
			break;
		case SENSOR_READ_GYRO :
			return 7;
			break;
		/* general */
        case DONE :
            return 15;
            break;
        default:
            return -1;
    }
}

t_msgType msgTypeDecode(int msgType, t_unitType unitType){
	if (unitType == MOTOR) {
		switch(msgType){
			case 0 :
				return ACK;
				break;
			case 1 :
				return MOTOR_MOVE_MS;
				break;
			case 2 :
				return MOTOR_TURN_MS;
				break;
			case 3 :
				return MOTOR_SET_SIDE_SPEED;
				break;
			case 4 :
				return MOTOR_SET_SERVO_ANGLE;
				break;
			case 5 :
				return MOTOR_STOP_MOTORS;
				break;
            case 6 :
                return MOTOR_GET_DIAG;
                break;
			case 15 :
				return DONE;
				break;
			default:
				// Wrong unit type
				return INV;
				break;
		}

	} else if (unitType == SENSOR) {
		switch(msgType){
			case 0 :
				return ACK;
				break;
			case 1 :
				return SENSOR_READ_IR_LEFT_FRONT;
				break;
			case 2 :
				return SENSOR_READ_IR_LEFT_BACK;
				break;
			case 3 :
				return SENSOR_READ_IR_RIGHT_FRONT;
				break;
			case 4 :
				return SENSOR_READ_IR_RIGHT_BACK;
				break;
			case 5 :
				return SENSOR_READ_IR_BACK;
				break;
			case 6 :
				return SENSOR_READ_LIDAR;
				break;
			case 7 :
				return SENSOR_READ_GYRO;
				break;
			case 15 :
				return DONE;
				break;
			default:
				// Wrong unit type
				return INV;
				break;
		}
	}
	
	// unreachable
	return INV;
}