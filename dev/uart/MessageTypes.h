/*
 * MessageTypes.h
 *
 * Created: 11/14/2016 15:05:46 AM
 *  Author: felha423
 */

#ifndef MESSAGETYPES_H_
#define MESSAGETYPES_H_

 typedef enum {ACK, DONE, INV} t_msgType;

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
 * (int) The message type corresponding to the given integer encoding. Returns INV if given integer is not a valid encoding.
 */
t_msgType msgTypeDecode(int msgType);

#endif /* MESSAGETYPES_H_ */