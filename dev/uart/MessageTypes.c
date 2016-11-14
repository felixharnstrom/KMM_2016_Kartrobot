/*
 * MessageTypes.c
 *
 * Created: 11/14/2016 15:05:27 AM
 *  Author: felha423
 */

 #include "MessageTypes.h"

int msgTypeEncode(t_msgType* msgType){
    switch(*msgType){
        case ACK :
            return 0;
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
        case 15 :
            return DONE;
            break;
        default:
            //That's impossible!
            return INV;
            break;
     }
}