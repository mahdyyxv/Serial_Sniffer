/*
 * config.h
 *
 *  Created on: May 30, 2026
 *      Author: ahmed mahdy
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

typedef enum{
    ENUM_BAUDE_4800 = 0,
    ENUM_BAUDE_9600,
    ENUM_BAUDE_19200,
    ENUM_BAUDE_38400,
    ENUM_BAUDE_57600,
    ENUM_BAUDE_115200
}eBaudRate;

typedef enum{
    ENUM_ONE_STOP_BIT = 0,
    ENUM_TWO_STOP_BIT
}eStopBits;

typedef enum{
    ENUM_NO_PARITY = 0,
    ENUM_EVEN_PARITY,
    ENUM_ODD_PARITY
}eParity;

typedef enum{
    ENUM_NO_RESPOND = 0,
    ENUM_RESPOND,
}eRespondType;

typedef struct{
    eBaudRate       BaudeRate;
    eStopBits       StopBit;
    eParity         Parity;
    eRespondType    respond;
}sRs485Conf;

typedef struct{
    eBaudRate       BaudeRate;
    eStopBits       StopBit;
    eParity         Parity;
    eRespondType    respond;
}sRs232Conf;

#endif /* INC_CONFIG_H_ */
