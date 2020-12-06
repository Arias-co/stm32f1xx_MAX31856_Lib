/*
 * Serial.h
 *
 *  Created on: Aug 24, 2020
 *      Author: Macbook
 */

#ifndef SRC_SERIAL_H_
#define SRC_SERIAL_H_

#include "Print.h"
#include "stm32f1xx_hal.h"


typedef enum
{
    TIMEOUT,
    INTER_BYTE_TIMEOUT
} Mode_timeout_t;

class Serial: public Print
{
private:

    uint8_t mode;

public:

    UART_HandleTypeDef *uart;
    Serial( UART_HandleTypeDef * uartx);
    void receive( uint8_t * data, uint16_t timeout , Mode_timeout_t modeTimeout);
    virtual void write( uint8_t * text );

};

#endif /* SRC_SERIAL_H_ */
