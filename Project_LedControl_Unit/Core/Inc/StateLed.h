/*
 * StateLed.h
 *
 *  Created on: Dec 13, 2020
 *      Author: musta
 */

#ifndef INC_STATELED_H_
#define INC_STATELED_H_
#include "stm32f4xx_hal.h"

typedef struct Mx_state
{
	uint8_t id;
	uint8_t data;

}MxMinMeasure_t;

typedef struct motorspeed
{
	uint8_t id;
	uint8_t data;

}MotorSpeed_t;

typedef enum state
{
	ERROR_S,
	HIGHSEQURITY,
	LOWSEQURITY,
	NO_OBJECT
}STATE_t;

#endif /* INC_STATELED_H_ */
