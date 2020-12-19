/*
 * StateLed.h
 *
 *  Created on: Dec 13, 2020
 *      Author: musta
 */

#ifndef INC_DSP_H_
#define INC_DSP_H_
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

#endif /* INC_DSP_H_ */
