/*
 * encoder.h
 *
 *  Created on: May 5, 2020
 *      Author: samper
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "main.h"
#include "tim.h"

#define ENCODER_CW				1
#define ENCODER_CCW				-1
#define ENCODER_NOTURN			0

#define BEEP_ON					1
#define BEEP_OFF				0

#define BEEP_DURATION_MS		25

typedef struct __Encoder_Handle_t {
	TIM_HandleTypeDef *htim;
	int32_t value;
	int32_t valueOld;
	int8_t direction;
	uint8_t beep;
	GPIO_TypeDef *switchPort;
	uint16_t switchPin;
} Encoder_Handle_t;

void beep(void);												// generate beep using buzzer on RepRap Smart Controller

void encoderInit(Encoder_Handle_t *hencoder, TIM_HandleTypeDef *htim);
void encoderGetDirection(Encoder_Handle_t *hencoder);

uint8_t buttonIsPressed(Encoder_Handle_t *hencoder);

#endif /* INC_ENCODER_H_ */
