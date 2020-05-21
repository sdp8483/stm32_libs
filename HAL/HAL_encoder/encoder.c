/*
 * encoder.c
 *
 *  Created on: May 5, 2020
 *      Author: samper
 */

#include "encoder.h"

void beep(void) {
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
	HAL_Delay(BEEP_DURATION_MS);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
}

void encoderInit(Encoder_Handle_t *hencoder, TIM_HandleTypeDef *htim) {
	HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);

	hencoder->value = 0;
	hencoder->valueOld = 0;
	hencoder->direction = ENCODER_NOTURN;
	hencoder->htim = htim;
	hencoder->beep = BEEP_ON;
}

void encoderGetDirection(Encoder_Handle_t *hencoder) {
	hencoder->value = hencoder->htim->Instance->CNT;

	if (hencoder->value >= (hencoder->valueOld + 4)) {
		hencoder->valueOld = hencoder->value;

		hencoder->direction = ENCODER_CW;

	} else if (hencoder->value <= (hencoder->valueOld - 4)) {
		hencoder->valueOld = hencoder->value;

		hencoder->direction = ENCODER_CCW;

	} else {

		hencoder->direction = ENCODER_NOTURN;
	}
}

uint8_t buttonIsPressed(Encoder_Handle_t *hencoder) {
	if (HAL_GPIO_ReadPin(hencoder->switchPort, hencoder->switchPin) == 0) {		// encoder switch was pressed
		HAL_Delay(20);
		if (HAL_GPIO_ReadPin(hencoder->switchPort, hencoder->switchPin) == 0) {
			if (hencoder->beep == BEEP_ON) {
				beep();
			}

			while (HAL_GPIO_ReadPin(hencoder->switchPort, hencoder->switchPin) == 0)
				; // wait for user to release button
			return 1;
		}
	}

	return 0;
} // end buttonIsPressed()
