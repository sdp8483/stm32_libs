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
	hencoder->htim = htim;
}


int8_t encoderGetDirection(Encoder_Handle_t *hencoder) {
	//hencoder->value = TIM4->CNT;
	hencoder->value = hencoder->htim->Instance->CNT;

	if (hencoder->value >= (hencoder->valueOld + 4)) {
		hencoder->valueOld = hencoder->value;
		return ENCODER_CW;
	}

	if (hencoder->value <= (hencoder->valueOld - 4)) {
		hencoder->valueOld = hencoder->value;
		return ENCODER_CCW;
	}

	return ENCODER_NOTURN;
}

uint8_t buttonIsPressed(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t beepOnPress) {
	if (HAL_GPIO_ReadPin(ENC_SW_GPIO_Port, ENC_SW_Pin) == 0) {		// encoder switch was pressed
		HAL_Delay(20);
		if (HAL_GPIO_ReadPin(ENC_SW_GPIO_Port, ENC_SW_Pin) == 0) {
			if (beepOnPress == BEEP_ON) {
				beep();
			}

			while (HAL_GPIO_ReadPin(ENC_SW_GPIO_Port, ENC_SW_Pin) == 0)
				; // wait for user to release button
			return 1;
		}
	}

	return 0;
} // end buttonIsPressed()
