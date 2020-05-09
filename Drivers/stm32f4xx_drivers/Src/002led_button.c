/*
 * 002led_button.c
 *
 *  Created on: May 9, 2020
 *      Author: sam
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio.h"

#define BUTTON_PRESSED 	0

void delay(void) {
	for (uint32_t i = 0; i < 500000/2; i++) {

	}
}

int main(void) {

	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN6;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);
	GPIO_WriteToOutputPin(GPIOA, GPIO_PIN6, GPIO_PIN_SET);

	GPIO_Handle_t GpioBtn;
	GpioBtn.pGPIOx = GPIOE;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN4;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOE, ENABLE);
	GPIO_Init(&GpioBtn);

	while (1) {

		if (GPIO_ReadFromInputPin(GPIOE, GPIO_PIN4) == BUTTON_PRESSED) {
			delay();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN6);
		}
	}

	return 0;
}
