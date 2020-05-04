/*
 * 001led_toggle.c
 *
 *  Created on: May 4, 2020
 *      Author: sam
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio.h"

void delay (void) {
	for(uint32_t i=0; i< 500000/2; i++) {

	}
}

int main (void) {

	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN6;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

	while(1) {
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN6);
		delay();
	}

	return 0;
}
