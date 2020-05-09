/*
 * 005button_interrupt.c
 *
 *  Created on: May 9, 2020
 *      Author: sam
 */

#include "string.h"
#include "stm32f407xx.h"
#include "stm32f407xx_gpio.h"

#define BUTTON_PRESSED 	0

void delay(void) {
	for (uint32_t i = 0; i < 500000/2; i++) { 			// ~200ms when clk is 16MHz

	}
}

int main(void) {

	GPIO_Handle_t GpioLed;
	memset(&GpioLed, 0, sizeof(GpioLed));				// avoid garbage values and initialize variable to 0

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
	memset(&GpioBtn, 0, sizeof(GpioBtn));				// avoid garbage values and initialize variable to 0

	GpioBtn.pGPIOx = GPIOE;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN4;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOE, ENABLE);
	GPIO_Init(&GpioBtn);

	// IRQ configurations
	GPIO_IRQITConfig(IRQ_NO_EXTI4, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI4, NVIC_IRQ_PRI15);

	while (1) {

	}

	return 0;
}

void EXTI4_IRQHandler(void) {
	delay();
	GPIO_IRQHandling(GPIO_PIN4);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN6);
}
