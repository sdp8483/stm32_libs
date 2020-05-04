/*
 * stm32f407xx_gpio.h
 *
 *  Created on: May 3, 2020
 *      Author: sam
 */

#ifndef INC_STM32F407XX_GPIO_H_
#define INC_STM32F407XX_GPIO_H_

#include "stm32f407xx.h"

/* this is a configuration structure for a GPIO pin */
typedef struct {
	uint8_t GPIO_PinNumber;					// GPIO pin number
	uint8_t GPIO_PinMode;					// possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;					// possible values from @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl;			// possible values from @GPIO_PIN_PUPD
	uint8_t GPIO_PinOPType;					// possible values from @GPIO_PIN_OUTPUT_TYPE
	uint8_t GPIO_PinAltFunMode;				// GPIO pin alternative function mode
} GPIO_PinConfig_t;

/* this is a handle structure for a GPIO pin */
typedef struct {
	GPIO_RegDef_t *pGPIOx; 					// pointer to hold the base address of the GPIO peripheral
	GPIO_PinConfig_t GPIO_PinConfig;		// this holds gpio pin configuration settings
} GPIO_Handle_t;

/* @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN0							0
#define GPIO_PIN1							1
#define GPIO_PIN2							2
#define GPIO_PIN3							3
#define GPIO_PIN4							4
#define GPIO_PIN5							5
#define GPIO_PIN6							6
#define GPIO_PIN7							7
#define GPIO_PIN8							8
#define GPIO_PIN9							9
#define GPIO_PIN10							10
#define GPIO_PIN11							11
#define GPIO_PIN12							12
#define GPIO_PIN13							13
#define GPIO_PIN14							14
#define GPIO_PIN15							15

/* @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_INPUT						0
#define GPIO_MODE_OUTPUT					1
#define GPIO_MODE_ALTFN						2
#define GPIO_MODE_ANALOG					3
#define GPIO_MODE_IT_FT						4
#define GPIO_MODE_IT_RT						5
#define GPIO_MODE_IT_RFT					6

/* @GPIO_PIN_OUTPUT_TYPE
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP						0	// push pull output mode
#define GPIO_OP_TYPE_OD						1	// open drain output mode

/* @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW						0
#define GPIO_SPEED_MEDIUM					1
#define GPIO_SPEED_FAST						2
#define GPIO_SPEED_HIGH						3

/* @GPIO_PIN_PUPD
 * GPIO pin pull-up pull-down configuration
 */
#define GPIO_NO_PUPD						0	// no pull-up/pull-down
#define GPIO_PIN_PU							1	// pull-up
#define GPIO_PIN_PD							2	// pull-down



/*
 * **********************************************************************
 *                    APIs supported by this driver
 *   For more information about the APIs check the function definitions
 * **********************************************************************
 */

/* Peripheral Clock Setup */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/* Initialize and Deinitialize */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/* data read and write */
uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/* IRQ configuration and ISR handling */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F407XX_GPIO_H_ */
