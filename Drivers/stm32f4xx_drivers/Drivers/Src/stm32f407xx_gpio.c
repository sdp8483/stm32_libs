/*
 * stm32f407xx_gpio.c
 *
 *  Created on: May 3, 2020
 *      Author: sam
 */

#include "stm32f407xx_gpio.h"

/* Peripheral Clock Setup */

/******************************************************************************
 * @fn						- 	GPIO_PeriClockControl
 *
 * @brief               	- 	this function enables or disables peripheral clock
 * 						  		for the given GPIO port
 *
 * @param[pGPIOx]			- 	base address of the GPIO peripheral
 * @param[EnorDi]			- 	ENABLE or DISABLE macros
 *
 * @return					- 	none
 *
 * @Note					- 	none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if(pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if(pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if(pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if(pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if(pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if(pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if(pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		} else if(pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		} else if(pGPIOx == GPIOI) {
			GPIOI_PCLK_EN();
		}
	} else {
		if(pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		} else if(pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if(pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if(pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if(pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if(pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		} else if(pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		} else if(pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		} else if(pGPIOx == GPIOI) {
			GPIOI_PCLK_DI();
		}
	}
}

/* Initialize and Deinitialize */
/******************************************************************************
 *  @fn						- 	GPIO_Init
 *
 * @brief               	- 	this function initializes GPIO pin
 *
 * @param[pGPIOHandle]		- 	handle structure for a GPIO pin
 *
 * @return					- 	none
 *
 * @Note					- 	none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
	uint32_t temp = 0;	// temp register

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		// 1. configure the mode of the gpio pin
		// the non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clear
		pGPIOHandle->pGPIOx->MODER |= temp;

	} else {
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_FT) {
			// 1. configure the FTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the corresponding RTSR bit
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RT) {
			// 1. configure the RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the corresponding FTSR bit
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RFT) {
			// 1. configure both FTSR and RTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// 2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADD_TO_CODE(pGPIOHandle->pGPIOx);

		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		// 3. enable the exti interrupt delivery using IMR
		EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	// 2. configure the speed
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clear
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	// 3. configure the pupd settings
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clear
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	// 4. configure the output type only if pin is output
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_OUTPUT) {
		temp = 0;
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clear
		pGPIOHandle->pGPIOx->OTYPER |= temp;
	}

	// 5. configure alternate mode
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		// configure the alt function registers
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); // clear
		pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2);
	}
}

/******************************************************************************
 *  @fn						- 	GPIO_DeInit
 *
 * @brief               	- 	this function deinitializes GPIO pin
 *
 * @param[pGPIOx]			- 	peripheral register definition structures
 *
 * @return					- 	none
 *
 * @Note					- 	none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	if(pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if(pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if(pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if(pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if(pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	} else if(pGPIOx == GPIOF) {
		GPIOF_REG_RESET();
	} else if(pGPIOx == GPIOG) {
		GPIOG_REG_RESET();
	} else if(pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	} else if(pGPIOx == GPIOI) {
		GPIOI_REG_RESET();
	}
}

/* data read and write */
/******************************************************************************
 *  @fn						- 	GPIO_ReadFromInputPin
 *
 * @brief               	- 	this function reads from a GPIO pin
 *
 * @param[pGPIOx]			- 	peripheral register definition structure
 * @param[PinNumber]		- 	GPIO pin number
 *
 * @return					- 	pin input state
 *
 * @Note					- 	none
 */
uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	uint8_t value;

	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x1);

	return value;
}

/******************************************************************************
 *  @fn						- 	GPIO_ReadFromInputPort
 *
 * @brief               	- 	this function reads from a GPIO port
 *
 * @param[pGPIOx]			- 	peripheral register definition structure
 *
 * @return					- 	port input state
 *
 * @Note					- 	none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	uint16_t value;

	value = (uint16_t) (pGPIOx->IDR);

	return value;
}

/******************************************************************************
 *  @fn						- 	GPIO_WriteToOutputPin
 *
 * @brief               	- 	this function writes to a GPIO pin
 *
 * @param[pGPIOx]			- 	peripheral register definition structure
 * @param[PinNumber]		- 	GPIO pin number
 * @param[Value]			- 	GPIO_PIN_SET or GPIO_PIN_RESET macros
 *
 * @return					- 	none
 *
 * @Note					- 	none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value) {
	if(Value == GPIO_PIN_SET) {
		pGPIOx->ODR |= (1 << PinNumber);	// write 1 to pin
	} else {
		pGPIOx->ODR &= ~(1 << PinNumber);	// write 0 to pin
	}
}

/******************************************************************************
 *  @fn						- 	GPIO_WriteToOutputPort
 *
 * @brief               	- 	this function writes to a GPIO port
 *
 * @param[pGPIOx]			- 	peripheral register definition structure
 * @param[Value]			- 	value to write to port
 *
 * @return					- 	none
 *
 * @Note					- 	none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {
	pGPIOx->ODR = Value;
}

/******************************************************************************
 *  @fn						- 	GPIO_ToggleOutputPin
 *
 * @brief               	- 	this function toggles a GPIO pin
 *
 * @param[pGPIOx]			- 	peripheral register definition structure
 * @param[PinNumber]		- 	GPIO pin number
 *
 * @return					- 	none
 *
 * @Note					- 	none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	pGPIOx->ODR ^= (1 << PinNumber);
}

/* IRQ configuration and ISR handling */
/******************************************************************************
 *  @fn						- 	GPIO_IRQITConfig
 *
 * @brief               	- 	this function enables or disables the GPIO IRQ
 *
 * @param[IRQNumber]		- 	IRQ Number
 * @param[EnorDi]			- 	ENABLE or DISABLE macros
 *
 * @return					- 	none
 *
 * @Note					- 	none
 */
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi) {
	if(EnorDi == ENABLE) {
		if (IRQNumber <= 31) {
			// program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			// program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			// program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	} else {
		if (IRQNumber <= 31) {
			// program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			// program ICER0 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			// program ICER0 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

/******************************************************************************
 *  @fn						- 	GPIO_IRQPriorityConfig
 *
 * @brief               	-
 *
 * @param[IRQNumber]		- 	IRQ Number
 * @param[IRQPriority]		- 	IRQ Priority
 *
 * @return					- 	none
 *
 * @Note					- 	none
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	// 1. find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}

/******************************************************************************
 *  @fn						- 	GPIO_IRQHandling
 *
 * @brief               	- 	this function handles the IRQ
 *
 * @param[PinNumber]		- 	GPIO pin number
 *
 * @return					- 	none
 *
 * @Note					- 	none
 */
void GPIO_IRQHandling(uint8_t PinNumber) {
	// clear the exti pr register corresponding to the pin number
	if (EXTI->PR & (1 << PinNumber)) {
		// clear
		EXTI->PR |= (1 << PinNumber);
	}
}
