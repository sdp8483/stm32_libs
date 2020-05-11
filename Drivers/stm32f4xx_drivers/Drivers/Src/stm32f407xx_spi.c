/*
 * stm32f4xx_spi.c
 *
 *  Created on: May 10, 2020
 *      Author: sam
 */

#include "stm32f407xx_spi.h"
/******************************************************************************
 *  @fn						- 	SPI_PeriClockControl
 *
 * @brief               	-
 *
 * @param[pSPIx]			- 	pointer to SPIx register definition
 * @param[EnorDi]			-	ENABLE or DISABLE macros
 *
 * @return					- 	none
 *
 * @Note					- 	none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {

}

/******************************************************************************
 *  @fn						- 	SPI_Init
 *
 * @brief               	-
 *
 * @param[pSPIHandle]		- 	pointer to SPIx handle
 *
 * @return					- 	none
 *
 * @Note					- 	none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle) {

}

/******************************************************************************
 *  @fn						- 	SPI_DeInit
 *
 * @brief               	-
 *
 * @param[pSPIx]			- 	pointer to SPIx register definition
 *
 * @return					- 	none
 *
 * @Note					- 	none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx) {

}

/******************************************************************************
 *  @fn						- 	SPI_SendData
 *
 * @brief               	- 	Send data over SPIx using blocking method
 *
 * @param[pSPIx]			- 	pointer to SPIx register definition
 * @param[pTxBuffer]		-	pointer to data buffer to send
 * @param[len]				-	number of bytes of data to send
 *
 * @return					- 	none
 *
 * @Note					- 	none
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len) {

}

/******************************************************************************
 *  @fn						- 	SPI_ReceiveData
 *
 * @brief               	- 	receive data over SPIx using blocking method
 *
 * @param[pSPIx]			- 	pointer to SPIx register definition
 * @param[pTxBuffer]		-	pointer to data buffer to receive
 * @param[len]				-	number of bytes of data to receive
 *
 * @return					- 	none
 *
 * @Note					- 	none
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len) {

}

/******************************************************************************
 *  @fn						- 	SPI_IRQITConfig
 *
 * @brief               	- 	this function enables or disables the SPIx IRQ
 *
 * @param[IRQNumber]		- 	IRQ Number
 * @param[EnorDi]			- 	ENABLE or DISABLE macros
 *
 * @return					- 	none
 *
 * @Note					- 	none
 */
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi) {

}

/******************************************************************************
 *  @fn						- 	SPI_IRQPriorityConfig
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {

}

/******************************************************************************
 *  @fn						- 	GPIO_IRQHandling
 *
 * @brief               	- 	this function handles the IRQ
 *
 * @param[pSPIHandle]		- 	pointer to SPIx handle
 *
 * @return					- 	none
 *
 * @Note					- 	none
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle) {

}
