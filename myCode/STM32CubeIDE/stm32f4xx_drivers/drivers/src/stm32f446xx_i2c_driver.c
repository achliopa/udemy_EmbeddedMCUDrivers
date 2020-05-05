/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: 4 Μαΐ 2020
 *      Author: achliopa
 */

#include "stm32f446xx_i2c_driver.h"

/*****************************************************************
 * @fn			- I2C_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral clock for an I2C peripheral
 *
 * @param[in]	- base address of the I2C periph
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 *
 * @Note		- note
 *
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if(EnorDi == ENABLE)
			{
				if(pI2Cx == I2C1)
				{
					I2C1_PCLK_EN();
				}else if (pI2Cx == I2C2)
				{
					I2C2_PCLK_EN();
				}else if (pI2Cx == I2C3)
				{
					I2C3_PCLK_EN();
				}
			}
			else
			{
				if(pI2Cx == I2C1)
				{
					I2C1_PCLK_DIS();
				}else if (pI2Cx == I2C2)
				{
					I2C2_PCLK_DIS();
				}else if (pI2Cx == I2C3)
				{
					I2C3_PCLK_DIS();
				}
			}
}

/*****************************************************************
 * @fn			- I2C_Init
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 *
 */
void I2C_Init(I2C_Handle_t *pI2CHandle){

}

/*****************************************************************
 * @fn			- I2C_DeInit
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 *
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx){
	if(pI2Cx == I2C1)
		{
			I2C1_REG_RST();
		}else if (pI2Cx == I2C2)
		{
			I2C2_REG_RST();
		}else if (pI2Cx == I2C3)
		{
			I2C3_REG_RST();
		}
}

/*****************************************************************
 * @fn			- I2C_PeripheralControl
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 *
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	} else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

/*****************************************************************
 * @fn			- I2C_IRQInterruptConfig
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 *
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi == ENABLE) {
			if (IRQNumber <= 31) {
				// program ISER0
				*NVIC_ISER0 |= (1 << IRQNumber);
			} else if (IRQNumber > 31 && IRQNumber <= 63 ) {
				// program ISER1
				*NVIC_ISER1 |= (1 << IRQNumber % 32);
			} else if (IRQNumber >=64 && IRQNumber < 96 ) {
				// program ISER2
				*NVIC_ISER2 |= (1 << IRQNumber % 64);
			}
		} else {
			if (IRQNumber <= 31) {
				// program ICER0
				*NVIC_ICER0 |= (1 << IRQNumber);
			} else if (IRQNumber > 31 && IRQNumber <= 63 ) {
				// program ICER1
				*NVIC_ICER1 |= (1 << IRQNumber % 32);
			} else if (IRQNumber >=64 && IRQNumber < 96 ) {
				// program ICER2
				*NVIC_ICER2 |= (1 << IRQNumber % 64);
			}
		}
}

/*****************************************************************
 * @fn			- I2C_IRQPriorityConfig
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-
 *
 * @Note		-
 *
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	//1. first lets find out the ipr register
	uint8_t	iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= ( IRQPriority << shift_amount);
}
