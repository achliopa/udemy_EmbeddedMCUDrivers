/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: 4 Μαΐ 2020
 *      Author: achliopa
 */

#include "stm32f446xx_i2c_driver.h"

// Private Helper Methods Prototypes
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);

// AHB prescaler values
uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
// APB1 prescaler values
uint8_t APB1_PreScaler[4] = {2,4,8,16};

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
 * @fn			- RCC_GetPLLOutputClock
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
uint32_t RCC_GetPLLOutputClock(void){
	// stub
	uint32_t pll = 16000000;
	return pll;
}

/*****************************************************************
 * @fn			- RCC_GetPCLK1Value
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
uint32_t RCC_GetPCLK1Value(void){
	uint32_t pclk1,SystemClk;
	uint8_t clksrc,temp,ahbp,apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);
	if(clksrc == 0){
		SystemClk = 16000000; //16Mhz
	}else if(clksrc == 1){
		SystemClk = 8000000; //8Mhz
	}else if(clksrc == 2){
		SystemClk = RCC_GetPLLOutputClock(); //PLL
	}

	temp = ((RCC->CFGR >> 4) & 0xF);
	if(temp < 8){
		ahbp = 1;
	}else{
		ahbp = AHB_PreScaler[temp-8];
	}

	temp = ((RCC->CFGR >> 10) & 0x7);
	if(temp < 4){
		apb1p = 1;
	}else{
		apb1p = APB1_PreScaler[temp-4];
	}

	pclk1 = (SystemClk/ahbp)/apb1p;

	return pclk1;
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
	uint32_t tempreg = 0;

	//ack control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//configure the FREQ field of CR2
	tempreg = 0;
	tempreg = RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//program the device own address
	tempreg = pI2CHandle->I2C_Config.I2C_DeviceAddress << 1 & 0x7F;
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		//mode is standard mode
		ccr_value = RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		tempreg |= (ccr_value & 0xFFF);
	}else{
		//mode is fast mode
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
			ccr_value = RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}else{
			ccr_value = RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;
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
 * @fn			- I2C_MasterSendData
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
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr){
	//1. Generate the Start Condition
	I2C_GenerateStartCondition();
	//2. confirm that start generation is completed by checking the SP flag in the SR1
	//  Note: Untill SB is cleared SCL will be stretched (pulled to LOW)
	while(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)){

	}
	//3. Send the address of the slave with r/w bit set to w(0) (total 8 bits)
	//4. confirm that address phase is completed by checking the ADDR flag in the SR1
	//5. clear he ADDR flag according to its software sequence
	//	Note: Untill ADDR is cleared SCL will be stretched (pulled to LOW)
	//6. send the data untill Len becomes 0
	//7. when Len becomes 0 wait for TXE=1 and BTF=1 before generating the STOP condition
	//	Note: TXE=1, BTF=1 means that both SR and DR are empty and next transmission should begin
	//	when BTF=1 SCL will be stretched (pulled to LOW)
	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	//	Note: generating STOP, automaticaly clears the BTF
}

/*****************************************************************
 * @fn			- I2C_MasterSendData
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
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
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
