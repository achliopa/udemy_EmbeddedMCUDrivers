/*
 * stm32f446xx_i2c_driver.h
 *
 *  Created on: 4 Μαΐ 2020
 *      Author: achliopa
 */

#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_

#include "stm32f446xx.h"

/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct {
	uint32_t 	I2C_SCLSpeed;
	uint8_t		I2C_DeviceAddress;
	uint8_t		I2C_ACKControl;
	uint16_t	I2C_FMDutyCycle;
}I2C_Config_t;

/*
 * Handle structure for I2C peripheral
 */
typedef struct{
	I2C_RegDef_t	*pI2Cx;
	I2C_Config_t	I2C_Config;
}I2C_Handle_t;

#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */
