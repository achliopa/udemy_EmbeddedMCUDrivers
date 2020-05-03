/*
 * 005spi_txonly_arduino.c
 *
 *  Created on: 2 Μαΐ 2020
 *      Author: achliopa
 */

#include <string.h>
#include "stm32f446xx.h"

//	* PB12 as SPI2-NSS  (AF5)
//	* PC02 as SPI2-MISO (AF5)
//	* PC03 as SPI2-MOSI (AF5)
//	* PB10 as SPI2-SCLK (AF5)

void GPIO_ButtonInits(void){
	GPIO_Handle_t GpioBtn;
	/* Button config */
	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GpioBtn);
}


void SPI2_GPIOInits(void){
	GPIO_Handle_t SPIPinsB;
	GPIO_Handle_t SPIPinsC;
	SPIPinsB.pGPIOx = GPIOB;
	SPIPinsC.pGPIOx = GPIOC;
	SPIPinsB.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPinsB.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPinsB.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	SPIPinsB.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	SPIPinsC.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPinsC.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPinsC.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	SPIPinsC.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPinsB.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GPIO_Init(&SPIPinsB);
	//MOSI
	SPIPinsC.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&SPIPinsC);
	//NSS
	SPIPinsB.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPinsB);
//	//MISO
//	SPIPinsC.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
//	GPIO_Init(&SPIPinsC);
}

void SPI2_Inits(void){

	SPI_Handle_t	SPI2handle;
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; //generates SCLK of 2MHz
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DIS; //SW slave management enabled for NSS pin
	SPI_Init(&SPI2handle);
}

void delay(void){
	for(uint32_t i = 0; i<500000;i++);
}

int main(void){

	char user_data[] = "We love a Vampire story, especially when we can choose the paths we walk after dark....";

	// enable button
	GPIO_ButtonInits();

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// this function is used to initialize SPI2 peripheral
	SPI2_Inits();

	// enable SSOE to control NSS pin
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1){
		// wait till button is pressed
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		// to avoid button debounce add delay
		delay();
		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,ENABLE);
		//first send length information
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2,&dataLen,1);
		//send data synchronous
		SPI_SendData(SPI2, (uint8_t *)user_data, strlen(user_data));
		// confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));
		//disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,DISABLE);
	}

	return 0;
}

