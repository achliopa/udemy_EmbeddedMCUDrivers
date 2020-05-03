/*
 * 007spi_cmd_handling_it.c
 *
 *  Created on: 3 Μαΐ 2020
 *      Author: achliopa
 */

#include <string.h>
#include <stdio.h>
#include "stm32f446xx.h"

// command codes
#define COMMAND_LED_CTRL	0x50
#define COMMAND_SENSOR_READ	0x51
#define COMMAND_LED_READ	0x52
#define COMMAND_PRINT		0x53
#define COMMAND_ID_READ		0x54

#define LED_ON		1
#define LED_OFF		0

// arduino analog pins
#define ANALOG_PIN0		0
#define ANALOG_PIN1		1
#define ANALOG_PIN2		2
#define ANALOG_PIN3		3
#define ANALOG_PIN4		4
#define ANALOG_PIN5		5

// arduino led
#define LED_PIN		9

// ack/nack
#define ACK			0xF5
#define NACK		0xA5

/*
 * Global Variables
 */

SPI_Handle_t SPI2handle;
uint8_t RcvBuff[100];
uint8_t ReadByte;
uint8_t RxContFlag = RESET;

// initialize NUCLEO board User Button

void GPIO_ButtonInit(void){
	GPIO_Handle_t GpioBtn;
	/* Button config */
	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GpioBtn);
}

//	* PB12 as SPI2-NSS  (AF5)
//	* PC02 as SPI2-MISO (AF5)
//	* PC03 as SPI2-MOSI (AF5)
//	* PB10 as SPI2-SCLK (AF5)

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
	//MISO
	SPIPinsC.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&SPIPinsC);
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
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DIS; //HW slave management enabled for NSS pin
	SPI_Init(&SPI2handle);
}

void delay(void){
	for(uint32_t i = 0; i<250000;i++);
}

uint8_t SPI_VerifyResponse(uint8_t ackbyte){
	if(ackbyte == (uint8_t)ACK){
		return 1;
	}else{
		return 0;
	}
}

int main(void){

	// enable button
	GPIO_ButtonInit();

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// this function is used to initialize SPI2 peripheral
	SPI2_Inits();

	// enable SSOE to control NSS pin
	SPI_SSOEConfig(SPI2, ENABLE);
	// enable SPI Interrupt
	SPI_IRQInterruptConfig(IRQ_NO_SPI2,ENABLE);
	// wait till button is pressed
	while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
	// to avoid button debounce add delay
	delay();
	//enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2,ENABLE);
	RxContFlag = SET;
	while(RxContFlag == SET){
		while ( ! (SPI_ReceiveDataIT(&SPI2handle,&ReadByte,1) == SPI_READY) );
	}

	// confirm SPI is not busy
	while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));
	//disable the SPI2 peripheral
	SPI_PeripheralControl(SPI2,DISABLE);

	return 0;
}

void SPI2_IRQHandler(void)
{

	SPI_IRQHandling(&SPI2handle);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	static uint32_t i =0;
	static uint8_t  rcv_start = 0;
	if(AppEv == SPI_EVENT_RX_CMPLT)
	{
		if(ReadByte == 0XF1)
		{
			rcv_start = 1;
		}else
		{
			if(rcv_start)
			{
				if(ReadByte == '\r')
				{
					RxContFlag = RESET;
					rcv_start =0;
					RcvBuff[i++] = ReadByte; //place the \r
					i=0;
				}else
				{
					RcvBuff[i++] = ReadByte;

				}
			}
		}


	}

}
