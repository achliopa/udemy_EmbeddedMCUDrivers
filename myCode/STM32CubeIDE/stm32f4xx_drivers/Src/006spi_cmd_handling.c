/*
 * 006spi_cmd_handling.c
 *
 *  Created on: 2 Μαΐ 2020
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
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DIS; //SW slave management enabled for NSS pin
	SPI_Init(&SPI2handle);
}

void delay(void){
	for(uint32_t i = 0; i<500000;i++);
}

uint8_t SPI_VerifyResponse(uint8_t ackbyte){
	if(ackbyte == ACK){
		return 1;
	}else{
		return 0;
	}
}

int main(void){

	uint8_t dummy_write,dummy_read,commandcode,ackbyte,args[2];
	dummy_write = 0xFF;

	// enable button
	GPIO_ButtonInit();

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

		//1. CMD_LED_CTRL <pin no(1)> <value(1)>
		commandcode = COMMAND_LED_CTRL;
		SPI_SendData(SPI2,&commandcode,1);
		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);
		// send dummy byte to slave to get back the ACK/NACK
		SPI_SendData(SPI2,&dummy_write,1);
		SPI_ReceiveData(SPI2,&ackbyte,1);
		if(SPI_VerifyResponse(ackbyte)){
			// send arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI2,args,2);
		}

		//2. CMD_SENSOR_READ <analog pin number(1)>
		// wait till button is pressed
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		// to avoid button debounce add delay
		delay();
		commandcode = COMMAND_SENSOR_READ;
		SPI_SendData(SPI2,&commandcode,1);
		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);
		// send dummy byte to slave to get back the ACK/NACK
		SPI_SendData(SPI2,&dummy_write,1);
		SPI_ReceiveData(SPI2,&ackbyte,1);
		if(SPI_VerifyResponse(ackbyte)){
			// send arguments
			args[0] = ANALOG_PIN2;
			SPI_SendData(SPI2,args,1);
			// do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2,&dummy_read,1);
			delay();
			// send dummy byte to slave to get back the analog val
			SPI_SendData(SPI2,&dummy_write,1);
			uint8_t analog_read;
			SPI_ReceiveData(SPI2,&analog_read,1);
			printf("CMD_SENSOR_READ %d\n",analog_read);
		}

		//3.  CMD_LED_READ 	 <pin no(1) >
		//wait till button is pressed
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		//to avoid button de-bouncing related issues 200ms of delay
		delay();
		commandcode = COMMAND_LED_READ;
		//send command
		SPI_SendData(SPI2,&commandcode,1);
		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);
		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);
		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);
		if( SPI_VerifyResponse(ackbyte)){
			args[0] = LED_PIN;
			//send arguments
			SPI_SendData(SPI2,args,1); //sending one byte of
			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2,&dummy_read,1);
			//insert some delay so that slave can ready with the data
			delay();
			//Send some dummy bits (1 byte) fetch the response from the slave
			SPI_SendData(SPI2,&dummy_write,1);
			uint8_t led_status;
			SPI_ReceiveData(SPI2,&led_status,1);
			printf("COMMAND_READ_LED %d\n",led_status);
		}

		//4. CMD_PRINT 		<len(2)>  <message(len) >
		//wait till button is pressed
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		//to avoid button de-bouncing related issues 200ms of delay
		delay();
		commandcode = COMMAND_PRINT;
		//send command
		SPI_SendData(SPI2,&commandcode,1);
		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);
		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);
		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);
		uint8_t message[] = "Hello ! How are you ??";
		if( SPI_VerifyResponse(ackbyte)){
			args[0] = strlen((char*)message);
			//send arguments
			SPI_SendData(SPI2,args,1); //sending length
			//send message
			SPI_SendData(SPI2,message,args[0]);
			printf("COMMAND_PRINT Executed \n");
		}

		//5. CMD_ID_READ
		//wait till button is pressed
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		//to avoid button de-bouncing related issues 200ms of delay
		delay();
		commandcode = COMMAND_ID_READ;
		//send command
		SPI_SendData(SPI2,&commandcode,1);
		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);
		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);
		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);
		uint8_t id[11];
		uint32_t i=0;
		if( SPI_VerifyResponse(ackbyte))
		{
			//read 10 bytes id from the slave
			for(  i = 0 ; i < 10 ; i++){
				//send dummy byte to fetch data from slave
				SPI_SendData(SPI2,&dummy_write,1);
				SPI_ReceiveData(SPI2,&id[i],1);
			}
			id[11] = '\0';
			printf("COMMAND_ID : %s \n",id);
		}

		// confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));
		//disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,DISABLE);
	}

	return 0;
}
