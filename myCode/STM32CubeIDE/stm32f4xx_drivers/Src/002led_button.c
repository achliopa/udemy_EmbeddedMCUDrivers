/*
 * 002led_button.c
 *
 *  Created on: Apr 18, 2020
 *      Author: achliopa
 */

#include "stm32f446xx.h"

void delay(void){
	for(uint32_t i = 0; i<500000;i++);
}

int main(void) {

	GPIO_Handle_t GpioLed, GpioBtn;

	/* LED config */
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

	/* Button config */
	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioBtn);

	while(1){
		if(!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)){
			delay();
			GPIO_TogglePin(GPIOA, GPIO_PIN_NO_5);
		}
	}
	return 0;
}
