#include "Board_LED.h"
#include "Board_Buttons.h"

void delay(void) {
	uint32_t i = 0;
	for(i=0;i<150000;i++);
}

void fun3(void) {
	void (*jump_addr) (void) = 0x00000000;
	jump_addr();
}

void fun2(void) {
	fun3();
}

void fun1(void) {
	fun2();
}

int main(void) {
	LED_Initialize();
	Buttons_Initialize();
	fun1();
	while(1){
		if(Buttons_GetState()){
			LED_On(0);
			delay();
			LED_Off(0);
			delay();
		} else {
			LED_Off(0);
		}
	}
	return 0;
}
