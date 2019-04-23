# Udemy Course: Mastering Microcontroller with Embedded Driver Development

* [Course Link](https://www.udemy.com/course/mastering-microcontroller-with-peripheral-driver-development/)
* [Course Repo](https://github.com/niekiran/MasteringMCU)

## Section 2 - Development board used in our courses

### Lecture 4 - About MCU Development board

* NUCLEO-F446RE from ST 
* ST has good tools. and support.
* check for on incircuit debugger
* check if Board supports the peripherals we are interested in 
* check flash size and on board ram size. aim for >100KB

### Lecture 5 - STM32F4 Discovery and Nucleo : Board Details

* See EmbeddedProg Course (same lecture)

### Lecture 6 - ST-Link Driver Installation

* See EmbeddedProg Course (same lecture)

### Lecture 7 - ST Link Firmware Upgrade

* See EmbeddedProg Course (same lecture)

## Section 3 - Hardware/Software Requirements

### Lecture 8 - Hardware/Software Requirements

* check list in 'Hardware-software-used-in-MCU1'

## Section 4 - OpenSTM32 System Workbench installation

* Same ad EmbeddedProgrammin Course

## Section 5 - KEIL-MDK-5 Setup For ARM Cortex M based MCUs

* Same ad EmbeddedProgrammin Course

## Section 6 - About the board

* Same ad EmbeddedProgrammin Course

## Section 7 - LED/Button Exercises using BSPs

### Lecture 2- Exercise: LED Toggling App using Board BSP APIs (Nucleo)

* first we have allok at schematic to see where the LED is connected (Port A - pin 5)
* we create a 'blinky' project
* in manage project we add boardsupport (led) selecting our dev board from the list
* add cubemx classic
* Board support is added with a file for LED offering API methods
* we import the header `#include "Board_LED.h"`
* we initialize LEDs `LED_Initialize();` which sets the GPIO
* we turn on the LED `LED_On(0);` passing in the index (we have only 1 in nucleo so 0)
* to toggle we add a delay method
* complete code
```
#include "Board_LED.h"

void delay(void) {
	uint32_t i = 0;
	for(i=0;i<150000;i++);
}

int main(void) {
	LED_Initialize();
	while(1){
		LED_On(0);
		delay();
		LED_Off(0);
		delay();
	}
	return 0;
}
```

### Lecture 25 - Exercise : Adding button support using board BSP APIs(Nucleo)

* in Keil MDK5 in Manage Run-Time Env we add boardsupport for buttons (NUCLEO)
* we import 'Board_Buttons.h' to main.c
* we add `Buttons_Initialize();`
* in the while loop we check button state `Buttons_GetState()` if it is 1 (pushed) we toggle
```
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
```
* according to schematic when button is not pressed the pin is driven High (VDD) if pressed Low (GND)
* if we implemented an custom interrupt handler the it should trigger on falling edge
* all this is taken care of the keil BSP

## Section 8 - LED/Button Exercises with OpenSTM32 SystemWorkbench

* Same ad EmbeddedProgrammin Course

## Section 9 - Embedded Code Debugging Tools n Tips

### Lecture 32 - Embedded Code Debugging Part-1

* most dev boards come with on board debugger
* to start debugger we click the button on keil
* in debug mode we have the source files with indication of where execution is
* we also have dissasembly window
* we use the butoon in menu to control the execution(reset, run, stepi in, step out, step ove)

### Lecture 33 - Embedded Code Debugging Part-2: Break points

* breakpoints halt the execution.
* we can put breakpoin in source code or dissassembly instruction

### Lecture 34 - Embedded Code Debugging Part-3: Step In/Step Over/Step Out

* there are 3 buttons.
* step in => enter the function (instr by instr execution)
* step over => execute code without entering the method or assembly code
* step out execute method and return www