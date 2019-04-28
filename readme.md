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
* using repetitevely breakpoints and step in i can do lin eby line debuggin

### Lecture 35 - Embedded Code Debugging Part-4 : Disable/Kill all Bkpts.

* if we forget where we have the breakpoints we can disable them all at once using the button. (we can reeanable them all by clicking again)
* we can also reove them all at once with kill all button (in debug mode)

### Lecture 36 - Embedded Code Debugging Part-5: Call Stack Usage

* Keil uvision in debug mode has the call stack window in bottom right
* to show its wuse we implement 3 nested dummy functions (3 level deep)
* in the internal method we try to jump to an address
```
void fun3(void) {
	void (*jump_addr) (void) = 0x00000000;
	jump_addr();
}
```
* in the code above we have afunction pointer (a pointer holding the address of another function)
* in our case that function takes no argument and returns nothing
* the function pointer is initialized to an invalid address
* by calling `jump_addr()` we dereference and jump to the address
* in `void (*jump_addr1) (void) = &delay;` we point the function pointer to a valid address (the address of func 'delay')
* we expect with the dereference of the invalid address to trap ourselfs in hard fault exception handler
* as we go deepr in the subroutting nest we see the addresses of the functions we enter added in the call stack. when we enter fun3 the function pointer is added as well
* even if i let the app run in debug mode and it halts we can press stop and see the last functions it visited in the call stacxk
* in our case feafor eth efault handler we were in fun3. we even see the invlaid meme address

### Lecture 37 - Embedded Code Debugging Part-6: Watch Windows

* in our debug sessions we many times want to watch the value of a variable
* in keil: view => watch windows
* then we enter the variable name and monitor its value (drag and drop them from code file to the wathc window)

### Lecture 38 - Embedded Code Debugging Part-7: Memory Windows

* mem window is in top right and has its  tab.
* we can type in a mem address and see its contents
* keil provides up to 4 em windows
* we can get the mem map for base addresses from proc manual
* eg get the base addr of flash mem and SRAM in our proc memmap
	* FLASH 0x08000000
	* SRAM1 0x20000000
* mem window has raw mem content
* flash has data (code static)
* ram is empty not many runtime data
* stack is in SRAM. it grows and shrincs as program is running
* we can write in the memory in the mewindow
* we can do direct write in mem from our program to see the mem window change `uint32_t *ram_pointer = 0x20000000` 
* then get the hardcoded val from mem dereferencing the pointer `int value = *(ram_pointer);`
* we get error as type has to be the same (int is 16bit uint32_t is 32bit) we need proper typecasting
```
uint32_t *ram_pointer = (uint32_t *) 0x20000000;
int value = *(ram_pointer);
```
* the contents ox 0x20000000 wil lbe assigned to value

### Lecture 39 - Exercise-Copying data from Flash to RAM

* we will copy data from FLASH to SRAM
* we add a prject 'data_copy' and add boilerplate main()
* we will put data in the flash memory. the easiest way is to add constant data.
* data declared with keyword 'const' go to the flash memory and are read-only.
* if i declare a string literal `char my_data[] = "I love embedded programming";` as global var it goes to RAM memory by default
* in main we copy 1st byte to a local var
```
int main(void) {
	
	char data;
	
	data = my_data[0];
	
	return 0;
}
```
* i go into debug mode and confirm that my_char array is stored in SRAM1 base address 0x20000000 and onwards (in mem window)
* in mem window we can RCLICK and  show as ASCII to see the letters
* we define as const char const my_data[] = "I love embedded programming"; and in debug mode we add it to watch .
* we confirm that it resides in flash memory (0x0800037E)
* we do the copy from FLASH TO RA in a for loop writing directly to mem address (we define it in a macro)
```
	for(int i=0;i<sizeof(my_data);i++) {
		*((uint8_t *) BASE_ADDRESS_OF_SRAM+i) = my_data[i];
	}
```

## Section 10 - Understanding MCU Memory Map

### Lecture 40 - Understanding Memory Map of the MCU: Part 1

* Memory Bus of MCU (STM32F446XX)
* ARM Cortex M4
* width of system bus is 32 bits
* the proc can produce 2^32 different addreses (4GB)
* available range is 0x0000_0000 to 0xFFFF_FFFF
* to see the mem map we see the ref manual of the microcontroller
* we can see the generic mem map of our MCU (cortex M4 gtenric) in section 2.2.2 of [ref manual](https://www.st.com/content/ccc/resource/technical/document/reference_manual/4d/ed/bc/89/b5/70/40/dc/DM00135183.pdf/files/DM00135183.pdf/jcr:content/translations/en.DM00135183.pdf) and mem map of device peripherals in STM32F446xx register boundary addresses table.
* in this table we have the register add4ess range. the bus, the reg name and a description
* eg the GPIOA reg base address is 0x4002_0000
* when the processor produces this address on the system bus (AHB1) it is actually referring to the GPIOA registers
* we must understand tha MCU peripherals resinde in SoC but are processor external and communicate with the proc through the vaious System Buses

### Lecture 41 - Understanding Memory Map of the MCU: Part 2

* we  dive into our MCUs datasheet (reference manual)
* in our previous course we talked in depth on the Cortex M4 me map which is common for all devices using it
* core feats like FLASH, SRAM , bitbanded areas are common
* peripeherals vary per mcu family

### Lecture 42 - Understanding Memory Map of the MCU: Part 3

* Questions to answer based on STM32F446XX Ref Manual:
	* What's the base address of AHB1 Bus peripherals? 0x4002_0000
	* What's the base address of GPIOA registers? 0x4002_0000
	* What's the base address of RCC engine registers of the MCU? 0x4002_3800
	* What's the base address of APB1 peripherals? 0x4000_0000
	* What's the base address of FLASH Memory? 0x08000_0000 
	* What's the base address of SRAM2? 0x2001_0000 (SRAM1 base addr 0x2000_0000 + 112KB)
	* What's the base address of ADC Registers? 0x4001_2000
* we see that System Bus Mem regions contain their peripherals MCU specific registers
* in the MCU header file we can use these addresses as Macros

## Section 11 - MCU Bus Interfaces

### Lecture 43 - MCU Bus Interfaces Explanation Part 1: I-Code/D-Code/S-Bus