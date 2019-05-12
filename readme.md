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

* we can havea top level look of the various buses in the MCU block diagram (Fig3) of [STM32F446 Datasheet](https://www.st.com/resource/en/datasheet/stm32f446re.pdf)
* Some notes on this MCU bus interfaces:
	* Cortex M4 Proc has several bus interfaces
	* Proc talks to Peripherals through the bus interfaces
* Cortex M proc exposses 3 types of bus interfaces:
	* I-BUS (instructions bus) = ICode Mem Interface (32bit AHB-Lite)
	* D-BUS (data bus) = DCode Mem Interface (32bit AHB-Lite)
	* S-BUS (system bus) = System Interface (32bit AHB-Lite bus)
* in a prev project we copied data from FLASH to RAM
* our app program is converted into instructions stored in the MCU program memory (FLASH mem)
* FLASH mem stores:
	* Instructions
	* const data (Read only)
	* vector tables
* on chip FLASH mem is external to the proc.
* proc uses the I-Bus to fetch instructions from the FLASH mem
* proc uses the D-Bus to read data from the ReadOnly area in the FLASH
* we rerun 'data_copy' project in debug mode
* our code reads data from FLASH and copies the to RAM. the read operation is done over the D-BUS
* onchip FLASH mem (512kB) has a FLASH mem controller interface that connects to the Proc bus interfaces through the on Chip AHB BUS MAtrix  7S8M
* More info on the Proc bus types we can get on the [Cortex M4 TechRefmanual](http://infocenter.arm.com/help/topic/com.arm.doc.100166_0001_00_en/arm_cortexm4_processor_trm_100166_0001_00_en.pdf) at section 2.3.1
* Cortex-M4 contains 3 AHB(Advanced High Performance Bus)-Lite bus interfaces and 1 APB(Advanced Peripheral Bus) Interface
	* I-Bus is used for instruction fetches from Code Mem Space (0x0000_0000 to 0x1FFF_FFFC)
	* D-Bus is used for data and debug access to Code mem space (0x0000_0000 to 0x1FFF_FFFF)
	* S-Bus instruction fetch, data and debug access to addr range 0x2000_0000 to 0xDFFF_FFFF and 0xE010_0000 to 0xFFFF_FFFF

### Lecture 44 - MCU Bus Interfaces Explanation Part 2: AHB/APB1/APB2

* if instructions are present in between mem locations 0x0000_0000 to 0x1FFF_FFFC the Cortex proc will fetch them using the ICode bus I/F
* if instructions are present outside mem locations 0x0000_0000 to 0x1FFF_FFFC the Cortex proc will fetch them using the System bus I/F
* if data are present in between mem locations 0x0000_0000 to 0x1FFF_FFFF the Cortex proc will access them using the DCode bus I/F
* if data are present outside mem locations 0x0000_0000 to 0x1FFF_FFFF the Cortex proc will access them using the System bus I/F
* So FLASH data are accessed via I and D bus while RAM data via S-Bus
* Peripherals fall outside the I and D bus region so in the S-Bus Region
* S-Bus is primarily AHB-Lite (AHB1) but it goes to an AHB/APB bridge being converted into 2 APB buses to access certain peripherals
* The peripherals register mem map tells us which bus they use
* APB is slower and more energy efficient
	* AHB-Lite max speed is 180Mhz
	* APB2 max speed is 90MHz
	* APB1 max speed is 45MHz
* GPIO ports hang under AHB1 so can run at full speed

## Lecture 45 -  MCU Bus Interfaces Explanation Part 3: Q/A session

* is it true that S-Bus is not connected to FLASH memory? yes (Figure 3 of MCU ref manual shows the FLASH mem controller)
* proc can fetch instructions from SRAM over I-code bus? False
* system bus can operate at speed up to 180MHz? true
* SRAMs are connected to System Bus I/F? True
* APB1 bus can operate a speed up to 180MHz ? false
* A peipheral  whose operate freq must be over 95MHz can connect to APB2 bus? false
* proc can fetch instructions and data simultaneously from SRAM ? False (only 1 bus)
* proc can fetch instructions and data simultaneously from FLASH ? True (I and D bus)
* whats the max val of HCLK of the MCU? 180Mhz
* whats the max val of P1CLK of the MCU? 45Mhz
* whats the max val of p2CLK of the MCU? 90Mhz
* GPIOs and proc comm over AHB1 bus? true
* USB OTG and proc comm over AHB2 bus? true
* USB OTG and GPIO can comm to proc simultaneously ? false (only 1 S-Bus)
* Proc can talk to flash mem and SRAP simultaneously ? true (diff buses)

### Lecture 46 - Understanding MCU Bus Matrix

* to understand it we will use the Fig7 of [STM32 DMA controller App note](https://www.st.com/content/ccc/resource/technical/document/application_note/27/46/7c/ea/2d/91/40/a9/DM00046011.pdf/files/DM00046011.pdf/jcr:content/translations/en.DM00046011.pdf) for STM32F4-7  MCU. For STM32F446 we can look at Fig1 from STM32F446 TRM
* horizontally we have bus masters and verticaly bus slaves
* the bus matrix is a matrix that shows the connections of masters to slaves
* we see the processor is the master of its 3 busses and how the peripherals are slaves to the S-Bus

## Section 12 - Understanding MCU Clocks and Details

### Lecture 47 - Understanding MCU Clocking System:Part1

* three different sources can drive the system clock (SYSCLK)
	* HSI oscillator clock
	* HSE oscillator clock
	* Main PLL (PLL) clock
* the device (MCU) have 2 secondary clock sources
	* 32kHz low-speed internatl RC (LSI RC) which drives the independent watchdog and optionally the RTC used for Auto-wakeup from Stop/Standby mode
	* 32.768 kHz low-speed external crystel (LSE crystal) which optionally drives the RTC clock (RTCCLK)
* Physical Clock Sources:
	* Crystal Oscillator: external component (HSE high speed extrnal)
	* RC Oscilator: in the SoC (HSI high speed internal)
	* THe PLL (Phase Locked Loop): in the SoC. generates higher frequency from low frequency

### Lecture 48 - Understanding MCU Clocking System:Part2

* in MCU ref manual we can see the clock tree
* we create a new project in MX cube for the NUCLEO board and go to Clock Configuration screen which is the same clock tree we see in ref manual but is configurable
* HSI is coming tfrom internal HSI RC oscillator at 16MHz
* HSE is going to IO pins (2) accepting a 4-26MHz crystal
* we can use the PLL to increase the speed and us this as input
* from UX the Clock is used to drive the SYSCLK.
* from this we can set various bus clocks etc using divisors 
* AHB bus clock (HCLK) uses a prescaler from the SYSCLK
* from HCLK we get PCLK1 (APB1) and PCLK2 (APB2)

### Lecture 49 - Exercise-Using HSI Clock and Clock Measurement:Part 1

* MCU comes with internal HSI RC oscillator.
* Also by default MCU uses HSI as source of SYSCLK
* even if we use HSE or PLL after reset MCU will revert bakc to HSI
* DISCO board has HSE. NUCLEO does not
* crystal oscillator is more accurate and more stable than RC
* RC freq drifts with temperature etc
* if we dont use CubeMX we can configure clocks with registers to set the prescalers
* we will use CubeMX to set HCLK at 8MHz
* CubeMX => New Project => Select Board => Clock Config
* By default is uses PLLCLK to upscale HSI RC
* We set HCLK to 8 and CubeMX calculates the scales.
* we select HSI in MUX se HCLK to 8 and scaler goes to /2
* we want to measure and confirm the clock. in Clock Config bottom we see that MCU gives IO so that we can measure the clock. its grey because its not enabled. we go to pinout => RCC section => enable master clock output 1 and 2. RCC_MCO1 is in pin  PA8 and RCC_MCO2 in pin PC9 
* MCO1 and 2 are now acticv in clock configuration. we use th mux to out put SYSCLK divided by 4
* we generate sourcecode => Project Manager
	* Project name : HCLK_measurement
	* project folder : Desktop (MDK code gen has problem with deep folder)
	* toolchain MDK-ARM V5
	* code generator => select: copy only necessary library files
	* generate code
* we go to project location => open MDK-ARM folder and open the project
* we build and flash the code

### Lecture 50 - Exercise-Using HSI Clock and Clock Measurement:Part 2

* We verify code with logic analyzer checking in pin C9
* analyzer goes up to 8 MHz

## Section 13 - Understanding MCU Peripheral Clock Control

### Lecture 51 - Understanding MCU Peripheral Clock control

* 