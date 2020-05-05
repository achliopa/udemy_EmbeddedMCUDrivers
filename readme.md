# Udemy Course: Mastering Microcontroller with Embedded Driver Development

* [Course Link](https://www.udemy.com/course/mastering-microcontroller-with-peripheral-driver-development/)
* [Course Repo](https://github.com/niekiran/MasteringMCU)

## Section 2 - Development board used in our courses

### Lecture 4 - About MCU Development board

* NUCLEO-F446RE from ST 
* ST has good tools. and support.
* check for on incircuit debugger
* prefer e board from themdu vendor. pick a vensor with good tools and libs
* check if Board supports the peripherals we are interested in 
* check flash size and on board ram size. aim for >100KB

### Lecture 5. STM32F4 Discovery and Nucleo : Board Details

* See EmbeddedProg Course (same lecture)

## Section 3 - Hardware/Software Requirements

### Lecture 6. Hardware/Software Requirements

* check list in 'Hardware-software-used-in-MCU1'

## Section 4 - Section 4: IDE installation

* Same ad EmbeddedProgrammin Course

### Lecture 7. Downloading STM32CUBEIDE

* we will use STMCubeIDE
* download from ST

### Lecture 8. Installation-Windows

* install all drivers

### Lecture 10. Embedded Target

* ST-Link drivers are installed with IDE

### Lecture 11. Documents required

* download MCU datasheet
* download MCU ref manual
* download Board user manual and schematic

## Section 5: Creating a project using STM32CUBEIDE

check the Embedded C course. its the same

## Section 6: Embedded Code Debugging Tips and tricks

### Lecture 16. Debugging options

* Embeeded code Debugging options
	* SWV (Serial Wire Viewer and data tracing (printf style debugging))
	* Single stepping, steping over and stepping out
	* Breakpoints (Inserting, deleting and skipping breakpoints)
	* Disassembly
	* Call stack
	* Expression and variable windows
	* Memory browser
	* Data watch-points
* SWV and ITM based printf style debugging
	* introduces very minimal tim-space overhead
	* ITM trace source of ARM Cortex M3/M4 proc supports printf style debugging to trace OS and application events and emits diagnostic info for the app captured by the host IDE to analyze behaviour when up is running

### Lecture 17. Single stepping

* we create an STM32 project SWV enabled 
* we cp the main.c code from course (buuble sort)
* when we debug , we get debug perspective
* step return = step out

### Lecture 18. Disassembly and Register windows

* to see the disassembler: window => Show View => disassembly
* instruction steping mode button for instruction single steping
* in disassembly left column shows the address of the instruction. they resinde in FLASH memory. to see the opcode together with the mnemonic of an instruction RCLICK => show opcodes
* to see the registers: Window => show view => registers

### Lecture 19. Breakpoints

* breakpoints are realized by HW address comparators that are there inside the processor debug unit. so the processor must support beakpoints to use them
* the breakpoints we will use are called HW because they are realized by the HW unit of processor
* there are also SW breakpoints realized through breakpoint instructions
* being in debug mode Window => Show view => Breakpoints
* max 6 breakpoints supported by HW unit

### Lecture 20. Expression and variable windows

* in debug mode passing cursor over a var shows expression window
* window => show view => variables window shows the variables window we can add vars to monitor. it gives the current functions stack frame
* in expression window we can do operations on variables

### Lecture 21. Memory browser windows

* mem window shows mem mapped content.
window => show view => memory window 
* we can select cell size and columns num of vars default is 4 words (32bit) per line
* with RCLICK radix we can change mem content representation

### Lecture 22. Call stack and fault analyzers

* if we set a function pointer to an empty memory location and try to call it, it raises illegal operation exception. we see the code trapped in an infinite loop
* we can use the fault analyzer (Window => show as => fault analyzer)

### Lecture 23. Data watch-points

* we want to see where our var is assigned a new val
* in break points => add watchpoint (add expression or read/read)
* it is a conditional breakpoint
* run code

### Lecture 24. SFR windows

* SFRs (Special Function Registers) are for MCU peripherals
* use window => show view => SFRs
* we can even mod them and see results on board

### Lecture 25. Other basic features of IDE

* terminate and rerun to shorten debug cycle
* left ctrl + space = code suggestions
* ctrl+o shows the methods of the source file

## Section 7: Understanding MCU Memory Map

### Lecture 26. Understanding Memory Map of the MCU: Part 1

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

### Lecture 27. Understanding Memory Map of the MCU: Part 2

* we  dive into our MCUs datasheet (reference manual)
* in our previous course we talked in depth on the Cortex M4 me map which is common for all devices using it
* core feats like FLASH, SRAM , bitbanded areas are common
* peripeherals vary per mcu family

### Lecture 28. Understanding Memory Map of the MCU: Part 3

* Questions to answer based on STM32F446XX Ref Manual:
	* What's the base address of AHB1 Bus peripherals? 0x4002_0000 start address, 0x4007 FFFF end address 
	* What's the base address of GPIOA registers? 0x4002_0000
	* What's the base address of RCC engine registers of the MCU? 0x4002_3800
	* What's the base address of APB1 peripherals? 0x4000_0000
	* What's the base address of FLASH Memory? 0x08000_0000 
	* What's the base address of SRAM2? 0x2001_0000 (SRAM1 base addr 0x2000_0000 + 112KB)
	* What's the base address of ADC Registers? 0x4001_2000
* we see that System Bus Mem regions contain their peripherals MCU specific registers
* in the MCU header file we can use these addresses as Macros

## Section 8: MCU Bus Interfaces

### Lecture 29. MCU Bus Interfaces Explanation Part 1: I-Code/D-Code/S-Bus

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

### Lecture 30. MCU Bus Interfaces Explanation Part 2: AHB/APB1/APB2

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

## Lecture 31.  MCU Bus Interfaces Explanation Part 3: Q/A session

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

### Lecture 32. Understanding MCU Bus Matrix

* to understand it we will use the Fig7 of [STM32 DMA controller App note](https://www.st.com/content/ccc/resource/technical/document/application_note/27/46/7c/ea/2d/91/40/a9/DM00046011.pdf/files/DM00046011.pdf/jcr:content/translations/en.DM00046011.pdf) for STM32F4-7  MCU. For STM32F446 we can look at Fig1 from STM32F446 TRM
* horizontally we have bus masters and verticaly bus slaves
* the bus matrix is a matrix that shows the connections of masters to slaves
* we see the processor is the master of its 3 busses and how the peripherals are slaves to the S-Bus

## Section 9: Understanding MCU Clocks and Details

### Lecture 33. Understanding MCU Clocking System:Part1

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

## Section 10: Understanding MCU Clock tree



### Lecture 34. Understanding MCU clock sources and HSE

* in MCU ref manual we can see the clock tree
* we create a new project for the NUCLEO board with target project type STM32Cube and do not do default config. this opens CubeMX
* and go to Clock Configuration screen which is the same clock tree we see in ref manual but is configurable
* HSE is going to IO pins (2) accepting a 4-26MHz crystal. NUCLEO has no external crystal. in ref manual it shows how to connect crystal. NUCLEO takes external 8MHz from STLINK and its crystal at pin PD0 (MCO)
* we can use the PLL to increase the speed and use this as input
* DISCO board has HSE. NUCLEO does not
* crystal oscillator is more accurate and more stable than RC

### Lecture 35. HSI and RCC registers

* HSI is coming tfrom internal HSI RC oscillator at 16MHz
* MCU comes with internal HSI RC oscillator.
* Also by default MCU uses HSI as source of SYSCLK
* even if we use HSE or PLL after reset MCU will revert back to HSI
* from MUX the HSI Clock is used to drive the SYSCLK.
* from this we can set various bus clocks etc using divisors 
* we cannot select HSE in CubeMX clock config. to do so we need to
	* go to pinout & configuration => system core => rcc => HSE clock source: Bypass (for Nucleo)
	* now we can select HSE as source of SYSCLK
* Sysclk is used as source for bus clocks
	* AHB bus clock (HCLK) uses a prescaler from the SYSCLK
	* from HCLK we get PCLK1 (APB1) and PCLK2 (APB2) and for peripherals
	* also it clocks SysTick timer through prescaler
	* also it clocks our processor clock FCLK
* HSI RC freq drifts with temperature etc
* if we dont use CubeMX we can configure clocks with registers to set the prescalers
* PLL is not used in this course. PLLCLK is used to upscale  using multiplier HSE or HSI. then it is used as source to SYSCLK
* RCC registers are used to configure clock
* I2S interface, USB,Ethernet needs PLL

### Lecture 36. Peripheral clock configuration

* we will see the RCC (Reset and Clock Control) engine of MCU
* to work with peripherals on MCU we have to first enable their clocks
* all peripherals drive clock from the bus they are connected to
* without clock enabled peripherals are dead (no energy consumption)
* RCC is a complete clock control engine allowing to enaable peripherals clocks
* the clock configuration in STM32CubeMX is stored in RCC registers
* we refer to the manual RCC reg section to say: enable ADC peripheral
	* in ADC_CR1 we want to set 8bit = 1
	* we create a new empty project 004PeriClockEnable
* we define with macro the reg
```
#define ADC_BASE_ADDR 			0x40012000UL
#define ADC_CR1_REG_OFFSET 		0x04
#define ADC_CR1_REG_ADDR 		(ADC_BASE_ADDR + ADC_CR1_REG_OFFSET)
```
* we set 8th bit
```
uint32_t *pAdcCr1Reg = (uint32_t*) ADC_CR1_REG_ADDR;
	*pAdcCr1Reg |= (1<<8);
```
* it has no effect when we run as clock is not enabled. we need to consult the manual and see on which bus it is connected (APB2) and see the reg. we set the macros to enable ADC1 clock and set it (this need to be done before)
```
#define RCC_BASE_ADDR			0x49923800UL
#define RCC_APB2_ENR_OFFSET		0x44UL
#define RCC_APB2_ENR_ADDR		(RCC_BASE_ADDR + RCC_APB2_ENR_OFFSET)
	uint32_t *pRccApb2Enr = (uint32_t*) RCC_APB2_ENR_ADDR;
	*pRccApb2Enr |= (1<<8);
```

### Lecture 37. Exercise : HSI measurements

* we will write a program to output HSI clock on MCU pin and measure it using oscilloscope or logic analyzer
* Steps to output a clock on MCU pin
	* select the desired clock for the MCOx signal (MCU Clock Output)
	* output the MCOx signal on the MCU pin
* we start a STM32 project 004Clock with target project cube (no defaults)
* CubeMX => New Project => Select Board => Clock Config
* there are 2 signals we can use to output the clock to pin MCO1 and MCO2
* we see that HSI can be outputed only at MCO1 and it is disabled by default
	* go to pinout => system core => RCC and enable MCO1 and MCO2
* we need to route these signals to pins
* we can divide the freq before outputing with CubeMX or RCC clock config register and config even source from reg (e.g. set MCO1 input to HSI (bit21:22 = 00))
* to output to pin we need MCU pin details (in MCU datasheet) at pin description table => alternate function mapping (16 in total for each pin)

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
	* MCO1 can be mapped to port PA8 with AF0 (alternate function 0)
	* PA8 to physical map is in the pin and ball definition and is package dependent
	* MCO2 can be mapped to port PC9 with AF0 (alternate function 0)

### Lecture 38. About USB logic analyzer

* We have it we know it

### Lecture 39. Code implementation

* we create a new STM32 Project without using CubeMX
* first we need to set HSI as source for MCO1 (RCC_CFGR bit21:22)
```
#define RCC_BASE_ADDR              0x40023800UL
#define RCC_CFGR_REG_OFFSET        0x08UL
#define RCC_CFGR_REG_ADDR          (RCC_BASE_ADDR + RCC_CFGR_REG_OFFSET )
uint32_t *pRccCfgrReg =  (uint32_t*) RCC_CFGR_REG_ADDR;
//1. Configure the RCC_CFGR MCO1 bit fields to select HSI as clock source
*pRccCfgrReg &= ~(0x3 << 21); //clear 21 and 22 bit positions
```
* we then have to config PA8 to behave as MCO1 (AF0 mode) but first we need to enable the peripheral clock of GPIOA peripheral
```
	 uint32_t *pRCCAhb1Enr = (uint32_t*)(RCC_BASE_ADDR + 0x30);
	*pRCCAhb1Enr |= ( 1 << 0); //Enable GPIOA peripheral clock
```
* we then need to configure the PA8 mode in GPIAMODE reg as alternate function
```
#define GPIOA_BASE_ADDR			0x40020000UL
	uint32_t *pGPIOAModeReg = (uint32_t*)(GPIOA_BASE_ADDR + 00);
	*pGPIOAModeReg &= ~( 0x3 << 16); //clear
	*pGPIOAModeReg |= ( 0x2 << 16);  //set
```
* we then set the alternate function high register for GPIOA 'GPIOA_AFRH' to select mode 0 for pin PA8
```
	uint32_t *pGPIOAAltFunHighReg = (uint32_t*)(GPIOA_BASE_ADDR + 0x24);
	*pGPIOAAltFunHighReg &= ~( 0xf << 0);
```
* we run the code and connect the logic analyzer...
* our logic analyzer cannot sample at 16Mhz
* we need to divide the clock using a prescaler RCC CFG reg.
```
	//1.5 Configure MCO1 prescaler divide by 4
	*pRccCfgrReg |= ( 1 << 25);
	*pRccCfgrReg |= ( 1 << 26);
```
* we confirm the 4MHz clock rate in the logic analyzer

### Lecture 40. Exercise : HSE measurements

* To do measurement of HSE clock through MCO1 (Discovery Board)
	* We need to enable the HSE clock using HSEON bit in RCC control register (RCC_CR)
	* Wait until HSE clock from the external crystal stabilizes HSERDY bit in RCC_CR becomes 1 (only if crystal is connected) (indicates if the high-speed external oscillator is stable or not)
	* Switch the systemclock to HSE (RCC_CFGR)
	* Do MCO1 settings to measure it
* To do measurement of HSE clock through MCO1 (NUCLEO Board)
	* We need to enable the HSE clock using HSEBYP bit in RCC control register (RCC_CR), to bypass the oscillator with an external clock
	* Enable the HSE clock using HSEON bit in (RCC_CR)
	* Switch the systemclock to HSE (RCC_CFGR)
	* Do MCO1 settings to measure it
* the code
```
#define RCC_BASE_ADDR       	0x40023800UL
#define RCC_CFGR_REG_OFFSET  	0x08UL
#define RCC_CR_REG_OFFSET    	0x00UL
#define RCC_CFGR_REG_ADDR     	(RCC_BASE_ADDR + RCC_CFGR_REG_OFFSET )
#define RCC_CR_REG_ADDR      	(RCC_BASE_ADDR + RCC_CR_REG_OFFSET )
#define GPIOA_BASE_ADDR			0x40020000UL

int main(void)
{
	uint32_t *pRccCrReg = (uint32_t*)RCC_CR_REG_ADDR;
	uint32_t *pRccCfgrReg = (uint32_t*)RCC_CFGR_REG_ADDR;
	//1.Enable the HSE bybass using HSEBYP bit (RCC_CR)
	*pRccCrReg |= ( 1 << 18);
	//2.Enable the HSE clock using HSEON bit (RCC_CR)
	*pRccCrReg |= ( 1 << 16);
	//3. Switch the system clock to HSE (RCC_CFGR)
	*pRccCfgrReg |= ( 1 << 0);
	//4. Configure the RCC_CFGR MCO1 bit fields to select HSE as clock source
	*pRccCfgrReg |= ( 1 << 22); //clear 21 and SET 22
	//5. Configure MCO1 prescaler // divisor as 4
	*pRccCfgrReg |= ( 1 << 25);
	*pRccCfgrReg |= ( 1 << 26);
	//6. Enable GPIOA peripheral clock in RCC AHB1 enable reg
	 uint32_t *pRCCAhb1Enr = (uint32_t*)(RCC_BASE_ADDR + 0x30);
	*pRCCAhb1Enr |= ( 1 << 0); //Enable GPIOA peripheral clock
	//7. Configure the mode of GPIOA pin 8 as alternate function mode (0x02)
	uint32_t *pGPIOAModeReg = (uint32_t*)(GPIOA_BASE_ADDR + 00);
	*pGPIOAModeReg &= ~( 0x3 << 16); //clear
	*pGPIOAModeReg |= ( 0x2 << 16);  //set
	//8. Configure the alternation function register to set the mode 0 for PA8
	uint32_t *pGPIOAAltFunHighReg = (uint32_t*)(GPIOA_BASE_ADDR + 0x24);
	*pGPIOAAltFunHighReg &= ~( 0xf << 0);
	for(;;);
}
```
* we test with Logic Analyzer on pin PA8 and it works (2MHz)

## Section 11: Understanding MCU Vector table

### Lecture 41. Understanding MCU Vector Table: Part-1

* a vector table is a table of ponters or addresses of the exception handlers
* exceptions are: system exceptions + interrupts/external exceptions
* ARM M4 core supports 15 sys exceptions + 240 interrupts (not all are implemented)
* in the ref manual if the MCU we can see the table and the addresses
	* position has to do with NVIC (fixed by ARM) = IRQnum
	* priority has to do with interrupt preemption
	* type of priority has to do if it is settable by programmer ir not
	* lower priority num = higher priority
	* address is the mem map address of the interrupt handler. they hold the function pointers of the handlers
* the first address 0x00000000 holds the initialization value of the MSP stack pointer (address). it is OUR duty in the startup file to put a valid address in 0x0000000 for the MSP stack
* handlers in asembly are set as weak so that we can override them
* note that addresses are stored always +1 for the Tbit

### Lecture 42. Understanding MCU Vector Table: Part-2

* NVIC in ARM Cortex M4 takes care of Interupt priorities etc and sends one interrupt signal to CPU core

## Section 12: Understanding MCU interrupt Design , NVIC, Interrupt handling

### Lecture 43. Understanding MCU interrupt Design , NVIC, Interrupt handling: Part 1

* we know that in NUCLEO board the User Button is connected to GPIO PortC bit 13
* is pulled up. when we press button it goes to GRND
* How GPIO pin interrupts the CPU??
	* first the go to EXTI engine
	* at external interrupt/event line mapping of EXTI engine for external interrupts which hangs on APB bus
	* the engine connects to NVIC interupt controller. the engine sends 23 lines
	* some peripherals (SPI) go directly to NVIC

### Lecture 44. Understanding MCU interrupt Design , NVIC, Interrupt handling: Part 2

* at external interrupt /event line mapping we see that PC13 goes to EXT13 mux together with the 13th pin of other ports.
* in the diagram we see that its configured in SYSCFG_EXTICR4 reg at bits 4-7 where we need to write 0010: PC[x] pin to enable the PC port pin
* this reg is for the MUX that produces the EXTI13 line. the cofig of the GPIO is another story
* we can confirm it if we make a CubeMX project for the board and  check the generated code. to configure the pin in Cube MX:Pinout&Config=>SystemCore=>GPIO
* at PC13 if we click we see the config options
	* external interrupt mode falling edge (if we change to rising edge the interupt will trigger when we release the button). no need for pull up/down its done externally
	* if we highlight the pin in CubeMX pinout display we see EXTI0 is configured... so we go to NVIC 
	* in NVIC we have the option to enable EXTI
	* in code geenration we are asked if we want a handler to be implemented for us
	* as clock source we choose HSI
* we generate code and view it
	* we see the interrupt handlers. we build debug and put a breakpoint to the handler.. press the button. at release we see  the program entering the handler

### Lecture 45. Understanding MCU interrupt Design , NVIC, Interrupt handling: Part 3

* we see EXTI base address in mem-map 0x40013C00
* we check EXTI_PR (offset 0x14)
* while running the program we view the 0x40013C14 mem content and see the bit for 13 raised.
* this tells us an interrupt is pending in line13
* its our responsibility to clear the bit in the handler so a nre interrupt can be queued for execution. we need to write an 1 to clear. this is done in the HAL code

### Lecture 46. Understanding MCU interrupt Design , NVIC, Interrupt handling: Part 4

* each EXTI line is ORed with EXTI_PR bit for this line before going to NVIC
* when we press button line and PR bit goes high. even if we release the button and line goes low PR bit is still high so NVIC still holds CPU to Handler mode
* that why we need to clear the bit before exiting the handler
* if we dont clear the CPU is stuck in the handler

## Section 13: Importance of "Volatile" Keyword

* we know this from embedded c course... 
* volatile must always be used for pointers to mem map space to protect from compiler optimization especially when we use higher optimization options in compiler
* volatile tells compiler that content is highly volatile in nature . it can change anytime. so it should not attempt any optimization but instead be aware that it is expected to read again and again from this memory
* these are cases that initialize once update/read often
* use volatile when you code is dealing with below scenarios
	* Memory-mapped peripheral registers of the MCUs
	* multiple tasks accessing global variables(read/write) in an RTOS multithreaded app
	* when a global var is used to share data between the main code and an ISR code

## Section 14: GPIO Must know concepts

### Lecture 49. GPIO pin and GPIO port

* GPIOs are used to
	* Read digital signals
	* issue interrupts
	* generate triggers for external components
	* wake up the processor
* GPIO port is a collection of pins (8,16,32)

### Lecture 50. GPIO behind the scene

* In MCU the pin is implemented with an input buffer (CMOS transistor gate) and an output buffer (CMOS transistor gate) with enable line. 
* their gates are opossite logic
* the enable line is register controlled

### Lecture 51. GPIO input mode with high impedance state

* Hi impedance or hi-Z mode when pin is in input mode is when we keep the pin floating not connecting it to high or low level (pull up/down)
* By default all GPIO pins are in High-Z mode
* in that state pin can pickup circuit noise
* also it can lead to current leakage

### Lecture 52. GPIO input mode with pull-up/down state

* floating state (High-Z) can be avoided by using internal or external pull up or down resistors
* its safe to keep unused GPIOs in one of two states to avoid picking up noise from circuit and lead to current leakage

### Lecture 53. GPIO output mode with open drain state

* pin output mode with open drain config is nothing but the top PMOS transistor not present
* when transistor is switched on, the output will be pulled LOW. when its off the drain will be open so the pin will be floating
* open drain has only 2 states: GND or float. this is useledd unless we provide a pull up resistance internal or external
* so OPEN DRAIN config has to be used WITH external or internal PULL UP
* Usage of OPEN DRAIN config to drive a LED: 
	* enable pull up
	* wire the LED to the GND 
	* when 1 is sent in NOT to 0 so transistor is off anf LED is high (ON)
	* when 0 is sent => ... => LED is OFF
* we can alternatively not use internal pull up and use an external one to VCC
* even if we drive I2C both SDA and SCL are open drain by default.. so we need pull up resistors on both lines
* prefer to use internal pull up/down res unless we have valid design ossued
* CHECK DESIGN if internal pull ups/down can fit

### Lecture 54. GPIO output mode with push pull state

* output mode with push-pull config is the default when we set a pin to output ode. 
* both CMOS (PMOS to VCC and NMOS to GND are used with reverse gates)
* in this case we dont need neither pull up or down res
	* top trans PMOS is ON / bottom trans NMOS is OFF when output drives HIGH
	* the opposite for LOW
* when we connect a LED to a push/pull output we need a limiting resistor to limit the current passing from the LED  and then to GND

### Lecture 55. Optimizing I/O power consumption

* in input pin when it is connected to VCC or GND there is no current leakage as the one CMOS is ON and the other OFF
* when it is open (no connection) voltage of input  can fluctuate (0.5 - 0.2Vcc). it can turn on both CMOS so small amount of current will leak

## Section 15: GPIO Programming structure and Registers

### Lecture 56. GPIO programming structure

* GPIO Registers stand between the Bus And the IO Port
	* Port Direction (Mode) reg
	* Port Speed Reg 
	* Port output type Reg
	* more ... config regs...
	* Port output Data reg
	* Port Input Data Reg

### Lecture 57. Exploring GPIO PORT and pins of STM32F4xx Discovery board

* The MCU of the Discovery board has 9 ports of 16 pins (144total) 80 pins available on the headers

### Lecture 58. GPIO Mode register(used to set mode for a pin)

* GPIO MODE REG in ref manual (GPIOx_MODER) at offset 0x00
* 2 bits per pin config 00: input, 01: output mode, 10: alternate function, 11: analog
* reset val is important for all registers

### Lecture 59. Input configuration of a Microcontroller's GPIO Pin

* some  pins cannot be used  for all purposes
* when MCU pin is configured as input it can be configured to issue an interrupt to the processor
* in GPIO general description we can see the full circuit of a GPIO pin
* for every AHB1 bus cycle in input mode the pin status will be passed to the input data register passing from the the TTL schmidt trigger. output driver is open circuit
* not using pull up or down will produce leakage and floating reads...
* if we connect a switch to ground we need a pull up
* for STM32F$ internal pull up and down is 40KOhn
* When an IO pin os programmed in Input Mode
	* the output buffer is disabled
	* schmitt trigger input is activated
	* pull up and pull down resistors are activated depending on the value in the GPIOx_PUPDR reg
	* the data present on the I/O pin are sampled into the input data register every AHB1 clock cycle
	* a read access to the input data reg provides the I/O state

### Lecture 60. Output Configuration of a GPIO Pin in Push pull mode

* 2 configs in output mode: open drain & push/pull
* in push/pull in bothe states we have 1 or 0 (no floating,no high impedance)
* in open drain 0 gives 0 (NMOS on => grounded), in 1 is floating
* in open drain we have to activate the pull up resistor (when we give 1) but the intrnal pull up resistanc eis very high so the current will be small and LED wont light up. we need and external pull up (smaller say 1KOhm)
	* in case 0 VCC will flow to ground through the NMOS (LED off)
	* in case of 1 VCC will flow through LED to GND (LED On)
* When we clearly want 2 states in output: use push/pull (default)
* when we want high-Z state use open drain (e.g I2C)

### Lecture 62. Input stage of a GPIO pin during output configuration

* in output mode the TTL schmitt trigger (input buffer) will be ON 
* so we can read the state of PIN while outputing

### Lecture 63. Alternate functionality Configuration of a GPIO pin

* Alternate Function Mode is all the Goodies in peripherals apart from GPIO
* in AF mode input and output data registers are bypassed and signal is going to the other peripheral after passing the buffers

### Lecture 64. GPIO out put type register explanation

* when GPIO is in OutMode this register GPIOx_OTYPER selects the type (opendrain, push pull) 1bit per pin

## Section 16: GPIO Registers : SPEED, PULL UP/DOWN, IDR and ODR

### Lecture 65. GPIO output speed register and its applicability

* GPIOxOSPEEDR reg is applicable only in Output Mode and configures the I/O output speed of transitioning from H to L (Tfall) and from L to H (Trise)
 (slew rate). 
* there are 4 options (2bit p/ pin) 00 (Low speed), 01 (Med Speed), 10 (High Speed), 11 (Very High Speed)
* check the manual for exact time
* Datasheet also the max possible frequency of rate and the times depending also per Vdd
* a case that we might need it is to support bitbnging to I2C if our MCU does not support it

### Lecture 66. GPIO Pull up and Pull down register

* self explanatory. GPIOx_PUPDR 0x0C the address offset, 2bits per pin
* options: 00: non pull up/down, 01: pull up, 10: pull down

### Lecture 67. GPIO input data register

* GPIOxIDR (offset 0x10) 1bit per pin. used to read the state of the input pin

### Lecture 68. GPIO output data register and summary of various modes discussed

* GPIOx_ODR (offset 0x14) 1bit per pin. used to control the state of out pin
* GPIO Functional Summary (possible configs for a GPIO pin)
	* Input floating
	* Input pull-up
	* Input pull-down
	* Analog
	* Output open drain with pull-up/down capability
	* Output push-pull with pull up/down capability
	* Alternate function push/pull with pull-up/down capability
	* Alternate function open drain with pull-up/down capability

## Section 17: GPIO Alternate functionality register and example of usage

### Lecture 69. Alternate functionality settings of a GPIO pin with example : Part 1

* up to 16 different Alternate Functionalities per pin (SPI,TIMER,UART,SPDIF,CAN,I2C,I2S)
* We can see them to MCU Datasheet @ Alternate Function Mapping

### Lecture 70. Alternate functionality settings of a GPIO pin with example : Part 2

* GPIOxAFRL (offset 0x20) gpio altern function low register is used for pins 0-7 and GPIOxAFRH (offset 0x24) gpio altern function high register is used for pins 8-15
* 4bits per pin to selct among 16 AF modes AF0-AF15

## Section 18: GPIO peripheral clock control

* to enable/disable thte GPIO peripheral clock:
	* we know from architecture of MCU that GPIO ports hang under AHB1 bus
	* so we need to set RCC_AHB1ENR registers GPIOx_EN bit for the GPIO port we want to enable
* if we dont do it the port is dead to save power and we cannot do any config or operation

## Section 19: GPIO driver development overview and Project creation

### Lecture 73. GPIO driver development overview

* High level Project Architecture:
* in these 2 courses we will develop the Driver Layer 
	* a source file & header file for each peripheral e.g `gpio_driver.h` + `gpio_driver.c` 
	* MCU device header `STM32f446xx.h` 
	* drivers stand on top of PHYSICAL layer and below Application
	* Drivers expose driver APIs to app

### Lecture 74. MCU Specific header file and its contents

* Device Header File is a C header file containing MCU specific details such as:
	* the base addresses of various memories present in MUC mem map (Flash,SRAM1,ROM etc)
	* the base addresses of various bus domains such as (AHBx domain, APBx domain)
	* base addresses of various peripherals of MCU
	* clock mamangement macros (clock enable, clock diaable)
	* IRQ definitions
	* Peripheral Register definition structs
	* Peripheral reg bit definitions
	* other MCU config macros
* used by both App and drivers

### Lecture 75. New project creation and creating MCU specific headerfile

* we create a new STM32 project in Workspace 'stm32f4xx_drivers' of empty project type
* inc folder is for header files
* click on project in tree and create new folder named 'drivers' and in it 2 folders 'inc' and 'src'
* then RCLICK on 'drivers' folder => Properties => C/C++ build and uncheck exclude from build
* in /inc we create a device header file 'stm32f446xx.h'

### Lecture 76. Include path settings

* we include the file in main.c but compile cannot find path
* in project => properties => c/c++build => tool settings => MCU GCC Compiler => include paths we add the folder drivers/inc to the path . we rebuild

## Section 20: Updating MCU specific header file with bus domain and peripheral details

### Lecture 78. Writing base address C macros for MCU's embedded memories : Part 1

* we start adding mem areas base addresses as macros in header file
```
#define FLASH_BASEADDR 		0x08000000U
#define SRAM1_BASEADDR		0x20000000U
#define SRAM2_BASEADDR		0x20001C00U
#define	ROM					0x1FFF0000U		/* system memory */
#define SRAM 				SRAM1_BASEADDR
```
* definitions by default are signed. mem addresses are unsigned. we should clearly cast to unsigned with U or using stdint.h uint32_t

### Lecture 80. Defining base addresses of different bus domains

* PERIPH_BASE is the bse for sifferent bus domains of STM32F4
	* APB1PERIPH_BASE 0x40000000
	* APB2PERIPH_BASE 0x40010000
	* AHB1PERIPH_BASE 0x40020000
	* AHB2PERIPH_BASE 0x50000000
* different peripherals are hanging on different busses
* AHB bus is used for those peripherals which need high speed data communication (e.g Camera if, GPIO)
* APB bus is for peripherals that do not need high speed comm

### Lecture 81. Defining base addresses of AHB1 Peripherals

* we implement the memory map as we see it in ref manual
* we will define only GPIO,SPI,I2C and UART peripherals of this course and also SYSCFG
```
#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2000)
```

### Lecture 82. Defining base addresses of APB1 and APB2 Peripherals

* same thing
* UART4 and 5 are not USART as they do not support synchronous comm
* use CAPS for macros
* HAL or DRV are prefixes used for HAL layer or DRV layer macros

## Section 21: Structuring peripheral registers

### Lecture 84. Address of peripheral registers

* we determine and write macros for the addresses of various registers of a peripheral using offsets from its bse address using the MCU reference manual

### Lecture 85. Structuring peripheral registers

* it is very tedious to write macros for each and every register
* we can use a C struct for a peripheral register set definition and then set its pointer to the peripheral address in mem space
* structure elements act as placeholders for the registers
```
typedef struct {
	volatile uint32_t MODER;			/*!< GPIO port mode register, Address Offset: 0x00 */
	volatile uint32_t OTYPER;		/*!< GPIO port output type register,  Address Offset: 0x04 */
	volatile uint32_t OSPEEDR;		/*!< GPIO port output speed register, Address Offset: 0x08 */
	volatile uint32_t PUPDR;			/*!< GPIO port pull-up/pull-down register, Address Offset: 0x0C */
	volatile uint32_t IDR;			/*!< GPIO port input data register, Address Offset: 0x10 */
	volatile uint32_t ODR;			/*!< GPIO port output data register, Address Offset: 0x14 */
	volatile uint32_t BSRR;			/*!< GPIO port bit set/reset register, Address Offset: 0x18 */
	volatile uint32_t LCKR;			/*!< GPIO port configuration lock register, Address Offset: 0x1C */
	volatile uint32_t AFR[2];		/*!<GPIO alternate function Address Offset Registers: 0x20-0x24 */
} GPIO_RegDef_t;
```

* then we have to map a pointer to the base addr `GPIO_RegDef_t *pGPIOA = (GPIO_RegDef_t*) GPIOA_BASEADDR;`
* and we can access reg as `pGPIOA->MODER`
* to build the structs go directly to the Periperal Register Map in Ref Manual
* use volatile as regs are highly volatile

### Lecture 86. Peripheral definition macros

* we ll define another macro for the pointers `#define GPIOA   	((GPIO_RegDef_t*)GPIOA_BASEADDR)`


## Section 22: Writing Clock enable and disable macros

### Lecture 87. Writing peripheral clock enable and disable C Macros

* we add a struct def for RCC regs and we add macros to enable clock for GPIOx peripherals `#define GPIOA_PCLK_EN()	(RCC->AHB1ENR |= (1<<0))` etc
* we do the same for other peripherals SPI, I2C etc
* we write clock disable macros `#define GPIOA_PCLK_DIS()	(RCC->AHB1ENR &= ~(1<<0))`

### Lecture 88. Project include settings and build

* rebuild the project

## Section 23: GPIO driver API requirements and handle structure

### Lecture 89. Creating GPIO driver header and source file

* in drivers/src we add 'stp32f446xx_gpio_driver.c' file
* in drivers/inc we add 'stp32f446xx_gpio_driver.h' file
* include the device header in gpio header file
* include the h in c

### Lecture 90. Defining GPIO handle and configuration structure

* For a User Application we need to configure
	* GPIO port name
	* GPIO pin number
	* GPIO mode
	* GPIO speed
	* GPIO outputtype
	* GPIO pullup/don
	* GPIO Alt. Function mode
* we have to offer APIs for al these and a config structure to confgi the above
* the struct will be passed to the API
* we will config 2 structures.
	* GPIO Handle
	* Config Struct
```
typedef struct {
	uint8_t	GPIO_PinNumber;
	uint8_t	GPIO_PinMode;
	uint8_t	GPIO_PinSpeed;
	uint8_t	GPIO_PinPuPdControl;
	uint8_t	GPIO_PinOPType;
	uint8_t	GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/*
 * this is a handle structure for a GPIO pin
 */

typedef struct {
	GPIO_RegDef_t		*pGPIOx;		/*!< This holds the base address of the GPIO to which the pin belongs >*/
	GPIO_PinConfig_t	GPIO_PinConfig; /*!< This holds GPIO pin condifguration settings >*/
}GPIO_Handle_t;
```

### Lecture 91. Driver API requirements and adding API prototypes

* A GPIO driver should give the following APIs to the App
	* GPIO Initialization
	* Enable/Disable GPIO port clock
	* Read from a GPIO pin
	* Write to a GPIO pin
	* Config Alternate Functionality
	* Interrupt Handling
* we add boilerplate function stubs for all these

### Lecture 92. Driver API input parameters and return types

* For peripheral clock control we need the GPIO port base address and a flag for enable or disable `void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);`
* the base adress is needed purely to know which port wea re dealing with
* for init pass the handler `void GPIO_Init(GPIO_Handle_t *pGPIOHandle);`
* for deinit we pass the base addr again to know which port it is`void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);` deinit is done through RCC_AHB1RST reseting a bit we clear all regs
* read write api stubs are in the same philosophy
```
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
```

* the stubs for IRQ are IRQ specific
```
void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);
```

### Lecture 93. Driver empty API implementation and documentation

* we ll start fleshing out the prototypes in .c driver file
* we cp the stubs make them functions and add comment descripitons

## Section 24: GPIO driver API Implementation : Clock control

### Lecture 94. Implementation of GPIO peripheral clock control API

* we need to enable or disable the clock for a port based on the base address (which GPIO) and the flag. the logic is simple
```
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){
	if(EnorDi == ENABLE) {
		if(pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		}
	} else {
		
	}
}
```

## Section 25: GPIO driver API Implementation : GPIO init and de-init

### Lecture 95. Writing user configurable macros

* initialization will be done based on the params passed in the config struct
	1. configure mode of gpio pin
	2. configure the speed
	3. configure the pupd settings
	4. configure the optype
	5. configure the alt func
* we need macros for all the options in the h file ....
* for pin mode we add also the interrupt types

### Lecture 96. Implementation of GPIO init AP

* we wont code for interupt modes in init
*  we should only touch the bits for the pin we want to config and leave the rest of the register untouched (clear the affected bits then set them)
* the pattern to set the registers using the config option that will be used extensively is
```
		uint32_t temp = 0;
		// the non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clear bits
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;
```

### Lecture 98. Configuring Alternate function registers

* AFR[0] is AFRlow  (pin0-7) and AFR[1] is AFRhigh (pin8-15)
* we divide pinNum by 8 to decide which reg to affect
* we also need the modulo for bit shifting
```
uint8_t temp1,temp2;	
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << temp2); //clear bits
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4* temp2));
```

### Lecture 99. GPIO de-init API implementation

* we need to reset all the registers for deinit
* this is done by seting the respective bit of RCC_AHB1RSTR and then reseting it
* we use the code of ClockControl changing the Reg
* we need to create the `GPIOx_REG_RST();` macros
* for GPIOA its `#define GPIOA_REG_RST() 	do{(RCC->AHB1RSTR |= (1<<0));	(RCC->AHB1RSTR &= ~(1<<0));}while(0)`
* the way to implement macros with multiple statements is #define MACRO() do{statement1; .. ; statementX;}while(0)

## Section 26: GPIO driver API Implementation : GPIO data read and write

### Lecture 100. Implementation of GPIO input port read and input pin read APIs

* we extract the bit from IDR reg in read bit shifting to 0 to make an uint8_t to return
```
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber){
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001); 
	return value;
}
```

### Lecture 101. Implementation of GPIO output port write and output pin write APIs

* easy set / reset bit
```
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value){
	if(Value == GPIO_PIN_SET) {
		// write 1 to the REG at respective bit
		pGPIOx->ODR |= (1 << PinNumber);
	} else {
		// write 1 to the REG at respective bit
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}
```

### Lecture 102. Implementation of GPIO pin toggle API

```
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber){
	pGPIOx->ODR ^= (1 << PinNumber);
}
```

## Section 27: Exercise

### Lecture 103. Exercise : LED toggling with PUSH PULL configuration

* Write a program to toggle the on board LED with some delay
	* 1. use push pull config for output pin
	* 2. use open drain config for output pin
* In Nucleo LED is in A5 and it has resistor
* we delete main.c
* we add a 001led_toggle.c in /src
* we import device header
* in device header end we include the driver header
* the code is straightforward.  set the config struct then call the api to init and toggle
```
int main(void) {

	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

	while(1){
		GPIO_TogglePin(GPIOA, GPIO_PIN_NO_5);
		delay();
	}
	return 0;
}
```

### Lecture 104. Exercise : LED toggling with OPEN DRAIN configuration

* we do 2 changes and rerun
```
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
```

* but we see a fading light because internal pull up is high ohm
* if we remove internal pull up we see no lght as the external resistor is not pull up
* to be able to drive the LED in opendrain we need an external pull up with smaller resistance (e.g 320ohm )

### Lecture 105. Exercise : Handling on board LED and Button

* Exercise: togle LED whenever the button is pressed (PC13) pressed is LOW, released is HIGH
* create a new file 002led_button.c and exclude from build the 001 file
* cp the code
* we create a new GPIO_Handle_t and config for input
```
/* Button config */
	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioBtn);
```
* we use `GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)` to probe the input
* we need delay for button debouncing
* in pushbuttons we have 4 pins. A-D are connected and B-C. when we push A-B-C-D are connected

### Lecture 106. Exercise : Connecting external button and circuit explanation

* use 2 free GPIOs for an external button and LED and conect them from breadboard to test
* use 22kohm pull up for ush button
* straigth leg of led is cathode to ground
* in anode use current limitign resistor e.g 500ohm
* go to [Tinkercad](tinkercad.com) to simulate the breadboard before wiring

## Section 28: GPIO pin Interrupt configuration

### Lecture 109. GPIO pin Interrupt configuration coding : Part 1

* we will implement init of 3 interrupt modes.. Fallinng Edge , Rising Edge and Both
* The Pins Interrupt Delivery to CPU in STM32F$ is
	* GPIO port is decided by SYSCFG_EXTICR register configuration (GPIOx_PIN0 ... GPIOx_PIN10_15)
	* upt to pin 4 there is dedicated bit config reg 4-9 share 1 bit also 10-15
	EXTI block does edge detection (FT,RT), enable and disable of interrupt delivery top proc
	* EXTTI0,EXTI1,EXTI2,EXTI3,EXTI4,EXTI5-9,EXTI!)-15 go to NVIC on their own IRQ num
	* NVIC does Enable/Disable of IRQs with NVIC regs
* GPIO Pin Interrupt Configuration
	* Pin must be in input configuration
	* Configure the edge trigger (RT,FT,RFT)
	* Eanble interrupt delivery from peripheral to processor (on peripheral side)
	* Identify the IRQ number on which the processor accepts the interrupt from that pin
	* Configure the IRQ priority for the identified IRQ number (Proc side)
	* Enable interrupt reception on that IRQ number (Proc side)
	* Implement IRQ handler

### Lecture 110. GPIO pin Interrupt configuration coding : Part 2

* we need to define the EXTI peripheral definition as stuct
```
/*
 * peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;    /*!< Give a short description,          	  	    Address offset: 0x00 */
	__vo uint32_t EMR;    /*!< TODO,                						Address offset: 0x04 */
	__vo uint32_t RTSR;   /*!< TODO,  									     Address offset: 0x08 */
	__vo uint32_t FTSR;   /*!< TODO, 										Address offset: 0x0C */
	__vo uint32_t SWIER;  /*!< TODO,  									   Address offset: 0x10 */
	__vo uint32_t PR;     /*!< TODO,                   					   Address offset: 0x14 */

}EXTI_RegDef_t;
```

* base addr we have already set
* we config edge trigger `#define EXTI		((EXTI_RegDef_t*) EXTI_BASEADDR)`
* to set SYSCFG registers we need the structure like EXTI
```
/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;       /*!< Give a short description,                    Address offset: 0x00      */
	__vo uint32_t PMC;          /*!< TODO,     									  Address offset: 0x04      */
	__vo uint32_t EXTICR[4];    /*!< TODO , 									  Address offset: 0x08-0x14 */
	uint32_t      RESERVED1[2];  /*!< TODO          							  Reserved, 0x18-0x1C    	*/
	__vo uint32_t CMPCR;        /*!< TODO         								  Address offset: 0x20      */
	uint32_t      RESERVED2[2];  /*!<                                             Reserved, 0x24-0x28 	    */
	__vo uint32_t CFGR;         /*!< TODO                                         Address offset: 0x2C   	*/
} SYSCFG_RegDef_t;
```

* base address we have we add a pointer macro `#define SYSCFG		((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)`

### Lecture 112. GPIO pin Interrupt configuration coding : Part 4

* we see that there are 4 registers . with 4 bits  per pin it gives up to 15 ports to enable
* so we can use at any moment an interupt on a pin number of a given port. we cannot fire interapt from the same pin num on another port (for GPIO peripherals)
	* 0000 for PortA -> 0111 for port H (in STM32F446)
* so we need the pin num and port base address to now port and pin to set the correct bits
* we will play with div and modulo
* we will add a macro to get the port bit mask based on portnum
```
#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA) ? 0 :\
									 (x == GPIOB) ? 1 :\
									 (x == GPIOC) ? 2 :\
									 (x == GPIOD) ? 3 :\
									 (x == GPIOE) ? 4 :\
									 (x == GPIOF) ? 5 :\
									 (x == GPIOG) ? 6 :\
									 (x == GPIOH) ? 7)
```
* macro is interesting as it takes argument and also it makes heavy use of ternary conditions
* we also need to enable syscfg clock
* and enable IMR bit
```
		// 2. config the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);
		// 3. enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
```

* We add macros for the IRQnum of the EXTI lines based on the ref manual
```
#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
```

### Lecture 113. GPIO pin Interrupt configuration coding : Part 5

* we flesh out the GPIO_IRQConfig method
* in Cortex M4 user guide we look for NVIC registers
* EXTI lines meet the NVIC so we need to program the NVIC
* 'NVIC_ISER' is used to enable(set) an interrupt on a specific IRQnum
* 'NVIC_ICER' is used to disable(clear) an interrupt on a specific IRQnum
* 'NVIC_IPR' is used to set the priority
* NVIC_ISER are 8 registers each 32bits that can used for 32 IRQnums 
* so again we need division by 32 and modulo to find the reg and bit to set
* same holds for ICER
* again we need a struct for NVIC regs
* he changes philosophy not using arrays of regs.......... and using if/then
```
if(EnorDi == ENABLE) {
		if (IRQNumber <= 31) {
			// program ISER0
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber <= 63 ) {
			// program ISER1
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		} else if (IRQNumber >=664 && IRQNumber < 96 ) {
			// program ISER2
			*NVIC_ISER2 |= (1 << IRQNumber % 64);
		}
	} else {
		if (IRQNumber <= 31) {
			// program ICER0
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber <= 63 ) {
			// program ICER1
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		} else if (IRQNumber >=664 && IRQNumber < 96 ) {
			// program ICER2
			*NVIC_ISER2 |= (1 << IRQNumber % 64);
		}
	}
```

### Lecture 114. GPIO pin Interrupt configuration coding : Part 6

* NVIC_IPR registers are 60 in number 0-59 and have four 8 bit masks for the priority of four IRQnumbers 4x60 = 240 IRQnums according to Cortex M4 Spec
* to find the reg we need to divide the IRQnum by 4 (shift 4 bit)
	* the 8 bit mask is the IRQpriority num on the upper 4 bits , lower 4 bits are not used
* we add a new API for priority config `void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)`
* we add a macro for priority base address `#define NVIC_PR_BASE_ADDR		((__vo uint32_t*)0xE000E400)`
```
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority){
	//1. first lets find out the ipr register
	uint8_t	iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= ( IRQPriority << shift_amount);
}
```

### Lecture 115. GPIO pin Interrupt configuration coding : Part 7

* we now the path and logic of interrupt from pin to CPU
	* there are 2 pending registers 1 in EXTI and 1 in NVIC (latches)
* the NVIC uses the vector table for the address that stores the handlers address depending on the IRQnum
* it is our responsibility to keep the ISR address in the vector tables reserved address for the IRQnum
* NVIC clears its pending bit and executes the ISR (if CPU is in Handler mode... priority takes effect and we get preemption or override)
* We need to
	* implement the ISR (ISR are application specific)
	* store its address in Vector table
* startup code in assembly contains default implemenmtation of ISRs, we simply use the same name in our app
* our responsibility is to clear the PR bit in EXIT reg (pending reuest) in the ISR
```
void GPIO_IRQHandling(uint8_t PinNumber){
	//clear the exti pr register corresponding to pinnum
	if(EXTI->PR & (1 << PinNumber)){
		// clear by setting the bit
		EXTI->PR |= (1 << PinNumber);
	}
}
```

## Section 29: Exercise : GPIO interrupts

### Lecture 116. Exercise : External button interrupt implementation

* Exercise: connect an external  button to pin and toggle the LED whenever interrup is triggered by the button press
* interrupt should be triggered durring falling edge of button press
* we will play with onboard LED (PA5) and PUSHbutton (PC13) because we are bored to use the breadboard
* we add '003button_interrupt.c' and cp  the 002 source file which we exclude from build
* in button config we set mode to `GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;`
* we do the IRQ config
* we add priority macros...just because...
```
GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);
```
* we implement the ISR (get name from startup assembly file in vector table)
```
void EXTI15_10_IRQHandler(){
	GPIO_IRQHandling(GPIO_PIN_NO_13);
	GPIO_TogglePin(GPIOA, GPIO_PIN_NO_5);
}
```
* we dont need debouncing
* to debug the registers we need to suspend the execution to see the current vals and see if values are what it should be
* the tutor gets corruption in register addresses
* this is because his led and bts are on same port and in Btn config he leaves a reg uninitialized (output type) so garbase from the local val went in the reg
* a good trick is to initialize all structs to 0 before starting filling them
* we use memset
```
	GPIO_Handle_t GpioLed, GpioBtn;
	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GpioBtn,0,sizeof(GpioBtn));
```

### Lecture 117. Exercise : Debugging the application : Part 1

* always buy a board with onboard debugger to start with
* check registers of MCU with program in standby
* be careful of variables and function arguments size especially when we apply bitshifting

### Lecture 118. Exercise : Debugging the application : Part 2

* the code below has a tricky bug `*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= ( IRQPriority << shift_amount);`
* NVIC_PR_BASE_ADDR macro is defined as uint32_t pointer sto when we increment it by 1 we actual move the pointer 4 bytes
* se we need to remove the 4 as it is not needed and corrupts memspace

## Section 30: SPI introduction and bus details

### Lecture 119. Introduction to SPI Bus

* Serial Peripheral Interphace (SPI Bus)
	* 1 master / 1-n slaves
* SPI Bus signals
	* SCLK 'Serial Clock' Master:SCLK => Slave:SCLK
	* MOSI 'Master Out Slave In' Master:MOSI => Slave:MOSI
	* MISO 'Master In Slave out' Slave:MISO => Master:MISO
	* 'Slave Control' Master:gpio1 => Slave:ss
* SPI Slaves: sensor,EEPROM,SDCARD,Display,Blowetooth ....
* 4 I/O pins are dedicated to SPI communication with external devices
	* MISO: Master In / Slave Out data. In the general case, this pin is used to transmit data in slave mode and receive data in master mode
	* MOSI: Master Out / Slave In data. In the general case, this pin is used to transmit data in master mode and receive data in slave mode 
	* SCK: Serial Clock output pin ifor SPI master and input pin for SPI slaves
	* NSS: Slave select pin. Depending on the SPI and NSS settings, this pin can be used to select an individual slave device for communication
* Data on MOSI/MISO data lines are synchornized with the SCK data comm clock
* SPI is synchronous comm where MAster produces the clock
* NSS is useful only when multiple slaves are hanging on the SPI bus

### Lecture 120. SPI comparison with other protocols

* SPI runs at fPCLK/2 speed
* I2C speed is 3.4Mbps in high speed mode
* SPI, I2C these peripherals communicate over TTL signaling in the voltage range 0V to 5V where as protocols like CAN,RS485 communicate over differential signaling (>10V). This is the reson why SPI and I2C are short distance comm interfaces

### Lecture 121. Importance of SPI slave select pin

* When a master wants to talk to a slave it uses one of its GPIOs to pull the slaves SS (SlaveSelect) pin to the ground
* Slave's communication lines (MOSI/MISO) will be to High-Z state or high impedance state unless SS line is pulled to GND
* when SS is pulled to Ground data lines can be used and a Clock must be sent along with the data

### Lecture 122. SPI Minimum bus configuration

* Minimal SPI Bus: SPI Bus allows the communication between one master device and one or more slave devices. In some applications SPI bus may consist of just two wires, one for the clock signal and the other for synchronous data transfer. Other signals can be added depending on the data exchange between SPI nodes and their slave select signal management

### Lecture 123. SPI behind the scene data communication principle

* On the MCU SPI peripheral we will find Shift Registers (typicaly just 1)
	* assume an 8 bit shift register on Master (MCU)
	* Slave Device also has an 8bit Shift Register
	* MISO,MOSI,SCLK are connected
* The goal is to send the 8 bit data from Master Shift Register (A) to Slave Shift Register (B) and the other way around
	* A shift reg  (master) has A8,A7,A6,A5,A4,A3,A2,A1,A0 bits loaded
	* B shift reg (slave) has B8,B7,B6,B5,B4,B3,B2,B1,B0 bits loaded
* in each SCLK clock cycle 1 bit (A0) is moved from Areg to Breg through MOSI, there is 1 step right bit shifting on the registers and 1 bit (B0) from Breg to Areg through MISO. also the new bit is added at 7th bit position that empty due to bit shifting (and transmition of the 0th bit along MOSI and MISO lines) After 1 clock cycle the register content will be:
	* A shift reg  (master) has B0,A7,A6,A5,A4,A3,A2,A1 bits loaded
	* B shift reg (slave) has A0,B7,B6,B5,B4,B3,B2,B1 bits loaded
* after 8 clock cycles we will have hte whole registers exchanged
* the reception of data along MISO is done Automaticly if MISO is connected
* SPI SHift regs are up to 16bit long

## Section 31: SPI bus configuration and functional block diagram

### Lecture 124. SPI bus configuration discussion : full duplex, half duplex and simplex

* The SPI allows the MCU to communicate using different configurations, depnding on the device targeted and application requirements
* FULL DUPLEX COMMUNICATION (default config)
	* in this config the shift regs on Master and Slave are linked using unidirectional lines between the MOSI and MISO pins
	* During SPI comm, data is shifted synchronously on the SCK clock edges provided by the master.
	* The master transmits the data to be sent to the slave via the MOSI line and receives data from the slave via the MISO line
* HALF-DUPLEX COMMUNICATION
	* in this config one single  cross connection line is used to link the shift registers of the master and slave together
	* during this communication, the data is synchronously shifted between the shift registers on the SCK clock edge in the transfer direction selected reciprocally by both master and slave
	* MOSI from slave is connected to MISO of slave with a resistance (1KOhn) in between
	* internally in devices MOSI line (in Master) and MISO line (in slave) are connected on both ends of the shift reg. when the device is in Rx mode it LSB res side is in Input state and the MSB side is in High-Z state. when it is in Tx mode MSB side is in Output state
	* NSS is optional... HALF DUPLEX is usually for 1-1 comm
	* Master and Slave alternate Tx and Rx roles (programmaticaly configured how to decide when to swap roles)
	* Th unused pins can be used as GPIOs
* SIMPLEX COMMUNICATION (one directional)
	* Master Tx only mode, Slave Rx only mode
	* the config settings are the same as for full-duplex. the application has to ignore the information captured on the unused pin. 
	* this pin can be used as standard GPIO
	* Only SCK and MOSI used

### Lecture 125. SPI functional block diagram explanation

* we consult STM32F4 ref manual
* we see the SPI block diagram @SPI functional desc
* we see that the Shift register stands between an Rx Buffer and a TX buffer
	* MCU address/data bus (APB) writes to Tx Buffer for transmition over SPI
	* MCU address/data bus (APB) reades from Rx Buffer data received from SPI  
* Buffers get/put to Shift Reg complete 8/16bit frames for the SPI datalines
* Data are read/write from buffers to Shift Reg when its not busy receiving/transmitting
* When Shift Reg becomes free we get an interrupt
* SPI interrupts
	* Tx buffer empty (we can send a frame)
	* RX buffer full (we can read a frame)
* IF MCU is Master it has control logic for clock
* SPI Data Block and Clock Control Block have their Registers in MemSpace

## Section 32: STM32 NSS pin settings and management

### Lecture 126. NSS settings in STM32 master and slave modes

* Slave Select (NSS) Pin management
	* When a device is in slave mode: the NSS works as a standard chip select input and lets the slave communicate with the master
	* When a device is master: NSS can be used either as output or input. As an input it can prevent multi-master bus collision, as an output it can drive a slave select signal of a single slave

### Lecture 127. STM32 SPI hardware and software slave managements

* 2 types of slave management in STM32
	* HW slave NSS management (SSM = 0 in SPIxCR1 reg): NSS pin must be pulled low to enable slave to comm with master. In Master Node if SSM = 0, NSS pin must be in output mode (NSS is managed by HW)
	* SW slave NSS management (SSM = 1 in SPIx_CR1 reg): in this config, slave select info is driven internally by the the SSI bit val in SPIx_CR1 reg. SSI bit = 0 is like grounding NSS pin. The external NSS pin is free for other uses
* In Multi Slave Bus Config we cannot use SW slave management. in that case NSS pin of Master if enabled must be connected to VDD to avoid accidentally going LOW

## Section 33: SPI CPOL and CPHA discussion

### Lecture 128. SPI CPOL and CPHA discussion

* SPI Communication Format depends on 3 main parameters:
	* Serial Clock (SCLK) Phase (CPHA)
	* Serial Clock (SCLK) Polarity (CPOL)
	* Data Frame Format (DFF) (16bit or 8bit)
* During SPI Comm, receive and transmit operations are perfomred simultaneously
* The Serial Clock (SCK) synchronizes the shifting and sampling of the information on the data lines
* The communication format depends on the clock phase, the clock polarity and the data frame format. To be able to comm together, master and slaves must follow the same comm format
* CPOL (Clock Polarity)
	* the CPOL (clock polarity) bit controls the idle state value of the clock when no data is being transferred
	* if CPOL is reset, the SCLK pin has a low-level idle state. If CPOL is set, the SCLK pin has a high level idle state
* CPHA (Clock Phase)
	* CPHA controls at which clock edge of the SCLK (1st or 2nd) the data should be sampled by the slave
	* The combination of CPOL (clock polarity) and CPHA (clock phase) bits selects the data capture clock edge
 * If CPHA=1 Master Places data after 1st falling edge for CPOL=1, rising for CPOL=0, Slave picks the data after 1st rising edge for CPOL=1, falling edge for CPOL=0. sampling repeats every cycle
 * If CPHA=0 Master Places data in SCLK idle state, Slave picks the data after 1st falling edge for CPOL=1, rising edge for CPOL=0. sampling repeats every cycle
 * The above is for MOSI line, the reverse holds for MISO line
 * SPI Modes
 	* 0: CPOL=0, CPHA=0
 	* 1: CPOL=0, CPHA=1
 	* 2: CPOL=1, CPHA=0
 	* 3: CPOL=1, CPHA=1
 * If CPHASE=1 data will be sampled on the trailing edge of the clock
 * If CPHASE=0 data will be sampled on the leading edge of the clock

### Lecture 129. SPI CPOL and CPHA waveform example

* We use Logic Analyzer to see an actual SPI in action. without connecting to hardware we click on Start Simulation.
* after collecting data we go to Analyzers => + => SPI and select channels
	* MOSI = channel1
	* MISO = channel2
	* Clock = channel0
	* Enable = channel3
* leave the rest to Standard or experiment with CPOL,CPHA etc
* click Save
* click Start Simulation
* we analyze the signals and see the protocol in action....
* in Analyze=>SPI => settings we can choose the decoding
* we see that data is on the line for capture during the edges that sampling is confgigured to happen
* comm is very fast so traces must be short

## Section 34: SPI serial clock discussion

### Lecture 130. SPI peripherals of your Microcontroller

* SPI peripherals are connected to ARM Cortex M4 via the APB peripheral. in STM32F446
	* SPI1 and 4 go via APB2
	* SPI2 and 3 go via APB1 

### 131. SPI Serial clock frequency

* The Max speed of SPIx SCLK is APBx/2
* SPIx uses a clock prescalar dividing the APBx clock freq. the min value is 2
* IF we use internal STM32F446 RC oscillator. SPI Clock Max speed is 8Mhz

## Section 35: SPI Driver : API requirements and configuration structure

### 132. SPI API requirements and configuration items

* we will add 2 more files in /drivers folder the 'stm32f446xx_spi_driver.c' and 'stm32f446xx_spi_driver.h'
* we will also add SPI specific content to device header 'stm32f446xx.h'
* our driver should offer the following APIs (methods)
	* SPI Initialization / peripheral clock control
	* SPI TX
	* SPI RX
	* SPI Interrupt config and handling
	* Other SPI management APIs
* SPI Configuration Items
	* SPI_DeviceMode (master, slave)
	* SPI_BusConfig (Comm mode, FULL DUPLEX etc)
	* SPI_DFF (8bit or 16bit)
	* SPI_CPHA (0 or 1)
	* SPI_CPOL (0 or 1)
	* SPI_SSM (SW or HW)
	* SPI_Speed
* Exercise:
	* Complete SPI register definition structure and other macros (peripheral base addresses, Device definitions, clock en, clock di, etc) in MCU specific header file
	* Complete SPI Configuration structure and SPI handle structure in SPI header file

### 133. updating MCU specific header file with SPI related details

* baseaddresses we have them already from memory map in Ref Manual
* we add the Registers struct
```
/*
 * peripheral register definition sctructure for SPI
 */
typedef struct
{
	__vo uint32_t CR1;        /*!< SPI control register 1, Address offset: 0x00 */
	__vo uint32_t CR2;        /*!< SPI control register 2, Address offset: 0x04 */
	__vo uint32_t SR;         /*!< SPI status register, Address offset: 0x08 */
	__vo uint32_t DR;         /*!< SPI data register, Address offset: 0x0C */
	__vo uint32_t CRCPR;      /*!< SPI CRC polynomial register, Address offset: 0x10 */
	__vo uint32_t RXCRCR;     /*!< SPI RX CRC register, Address offset: 0x14 */
	__vo uint32_t TXCRCR;     /*!< SPI TX CRC register, Address offset: 0x18 */
	__vo uint32_t I2SCFGR;    /*!< SPI_I2S configuration register, Address offset: 0x1C */
	__vo uint32_t I2SPR;      /*!< SPI_I2S prescaler register, Address offset: 0x20 */
} SPI_RegDef_t;
```
* we add the config and handler structs
```
/*
 * Configuration structure for SPIx peripheral
 */
typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 */
typedef struct {
	SPI_RegDef_t	*pSPIx;		/*!< This holds the base address of SPIx(x:1,2,3,4) peripheral>*/
	SPI_Config_t	SPIConfig;
}SPI_Handle_t;
```
* and the Periph pointer macros `#define SPI1 ((SPI_RegDef_t*) SPI1_BASEADDR)`

### 134. SPI adding API prototypes to driver header file

* we cp the pripheral clock control and init/deinit prototypes from GPIO to SPI and adapt them
* for Data Read and Write we consider the following
* in Peripheral Comms we have 3 types of Tx/Rx methods
	* Polling based (blocking type)
	* Interrupt based (non-blocking type)
	* DMA based
* our blocking calls are
```
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Length);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t Length);
```
* length should always be 32bit or more
* we cp the GPIO IRQ config and Handlers and mod them
* in handler we pass the handle struct pointer `void SPI_IRQHandling(SPI_Handle_t *pHandle);`

## Section 36: SPI Driver API Implementation : Clock control

### 135. Implementation of SPI peripheral clock control API

* we add the stubs in C for all APIs so far.
* code for periph clock enable is the same like GPIO we cp and mod it
* code for deinit is also same for SPI like for GPIO we cp it and the macros making the changes necessary

## Section 37: SPI Driver API Implementation : SPI init

### Lecture 136. SPI user configuration options writing and register bit definition macros

* we need to config ALOT of SPI regs
* Control Regs config the behaviour of the SPI
* Status Reg has the Status Flags og the buffer + error flags
* we define macros for config options
* for mode we have master and slave
* for bus config we have full duplex, half duplex, simplex tx only, simplex rx only
* for clockspeed we see the VR1 baudrate bits to see the options. they are the clock dividers (prescalars) 2,4,8,16,21,64,128,256
* we add macros for DFF,CPOL,CPHA,SSM with the defaults to 0.
* we check the regs for the relevant bitfields to set/clear like we did in GPIO
* periph by default is in slave mode and not producing SCLK
* for master/slave we set bit MSTR of CR1
* for bidirectional mode the bits BIDIOE and BIDIMODE of CR1
	* 2 line unidierectional is Full Duplex
	* 1 line bidir is HALF DUPLEX
	* in HALF DUPLEX BIDIOE is used to change programmatically to receive only and transmit only mode
* SIMplex RX is configured if we set RXONLY while BIDIMODE is 0
* To make Full DUplex setup Simplex TX only we dont connect the MISO line from master to miso line of Slave
* To make SImplex Rx only we cannot just disconnect MOSI in full duplex mode. Even in RX mode Master has to provide the clock. Master in order to produce clock data must be on the MOSI line so we cannot just disconnect it. thats why RXONLY reg bit is needed. so Periph will output clock despite data being absent from MOSI

### Lecture 137. Implementation of SPI init API : Part 1

* SPI_Init is all about translating the Config struct options into bit set/clear in the relevant bitfields of the Control Registers
* we use macros for the bit positions

### Lecture 138. Implementation of SPI init API : Part 2

* we do the same for CR2 and SR adding macros in device header for bitfield positions

## Section 38: SPI Driver API Implementation : Send Data

### Lecture 139. Implementation of SPI send data API : Part 1

* we will implement a low performance blocking call where the function called will wait till all the bytes for transmission are sent
* the logic is checking data sent compared to the length passed if there are more data they are placed to DR reg for transmission ONLY if Tx buffer is empty
* in Status Reg TXE bit (1) tells us if Tx buffer is free when is is SET
* TX buffer and RX buffer are peripheral internal. we access them with the DR (Data Register)
* to read from DR we check the bit RXNE in SR if it is SET. this meand there are data for us waiting from RX Buff

### Lecture 140. Implementation of SPI send data API : Part 2

* we implement the method
* we check TXE flag and wait. 
* we add a helper method to check bit flags where we pass the periph handle pointer and the FLAG as a macro
```
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName){
	if(pSPIx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
```
* this can work for all SR flags

### Lecture 141. Implementation of SPI send data API : Part 3

* the full blocking call
```
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){
	while(Len > 0){
		//1. wait till TXE is SET
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);
		//2. check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)){
			//16bit DFF
			//1. load the data into the DR reg
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len = Len -2;
			(uint16_t*)pTxBuffer++;
		} else {
			//8bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}
```

* it is possible that while loop will hang forever. t hats why we need to avoid using this method
* we add a empty main in a new source file and build

## Section 39: Exercise : SPI Send Data

### Lecture 143. Exercise to test SPI Send Data API

* Test the SPI_SendData API to send the string "Hello World" and use the below config
	* SPI-2 Master mode
	* SCLK=max possible
	* DFF=0 and DFF=1
* we will not use a slave so we need sclk and mosi
* so we will config 2 pins but we need to decide on which pins to use

### Lecture 144. Finding out microcontroller pins to communicate over SPI2

* finding PINs is Board Specific
* for NUCLEO-F446RE we will use SPI2 on:
	* PB12 as SPI2-NSS (AF5)
	* PC02 as SPI2-MISO (AF5)
	* PC03 as SPI2-MOSI (AF5)
	* PB10 as SPI2-SCLK (AF5)

### Lecture 145. Exercise : Code implementation : Part 1

* first we will configure the pins for SPI
* we use GPIO_Init() and 2 handle structs as we use 2 ports B and C (in )

### Lecture 146. Exercise : Code implementation : Part 2

* we implement SPI initialization
```
	SPI_Handle_t	SPI2handle;
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; //generates SCLK of 8MHz
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN; //SW slave management enabled for NSS pin
	SPI_Init(&SPI2handle);
```
* we add peripheralclock enable in Init methods of drivers
* we send some date with blocking call
* we implement a small buffer (string) and send data
```
char user_data[] = "Hello world";
SPI_SendData(SPI2, (uint8_t *)user_data, strlen(user_data));
```

### Lecture 147. Exercise : Code implementation : Part 3

* we have enabled SPI periph clock and all the config but SPI is yet not enabled
* we need to SET the SPE bit (6) in SPI CR1
* we need to configure the SPI regs and before sending data enable it
* when we enable SPE periph is busy with comm we cannot configure it anymore
* we add an API just for that

### Lecture 148. Exercise : Testing

* we connect Logic Analyzer 
* we dive with 16 to lower clock to 1mhz so that our L.A can capture it
* we flash . prepare the capture reset and let it capture the comm
* we set to capture 4 secs but see no data. spi is not working
* we fire up debugger and put breakpoint after initialization and check the SPI2 registers which look good.
	* our spi is master
	* ssm is 1
	* baudrate is ok
	* cr2 is 0
	* txe is set in SR
* we check RCC. looks good
* we enable the SPI and recheck the registers
* we see that SPI2 is no longer master MSTR in CR1 is 0
* this should not happen. unless SPI2 is disabled SPE=0 it should be master. we see that SPE is also 0
* we see that in SR MODF bit is set... an error occured so SPE and MSTR is cleared
* we look in the manual and see it has to do with NSS and SSi bit in NSS is in SW mode. we need to set SSI bit 1
* we add a method 'SPI_SSIConfig' where we set or reset the bit and add it as API while we should put it in Init 
* it works
* we need to disable the peripheral (SPE of CR1=0) after finishing using it to avoid errors
* it seems MCU is picking up noise in MISO as RX buffer is getting data and creating overrun

## Section 40: Exercise : STM32 master and Arduino Slave communication

### Lecture 149. Exercise : Communicating with Arduino slave

* when the button on the master is pressed, master should send string of data to the arcuino slave connected. the data received by the arduino will be displayed on the arduino serial port
	* Use SPI full duplex mode
	* ST board will be SPI master and Arduino will be configured SPI slave
	* use DFF =0
	use HW Slave management (SSM=0)
	* SCLK speed 2MHz fclk = 16MHz
* master wont recieve anything from slave so we may not configure MISO
* slave does not know how many bytes of data master is going to send. 
* master first sends number of bytes info which slave is going to receive next
* STboard will connct to Arduino board and this to PC (serial Monitor)
* connect their grounds (ST to Arduino)
* connect Logic Anlyzer
* we need to power up arduino and dowload slave sketch '001SPISlaveRxString.ino'

### Lecture 150. Exercise : Coding Part 1

* copy everything from main file 004spi_tx_testing.c to 005spi_txonly_arduino.c
* we uncomment NSS pin config code
* we put the SPEED_DIV to 8 (2MHz)
* SPI_SSM to Disable (HW control)
* Also Comment the SSI Config calls and test
* slave wont respond unless NSS is pulled to low
* when SSM = 0 in ST whenever we set SPE to 1, NSS will be pulled to low automatically. when we put SPE to 0 NSS is pulled to high
* to enable NSS output there is SSOE. we need to set it 1 so that NSS pin will be controlled by SPE. SSOE=0 is used for multimaster comm
* we add 1 more api call to control SSOE

### Lecture 151. Exercise : Coding Part 2

* we add button code from previous project to trigger transmission on User Button Press
* we wait till button is pressed and then send. this is in a while loop so we can send multiple times
* disabling the SPI immediatly after sending last byte might create problems. we should wait till its received successfuly
* we need to make sure comm is not busy before disabling the peripheral

### Lecture 152. Exercise : Coding Part 3

* we need to check the busy flag in status reg before disabling the SPI `while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));`
* also we need to send the length of the string to the slave to know when to close its peripheral
```
	uint8_t dataLen = strlen(user_data);
	SPI_SendData(SPI2,&dataLen,1);
```

### Lecture 153. Exercise : Testing

* It WORKS!!!
* in his example its not working so its setting PIN intrnal Pull UP in GPIO
* in our case we use no pullups and it works. we leave it as is
* as we drive a slave its good to use internal pull ups. we confirm it works better and its more stable to noise

## Section 41: SPI Driver API : Receive data

### Lecture 154. Implementation of SPI data receive API : Part 1

* receive data is a similar process like transmit data
* we will first do it in a blocking call
	* we check if LEn==0 if yes exit
	* if Len > 0 wait till Rx Buffer is non empty RXNE flag in SPI SR
	* if RXNE=1 depending on DFF we read from DR 1 or 2 bytes of data and increment our rx buffer address. 
	* we decrease the Len 1 or 2 and repeat
* we need to get the length of transmitted data as first data to set Len

### Lecture 155. Implementation of SPI data receive API : Part 2

* we ll implement the ReceiveData function
* we cp the code from SendData to mod it.
* its almost the same. we read instead of write to DR and instead of TXE we check RXNE

## Section 42: Exercise : SPI receive data

### Lecture 156. Exercise : SPI command and response based communication

* SPI Master (STM) and SPI Slave(Arduino) command & response based communication
* When the button on the master is pressed, master sends a command to the slave and slave responds as per the command implementation
	* Use the SPI Full Duplex mode
	* ST board will be in SPI master mode and Arduino will be configued for SPI slave mode
	* Use DFF = 0
	* Use HW slave management (SSM=0)
	* SCLK speed = 2MHz, fclk = 16MHz
* we need to connect MISO and also probe it it Logic Analyzer
* we need to upload sketch '002SPISlaveCmdHandling.ino' in Arduino UNO
* The protocol we will implement is
	* Master => Slave : Master Sends a Command
	* Slave => Master : Slave responds with NACK (0xA5) or ACK (0xF5) (1byte)
	* if NACK: Master Display Error Msg
	* if ACK: Master => Slave: Master sends 1 or more command Arguments
	* Slave takes action according to command
	* Slave => Master: Slave response for data read command
* Command Formats: <command_code(1)> <arg1> <arg2>
1. CMD_LED_CTRL <pin no(1)> <value(1)>
	* <pin no> : digital pin number of the arduino board (0to9) 1byte
	* <value> : 1 = ON, 0 = OFF (1 byte)
	* Slave Action : control the digital pin ON or OFF
	* Slave returns : nothing
2. CMD_SENSOR_READ <analog pin number(1)>
	* <analog pin number> : analog pin number of the Arduino board (A0 to A5) (1 byte)
	* Slave action : slave should read the analog value of the supplied pin
	* Slave returns : 1 byte of analog read value 
3. CMD_LED_READ <pin no(1)>
	* <pin no> : digital pin number of the arduino board (0to9) 1byte
	* Slave Action : Read the status of the supplied pin number
	* Slave returns : 1 byte of led status. 1 = ON, 0 = OFF
4. CMD_PRINT <len(2)> <message(len)>
	* <len> : 1 byte of length infformation of the message to follow
	* <message> : message of 'len' bytes
	* Slave Action : Receive the message and display via serial port
	* Slave returns : Nothing
5. CMD_ID_READ
	* Slave Returns : 10 bytes of board id string
* connect to pin num 9 of Arduino a 470ohm resistor => A LED => Ground
* Application flow chart:
* Start
* Enter main()
* All inits
* While loop
	* Wait till button is pressed
	* execute CMD_LED_CTRL 
	* Wait till button is pressed
	* execute CMD_SENSOR_READ
	* Wait till button is pressed
	* Execute CMD_LED_READ
	* Wait till button is pressed
	* Execute CMD_PRINT
	* Wait till button is pressed
	* Execute CMD_ID_READ

### Lecture 157. Exercise : Coding Part 1

* create a new source file for main
* cp code from prev exercise
* uncomment MISO
* we add macros for protocol commands
* in main we start implementing the flow
* we sent the first command
```
		uint8_t commndcode = COMMAND_LED_CTRL;
		SPI_SendData(SPI2,&commndcode,1);
```
* slave receives the command and loads its SPI Shift reg with ACK or NACK.
* SPI slave wont initiate data transmision. as we have seen we have to send 1 byte to get 1 byte back. as MOSI and MISO work in ame time on both shift regs...
* so we will send a dymmy byte to get the loaded data back from slave
* the size of dummy data depends on the size of reg (DFF) `SPI_SendData(SPI2,&dummy_byte,1);`
* then we read back the response
* In SPI comm when Master or Slave sends 1 byte it also receives 1 byte (the content of the other device's shift reg)
* we need dummy reads to make sure that when we will need to get meaning full data we will have them.
* this will clear the RXNE flag to signal that data is read
* we test the code and it works!! (trace is OK, we need to reset the boards and use USER Button)
```
		//1. CMD_LED_CTRL <pin no(1)> <value(1)>
		commndcode = COMMAND_LED_CTRL;
		SPI_SendData(SPI2,&commndcode,1);
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
```

### Lecture 158. Exercise : Coding Part 2

* we will implement the second command in the same fashion
* to test analog read we connect A0 to GND to read 0 and then we connect A0 to 5V to read 255, if we connect to 3.3V we read a value between 0 to 255
* we can also use printf using trace on stm to print the value of analog using the ITM unit (see embedded C course lecture for instructions)
* we see that we get back the pin number and not the analog value.
* we get back what we sent..
* this is because slave has no time to get the analog value and load it so that we can read it with SPI from master
* it has to do ADC conv.
* we add some delay and retry. IT F...... WORKS!!!

### Lecture 159. Exercise : Coding Part 3

* we add the rest of the commands its the same
* here the lecturer uses semihosting to printf trace on terminal while running the app on device in debug mode
* Using Semihosting
	* linker argument settings `-specs=rdimon.specs -lc -lrdimon`
	*  debug configuration of our app: monitor arm semihosting enable
	* in main.c use code below
```
extern void initialise_monitor_handles();
initialise_monitor_handles();
```
* also this needs OpenOCD
* we have clear instructions for this in embedded C course for the CubeIDE
* if we try to printf floats MCU will go to use the FPU. if we have not enable it it throws exception. we need to tell compiles to not use it and use SW instead (see embedded C course)

## Section 43: SPI interrupts

### Lecture 160. SPI peripheral interrupting the processor

* During SPI communication, interrupts can be generated by the following events:
	* Transmit Tx buffer ready to be loaded (TXE) : Enable Interrupt Control Bit TXEIE 
	* Data received in Rx buffer (RXNE) : Enable Interrupt Control bit RXNEIE
	* Master mode fault MODF (in single master case we must avoid this error happening), Overrun error (OVR), CRC Error (CRCERR), TI Frame Format Error (FRE) : Enable Interrupt Control Bit ERRIE

* Interrupts can be enabled and disabled separately
* SPI interrupts the rpocessor through a unique interrupt line per SPI peripheral that goes to NVIC. no EXTI etc
* we will compete SPI IRQnumber def macros in MCU device header
* we then have to implement the IRQ config API in SPI drivers c file

## Section 44: SPI interrupt mode APIs

### Lecture 161. SPI interrupt mode API implementation and changes to handle structure

* we will create 2 more APIs to send receive data using the interrupts 'SPI_SendDataIT' and 'SPI_ReceiveDataIT'
* we pass it the Handle struct where we ewill add more config options for Interrupts
* in send data with interrupts
	1. save the tx buffer address and Len information in some global variables
	2. Mark the SPI state as busy in transmission so that no other code can take over same SPI peripheral until transmission is over
	3. enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
	4. Data Transmission will be handled by the ISR code (will implement later)
* we need global vars as our code will switch from iSR to main and back
* first we have to create some place holder variables to save applications Tx address, len ans SPI state.
* we will add in hadle struct
	* Tx buffer address
	* Rx buffer address
	* Tx length
	* Rx length
	* Tx state
	* Rx state
* The possible SPI app states are
	* SPI_READY
	* SPI_BUSY_IN_TX
	* SPI_BUSY_IN_RX

### Lecture 162. SPI send data with interrupt API implementation

* we implement the SendIT method implementing the 3 of 4 steps described
```
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len){
	uint8_t	state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX){
		//1. save the tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		//2. Mark the SPI state as busy in transmission so that
		// no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		//3. enable the TXEIE control bit in CR2 to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;
}
```

### Lecture 163. SPI receive data with interrupt implementation

* we do the same for ReceiveIT (same code, just instead of Tx => Rx)

## Section 45: SPI Driver API : IRQ handling

### Lecture 164. SPI : Handling of interrupts

* we will implement the SPI IRQ handle
* the flow of actions in SPI IRQ handler:
* Enter ISR
* Check which event caused interrupt to trigger (Check the SPI SR)
	* interrupt is due to setting of RXNE flag: Handle RXNE event (receive)
	* interrupt is due to setting of TXE flag: Handle TXE event (transmit)
	* interrupt is due to setting of an ERROR flag: Handle error	
* Handling the TXE interrupt: (8bit or 16bit mode)
	* write 1 byte (8bit) or 2bytes (16bit) to SPI DR
	* Len-- (8bit) or Len -=2 (16bit)
	* Len == 0 ? Close the SPI TX : wait for next TXE interrupt

### Lecture 165. SPI IRQ handler implementation : Part 1

* we implement the handler where based on SR flags we invoke the specific handler
* we check for RXNE, TXE and OVR
* Overrun error happens when Master or Slave completes reception of next data frame while read operation of previous frame from RX buffer is not complete. data is NOT updated. so a read operation will retturn the previously received frame. al other transmitted data is lost
* to clear we need to read the DR and SR reg

### Lecture 166. SPI IRQ handler implementation : Part 2

* we add the prototypes of 2 specific ISRs in driver.c as we dont want to expose them
* we define them as static to forbid external calls (compiler error)
* we implement the TXE ISR handler code based on the activity flow we saw already
* we copy code from SendData blocking call as its the same the part when we load DR and change pointer and len
* also when Len==0 we end transmission by disablind the TXIE (TXE interrupt) and releasing the buffer (assign it to NULL)
* to use NULL keyword we `#include <stddef.h>`
* we also call a callback to inform the app
```
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)){
		//16bit DFF
		//1. load the data into the DR reg
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen -= 2;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	} else {
		//8bit DFF
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}
	if(!pSPIHandle->TxLen){
		// if TxLen = 0 close the SPI transmission and inform the app
		pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
		pSPIHandle->pTxBuffer = NULL;
		pSPIHandle->TxState = SPI_READY;
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}
}
```

* we implement the callback API method
* possible events for the callback defined as macros: tx compl, rx comp, ovr err, crc err
* we do the same for Receive (similar code)

### Lecture 167. SPI IRQ handler implementation : Part 3

* we implement OVR error interrupt handler
* we will read DR and SR to clear the flag and inform the app with the Callback
```
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle){
	uitn8_t temp;
	//1. clear the flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}
```
* the check is place to protect the app from a reset that will hinder transmission. in this case the callback gives the app responsibility to handle the error
* we add 3 more calls to the API
```
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pHandle);
void SPI_CloseReception(SPI_Handle_t *pHandle);
```
* in close Tx and Rx methods we just cp the code from ISR handlers when Len=0
* they are used for emergency aborting tx or rx
* so if app is busy transmitting and OVR happens app is notified with the callback. then app calls the ClearOVRflag where we just read from DR and SR to clear the flag
* when we dont assign a var and want to get rd of compiler warning we cast it to void `(void)temp;`
* we add a prototype for callback
* we add a weak implementation of the callback in driver.c file and let the app to implement it at will `__attribute__((weak))`
* we add a new main file '007spi_cmd_handling_it.c' and cp the code from last file
* we add global variables for SPI Comm (data passed between main and ISR)
```
SPI_Handle_t SPI2handle;
uint8_t RcvBuff[100];
uint8_t ReadByte;
uint8_t RxContFlag = RESET;
```
* in main the example is only for receving data.. it is not even sendind dummy data.... its crap but we get an idea how to do it if needed. Big Disapointment!!
* Be careful with Interrupt Priorities!!!!

## Section 46: Common problems in SPI

### Lecture 168. Common problems in SPI and Debugging Tips

* Troubleshooting SPI:
	* Master Mode bit must be enabled in the CR1 if we are configing SPI as Master
	* SPIpeiph enable bit must be enabled (SPE) last after all config
	* SPI peripheral clock must be enabled from RCC
* Problem1: master cannot produce clock and data
	* Reason1: non-proper config of GPIO I/O lines
	* Reason2: configuration override. Debug Tip: Dump out all the required register contents just before you begin the transmission
* Problem2: master is sending data but slave is not receiveing data
	* Reason1: not pulling down the slave select to ground before sending data to the slave. Debug Tip: probe with logic analyzer to confirm NSS going down during comm
	* Reason2: non-proper configuration of I/O lines for Alternate Functionality. Debug Tip: PRobe the GPIO AF register
	* Reason3: not enabling the peripheral IRQ num in the NVIC. Debug Tip: Probe the NVIC Irq Mask reg  to see whether the bit position corr to the IRQ num is set or not
* Problem 3: SPI interrupts do not trigger
	* Reason1: Not enabling the TXE or RXNE interrupt in the SPI configuration register. Debug Tip: Check the SPI config register to see TXEIE and RXNEIE bits are set to enable IRQ
	* Reason 2: not enabling the SPI IRQ in NVIC. Debug Tip: Probe NVIC REG to see if bits are set
* Problem 4: Master is producing right data but slave is receiving different data
	* Reason1: Using Long Wires in High Freq comm Debug Tip: use shorter wires, reduce the SPI serial freq to 500kHz and check

## Section 47: I2C introduction and I2C signals

### Lecture 169. I2C introduction and differences with SPI

* I2C: a protocol to achieve serial data communication between integrated circuits (ICs) which are very close to each other. A more serious protocol than SPI. Its design spec came from Philips Semoconductors in 1980s
* I2C details more complex than SPI
    * how data should be sent
    * how data should be received
    * how hand shaking should take place between sender/receiver, 
    * error handling 
* I2C spec is available from [NXP](https://www.nxp.com/docs/en/user-guide/UM10204.pdf) SPI has no universal spec
* Differences from SPI:
    * Multi-Master Capability: I2C is multimaster capable. SPI depends on MCU designers. STM SPI Peripherals can be used in MultiMaster config but arbitrations should be handled in SW
    * ACK: I2C HW automatically ACKs every byte received. SPI does not support any auto ACKing
    * Pins: I2C needs only 2 pins for comm. SPI needs 4 ane even more for multiple slaves. IN I2C Connect the 2 lines to VCC through resistors (Pull High) and connect devices
    * Addressing: In I2C master talks to slaves based on slave addresses. whereas in SPI a dedicated pin is used to select the slave
    * Communication Mode: I2C is HalfDuplex. SPI is Full Duplex
    * Speed: I2C Max Speed in 4MHz in ultra speed plus. For some STM MCUs max speed is only  400kHz. for SPI its Busclock/2 so can go very high
    * Slave's control over serial clock: In I2C slave can make master wait by holding the clock down if its busy, thanks to clock stretching feat of I2C. In SPI slave has no control over clock, programmers have to do tricks to overcome this situation
* Data rate (bitrate) is much less in I2C than SPI (for some STM32F4XX >50faster)
* I2C Terminology:
    * Transmitter: device sending data to bus
    * Receiver: device receiving data from bus
    * Master: device that initiates transfer, generates clock signals and terminates transfer
    * Slave device addressed by the master
    * Multi-master: >1 master can attempt to control the bus at the same time without corrupting the message
    * Arbitration: procedure to ensure tat, if >1 master at same time tries to control the bus, only one is allowed to do so and the winning message os not corrupted
    * Synchronization: procedure to sync the clock signals of 2 or more devices

### Lecture 170. I2C SDA and SCL signals

* Both SDA and SCL are bidirectional lines connected to a positive supply voltage via pull up resistors. when bus is free both lines are held HIGH
* The output stages of devices connected to the bus must have an open drain or open-collector configuration
* the bus capacitance limits the number of interfaces connected to the bus
* I2C pin configuration:
    * for proper functioning of the I2C bus, pull up resistor value has to be calculated according to the 12C formula. so in most cases we cannot use internal Pull up
    * chose Open Drain Config (only N-Mos)
* First Troubleshooting of I2C lines is to measure the voltage to ground when idle. (it should be VCC = 3.3V or 1.8V depending on the board)

## Section 48: I2C modes

### Lecture 171. I2C standard and fast mode

* 4 i2C Modes:
    * Standard Mode: Up to 100kbps - Suported by STM32F4x
    * Fast Mode: Up to 400kbps - Suported by STM32F4x
    * Fast Mode+ : Up to 1Mbps - Suported by some STM32F4x (see RM)
    * High Speed mode: Up to 3.4Mbps - Not Suported by STM32F4x

* we will use standard mode and fast mode
* Standard Mode
    * std mode devices are not forward compatible. cannot talk to fast mode devices
* Fast Mode
    * fast mode devices are downward compatible and can talk to std mode devices in a 0-100kbps i2C system. but in that case the bus is STD Mode

## Section 49: Understanding I2C Protocol

### Lecture 172. I2C Protocol explanation

* in I2C data transfer is always initiated by the master by producing the start condition in SDA
* after start condition the address phase follows (8bits)
    * 7 bits is the address of the slave
    * 8 bit is the R/W flag (1 is data request (READ), 0 is transsmission (WRITE)
* then an ACK bit  which is sent by the Slave to the Master after it matches the address with its own
* if its a write operations it is followed by 8 data bits (data phase) sent by the master to the slave
* then the ACK bit send by slave to master after it receives the data
* then master generates the stop condition if it wants to end comm (release the bus)
* Every byte put on SDA line must be 8 bits long
* ech byte must be followed by an Ack bit
* data is transferred with the MSB first
* start and stop condition is generated by a master which takes over the bus

### Lecture 173. I2C START and STOP conditions

* All transactions begin with a START (S) and terminate with a STOP (P)
* A HIGH to LOW transition on the SDA line while SCL is HIGH defines a START condition
* A LOW to HIGH transition on the SDA line while SCL is HIGH defines a START condition
* Once a master generates the Start condition it claims the bus. no other master can claim it. they have to wait till bus is released
* If the state of SDA is low at 9 clock cycle (after address pahase) it is considered an ACK. NACK isf its HIGH (default)
* If master wants to initiate a second Comm cycle it generates a new START condition (repeated Start) before issuing a STOP condition
* Most of the MCU's I2C perpherals support both master and slave mode. You need not to configure the mode because when the peripheral generates the start condition it automatically becomes the master and when it generates the stop condition it goes back to slave mode

### Lecture 174. I2C ACK and NACK

* 8 bit is R/W bit (8th clock cycle)
* in  9th clock cycle ACK/NAck is produced
* ACK signal: the transmitter releases the SDA line during the acknowledge clock pulse so that the receiver can pull the SDA line LOW and it remains stable LOW during the HIGH period of this SCL clock pulse 
* Ack takes place after each byte
* ack allows receiver that byte was successfully received so it may send next
* Master is the one generating the clock pulce
* IF SDA is HIGH during SCL 9th pulse HIGh state is a NACK. master can then generate STOP condition or repeated START condition

### Lecture 175. I2C Data validity

* The data on the SDA line must be stable during the HIGH period of the SCL clock cycle. The rising/falling edge of SDA is allowd only on the SCL LOW period of clock cycle. 
* One SCL cycle per data bit on SDA line
* This Does not hold for START/STOP condition

## Section 50: I2C master and slave communication

### Lecture 176. Example of master writing to slave

* Master Write
	* first master generates the start condition
	* sends the adress 7bit, the R/W byte (W=0) and gets an ACk from slave
	* then he can send consecutive bytes to slave which ACks them
	* then sends the stop condition
* Master write
	* first master generates the start condition
	* sends the adress 7bit, the R/W byte (R=1) and gets an ACk from slave
	* then slave sends consecutive bytes to master which ACks them
	* on last byte send by slave master NAcks
	* slave stops sending
	* then sends the stop condition

### Lecture 177. Understanding repeated START condition

* Repeated start without a stop: 
* Master reads the contents of EEPROM at address 0x45
	* master writes to EEPROM the address of memory to get contents Start => I2C Address of EPROM (W) => Data(Mem Address) => STOP
	* slave is ready to send the data
	* Master Reads the contents from slave: Start => I2C Address of EPROM (R) => Data(Content) => Stop
* This is bad as someone else can alter the EEPROM Content in between or ocupy the bus and introduce delay. He can instead of stop start in between use Repeated Start
* If the bus has 1 master it realy doesnt matter if we use repeated start at all

## Section 51: STM32 I2C functional block diagram

### Lecture 178. I2C functional block diagram

* STM32F446xx has 3 I2C IFs in APB1
* SDA and SCL pins go to Noise Filters to smooth out signals
	* SDA to Data Control connected to the Data Shift Register
	* SCL tp Clock Control
* I2C is half duplex so it has only one Data Register from where data is copied to Data Shift reg to go out from SDA through Data Control block. reverse also holds for read
* address reg is used to store the device address (when in slave mode)
* Clock control block is cofiged by Clock Control Reg (CCR) and control logic which is configed by CR1 and Cr2 and its status goes to Status Reg

## Section 52: I2C driver API requirements and config structures

### Lecture 179. I2C driver API requirements

* we add 2 more files in project for i2c .c and .h and start the drill
* for the I2C driver we will need the folloing API calls
	* I2C initialization
	* I2C master TX
	* I2C Master RX
	* I2C Slave Tx
	* I2C Slave Rx
	* I2C Error Interrupt Handling
	* I2C Event Interrupt Handling
* also the standard apis like peripheral enable and all
* Configurable Items for I2C
	* I2C_SCLSpeed
	* I2C_DeviceAddress
	* I2C_ACKControl
	* I2C_FMDutyCycle.
* IN Fast Mode the clock duty cycle is configurable
* Tasks to complete
* create the 2 files for driver
* add i2c details to MCU header file
	* i2c periph reg def struct
	* i2c base address macros
	* i2cx peripheral def macros
	* macros to enable /disable i2cx perih clock
	* bit position defs of i2c periph
* we cp them from courseRepo

### Lecture 180. I2C handle and configuration structure

* we add config struct and handle struct indriver.h
```
typedef struct {
	uint32_t 	I2C_SCLSpeed;
	uint8_t		I2C_DeviceAddress;
	uint8_t		I2C_ACKControl;
	uint16_t	I2C_FMDutyCycle;
}I2C_Config_t;

typedef struct{
	I2C_RegDef_t	*pI2Cx;
	I2C_Config_t	I2C_Config;
}I2C_Handle_t;
```

### Lecture 181. I2C user configurable macros

* we add options for the cofig attributes in Headers as macros
* we can define speeds below the max speed available
* in CR1 definition we see that ACK is disabled by default

### Lecture 182. I2C API prototypes

* we dd the api stubs in .h
* we cp the SPI API stubs to i2c to mod
* we keep the essentials only
* we impement PeriphCntrol and Deinit, IRQInterConfig and PiorityConfig, Periph Control
* the SPE equivalent for I2C is PE bit in CR1

### Lecture 183. Steps for I2C init implementation

* In I2C Init we need to
	* configure the mode (std or fast)
	* config the speed of periph clock (SCL) (the faster the shorter the wires)
	* config device address (applicable when device is slave)
	* Enable ACKing
	* cnfig rise time  for I2C pins (skew rate)
* we MUST do this while periph is disabled in CR1 (PE=0)
* In STM32F$x I2C periph, CR2 and CCR regs are used to cotnrol the I2C serial clock sets and timing of I2C like setup time and hold time
	* 6 FREQ bits in CR2 (0-5)
	* 12 CC bits in CCR (0-11)
	
## Section 53: I2C serial clock discussion(SCLK)

### Lecture 184. I2C serial clock settings with explanation

* in CR2 FREQ bits we pass the APB1 clock frequency in MHz
* FREQ val combined with CCR produces the SCL freq
* To generate a 100khw SCL clock with APB1 clock PCLK1 = 16MHz
	* We configure the mode in CCR reg (15th bit) as SM
	* Program FREQ field of CR2 with the value of PCLK1 (16 or 0x10)
	* Calculate and Program CCR value in CCR field of CCR register: For SM => 
		* Thigh(SCL)=CCR * TPLC1
		* Tlow(SCL)=CCR * TPLC1 (80 = 0x50)
* In FM Mode, generate a 200kHZ SCL freq, APB1 Clock (PCLK1) = 16MHz
	* Configure the mode i CCR register (15th bit) as FM
	* Select the duty cycle of Fast mode SCL in CCR reg (14th bit)
	* Program FREQ field of CR2 with the value of PCLK1 (16 or 0x10)
	* Calculate and Program CCR value in CCR field of CCR register: For SM => 
		* If DUTY = 0: Thigh(SCL)=CCR * TPLC1, Tlow(SCL)=2 * CCR * TPLC1
		* If DUTY = 1: Thigh(SCL)=9 * CCR * TPLC1, Tlow(SCL)=16 * CCR * TPLC1
* There are minimum requirements for Tlow and Thigh for each mode in the I2C Spec. we must be sure our times are ok with the spec

### Lecture 185. Clock Stretching

* Clock stretching means holding the clock to 0 (GND level)
* The moment clock is held to low, the the whole I2C interface pauses untill clock is given up to its normal operation level
* I2C devices, either Master or Slave, uses this feature to slow down the communication by stretching SCL to low, which prevents the clock to Rise high again and the I2C comm stops for a while

* Clock speed is set by Master.
* there are cases where I2C slave is not able to co-op with the clock speed given by master and needs to slow down
* slave need time to keep up. it uses clock stretching, holding clock to low. this pauses the I2C operation for a while
* holding down the clock does not count as clock cycle for the other participant to decode whants sent (there is no edge in SCl after all) when clock resumes clock cycles counting resumes

## Section 54: I2C Driver API : I2C Init

### Lecture 186. Implementation of I2C init API : Part 1

* we need to translate Config options to bit flags in regs
* we have to calculate APB1 speed
* clock is divided by prescalers in RCC registers
* RCC_CFGR reg SWS bits to detetmine clock source
* if its PLL we need a new function to calculate the clock
* HPRE bits in RCC_CFGR give the AHB prescaler
* APB prescaler is also in RCC_CFGR PPRE1 bits

### Lecture 187. Implementation of I2C init API : Part 2

* mask out the vals when setting bit fields for safety
* we set ADD[7:1] (slave address) in I2C_OAR1 reg
* setting ADD0 only when using 10bit address
* we leave ADDMODE bit (15) to 0 for 7b address
* also manual says that we must set 14bit to 1

### Lecture 188. Implementation of I2C init API : Part 3

* we go on to configure CCR based on the FM mode and the APB clock and the SCL clock speeds

## Section 55: I2C Driver API : I2C Master send data

### 189. I2C transfer sequence diagram for master sending data

* 