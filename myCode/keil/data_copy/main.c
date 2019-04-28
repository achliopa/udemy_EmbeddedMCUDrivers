#include <stdint.h>

// this is a const data. read only and stored in MCU flash memory
char const my_data[] = "I love embedded programming";

#define BASE_ADDRESS_OF_SRAM 0x20000000

int main(void) {
	
	for(int i=0;i<sizeof(my_data);i++) {
		*((uint8_t *) BASE_ADDRESS_OF_SRAM+i) = my_data[i];
	}
	
	return 0;
}
