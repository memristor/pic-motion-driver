#include "hw.h"
#include "../bootloader.h"


/*
Available builtin asm substitutes:
	unsigned int __builtin_tbloffset(const void *p);
	unsigned int __builtin_tblpage(const void *p);
	unsigned int __builtin_tblrdh(unsigned int offset);
	unsigned int __builtin_tblrdl(unsigned int offset);
	void __builtin_tblwth(unsigned int offset, unsigned int data);
	void __builtin_tblwtl(unsigned int offset, unsigned int data);
*/


// const uint8_t space[512] __attribute__((space(psv), address(0xfc00*3)));
int eeprom_initialized() {
	return 0;
}
void eeprom_save() {
	// save to file
}
void eeprom_load() {
	// load from file
}
int8_t* eeprom_get_ptr() {
	return 0;
}

void BOOT bootloader_start() {

}

void BOOT bootloader_erase_page(uint32_t adr) {

}

void BOOT bootloader_write_row(uint32_t adr, uint8_t* data, int len) {

}

void BOOT bootloader_write_sequence() {

}

void BOOT bootloader_write_config() {
}

// 
void BOOT read_prog_mem(uint32_t adr, uint8_t* data, int num) {
	
}
