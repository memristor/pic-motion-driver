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
void sw_reset() {
}

int8_t space[EEPROM_SIZE] __attribute__((space(psv), address(0xa7fe)));
int8_t eeprom_ram[EEPROM_SIZE];
int eeprom_initialized() {
	return space[EEPROM_SIZE-1];
	//return 0;
}
void eeprom_save() {
	int i;
	
	uint32_t page = 0xa700;
	
	for(i=0; i < EEPROM_SIZE; i++) {
		if(i % 256 == 0) {
			bootloader_erase_page(page+i);
		}
		//space[i] = eeprom_ram[i];
	}
	//space[EEPROM_SIZE-1] = 1;
}
void eeprom_load() {
	int i;
	for(i=0; i < EEPROM_SIZE; i++) {
		eeprom_ram[i] = space[i];
	}
}
int8_t* eeprom_get_ptr() {
	return eeprom_ram;
	//return 0;
}

void BOOT bootloader_start() {
	/*
	_DISI = 1;
	__builtin_disable_interrupts();
	SRbits.IPL = 7;
	
	Packet* pkt;
	// uint8_t d[64*3];
	bootloader_erase_page(adr);
	start_packet(']');
		put_long(adr);
	end_packet();
	int n=0;
	while(n < 8) {
		pkt=try_read_packet();
		if(pkt && pkt->type == '[' && pkt->size == 64) {
			start_packet(']');
				put_long(adr);
			end_packet();
			bootloader_write_row(adr, pkt->data, 21);
			n++;
		}
	}
	
	start_packet('[');
		put_long(adr);
	end_packet();
	*/
}

void BOOT bootloader_erase_page(uint32_t adr) {
	uint8_t pag = (adr >> 16) & 0xff;
	asm("mov %0, TBLPAG" :: "r"(pag));
	uint16_t wadr = adr & 0xffff;
	asm("tblwtl %0, [%0]" :: "r"(wadr));
	NVMCONbits.NVMOP = 2;
	NVMCONbits.ERASE = 1;
	bootloader_write_sequence();
	NVMCONbits.ERASE = 0;
}

/*
	data = [(8b 16b) (8b 16b)] => read 3 bytes at time
*/
void BOOT bootloader_write_row(uint32_t adr, uint8_t* data, int len) {
	
	// row is 64 instr or 192 bytes
	uint8_t pag = (adr >> 16) & 0xff;
	
	asm("mov %0, TBLPAG" :: "r"(pag));
	uint16_t wadr = adr & 0xffff;
	
	int i=64;
	while(--i >= 0) {
		/*
		uint8_t d;
		d = data[0];
		asm("tblwth.b %1, [%0]" : "+r"(wadr) : "r"(d));
		d = data[2];
		asm("tblwtl.b %1, [%0++]" : "+r"(wadr) :"r"(d));
		d = data[1];
		asm("tblwtl.b %1, [%0++]" : "+r"(wadr) : "r"(d));
		*/
		// data += 2;
		/*
		void __builtin_tblwth(unsigned int offset, unsigned int data);
		void __builtin_tblwtl(unsigned int offset, unsigned int data);
		*/
		__builtin_tblwth(wadr, data[0]);
		__builtin_tblwtl(wadr, (data[1] << 8) | data[2]);
		data += 2;
	}
	
	NVMCONbits.NVMOP = 1;
	bootloader_write_sequence();
}

void BOOT bootloader_write_sequence() {
	NVMCONbits.WREN = 1;
	__builtin_disi(5);
	NVMKEY = 0x55;
	NVMKEY = 0xAA;
	NVMCONbits.WR = 1;
	asm("nop");
	asm("nop");
	while(NVMCONbits.WR);
	NVMCONbits.WREN = 0;
}

void BOOT bootloader_write_config() {
	// tblwtl.b [WBUFPTR], [WADDR]	
	NVMCONbits.NVMOP = 0;
	bootloader_write_sequence();
}

// 
void BOOT read_prog_mem(uint32_t adr, uint8_t* data, int num) {
	uint8_t pag = (adr >> 16) & 0xff;
	
	// set page
	asm("mov %0, TBLPAG" :: "r"(pag));
	uint16_t wadr = adr & 0xffff;
	
	while(--num >= 0) {
		uint8_t d8 = __builtin_tblrdh(wadr);
		uint16_t d16 = __builtin_tblrdl(wadr);

		*data++ = d16;
		*data++ = d16 >> 8;
		*data++ = d8;
		wadr+=2;
	}
	// asm("mov %0, TBLPAG" :: "r"(cur_page));
	/*
		page 73 pic docs
		page 230 compiler docs for asm inline
	*/
}
