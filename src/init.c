#include "init.h"
void initialize(void) {
	config_init();
	packet_init();
	
	hw_init();
	
	regulator_init();
	
	// must come after regulator_init !
	config_load();
	
	reset_driver();
	motor_init();
	set_speed(0x32);
}
