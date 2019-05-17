#include "init.h"
void initialize(void) {
	config_init();
	packet_init();
	regulator_init();
	
	
	hw_init();
	// must come after regulator_init !
	config_load();
	
	reset_driver();
	motor_init();
}
