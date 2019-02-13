#include "config.h"
#include "chip_config.h"
#include "regulator.h"
#include "init.h"
#include "com/packet.h"
#include "drive/motor.h"

#ifdef SIM
#include <unistd.h>
char* can_iface = "can0";
int main(char argc, char* argv[]) {
#else
int main(void) {
#endif

	int16_t tmpX, tmpY, tmp, tmpO;
	char command, v, direction;

	
	#ifdef SIM
	if(argc > 1) {
		can_iface = argv[1];
	}
	#endif
	
	initialize();
	
	start_packet('L');
		put_byte('L');
	end_packet();
	while(1)
	{
		#ifdef SIM
		usleep(1000);
		#endif
		Packet* pkt = pkt=try_read_packet();
		if(!pkt) continue;
		#ifdef SIM
		printf("got pkt: %c (%x)\n", pkt->type, pkt->type);
		#endif
		command = pkt->type;
		reset_stuck();
		switch(command)
		{
			case CMD_SET_POSITION_AND_ORIENTATION:
				// x [mm], y [mm], orientation
				tmpX = get_word();
				tmpY = get_word();
				tmpO = get_word();
				set_position(tmpX, tmpY, tmpO);
				break;
			
			case CMD_SET_CONFIG:
				config_load(pkt->size, pkt->data);
				break;
			
			case CMD_GET_CONFIG: {
				int key = get_byte();
				uint32_t val;
				int exponent;
				int sign;
				config_get_as_fixed_point(key, (int32_t*)&val, &exponent, &sign);
				start_packet(CMD_GET_CONFIG);
					put_word(val >> 16);
					put_word(val);
					put_byte(sign);
					put_byte(exponent);
				end_packet();
				break;
			}
			
			case CMD_SET_CONFIG_HASH: {
				int hash = get_word();
				int32_t val = get_long();
				int exp=get_byte();
				int key = config_get_key(hash);
				config_set_as_fixed_point(key, val, exp);
				break;
			}
				
			case CMD_GET_CONFIG_HASH: {
				int hash = get_word();
				uint32_t val;
				int exponent;
				int sign;
				int key = config_get_key(hash);
				config_get_as_fixed_point(key, (int32_t*)&val, &exponent, &sign);
				
				start_packet(CMD_GET_CONFIG);
					put_word(val >> 16);
					put_word(val);
					put_byte(sign);
					put_byte(exponent);
				end_packet();
				break;
			}
			case CMD_SEND_STATUS_AND_POSITION:
				// read status and position
				send_status_and_position();
				break;

			case CMD_SET_SPEED:
				// set speed; Vmax(0-255)
				set_speed(get_byte());
				break;

			case CMD_SET_ROTATION_SPEED: {
				uint8_t max_speed = get_byte(); 
				uint8_t max_accel = get_byte(); 
				set_rotation_speed(max_speed, max_accel);
				break;
			}
				
			case CMD_FORWARD:
				// move forward [mm]
				tmp = get_word();
				forward(tmp);
				break;
				
			case CMD_FORWARD_LAZY:
				tmp = get_word();
				v = get_byte();
				forward_lazy(tmp, v);
				break;

				
			case CMD_ABSOLUTE_ROTATE:
				// absolute angle [degrees]
				tmp = get_word();
				rotate_absolute_angle(tmp);
				break;

			case CMD_RELATIVE_ROTATE:
				// relative angle [degrees]
				tmp = get_word();
				turn(tmp);
				break;
				
			case CMD_TURN_AND_GO:
				// rotate to and then move to point (Xc, Yc, v, direction) [mm]
				tmpX = get_word();
				tmpY = get_word();
				direction = get_byte(); // + means forward, - means backward
				turn_and_go(tmpX, tmpY, direction); //(x, y, end_speed, direction)
				break;
				     
			case CMD_CURVE:
				tmpX = get_word();
				tmpY = get_word();
				tmpO = get_word();
				tmp = get_byte();
				// void arc(long Xc, long Yc, int Fi, char direction)
				arc(tmpX, tmpY, tmpO, tmp);
				break;
				
			case CMD_CURVE_RELATIVE:
				tmpX = get_word();
				tmpO = get_word();
				// void arc_relative(int R, int Fi)
				arc_relative(tmpX, tmpO);
				break;
				
			case CMD_DIFF_DRIVE:
				tmpX = get_word();
				tmpY = get_word();
				tmpO = get_word();
				diff_drive( tmpX, tmpY, tmpO );
				break;
			
			case CMD_MOVE_TO: {
				// x [mm], y [mm], direction {-1 - backwards, 0 - pick closest, 1 - forward}
				tmpX = get_word();
				tmpY = get_word();
				int radius = get_word();
				int direction = get_byte();
				
				move_to(tmpX, tmpY, radius, direction);
				break;
			}
			case CMD_HARD_STOP:
				stop();
				break;

			case CMD_SOFT_STOP:
				// stop and kill PWM
				soft_stop();
				break;
				
			case CMD_RESET_DRIVER:
				// reset position, status and speed
				reset_driver();
				break;
			
			case CMD_UNSTUCK:
				reset_stuck();
				break;
			
			case CMD_LINEAR_OPTOCOUPLER:
				cmd_pwm_opto();
				break;
			
			
			case 'E':
				start_packet('E');
				put_byte(pkt->size);
				end_packet();
				break;
			/*
			case 'e':{
					uint8_t cnt = 0;
					while(1) {
						start_packet('e');
							put_byte(cnt);
						end_packet();
						cnt++;
						__delay_ms(1);
					}
				}
				break;
			*/
			
			case CMD_MOTOR: {
				tmpX = get_word();
				tmpY = get_word();
				motor_const(tmpX, tmpY);
				break;
			}
			
			case CMD_KEEP_SPEED: {
				tmpX = (int16_t)get_word();
				tmpY = (int16_t)get_word();
				speed_const(tmpX, tmpY);
				break;
			}
			
			case CMD_MOTOR_INIT: {
				motor_init();
				break;
			}
			
			/*
			// debugging bootloader
			case '.': {
				uint32_t adr = get_long();
				uint16_t len = get_word();
				
				uint8_t d[64*3];
				
				_DISI = 1;
				// SRbits.IPL = 7;
				while(len > 0) {
					int block = 64;
					if(len < block) block = len;
					read_prog_mem(adr, d, block);
					
					int i;
					int left = len*3;
					int ofs = 0;
					int c = 0;
					while(left > 0) {
						int l = 7;
						if(left < l) l = left;
						start_packet('r');
							for(i=0; i < l; i++) {
								put_byte(d[ofs+i]);
							}
						end_packet();
						__delay_ms(1);
						left -= l;
						ofs += l;
					}
					adr += 64*2;
					len -= block;
				}
				break;
			}
			case ']': {
				uint32_t adr = get_long();
				
				
				
				// start_packet('[');
				// end_packet();
				
				// uint8_t d[6];
				// int i;
				// for(i=0; i < 3; i++) {
					// d[i] = get_byte();
				// }
				
				// _DISI = 1;
				
				// _GIE = 1;
				
				break;
			}
			*/

			default:
				force_status(STATUS_ERROR);
		}
		
		report_status();
	}

	return 0;
}
