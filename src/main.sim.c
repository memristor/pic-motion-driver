#include "regulator.h"
#include "config.h"
#include "com/packet.h"
#include "bootloader.h"
#include "init.h"

   
int main(void) {
	int16_t tmpX, tmpY, tmp, tmpO;
	char command, v, direction, tmpSU;

	initialize();
	
	start_packet('L');
		put_byte('L');
	end_packet();
	while(1)
	{
		Packet* pkt = pkt=try_read_packet();
		if(!pkt) continue;
		
		command = pkt->type;
		
		printf("got pkt: %c (%x)\n", pkt->type, pkt->type);
		reset_stuck();
		switch(command)
		{
			// set position and orientation
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
				// read status and position
			case CMD_SEND_STATUS_AND_POSITION:
				send_status_and_position();
				break;

				// set speed; Vmax(0-255)
			case CMD_SET_SPEED:
				set_speed(get_byte());
				break;

			case CMD_SET_ROTATION_SPEED: {
				uint8_t max_speed = get_byte(); 
				uint8_t max_accel = get_byte(); 
				set_rotation_speed(max_speed, max_accel);
				break;
			}
				
				// move forward [mm]
			case CMD_FORWARD:
				tmp = get_word();
				printf("gw: %d\n", tmp);
				v = get_byte();
				forward(tmp, v);
				break;

				// relative angle [degrees]
			case CMD_RELATIVE_ROTATE:
				tmp = get_word();
				turn(tmp);
				break;
				
				// absolute angle [degrees]
			case CMD_ABSOLUTE_ROTATE:
				tmp = get_word();
				rotate_absolute_angle(tmp);
				break;

				// rotate to and then move to point (Xc, Yc, v, direction) [mm]
			case CMD_TURN_AND_GO:
				
				tmpX = get_word();
				tmpY = get_word();
				v = get_byte();
				direction = get_byte(); // + means forward, - means backward
				turn_and_go(tmpX, tmpY, v, direction); //(x, y, end_speed, direction)
				break;
				     
			case CMD_CURVE:
				tmpX = get_word();
				tmpY = get_word();
				
				tmpO = get_word();
				tmpSU = get_word();
				direction = get_byte();

				// void arc(long Xc, long Yc, int Fi, char direction_angle, char direction)
				arc(tmpX, tmpY, tmpO, tmpSU, direction);

				break;
			
				// x [mm], y [mm], direction {-1 - backwards, 0 - pick closest, 1 - forward}
			case CMD_MOVE_TO: {
				tmpX = get_word();
				tmpY = get_word();
				int direction = get_byte();
				int radius = 0x7fff;
				if(pkt->size >= 6) {
					radius = get_word();
				}
				
				move_to(tmpX, tmpY, direction, radius);
				break;
			}
				// stop
			case CMD_HARD_STOP:
				stop();
				break;

				// stop and kill PWM
			case CMD_SOFT_STOP:
				soft_stop();
				break;
				
				// reset position, status and speed
			case CMD_RESET_DRIVER:
				reset_driver();
				break;
			
			case CMD_UNSTUCK:
				reset_stuck();
				break;
			
			case 'E':
				start_packet('E');
				put_byte(pkt->size);
				end_packet();
				break;
				
				
			case CMD_MOTOR: {
				tmpX = get_word();
				tmpY = get_word();
				motor_const(tmpX, tmpY);
				break;
			}
			
			
			

			default:
				force_status(STATUS_ERROR);
				break;
		}
		
		report_status();
	}

	return 0;
}
