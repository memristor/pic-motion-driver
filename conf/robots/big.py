conf_floats={
		'wheel_distance': 252.31,
		'wheel_R1': 63.3,
		'wheel_R2': 63.3,
		
		'PID_d_p': 2.7,
		'PID_d_d': 70,
		'PID_r_p': 0.9,
		'PID_r_d': 100,
		'pid_r_i': 0.001,
		'accum_speed': 0.9,
		'accum_clip':10000,
		
		'vmax': 0x32,
		'omega': 0x32,
		
		'accel': 0x32,
		'alpha': 0x32,
		'slowdown': 1.0,
		'slowdown_angle': 1.1,
		'angle_speedup': 20,
		'speed_drop': 0.1,
		
		'encoder1': 0,
		'encoder2': 0,
		'encoder1_max':1231145,
		'encoder2_max':1231145,
		
		'setpoint1': 0,
		'setpoint2': 0,
		'speed1': 0.5,
		'speed2': 0.5,
		'pid_i1': 1.0,
		'pid_i2': 1.0,
		'pid_lin1': 0.5,
		'pid_lin2': 0.5,
	}
		
conf_integers={
		'stuck_distance_jump': 400, # in milimeters
		'stuck_rotation_jump': 180, # in degrees
		
		'stuck_distance_max_fail_count': 200,
		'stuck_rotation_max_fail_count': 200,
		
		# 3200 is max motor speed
		'motor_speed_limit': 3200,
		'left_motor_speed_limit': 3200,
		'right_motor_speed_limit': 3200,
		'motor_rate_of_change': 3200,
		'send_status_interval': 0,
		'motor_const_roc': 100,
		'tol1': 100,
		'tol2': 100
	}

conf_bytes={
		'distance_regulator': 1,
		'rotation_regulator': 1,
		'enable_stuck': 0,
		'stuck': 0, # is it stuck right now
		'debug': 0,
		'status_change_report': 1,
		'keep_count': 100,
		'tmr': 20,
		'motor_connected': 1,
		'debug_encoders': 0,
		'linear_mode': 0,
		'motor_flip_left': 1,
		'motor_flip_right': 1
	}
