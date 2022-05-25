
px4_add_board(
	PLATFORM nuttx
	VENDOR stalya
	MODEL trap_v2
	LABEL default
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m7
	ROMFSROOT trapfmu
	BUILD_BOOTLOADER
	IO stalya_io-v2_default
	UAVCAN_INTERFACES 2
	SERIAL_PORTS
		TEL1:/dev/ttyS0
		TEL2:/dev/ttyS5
		TEL3:/dev/ttyS6
		WIFI:/dev/ttyS2
		GPS1:/dev/ttyS3
		GPS2:/dev/ttyS1
		#PX4IO:/dev/ttyS4
	DRIVERS
		adc/board_adc
		barometer/ms5611
		#batt_smbus
		#camera_capture
		#camera_trigger
		#differential_pressure
		#distance_sensor
		distance_sensor/nra15
		dshot
		gps
		heater
		imu/bosch/bmi088
		imu/stalya/icm20689
		irlock
		lights # all available light drivers
		magnetometer/lis3mdl
		#optical_flow
		#osd
		#pca9685
		#pca9685_pwm_out
		#power_monitor/ina226
		#protocol_splitter
		freq_input
		pwm_input
		pwm_out_sim
		pwm_out
		px4io
		roboclaw
		rpm
		safety_button
		#smart_battery/batmon
		telemetry # all available telemetry drivers
		tone_alarm
		#uavcan
		pca9557
		mr72radar
		usbhub/usb2514
	MODULES
		airspeed_selector
		attitude_estimator_q
		battery_status
		#camera_feedback
		commander
		dataman
		ekf2
		esc_battery
		events
		flight_mode_manager
		fw_att_control
		fw_pos_control_l1
		gyro_calibration
		gyro_fft
		land_detector
		landing_target_estimator
		load_mon
		local_position_estimator
		logger
		mavlink
		mc_att_control
		mc_hover_thrust_estimator
		mc_pos_control
		mc_rate_control
		#micrortps_bridge
		navigator
		rc_update
		rover_pos_control
		sensors
		sih
		temperature_compensation
		#uuv_att_control
		#uuv_pos_control
		vmount
		vtol_att_control
	SYSTEMCMDS
		bl_update
		dmesg
		dumpfile
		esc_calib
		gpio
		hardfault_log
		i2cdetect
		led_control
		mft
		mixer
		motor_ramp
		motor_test
		mtd
		nshterm
		param
		perf
		pwm
		reboot
		reflect
		#sd_bench
		#serial_test
		system_time
		top
		topic_listener
		tune_control
		uorb
		usb_connected
		ver
		work_queue
	EXAMPLES
		#fake_gps
		#fake_imu
		#fake_magnetometer
		#fixedwing_control # Tutorial code from https://px4.io/dev/example_fixedwing_control
		#hello
		#hwtest # Hardware test
		#matlab_csv_serial
		#px4_mavlink_debug # Tutorial code from http://dev.px4.io/en/debug/debug_values.html
		#px4_simple_app # Tutorial code from http://dev.px4.io/en/apps/hello_sky.html
		#rover_steering_control # Rover example app
		#uuv_example_app
		#work_item
	)
