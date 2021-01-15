
px4_add_board(
	PLATFORM nuttx
	VENDOR px4
	MODEL fmu-v5
	LABEL default
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m7
	ROMFSROOT px4fmu_common
	#IO px4_io-v2_default
	TESTING
	UAVCAN_INTERFACES 2
	SERIAL_PORTS
		GPS1:/dev/ttyS0
		TEL1:/dev/ttyS1
		TEL2:/dev/ttyS2
		TEL4:/dev/ttyS3
	DRIVERS
		adc/board_adc
		distance_sensor # all available distance sensor drivers
		imu/bosch/bmi055
		imu/invensense/icm20602
		imu/invensense/icm20689
		test_ppm
		pwm_out
	MODULES
		commander
		dataman
		ekf2
		mavlink
		navigator
		sensors
	SYSTEMCMDS
		mft
		mtd
		nshterm
		param
		perf
		#pwm
		reboot
		tests # tests and test runner
		top
		#topic_listener
		tune_control
		usb_connected
		work_queue
	EXAMPLES
		gyro_fft
		hello
		hwtest # Hardware test
		#matlab_csv_serial
	)
