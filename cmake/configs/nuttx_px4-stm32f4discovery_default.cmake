include(nuttx/px4_impl_nuttx)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-none-eabi.cmake)

set(config_module_list
	#
	# Board support modules
	#
	drivers/device
	drivers/stm32
	drivers/stm32/adc
	drivers/stm32/tone_alarm
	drivers/led
	drivers/px4fmu
#DELDEL	drivers/px4io
	drivers/boards/px4-stm32f4discovery
#DELDEL	drivers/rgbled
#DELDEL	drivers/mpu6000
	drivers/mpu9250
#DELDEL	drivers/lsm303d
#DELDEL	drivers/l3gd20
#DELDEL	drivers/hmc5883
	drivers/ms5611
#DELDEL	drivers/mb12xx
#DELDEL	drivers/srf02
	drivers/sf0x
#DELDEL	drivers/ll40ls
#DELDEL	drivers/trone
	drivers/gps
	drivers/pwm_out_sim
	drivers/hott
	drivers/hott/hott_telemetry
	drivers/hott/hott_sensors
#DELDEL	drivers/blinkm
#DELDEL	drivers/airspeed
#DELDEL	drivers/ets_airspeed
#DELDEL	drivers/meas_airspeed
#DELDEL	drivers/frsky_telemetry
	modules/sensors
	#drivers/mkblctrl
#DELDEL	drivers/px4flow
#DELDEL	drivers/oreoled
	drivers/gimbal
	drivers/pwm_input
#DELDEL	drivers/camera_trigger

	#
	# System commands
	#
	systemcmds/bl_update
	systemcmds/mixer
	systemcmds/param
	systemcmds/perf
	systemcmds/pwm
	systemcmds/esc_calib
	systemcmds/reboot
	systemcmds/topic_listener
	systemcmds/top
	systemcmds/config
	systemcmds/nshterm
	systemcmds/mtd
	systemcmds/dumpfile
	systemcmds/ver
	systemcmds/tests

	#
	# General system control
	#
	modules/commander
	modules/navigator
	modules/mavlink
#DELDEL	modules/gpio_led
#DELDEL	modules/uavcan
	modules/land_detector

	#
	# Estimation modules (EKF/ SO3 / other filters)
	#
	# Too high RAM usage due to static allocations
	# modules/attitude_estimator_ekf
	modules/attitude_estimator_q
	modules/ekf_att_pos_estimator
	modules/position_estimator_inav

	#
	# Vehicle Control
	#
	# modules/segway # XXX Needs GCC 4.7 fix
	modules/fw_pos_control_l1
	modules/fw_att_control
	modules/mc_att_control
	modules/mc_pos_control
	modules/vtol_att_control

	#
	# Logging
	#
	modules/sdlog2

	#
	# Library modules
	#
	modules/param
	modules/systemlib
	modules/systemlib/mixer
	modules/controllib
	modules/uORB
	modules/dataman

	#
	# Libraries
	#
	#lib/mathlib/CMSIS
	lib/mathlib
	lib/mathlib/math/filter
	lib/ecl
	lib/external_lgpl
	lib/geo
	lib/geo_lookup
	lib/conversion
	lib/launchdetection
	lib/terrain_estimation
	lib/runway_takeoff
	platforms/nuttx

	# had to add for cmake, not sure why wasn't in original config
	platforms/common 
	platforms/nuttx/px4_layer

	#
	# OBC challenge
	#
	modules/bottle_drop

	#
	# Rover apps
	#
	examples/rover_steering_control

	#
	# Demo apps
	#
	#examples/math_demo
	# Tutorial code from
	# https://px4.io/dev/px4_simple_app
	examples/px4_simple_app

	# Tutorial code from
	# https://px4.io/dev/daemon
	#examples/px4_daemon_app

	# Tutorial code from
	# https://px4.io/dev/debug_values
	#examples/px4_mavlink_debug

	# Tutorial code from
	# https://px4.io/dev/example_fixedwing_control
	#examples/fixedwing_control

	# Hardware test
	examples/hwtest
)

set(config_extra_builtin_cmds
	serdis
	sercon
)

#DELDEL set(config_io_board
#DELDEL 	px4io-v2
#DELDEL 	)

set(config_extra_libs
 	${CMAKE_SOURCE_DIR}/src/lib/mathlib/CMSIS/libarm_cortexM4lf_math.a
#DELDEL 	uavcan
#DELDEL 	uavcan_stm32_driver
 	)

#DELDEL set(config_io_extra_libs
#DELDEL 	#${CMAKE_SOURCE_DIR}/src/lib/mathlib/CMSIS/libarm_cortexM3l_math.a
#DELDEL 	)

add_custom_target(sercon)
set_target_properties(sercon PROPERTIES
	MAIN "sercon" STACK "2048")

add_custom_target(serdis)
set_target_properties(serdis PROPERTIES
	MAIN "serdis" STACK "2048")
