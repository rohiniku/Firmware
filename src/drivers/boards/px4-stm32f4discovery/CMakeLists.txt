############################################################################
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################
px4_add_module(
	MODULE drivers__boards__px4-stm32f4discovery
	SRCS
		px4discovery_init.c
		px4discovery_led.c
		stm32f4discovery_pwm_servo.c
		stm32f4discovery_spi.c
		stm32_autoleds.c
		stm32_bmp180.c
		stm32_boot.c
		stm32_bringup.c
		stm32_buttons.c
		stm32_ethernet.c
		stm32_extmem.c
		stm32_idle.c
		stm32_max31855.c
		stm32_max6675.c
		stm32_nsh.c
		stm32_pm.c
		stm32_pmbuttons.c
		stm32_pwm.c
		stm32_qencoder.c
		stm32_sdio.c
#		stm32_spi.c
		stm32_ssd1289.c
		stm32_ssd1351.c
		stm32_timer.c
		stm32_ug2864ambag01.c
		stm32_ug2864hsweg01.c
		stm32_uid.c
		stm32_usb.c
		stm32_userleds.c
		stm32_watchdog.c
		stm32_zerocross.c
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
