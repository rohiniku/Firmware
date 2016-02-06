/* /\**************************************************************************** */
/*  * */
/*  *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved. */
/*  * */
/*  * Redistribution and use in source and binary forms, with or without */
/*  * modification, are permitted provided that the following conditions */
/*  * are met: */
/*  * */
/*  * 1. Redistributions of source code must retain the above copyright */
/*  *    notice, this list of conditions and the following disclaimer. */
/*  * 2. Redistributions in binary form must reproduce the above copyright */
/*  *    notice, this list of conditions and the following disclaimer in */
/*  *    the documentation and/or other materials provided with the */
/*  *    distribution. */
/*  * 3. Neither the name PX4 nor the names of its contributors may be */
/*  *    used to endorse or promote products derived from this software */
/*  *    without specific prior written permission. */
/*  * */
/*  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS */
/*  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT */
/*  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS */
/*  * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE */
/*  * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, */
/*  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, */
/*  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS */
/*  * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED */
/*  * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT */
/*  * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN */
/*  * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE */
/*  * POSSIBILITY OF SUCH DAMAGE. */
/*  * */
/*  ****************************************************************************\/ */

/* /\** */
/*  * @file board_config.h */
/*  * */
/*  * PX4-STM32F4Discovery internal definitions */
/*  *\/ */

/* #pragma once */

/* /\**************************************************************************************************** */
/*  * Included Files */
/*  ****************************************************************************************************\/ */

/* #include <nuttx/config.h> */
/* #include <nuttx/compiler.h> */
/* #include <stdint.h> */

/* __BEGIN_DECLS */

/* /\* these headers are not C++ safe *\/ */
/* #include <stm32.h> */
/* #include <arch/board/board.h> */

/* #define UDID_START		0x1FFF7A10 */

/* /\**************************************************************************************************** */
/*  * Definitions */
/*  ****************************************************************************************************\/ */
/* /\* Configuration ************************************************************************************\/ */

/* /\* PX4-STM32F4Discovery GPIOs ***********************************************************************************\/ */
/* /\* LEDs *\/ */
/* // LED1 green, LED2 orange, LED3 red, LED4 blue */


/* #define GPIO_LED1       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\ */
/* 			 GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN12) */
/* #define GPIO_LED2       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\ */
/* 			 GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN13) */
/* #define GPIO_LED3       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\ */
/* 			 GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN14) */
/* #define GPIO_LED4       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\ */
/* 			 GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN15) */

/* /\* External interrupts *\/ */
/* #define GPIO_EXTI_ACCEL1_DRDY	(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTE|GPIO_PIN0) */
/* #define GPIO_EXTI_ACCEL2_DRDY	(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTE|GPIO_PIN1) */
/* #define GPIO_EXTI_MPU_DRDY	(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN4) */

/* /\* Data ready pins off *\/ */
/* #define GPIO_ACCEL1_DRDY_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTE|GPIO_PIN0) */
/* #define GPIO_ACCEL2_DRDY_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTE|GPIO_PIN1) */
/* #define GPIO_EXTI_MPU_DRDY_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_EXTI|GPIO_PORTC|GPIO_PIN4) */

/* /\* SPI1 off *\/ */
/* #define GPIO_SPI1_SCK_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTA|GPIO_PIN5) */
/* #define GPIO_SPI1_MISO_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTA|GPIO_PIN6) */
/* #define GPIO_SPI1_MOSI_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTA|GPIO_PIN7) */

/* /\* SPI1 chip selects off *\/ */
/* #define GPIO_SPI_CS_ACCEL_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTE|GPIO_PIN3) */
/* #define GPIO_SPI_CS_MPU_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTC|GPIO_PIN5) */

/* /\* SPI chip selects *\/ */
/* #define GPIO_SPI_CS_ACCEL	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3) */
/* #define GPIO_SPI_CS_MPU 	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN5) */

/* #define PX4_SPI_BUS_SENSORS	1 */

/* /\* Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI1 *\/ */
/* #define PX4_SPIDEV_ACCEL	0 */
/* #define PX4_SPIDEV_MPU  	1 */

/* /\* ---------------> *\/ */
/* #define GPIO_GPS_TIMEPULSE	(GPIO_INPUT|GPIO_FLOAT|GPIO_PORTC|GPIO_PIN4) */
/* #define GPS_DEFAULT_UART_PORT	"/dev/ttyS0" */

/* /\* SPI3--Sensors *\/ */
/* /\* #define PX4_SPI_BUS_SENSORS	3 *\/ */
/* /\* #define GPIO_SPI_CS_ACCEL_MAG	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN2) *\/ */
/* /\* #define GPIO_SPI_CS_GYRO	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3) *\/ */
/* /\* #define GPIO_SPI_CS_BARO	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN4) *\/ */

/* /\* SPI4--Ramtron *\/ */
/* /\* #define PX4_SPI_BUS_RAMTRON	4 *\/ */

/* /\* Nominal chip selects for devices on SPI bus #3 *\/ */
/* /\* #define PX4_SPIDEV_ACCEL_MAG	0 *\/ */
/* /\* #define PX4_SPIDEV_GYRO		1 *\/ */
/* /\* #define PX4_SPIDEV_BARO		2 *\/ */

/* /\* User GPIOs *\/ */
/* #define GPIO_GPIO0_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN0) */
/* #define GPIO_GPIO1_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN1) */
/* #define GPIO_GPIO3_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN1) */
/* #define GPIO_GPIO4_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN2) */
/* #define GPIO_GPIO5_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN3) */
/* #define GPIO_GPIO6_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN12) /\* LED GREEN *\/ */
/* #define GPIO_GPIO7_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN13) /\* LED ORANGE *\/ */
/* #define GPIO_GPIO8_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN14) /\* LED RED *\/ */
/* #define GPIO_GPIO9_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN15) /\* LED BLUE *\/ */
/* #define GPIO_GPIO10_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN5) */
/* #define GPIO_GPIO11_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN8) */

/* #define GPIO_GPIO0_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN0) */
/* #define GPIO_GPIO1_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN1) */
/* #define GPIO_GPIO3_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN1) */
/* #define GPIO_GPIO4_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN2) */
/* #define GPIO_GPIO5_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN3) */
/* #define GPIO_GPIO6_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN12) /\* LED GREEN *\/ */
/* #define GPIO_GPIO7_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN13) /\* LED ORANGE *\/ */
/* #define GPIO_GPIO8_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN14) /\* LED RED *\/ */
/* #define GPIO_GPIO9_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN15) /\* LED BLUE *\/ */
/* #define GPIO_GPIO10_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN5) */
/* #define GPIO_GPIO11_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN8) */

/* /\* */
/*  * ADC channels */
/*  * */
/*  * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver */
/*  *\/ */
/* #define ADC_CHANNELS (1 << 10) | (1 << 11) | (1 << 12) | (1 << 13) */

/* // ADC defines to be used in sensors.cpp to read from a particular channel */
/* #define ADC_BATTERY_VOLTAGE_CHANNEL	10 */
/* #define ADC_BATTERY_CURRENT_CHANNEL	((uint8_t)(-1)) */
/* #define ADC_AIRSPEED_VOLTAGE_CHANNEL	((uint8_t)(-1)) */
/* /\* <--------------- *\/ */




/* /\* USB OTG FS */
/*  * */
/*  * PA9  OTG_FS_VBUS VBUS sensing */
/*  *\/ */
/* #define GPIO_OTGFS_VBUS		(GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9) */

/* /\* PWM */
/*  * */
/*  * Eight PWM outputs are configured. */
/*  * */
/*  * Pins: */
/*  * */
/*  * CH1 : PE9  : TIM1_CH1 */
/*  * CH2 : PE11 : TIM1_CH2 */
/*  * CH3 : PE13 : TIM1_CH3 */
/*  * CH4 : PE14 : TIM1_CH4 */
/*  * CH5 : PB4  : TIM3_CH1 */
/*  * CH6 : PB5  : TIM3_CH2 */
/*  * CH7 : PC8  : TIM3_CH3 */
/*  * CH8 : PC9  : TIM3_CH4 */
/*  *\/ */
/* #define GPIO_TIM1_CH1OUT	GPIO_TIM1_CH1OUT_2 */
/* #define GPIO_TIM1_CH2OUT	GPIO_TIM1_CH2OUT_2 */
/* #define GPIO_TIM1_CH3OUT	GPIO_TIM1_CH3OUT_2 */
/* #define GPIO_TIM1_CH4OUT	GPIO_TIM1_CH4OUT_2 */
/* #define GPIO_TIM3_CH1OUT	GPIO_TIM3_CH1OUT_2 */
/* #define GPIO_TIM3_CH2OUT	GPIO_TIM3_CH2OUT_2 */
/* #define GPIO_TIM3_CH3OUT	GPIO_TIM3_CH3OUT_2 */
/* #define GPIO_TIM3_CH4OUT	GPIO_TIM3_CH4OUT_2 */

/* /\* High-resolution timer *\/ */
/* #define HRT_TIMER		8	/\* use timer8 for the HRT *\/ */
/* #define HRT_TIMER_CHANNEL	1	/\* use capture/compare channel *\/ */

/* /\* PWM input driver. Use PB7 pins attached to timer4 channel 2 *\/ */
/* #define PWMIN_TIMER		4 */
/* #define PWMIN_TIMER_CHANNEL	2 */
/* #define GPIO_PWM_IN		GPIO_TIM4_CH2IN_1 */

/* /\**************************************************************************************************** */
/*  * Public Types */
/*  ****************************************************************************************************\/ */

/* /\**************************************************************************************************** */
/*  * Public data */
/*  ****************************************************************************************************\/ */

/* #ifndef __ASSEMBLY__ */

/* /\**************************************************************************************************** */
/*  * Public Functions */
/*  ****************************************************************************************************\/ */

/* /\**************************************************************************************************** */
/*  * Name: stm32_spiinitialize */
/*  * */
/*  * Description: */
/*  *   Called to configure SPI chip select GPIO pins for the PX4FMU board. */
/*  * */
/*  ****************************************************************************************************\/ */

/* extern void stm32_spiinitialize(void); */

/* extern void stm32_usbinitialize(void); */

/* /\**************************************************************************** */
/*  * Name: nsh_archinitialize */
/*  * */
/*  * Description: */
/*  *   Perform architecture specific initialization for NSH. */
/*  * */
/*  *   CONFIG_NSH_ARCHINIT=y : */
/*  *     Called from the NSH library */
/*  * */
/*  *   CONFIG_BOARD_INITIALIZE=y, CONFIG_NSH_LIBRARY=y, && */
/*  *   CONFIG_NSH_ARCHINIT=n : */
/*  *     Called from board_initialize(). */
/*  * */
/*  ****************************************************************************\/ */

/* #ifdef CONFIG_NSH_LIBRARY */
/* int nsh_archinitialize(void); */
/* #endif */

/* #endif /\* __ASSEMBLY__ *\/ */

/* __END_DECLS */
