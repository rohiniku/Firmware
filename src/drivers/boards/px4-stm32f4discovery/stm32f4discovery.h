/****************************************************************************
 * configs/stm32f4discovery/src/stm32f4discovery.h
 *
 *   Copyright (C) 2011-2012, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __CONFIGS_STM32F4DISCOVERY_SRC_STM32F4DISCOVERY_H
#define __CONFIGS_STM32F4DISCOVERY_SRC_STM32F4DISCOVERY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <arch/stm32/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration *************************************************************/
/* How many SPI modules does this chip support? */

#if STM32_NSPI < 1
#  undef CONFIG_STM32_SPI1
#  undef CONFIG_STM32_SPI2
#  undef CONFIG_STM32_SPI3
#elif STM32_NSPI < 2
#  undef CONFIG_STM32_SPI2
#  undef CONFIG_STM32_SPI3
#elif STM32_NSPI < 3
#  undef CONFIG_STM32_SPI3
#endif

#define PCA9635_I2CBUS  1
#define PCA9635_I2CADDR 0x40

/* Assume that we have everything */

#define HAVE_USBDEV     1
#define HAVE_USBHOST    1
#define HAVE_USBMONITOR 1
#define HAVE_SDIO       1
#define HAVE_RTC_DRIVER 1
#define HAVE_ELF        1
#define HAVE_NETMONITOR 1

/* Can't support USB host or device features if USB OTG FS is not enabled */

#ifndef CONFIG_STM32_OTGFS
#  undef HAVE_USBDEV
#  undef HAVE_USBHOST
#  undef HAVE_USBMONITOR
#endif

/* Can't support USB device monitor if USB device is not enabled */

#ifndef CONFIG_USBDEV
#  undef HAVE_USBDEV
#  undef HAVE_USBMONITOR
#endif

/* Can't support USB host is USB host is not enabled */

#ifndef CONFIG_USBHOST
#  undef HAVE_USBHOST
#endif

/* Check if we should enable the USB monitor before starting NSH */

#if !defined(CONFIG_USBDEV_TRACE) || !defined(CONFIG_SYSTEM_USBMONITOR)
#  undef HAVE_USBMONITOR
#endif

/* Can't support MMC/SD features if mountpoints are disabled or if SDIO support
 * is not enabled.
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_STM32_SDIO)
#  undef HAVE_SDIO
#endif

#undef SDIO_MINOR      /* Any minor number, default 0 */
#define SDIO_SLOTNO 0  /* Only one slot */

#ifdef HAVE_SDIO

#  if defined(CONFIG_NSH_MMCSDSLOTNO) && CONFIG_NSH_MMCSDSLOTNO != 0
#    warning "Only one MMC/SD slot, slot 0"
#    undef CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO SDIO_SLOTNO
#  endif

#  if defined(CONFIG_NSH_MMCSDMINOR)
#    define SDIO_MINOR CONFIG_NSH_MMCSDMINOR
#  else
#    define SDIO_MINOR 0
#  endif

  /* SD card bringup does not work if performed on the IDLE thread because it
   * will cause waiting.  Use either:
   *
   *  CONFIG_NSH_ARCHINIT=y, OR
   *  CONFIG_BOARD_INITIALIZE=y && CONFIG_BOARD_INITTHREAD=y
   */

#  if defined(CONFIG_BOARD_INITIALIZE) && !defined(CONFIG_BOARD_INITTHREAD)
#    warning "SDIO initialization cannot be perfomed on the IDLE thread"
#    undef HAVE_SDIO
#  endif
#endif

/* Check if we can support the RTC driver */

#if !defined(CONFIG_RTC) || !defined(CONFIG_RTC_DRIVER)
#  undef HAVE_RTC_DRIVER
#endif

/* ELF */

#if defined(CONFIG_BINFMT_DISABLE) || !defined(CONFIG_ELF)
#  undef HAVE_ELF
#endif

/* NSH Network monitor  */

#if !defined(CONFIG_NET) || !defined(CONFIG_STM32_EMACMAC)
#  undef HAVE_NETMONITOR
#endif

#if !defined(CONFIG_NSH_NETINIT_THREAD) || !defined(CONFIG_ARCH_PHY_INTERRUPT) || \
    !defined(CONFIG_NETDEV_PHY_IOCTL) || !defined(CONFIG_NET_UDP) || \
     defined(CONFIG_DISABLE_SIGNALS)
#  undef HAVE_NETMONITOR
#endif

/* The NSH Network Monitor cannot be used with the STM32F4DIS-BB base board.
 * That is because the LAN8720 is configured in REF_CLK OUT mode.  In that
 * mode, the PHY interrupt is not supported.  The NINT pin serves instead as
 * REFLCK0.
 */

#ifdef CONFIG_STM32F4DISBB
#  undef HAVE_NETMONITOR
#endif

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* STM32F4 Discovery GPIOs **************************************************/
/* LEDs */

#define GPIO_LED1       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN12)
#define GPIO_LED2       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN13)
#define GPIO_LED3       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN14)
#define GPIO_LED4       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN15)

/* BUTTONS -- NOTE that all have EXTI interrupts configured */

#define MIN_IRQBUTTON   BUTTON_USER
#define MAX_IRQBUTTON   BUTTON_USER
#define NUM_IRQBUTTONS  1

#define GPIO_BTN_USER   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTA|GPIO_PIN0)

/* ZERO CROSS pin definiton */

#define GPIO_ZEROCROSS  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN0)

/* PWM
 *
 * The STM32F4 Discovery has no real on-board PWM devices, but the board can be
 * configured to output a pulse train using TIM4 CH2 on PD13.
 */

#define STM32F4DISCOVERY_PWMTIMER   4
#define STM32F4DISCOVERY_PWMCHANNEL 2

/* SPI chip selects */

#define GPIO_CS_MEMS      (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)

#define GPIO_MAX31855_CS  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN8)

#define GPIO_MAX6675_CS  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN8)

//-->
/* External interrupts */
#define GPIO_EXTI_ACCEL1_DRDY	(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTE|GPIO_PIN0)
#define GPIO_EXTI_ACCEL2_DRDY	(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTE|GPIO_PIN1)
#define GPIO_EXTI_MPU_DRDY	(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN4)

/* Data ready pins off */
#define GPIO_ACCEL1_DRDY_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTE|GPIO_PIN0)
#define GPIO_ACCEL2_DRDY_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTE|GPIO_PIN1)
#define GPIO_EXTI_MPU_DRDY_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_EXTI|GPIO_PORTC|GPIO_PIN4)

/* SPI1 off */
#define GPIO_SPI1_SCK_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTA|GPIO_PIN5)
#define GPIO_SPI1_MISO_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTA|GPIO_PIN6)
#define GPIO_SPI1_MOSI_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTA|GPIO_PIN7)

/* SPI2 off */
#define GPIO_SPI2_SCK_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTB|GPIO_PIN13)
#define GPIO_SPI2_MISO_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTB|GPIO_PIN14)
#define GPIO_SPI2_MOSI_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTB|GPIO_PIN15)

/* SPI1 chip selects off */
#define GPIO_SPI_CS_ACCEL_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTE|GPIO_PIN3)
#define GPIO_SPI_CS_MPU_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTC|GPIO_PIN5)

/* SPI chip selects */
#define GPIO_SPI_CS_ACCEL	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)
#define GPIO_SPI_CS_MPU 	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN5)
#if defined(CONFIG_MMCSD_SPI) && defined(CONFIG_MMCSD)
#define GPIO_SPI_CS_SDCARD	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN12)
#elif defined(CONFIG_MTD) && defined(CONFIG_MTD_W25)
#define GPIO_SPI_CS_W25         (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN12)
#endif

#define PX4_SPI_BUS_SENSORS	1  /* sensors via SPI1 */
#define PX4_SPI_BUS_W25         2  /* Winbond W25 SPI Flash memory via SPI2 */

/* Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI1 */
#define PX4_SPIDEV_ACCEL	1
#define PX4_SPIDEV_MPU  	2

/* Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI2 */
#if defined(CONFIG_MMCSD_SPI) && defined(CONFIG_MMCSD)
#define PX4_SPIDEV_SD           1
#elif defined(CONFIG_MTD) && defined(CONFIG_MTD_W25)
#define PX4_SPIDEV_W25  	1
#endif
//<--

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing (also connected to the green LED)
 * PC0  OTG_FS_PowerSwitchOn
 * PD5  OTG_FS_Overcurrent
 */

#define GPIO_OTGFS_VBUS   (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|\
                           GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)
#define GPIO_OTGFS_PWRON  (GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|\
                           GPIO_PUSHPULL|GPIO_PORTC|GPIO_PIN0)

#ifdef CONFIG_USBHOST
#  define GPIO_OTGFS_OVER (GPIO_INPUT|GPIO_EXTI|GPIO_FLOAT|\
                           GPIO_SPEED_100MHz|GPIO_PUSHPULL|\
                           GPIO_PORTD|GPIO_PIN5)

#else
#  define GPIO_OTGFS_OVER (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|\
                           GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN5)
#endif

/* UG-2864AMBAG01 or UG-2864HSWEG01 OLED Display (SPI 4-wire):
 *
 * --------------------------+----------------------------------------------
 * Connector CON10 J1:      | STM32F4Discovery
 * --------------+-----------+----------------------------------------------
 * CON10 J1:     | CON20 J2: | P1/P2:
 * --------------+-----------+----------------------------------------------
 * 1  3v3        | 3,4 3v3   | P2 3V
 * 3  /RESET     | 8 /RESET  | P2 PB6 (Arbitrary selection)
 * 5  /CS        | 7 /CS     | P2 PB7 (Arbitrary selection)
 * 7  A0|D/C     | 9 A0|D/C  | P2 PB8 (Arbitrary selection)
 * 9  LED+ (N/C) | -----     | -----
 * 2  5V Vcc     | 1,2 Vcc   | P2 5V
 * 4  DI         | 18 D1/SI  | P1 PA7 (GPIO_SPI1_MOSI == GPIO_SPI1_MOSI_1 (1))
 * 6  SCLK       | 19 D0/SCL | P1 PA5 (GPIO_SPI1_SCK == GPIO_SPI1_SCK_1 (1))
 * 8  LED- (N/C) | -----     | ------
 * 10 GND        | 20 GND    | P2 GND
 * --------------+-----------+----------------------------------------------
 * (1) Required because of on-board MEMS
 * -------------------------------------------------------------------------
 */

#if defined(CONFIG_LCD_UG2864AMBAG01) || defined(CONFIG_LCD_UG2864HSWEG01) || \
    defined(CONFIG_LCD_SSD1351)
#  define GPIO_OLED_RESET (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN6)
#  define GPIO_OLED_CS    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN7)
#  define GPIO_OLED_A0    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN8)
#  define GPIO_OLED_DC    GPIO_OLED_A0
#endif

/* STM32F4DIS-BB MicroSD
 *
 * ---------- ------------- ------------------------------
 * PIO        SIGNAL        Comments
 * ---------- ------------- ------------------------------
 * PB15       NCD           Pulled up externally
 * PC9        DAT1          Configured by driver
 * PC8        DAT0          "        " "" "    "
 * PC12       CLK           "        " "" "    "
 * PD2        CMD           "        " "" "    "
 * PC11       CD/DAT3       "        " "" "    "
 * PC10       DAT2          "        " "" "    "
 * ---------- ------------- ------------------------------
 */

#if defined(CONFIG_STM32F4DISBB) && defined(CONFIG_STM32_SDIO)
#  define GPIO_SDIO_NCD   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|\
                           GPIO_PORTB|GPIO_PIN15)
#endif

/* STM32F4DIS-BB LAN8720
 *
 * ---------- ------------- ------------------------------
 * PIO        SIGNAL        Comments
 * ---------- ------------- ------------------------------
 * PB11       TXEN           Configured by driver
 * PB12       TXD0          "        " "" "    "
 * PB13       TXD1          "        " "" "    "
 * PC4        RXD0/MODE0    "        " "" "    "
 * PC5        RXD1/MODE1    "        " "" "    "
 * PA7        CRS_DIV/MODE2 "        " "" "    "
 * PA2        MDIO          "        " "" "    "
 * PC1        MDC           "        " "" "    "
 * PA1        NINT/REFCLK0  "        " "" "    "
 * PE2        DAT2          "        " "" "    "
 * ---------- ------------- ------------------------------
 */

#if defined(CONFIG_STM32F4DISBB) && defined(CONFIG_STM32_ETHMAC)
#  define GPIO_EMAC_NINT  (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|\
                           GPIO_PORTA|GPIO_PIN1)
#  define GPIO_EMAC_NRST  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN2)
#endif

//-->
/* Tone alarm output */
#define TONE_ALARM_TIMER	2	/* timer 2 */
#define TONE_ALARM_CHANNEL	1	/* channel 1 */
#define GPIO_TONE_ALARM_IDLE	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN15)
#define GPIO_TONE_ALARM		(GPIO_ALT|GPIO_AF1|GPIO_SPEED_2MHz|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN15)

/* PWM
 *
 * Eight PWM outputs are configured.
 *
 * Pins:
 *
 * CH1 : PE9  : TIM1_CH1
 * CH2 : PE11 : TIM1_CH2
 * CH3 : PE13 : TIM1_CH3
 * CH4 : PE14 : TIM1_CH4
 * CH5 : PB4  : TIM3_CH1
 * CH6 : PB5  : TIM3_CH2
 * CH7 : PC8  : TIM3_CH3
 * CH8 : PC9  : TIM3_CH4
 */
#define GPIO_TIM1_CH1OUT	GPIO_TIM1_CH1OUT_2
#define GPIO_TIM1_CH2OUT	GPIO_TIM1_CH2OUT_2
#define GPIO_TIM1_CH3OUT	GPIO_TIM1_CH3OUT_2
#define GPIO_TIM1_CH4OUT	GPIO_TIM1_CH4OUT_2
#define GPIO_TIM3_CH1OUT	GPIO_TIM3_CH1OUT_2
#define GPIO_TIM3_CH2OUT	GPIO_TIM3_CH2OUT_2
#define GPIO_TIM3_CH3OUT	GPIO_TIM3_CH3OUT_2
#define GPIO_TIM3_CH4OUT	GPIO_TIM3_CH4OUT_2

/* High-resolution timer */
#define HRT_TIMER		8	/* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL	1	/* use capture/compare channel */

/* PWM input driver. Use PB7 pins attached to timer4 channel 2 */
#define PWMIN_TIMER		4
#define PWMIN_TIMER_CHANNEL	2
#define GPIO_PWM_IN		GPIO_TIM4_CH2IN_1

/* User GPIOs */
#define GPIO_GPIO0_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN1)
#define GPIO_GPIO1_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN0)
#define GPIO_GPIO3_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN2)
#define GPIO_GPIO4_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN7)
#define GPIO_GPIO5_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN8)
#define GPIO_GPIO6_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN15)
#define GPIO_GPIO7_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN11)
#define GPIO_GPIO8_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN6)
#define GPIO_GPIO9_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN4)
#define GPIO_GPIO10_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN5)
#define GPIO_GPIO11_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN8)

#define GPIO_GPIO0_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN1)
#define GPIO_GPIO1_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN0)
#define GPIO_GPIO3_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN2)
#define GPIO_GPIO4_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN7)
#define GPIO_GPIO5_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN8)
#define GPIO_GPIO6_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN15)
#define GPIO_GPIO7_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN11)
#define GPIO_GPIO8_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN6)
#define GPIO_GPIO9_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN4)
#define GPIO_GPIO10_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN5)
#define GPIO_GPIO11_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN8)

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver
 */
#define ADC_CHANNELS (1 << 10) | (1 << 11) | (1 << 12) | (1 << 13)

// ADC defines to be used in sensors.cpp to read from a particular channel
#define ADC_BATTERY_VOLTAGE_CHANNEL	10
#define ADC_BATTERY_CURRENT_CHANNEL	((uint8_t)(-1))
#define ADC_AIRSPEED_VOLTAGE_CHANNEL	((uint8_t)(-1))

#define UDID_START		0x1FFF7A10
//<--

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the stm32f4discovery
 *   board.
 *
 ****************************************************************************/

void weak_function stm32_spiinitialize(void);

/****************************************************************************
 * Name: stm32_bmp180initialize
 *
 * Description:
 *   Called to configure an I2C and to register BMP180 for the stm32f4discovery
 *   board.
 *
 ****************************************************************************/

#ifdef CONFIG_BMP180
int stm32_bmp180initialize(FAR const char *devpath);
#endif

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_INITIALIZE=y :
 *     Called from board_initialize().
 *
 *   CONFIG_BOARD_INITIALIZE=y && CONFIG_NSH_ARCHINIT=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_usbinitialize very early in initialization to setup
 *   USB-related GPIO pins for the STM32F4Discovery board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_OTGFS
void weak_function stm32_usbinitialize(void);
#endif

/****************************************************************************
 * Name: stm32_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality. This function will start a thread that will monitor for
 *   device connection/disconnection events.
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_OTGFS) && defined(CONFIG_USBHOST)
int stm32_usbhost_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_extmemgpios
 *
 * Description:
 *   Initialize GPIOs for external memory usage
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm32_extmemgpios(const uint32_t *gpios, int ngpios);
#endif

/****************************************************************************
 * Name: stm32_extmemaddr
 *
 * Description:
 *   Initialize adress line GPIOs for external memory access
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm32_extmemaddr(int naddrs);
#endif

/****************************************************************************
 * Name: stm32_extmemdata
 *
 * Description:
 *   Initialize data line GPIOs for external memory access
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm32_extmemdata(int ndata);
#endif

/****************************************************************************
 * Name: stm32_enablefsmc
 *
 * Description:
 *  enable clocking to the FSMC module
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm32_enablefsmc(void);
#endif

/****************************************************************************
 * Name: stm32_disablefsmc
 *
 * Description:
 *  enable clocking to the FSMC module
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm32_disablefsmc(void);
#endif

/****************************************************************************
 * Name: stm32_led_pminitialize
 *
 * Description:
 *   Enable logic to use the LEDs on the STM32F4Discovery to support power
 *   management testing
 *
 ****************************************************************************/

#ifdef CONFIG_PM
void stm32_led_pminitialize(void);
#endif

/****************************************************************************
 * Name: stm32_pm_buttons
 *
 * Description:
 *   Configure the user button of the STM32f4discovery board as EXTI,
 *   so it is able to wakeup the MCU from the PM_STANDBY mode
 *
 ****************************************************************************/

#if defined(CONFIG_PM) && defined(CONFIG_ARCH_IDLE_CUSTOM) && defined(CONFIG_PM_BUTTONS)
void stm32_pm_buttons(void);
#endif

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_INITIALIZE=y :
 *     Called from board_initialize().
 *
 *   CONFIG_BOARD_INITIALIZE=n && CONFIG_NSH_ARCHINIT=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_STM32_SDIO)
int stm32_sdio_initialize(void);
#endif

/************************************************************************************
 * Name: stm32_netinitialize
 *
 * Description:
 *   Configure board resources to support networking.
 *
 ************************************************************************************/

#ifdef HAVE_NETMONITOR
void weak_function stm32_netinitialize(void);
#endif

/****************************************************************************
 * Name: stm32_zerocross_initialize
 *
 * Description:
 *   Initialize and register the zero cross driver
 *
 ****************************************************************************/

#ifdef CONFIG_ZEROCROSS
int stm32_zerocross_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_max6675initialize
 *
 * Description:
 *   Initialize and register the max6675 driver
 *
 ****************************************************************************/

#ifdef CONFIG_MAX6675
int stm32_max6675initialize(FAR const char *devpath);
#endif

/****************************************************************************
 * Name: stm32_pca9635_initialize
 *
 * Description:
 *   This function is called by board initialization logic to configure the
 *   LED PWM chip.  This function will register the driver as /dev/leddrv0.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_PCA9635PW
int stm32_pca9635_initialize(void);
#endif

/****************************************************************************
 * Name: up_timer_init
 *
 * Description:
 *   Configure the timer driver.
 *
 * Input Parameters:
 *   devpath - The full path to the timer device.  This should be of the
 *             form /dev/timer0
 *   timer   - The timer's number.
 *
 * Returned Values:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_TIMER
int up_timer_init(FAR const char *devpath, int timer);
#endif

/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization for NSH.
 *
 *   CONFIG_NSH_ARCHINIT=y :
 *     Called from the NSH library
 *
 *   CONFIG_BOARD_INITIALIZE=y, CONFIG_NSH_LIBRARY=y, &&
 *   CONFIG_NSH_ARCHINIT=n :
 *     Called from board_initialize().
 *
 ****************************************************************************/

#ifdef CONFIG_NSH_LIBRARY
int nsh_archinitialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_STM32F4DISCOVERY_SRC_STM32F4DISCOVERY_H */
