/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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

/**
 * @file board_config.h
 *
 * Board internal definitions
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <stm32_gpio.h>

/* PX4IO connection configuration */
#define BOARD_USES_PX4IO_VERSION       2
#define PX4IO_SERIAL_DEVICE            "/dev/ttyS4"
#define PX4IO_SERIAL_TX_GPIO           GPIO_USART6_TX
#define PX4IO_SERIAL_RX_GPIO           GPIO_USART6_RX
#define PX4IO_SERIAL_BASE              STM32_USART6_BASE
#define PX4IO_SERIAL_VECTOR            STM32_IRQ_USART6
#define PX4IO_SERIAL_TX_DMAMAP         DMAMAP_USART6_TX
#define PX4IO_SERIAL_RX_DMAMAP         DMAMAP_USART6_RX
#define PX4IO_SERIAL_RCC_REG           STM32_RCC_APB2ENR
#define PX4IO_SERIAL_RCC_EN            RCC_APB2ENR_USART6EN
#define PX4IO_SERIAL_CLOCK             STM32_PCLK2_FREQUENCY
#define PX4IO_SERIAL_BITRATE           1500000               /* 1.5Mbps -> max rate for IO */

/* LEDs */
#define GPIO_LED_BLUE	/* PE4  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN4)
#define GPIO_LED_RED 	/* PD5 */ 	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN5)
#define GPIO_LED_GREEN  /* PE3 */  	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN3)

#define BOARD_HAS_CONTROL_STATUS_LEDS  	1
#define BOARD_ARMED_STATE_LED  	LED_BLUE

#define PIN_ADC1_INP4			(GPIO_ANALOG|GPIO_PORTC|GPIO_PIN4)
#define PIN_ADC1_INP5			(GPIO_ANALOG|GPIO_PORTB|GPIO_PIN1)
#define PIN_ADC1_INP8			(GPIO_ANALOG|GPIO_PORTC|GPIO_PIN5)
#define PIN_ADC1_INP9			(GPIO_ANALOG|GPIO_PORTB|GPIO_PIN0)
#define PIN_ADC1_INP10			(GPIO_ANALOG|GPIO_PORTC|GPIO_PIN0)
#define PIN_ADC1_INP11			(GPIO_ANALOG|GPIO_PORTC|GPIO_PIN1)

/* ADC channels */
#define PX4_ADC_GPIO  \
	/* PC4 */  PIN_ADC1_INP4,  \
	/* PC5 */  PIN_ADC1_INP8,  \
	/* PB0 */  PIN_ADC1_INP9,  \
	/* PB1 */  PIN_ADC1_INP5,  \
	/* PC0 */  PIN_ADC1_INP10, \
	/* PC1 */  PIN_ADC1_INP11

/* Define Channel numbers must match above GPIO pins */
#define ADC_BATTERY1_VOLTAGE_CHANNEL         8 /* PC5: BATT_VOLTAGE_SENS */
#define ADC_BATTERY1_CURRENT_CHANNEL         4 /* PC4: BATT_CURRENT_SENS */
#define ADC_BATTERY2_VOLTAGE_CHANNEL         5 /* PB1: AUX_VOLTAGE_SENS */
#define ADC_BATTERY2_CURRENT_CHANNEL         9 /* PB0: AUX_CURRENT_SENS */
#define ADC_PRESSURE_VOLTAGE_CHANNEL        10 /* PC0: PRESSURE_SENS */
#define ADC_AIRSPEED_VOLTAGE_CHANNEL        11 /* PC1: AIRSPEED_SENS */

#define ADC_CHANNELS \
	((1 << ADC_BATTERY1_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_BATTERY1_CURRENT_CHANNEL)       | \
	 (1 << ADC_BATTERY2_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_BATTERY2_CURRENT_CHANNEL)       | \
	 (1 << ADC_PRESSURE_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_AIRSPEED_VOLTAGE_CHANNEL))

#define SYSTEM_ADC_BASE STM32_ADC1_BASE

/* HW has to large of R termination on ADC todo:change when HW value is chosen */
#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

/* CAN Silence Silent mode control */
#define GPIO_CAN1_SILENT_S0  	/* PD3 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN3)
#define GPIO_CAN2_SILENT_S1  	/* PB7 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN7)


/* PWM */
#define DIRECT_PWM_OUTPUT_CHANNELS	7
#define BOARD_NUM_IO_TIMERS			2

/* Power supply control and monitoring GPIOs */
#define BOARD_NUMBER_BRICKS     	2

#define GPIO_nVDD_BRICK1_VALID	/* PB14 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTB|GPIO_PIN14)  // VDD_BRICK_VALID
#define GPIO_nVDD_BRICK2_VALID  /* PB15 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTB|GPIO_PIN15)  // VDD_BACKUP_VALID
#define GPIO_nVDD_USB_VALID   	/* PD10 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTD|GPIO_PIN10)  // VBUS_VALID
#define GPIO_nVDD_5V_PERIPH_OC  /* PD11 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTD|GPIO_PIN11)  // VDD_5V_PERIPH_OC
#define GPIO_nVDD_5V_HIPOWER_OC /* PD4  */ (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTD|GPIO_PIN4)   // VDD_5V_HIPOWER_OC

/* Tone alarm output */
#define TONE_ALARM_TIMER        	2  /* Timer 2 */
#define TONE_ALARM_CHANNEL      	2  /* PB3 TIM2_CH2 */

#define GPIO_TONE_ALARM_IDLE    /* PB3 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN3) // ALARM
#define GPIO_TONE_ALARM         GPIO_TIM2_CH2OUT_2

/* PWM input driver. Use FMU AUX5 pins attached to Timer1 Channel 1 */
#define PWMIN_TIMER                        1
#define PWMIN_TIMER_CHANNEL    	/* T1C1 */ 1
#define GPIO_PWM_IN            	/* PA8  */ GPIO_TIM1_CH1IN_1

/* FREQ input driver. Use FMU AUX6 pins attached to Timer1 Channel 2 */
#define FREQIN_TIMER                       1
#define FREQIN_TIMER_CHANNEL    /* T1C2 */ 1
#define GPIO_FREQ_IN            /* PA9  */ GPIO_TIM1_CH2IN_1

/* Safety LED is only on PX4IO */

#define GPIO_SAFETY_SWITCH_IN	/* PD6  */ (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTD|GPIO_PIN6)

/* Enable the FMU to use the switch it if there is no px4io fixme:This should be BOARD_SAFTY_BUTTON() */
#define GPIO_BTN_SAFETY 	GPIO_SAFETY_SWITCH_IN  /* Enable the FMU to control it if there is no px4io */

/* High-resolution timer */
#define HRT_TIMER               8  /* use Timer8 for the HRT */
#define HRT_TIMER_CHANNEL       3  /* use Capture/Compare Channel 3 */

/* By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED 	(!px4_arch_gpioread(GPIO_nVDD_USB_VALID))
#define BOARD_ADC_USB_VALID     	(!px4_arch_gpioread(GPIO_nVDD_USB_VALID))
#define BOARD_ADC_BRICK1_VALID  	(!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#define BOARD_ADC_BRICK2_VALID  	(!px4_arch_gpioread(GPIO_nVDD_BRICK2_VALID))
#define BOARD_ADC_SERVO_VALID   	(1)
#define BOARD_ADC_PERIPH_5V_OC  	(!px4_arch_gpioread(GPIO_nVDD_5V_PERIPH_OC))
#define BOARD_ADC_HIPOWER_5V_OC 	(!px4_arch_gpioread(GPIO_nVDD_5V_HIPOWER_OC))

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 	5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET 			1

#define BOARD_HAS_STATIC_MANIFEST 	1

#define BOARD_HAS_PWM  				DIRECT_PWM_OUTPUT_CHANNELS
#define BOARD_NUM_IO_TIMERS 		2

#define BOARD_DSHOT_MOTOR_ASSIGNMENT	{3, 2, 1, 0, 4, 5, 6};

/* Internal IMU Heater
 *
 * Connected to the IO Expander; tell compiler to enable support
 */
#define IOE_HEATER_ENABLED
#define IOE_HEATER_OUTPUT		4

/* GPS System Power Control
 *
 * Connected to the IO Expander; tell compiler to enable support
 */
#define IOE_GPSPWR_OUTPUT		2

/* USB Hub power control
 *
 * Connected to the IO Expander; tell compiler to enable support
 */
#define USBHUB_PWR_IOE_PIN		6

/* USB Hub reset control
 *
 * Connected to the MCU Expander; tell compiler to enable support
 */
#define USBHUB_RST_GPIO_PIN /* PA15 */	(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN15)

/* I/O expander reset control */
#define IOE_RST_GPIO_PIN 	/* PE14 */	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN14)

/* SPI bus item count */
#define BOARD_SPI_BUS_MAX_BUS_ITEMS	2

#define BOARD_ENABLE_CONSOLE_BUFFER

#define BOARD_USB_VBUS_SENSE_DISABLED

#define BOARD_I2C_BUS_CLOCK_INIT {0, 100000, 0, 100000}


#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_FDCAN1_TX,                   \
		GPIO_FDCAN1_RX,                   \
		GPIO_FDCAN2_TX,                   \
		GPIO_FDCAN2_RX,                   \
		GPIO_CAN1_SILENT_S0,              \
		GPIO_CAN2_SILENT_S1,              \
		USBHUB_RST_GPIO_PIN,		      \
		IOE_RST_GPIO_PIN,				  \
		GPIO_nVDD_BRICK1_VALID,           \
		GPIO_nVDD_BRICK1_VALID,           \
		GPIO_nVDD_USB_VALID,              \
		GPIO_nVDD_5V_PERIPH_OC,           \
		GPIO_nVDD_5V_HIPOWER_OC,          \
		PX4_GPIO_PIN_OFF(GPIO_SDMMC1_D0), \
		PX4_GPIO_PIN_OFF(GPIO_SDMMC1_CMD),\
		GPIO_TONE_ALARM_IDLE,             \
		GPIO_SAFETY_SWITCH_IN, 		      \
	}

__BEGIN_DECLS
#ifndef __ASSEMBLY__

extern void stm32_spiinitialize(void);
extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>
#endif /* __ASSEMBLY__ */
__END_DECLS
