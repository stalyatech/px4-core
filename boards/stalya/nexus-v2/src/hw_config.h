/*
 * hw_config.h
 *
 *  Created on: May 17, 2015
 *      Author: david_s5
 */

#ifndef HW_CONFIG_H_
#define HW_CONFIG_H_

/* Boot device selection list*/

#define USB0_DEV       0x01
#define SERIAL0_DEV    0x02
#define SERIAL1_DEV    0x04

#define APP_LOAD_ADDRESS               0x00020000
#define FLASH_START_ADDRESS            APP_LOAD_ADDRESS
#define BOOTLOADER_DELAY               5000
#define INTERFACE_USB                  1
#define INTERFACE_USB_CONFIG           "/dev/ttyACM0"

#define INTERFACE_USART                1
#define INTERFACE_USART_CONFIG         "/dev/ttyS0,115200"
#define BOOT_DELAY_ADDRESS             0x00000400
#define _FLASH_KBYTES                  (5 * 1024)
#define BOARD_FLASH_SIZE               (_FLASH_KBYTES * 1024)

#define BOARD_PIN_LED_ACTIVITY         GPIO_nLED_BLUE
#define BOARD_PIN_LED_BOOTLOADER       GPIO_nLED_RED

#define BOARD_LED_ON                   0
#define BOARD_LED_OFF                  1

#define SERIAL_BREAK_DETECT_DISABLED   1

#endif /* HW_CONFIG_H_ */
