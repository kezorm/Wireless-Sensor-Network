/**
 * \file
 *
 * \brief SSD1306 OLED display controller driver.
 *
 * Copyright (c) 2012-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#ifndef SSD1306_H_INCLUDED
#define SSD1306_H_INCLUDED

#include <compiler.h>
#include <port.h>
#include <spi.h>
#include <delay.h>
#include <tc.h>

// controller and OLED configuration file
#include "conf_ssd1306.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \defgroup ssd1306_oled_controller_group SSD1306 OLED Controller Low-level\
 * driver
 *
 * This is a low level driver for the SSD1306 OLED controller through 4-wire SPI.
 * It provides basic functions for initializing and writing to the OLED
 * controller. In addition to hardware control and use of the OLED controller
 * internal functions.
 *
 * Before writing data to the display call \ref ssd1306_init() which will set up
 * the physical interface and the OLED. A file named \ref conf_ssd1306.h is needed
 * to define which interface to use. For more information see the Interface
 * selection section. In addition one also need to define the pins
 * \ref SSD1306_DC_PIN, \ref SSD1306_CS_PIN and \ref SSD1306_RES_PIN and the
 * display \ref SSD1306_CLOCK_SPEED.
 *
 * \warning This driver is not reentrant and can not be used in interrupt\
 * service routines without extra care.
 *
 *
 * An example \ref conf_ssd1306.h file could look like
 * \code
	 // interface selection
	 #define SSD1306_SPI           SERCOM2

	 #define SSD1306_CLOCK_SPEED          1000000

	 #define SSD1306_DC_PIN               PIN_PB24
	 #define SSD1306_CS_PIN               PIN_PB27
	 #define SSD1306_RES_PIN              PIN_PA17
\endcode
 *
 * \section dependencies Dependencies
 * This driver depends on the following modules:
 * - \ref asfdoc_sam0_port_group for IO port control.
 * - \ref asfdoc_sam0_system_group for getting system clock speeds for init functions.
 * - \ref asfdoc_sam0_sercom_spi_group for communication with the OLED controller
 * - \ref asfdoc_sam0_sercom_spi_group for communication with the OLED controller
 * @{
 */

/**
 * \name Interface selection
 *
 * The OLED controller support both serial and parallel mode, that means there
 * is a number of possible ways of interfacing the controller using different
 * SAM peripherals. The different interfaces can be selected using different
 * defines. This driver supports the serial communication mode using an
 * SPI Master.
 *
 * \note The current driver only support serial mode.
 */
/** @{@} */

extern struct spi_module ssd1306_master;
extern struct spi_slave_inst ssd1306_slave;

//! \name OLED controller functions
//@{
void ssd1306_clear_screen(void);

void ssd1306_write_line(uint8_t line, uint8_t* disp_buffer);
void ssd1306_write_lines(uint8_t start_line, uint8_t line_count, uint8_t* disp_buffer);

void ssd1306_maintain_screen(void);
//@}

//! \name Display hardware control
//@{
/**
 * \brief Turn the OLED display on
 *
 * This function will turn on the OLED.
 */
void ssd1306_display_on(void);

/**
 * \brief Turn the OLED display off
 *
 * This function will turn off the OLED.
 */
void ssd1306_display_off(void);


//! \name Initialization
//@{
void ssd1306_init(void);
//@}

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* SSD1306_H_INCLUDED */
