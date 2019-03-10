/**
 * \file
 *
 * \brief SSD1306 OLED display controller driver.
 *
 * Copyright (c) 2012-2016 Atmel Corporation. All rights reserved.
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
#include "ssd1306.h"

struct spi_module ssd1306_master;
struct spi_slave_inst ssd1306_slave;

static uint32_t delay_10uS;

static void assert_chip_select(void)
{
    // Assert the chip select pin (spi driver assumes negative logic)
    spi_select_slave(&ssd1306_master, &ssd1306_slave, false);

    delay_cycles(delay_10uS); // At lest 10us    
}

static void deassert_chip_select(void)
{
    delay_cycles(delay_10uS); // At lest 10us

    // Assert the chip select pin (spi driver assumes negative logic)
    spi_select_slave(&ssd1306_master, &ssd1306_slave, true);
}

/**
 * \internal
 * \brief Initialize the hardware interface
 *
 * Depending on what interface used for interfacing the OLED controller this
 * function will initialize the necessary hardware.
 */
static void ssd1306_interface_init(void)
{
	struct spi_config config;
	struct spi_slave_inst_config slave_config;

	spi_slave_inst_get_config_defaults(&slave_config);
	slave_config.ss_pin = SSD1306_CS_PIN;
	spi_attach_slave(&ssd1306_slave, &slave_config);

	spi_get_config_defaults(&config);

	config.mux_setting = SSD1306_SPI_PINMUX_SETTING;
	config.pinmux_pad0 = SSD1306_SPI_PINMUX_PAD0;
	config.pinmux_pad1 = SSD1306_SPI_PINMUX_PAD1;
	config.pinmux_pad2 = SSD1306_SPI_PINMUX_PAD2;
	config.pinmux_pad3 = SSD1306_SPI_PINMUX_PAD3;
	config.mode_specific.master.baudrate = SSD1306_CLOCK_SPEED;
    config.data_order  = SPI_DATA_ORDER_LSB;

	spi_init(&ssd1306_master, SSD1306_SPI, &config);
	spi_enable(&ssd1306_master);

	struct port_config pin;
	port_get_config_defaults(&pin);
	pin.direction = PORT_PIN_DIR_OUTPUT;

	port_pin_set_config(SSD1306_CS_PIN, &pin);
	port_pin_set_config(SSD1306_POWER_PIN, &pin);
	port_pin_set_config(SSD1306_DISPEN_PIN, &pin);
}

/**
 * \brief Initialize the OLED controller
 *
 * Call this function to initialize the hardware interface and the OLED
 * controller. When initialization is done the display is turned on and ready
 * to receive data.
 */
void ssd1306_init(void)
{
	// Initialize delay routine
	delay_init();

    delay_10uS = 10 * (system_gclk_gen_get_hz(0)/1000000);

	// Initialize the interface
	ssd1306_interface_init();

	// Set the chip select pin to the default state (spi driver assumes negative logic)
	spi_select_slave(&ssd1306_master, &ssd1306_slave, true);

	// Set the display enable pin to the default state
	port_pin_set_output_level(SSD1306_DISPEN_PIN, false);

	// Set the power pin to the default state
	port_pin_set_output_level(SSD1306_POWER_PIN, false);

	ssd1306_display_on();

	// Do a hard reset of the OLED display controller
	ssd1306_clear_screen();
}

void ssd1306_clear_screen(void)
{
    assert_chip_select();

    while(!spi_is_ready_to_write(&ssd1306_master));
	spi_write(&ssd1306_master, SSD1306_CMD_CLEAR_SCREEN);
    while(!spi_is_ready_to_write(&ssd1306_master));
    spi_write(&ssd1306_master, 0);
    while(!spi_is_ready_to_write(&ssd1306_master));
    spi_write(&ssd1306_master, 0);

    deassert_chip_select();
}

void ssd1306_write_line(uint8_t line, uint8_t* disp_buffer)
{
    disp_buffer += (16*line);
    line++;     // LCD line addesses are 1-indexed;
    
    assert_chip_select();

    while(!spi_is_ready_to_write(&ssd1306_master));
    spi_write(&ssd1306_master, SSD1306_CMD_UPDATE_DATA);
    while(!spi_is_ready_to_write(&ssd1306_master));
    spi_write(&ssd1306_master, line);
    for(uint8_t i=0; i<16; i++)
    {
        while(!spi_is_ready_to_write(&ssd1306_master));
        spi_write(&ssd1306_master, *disp_buffer++);
    }
    while(!spi_is_ready_to_write(&ssd1306_master));
    spi_write(&ssd1306_master, 0);
    while(!spi_is_ready_to_write(&ssd1306_master));
    spi_write(&ssd1306_master, 0);

    deassert_chip_select();
}

void ssd1306_write_lines(uint8_t start_line, uint8_t line_count, uint8_t* disp_buffer)
{
    disp_buffer += (16*start_line);
    start_line++;     // LCD line addesses are 1-indexed;

    assert_chip_select();

    while(!spi_is_ready_to_write(&ssd1306_master));
    spi_write(&ssd1306_master, SSD1306_CMD_UPDATE_DATA);
    for( ; line_count > 0; line_count--, start_line++ )
    {
        while(!spi_is_ready_to_write(&ssd1306_master));
        spi_write(&ssd1306_master, start_line);
        for(uint8_t i=0; i<16; i++)
        {
            while(!spi_is_ready_to_write(&ssd1306_master));
            spi_write(&ssd1306_master, *disp_buffer++);
        }
        while(!spi_is_ready_to_write(&ssd1306_master));
        spi_write(&ssd1306_master, 0);        
    }    
    while(!spi_is_ready_to_write(&ssd1306_master));
    spi_write(&ssd1306_master, 0);

    deassert_chip_select();
}

void ssd1306_maintain_screen(void)
{
    assert_chip_select();

    while(!spi_is_ready_to_write(&ssd1306_master));
    spi_write(&ssd1306_master, SSD1306_CMD_MAINTAIN_SCREEN);
    while(!spi_is_ready_to_write(&ssd1306_master));
    spi_write(&ssd1306_master, 0);
    while(!spi_is_ready_to_write(&ssd1306_master));
    spi_write(&ssd1306_master, 0);

    deassert_chip_select();
}
