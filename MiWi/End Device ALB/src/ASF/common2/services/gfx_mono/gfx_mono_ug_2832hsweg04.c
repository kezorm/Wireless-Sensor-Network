/**
 * \file
 *
 * \brief Haven Display UG 2832HSWEG04 display glue code for display controller
 *
 * Copyright (c) 2013-2015 Atmel Corporation. All rights reserved.
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
#include "gfx_mono_ug_2832hsweg04.h"

/* If we are using a serial interface without readback, use framebuffer */

#ifdef CONFIG_SSD1306_FRAMEBUFFER
static uint8_t framebuffer[GFX_MONO_LCD_FRAMEBUFFER_SIZE];
#endif

/**
 * \brief Initialize SSD1306 controller and LCD display.
 * It will also write the graphic controller RAM to all zeroes.
 *
 * \note This function will clear the contents of the display.
 */
void gfx_mono_ssd1306_init(void)
{
	uint8_t x;
	uint8_t y;

	gfx_mono_set_framebuffer(framebuffer);

	/* Initialize the low-level display controller. */
	ssd1306_init();

	/* Clear the contents of the display.
	 * If using a framebuffer (SPI interface) it will both clear the
	 * controller memory and the framebuffer.
	 */
	for (y = 0; y < GFX_MONO_LCD_HEIGHT; y++) {
		for (x = 0; x < GFX_MONO_LCD_WIDTH; x+=8) {
			gfx_mono_framebuffer_put_byte(x, y, 0x00);
		}
	}
}

/**
 * \brief Put framebuffer to LCD controller
 *
 * This function will output the complete framebuffer from RAM to the
 * LCD controller.
 *
 * \note This is done automatically if using the graphic primitives. Only
 * needed if you are manipulating the framebuffer directly in your code.
 */
void gfx_mono_ssd1306_put_framebuffer(void)
{
    ssd1306_write_lines(0, 128, framebuffer);
}
