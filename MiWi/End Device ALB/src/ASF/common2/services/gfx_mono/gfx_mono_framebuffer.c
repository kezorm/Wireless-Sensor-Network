/**
 * \file
 *
 * \brief Local framebuffer
 *
 * Copyright (c) 2011-2015 Atmel Corporation. All rights reserved.
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

#include "gfx_mono_framebuffer.h"

/* Pointer to the framebuffer; updated by the gfx_mono_set_framebuffer function */
static uint8_t *fbpointer;

/**
 * \brief Set the LCD framebuffer.
 *
 * \param[in] framebuffer A pointer to an allocated area of RAM that can hold the
 * framebuffer.
 *
 * A small example:
 * \code
	uint8_t framebuffer[FRAMEBUFFER_SIZE];
	gfx_mono_set_framebuffer(framebuffer);
\endcode
 */
void gfx_mono_set_framebuffer(uint8_t *framebuffer)
{
	fbpointer = framebuffer;
}

/**
 * \brief Draw pixel to framebuffer
 *
 * \param[in] x         X coordinate of the pixel
 * \param[in] y         Y coordinate of the pixel
 * \param[in] color     Pixel operation
 *
 */
void gfx_mono_framebuffer_draw_pixel(gfx_coord_t x, gfx_coord_t y,
		gfx_mono_color_t color)
{
	uint8_t pixel_mask;
	uint8_t pixel_value;

	/* Discard pixels drawn outside the screen */
	if ((x > GFX_MONO_LCD_WIDTH - 1) || (y > GFX_MONO_LCD_HEIGHT - 1)) {
		return;
	}

	pixel_mask = (1 << (x & 0x7));

	/*
	 * Read the byte containing the pixel in interest, then perform the
	 * requested action on this pixel before writing the byte back to the
	 * display.
	 */
	pixel_value = gfx_mono_framebuffer_get_byte(x, y);

	switch (color) {
	case GFX_PIXEL_SET:
		pixel_value |= pixel_mask;
		break;

	case GFX_PIXEL_CLR:
		pixel_value &= ~pixel_mask;
		break;

	case GFX_PIXEL_XOR:
		pixel_value ^= pixel_mask;
		break;

	default:
		break;
	}

	gfx_mono_framebuffer_put_byte(x, y, pixel_value);
}

/**
 * \brief Get the pixel value at x,y in framebuffer
 *
 * \param[in] x      X coordinate of pixel
 * \param[in] y      Y coordinate of pixel
 * \return Non zero value if pixel is set.
 *
 */
uint8_t gfx_mono_framebuffer_get_pixel(gfx_coord_t x, gfx_coord_t y)
{
	uint8_t pixel_mask;

	if ((x > GFX_MONO_LCD_WIDTH - 1) || (y > GFX_MONO_LCD_HEIGHT - 1)) {
		return 0;
	}

	pixel_mask = (1 << (x & 0x7));

	return gfx_mono_framebuffer_get_byte(x, y) & pixel_mask;
}

/**
 * \brief Put a byte to the framebuffer
 *
 * \param[in] x      X coordinate of pixel in byte
 * \param[in] y      Y coordinate of pixel
 * \param[in] data   Data to be written
 *
 * This example will put the value 0xFF to the first byte in the framebuffer
 * \code
	gfx_mono_framebuffer_put_byte(0, 0, 0xFF);
\endcode
 */
void gfx_mono_framebuffer_put_byte(gfx_coord_t x, gfx_coord_t y,
		uint8_t data)
{
	*(fbpointer + (y * GFX_MONO_LCD_BYTES_PER_ROW) + (x / 8)) = data;
}

/**
 * \brief Get a byte from the framebuffer
 *
 * \param[in] x      X coordinate of pixel in byte
 * \param[in] y      Y coordinate of pixel
 * \return       data from LCD controller or framebuffer.
 *
 * The following code will read the first byte of the framebuffer
 * \code
	data = gfx_mono_framebuffer_get_byte(0, 0);
\endcode
 */
uint8_t gfx_mono_framebuffer_get_byte(gfx_coord_t x, gfx_coord_t y)
{
	return *(fbpointer + (y * GFX_MONO_LCD_BYTES_PER_ROW) + (x / 8));
}

/**
 * \brief Read/Modify/Write a byte in the framebuffer
 *
 * This function will read the byte from the framebuffer and
 * do a mask operation on the byte according to the pixel operation selected
 * by the color argument and the pixel mask provided.
 *
 * \param[in] x          X coordinate of pixel in byte
 * \param[in] y          Y coordinate of pixel
 * \param[in] pixel_mask Mask for pixel operation
 * \param[in] color      Pixel operation
 *
 * A small example that will XOR the first byte of the framebuffer with 0xAA
 * \code
	gfx_mono_framebuffer_mask_byte(0,0,0xAA,GFX_PIXEL_XOR);
\endcode
 */
void gfx_mono_framebuffer_mask_byte(gfx_coord_t x, gfx_coord_t y,
		gfx_mono_color_t pixel_mask, gfx_mono_color_t color)
{
	gfx_mono_color_t temp;

	temp = gfx_mono_get_byte(x, y);

	switch (color) {
	case GFX_PIXEL_SET:
		temp |= pixel_mask;
		break;

	case GFX_PIXEL_CLR:
		temp &= ~pixel_mask;
		break;

	case GFX_PIXEL_XOR:
		temp ^= pixel_mask;
		break;
	}

	gfx_mono_put_byte(x, y, temp);
}
