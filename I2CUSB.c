/*
  I2CUSB.c - USB to I2C interface
  Copyright (c) 2022 Jarkko Sonninen.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*
             LUFA Library
     Copyright (C) Dean Camera, 2021.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2010  OBinou (obconseil [at] gmail [dot] com)
  Copyright 2021  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the I2C-Mega-USB program. This file contains the main tasks of
 *  the project and is responsible for the initial application hardware configuration.
 */

#include "I2CUSB.h"
#include "twi.h"

#include <LUFA/Drivers/Peripheral/Serial.h>

#define TWI_TIMEOUT 1500 // us

/* Name: main.c
   Project: i2c-tiny-usb
   Author: Till Harbaum
   Tabsize: 4
   Copyright: (c) 2005 by Till Harbaum <till@harbaum.org>
   License: GPL
*/

/* commands from USB, must e.g. match command ids in kernel driver */
#define CMD_ECHO       0
#define CMD_GET_FUNC   1
#define CMD_SET_DELAY  2
#define CMD_GET_STATUS 3

#define CMD_I2C_IO     4
#define CMD_I2C_BEGIN  1  // flag fo I2C_IO
#define CMD_I2C_END    2  // flag fo I2C_IO

/* linux kernel flags */
#define I2C_M_TEN   0x10  /* we have a ten bit chip address */
#define I2C_M_RD    0x01
#define I2C_M_NOSTART   0x4000
#define I2C_M_REV_DIR_ADDR  0x2000
#define I2C_M_IGNORE_NAK  0x1000
#define I2C_M_NO_RD_ACK   0x0800

/* To determine what functionality is present */
#define I2C_FUNC_I2C      0x00000001
#define I2C_FUNC_10BIT_ADDR   0x00000002
#define I2C_FUNC_PROTOCOL_MANGLING  0x00000004 /* I2C_M_{REV_DIR_ADDR,NOSTART,..} */
#define I2C_FUNC_SMBUS_HWPEC_CALC 0x00000008 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_READ_WORD_DATA_PEC  0x00000800 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_WRITE_WORD_DATA_PEC 0x00001000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_PROC_CALL_PEC  0x00002000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_BLOCK_PROC_CALL_PEC 0x00004000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_BLOCK_PROC_CALL  0x00008000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_QUICK    0x00010000
#define I2C_FUNC_SMBUS_READ_BYTE  0x00020000
#define I2C_FUNC_SMBUS_WRITE_BYTE 0x00040000
#define I2C_FUNC_SMBUS_READ_BYTE_DATA 0x00080000
#define I2C_FUNC_SMBUS_WRITE_BYTE_DATA  0x00100000
#define I2C_FUNC_SMBUS_READ_WORD_DATA 0x00200000
#define I2C_FUNC_SMBUS_WRITE_WORD_DATA  0x00400000
#define I2C_FUNC_SMBUS_PROC_CALL  0x00800000
#define I2C_FUNC_SMBUS_READ_BLOCK_DATA  0x01000000
#define I2C_FUNC_SMBUS_WRITE_BLOCK_DATA 0x02000000
#define I2C_FUNC_SMBUS_READ_I2C_BLOCK 0x04000000 /* I2C-like block xfer  */
#define I2C_FUNC_SMBUS_WRITE_I2C_BLOCK  0x08000000 /* w/ 1-byte reg. addr. */
#define I2C_FUNC_SMBUS_READ_I2C_BLOCK_2  0x10000000 /* I2C-like block xfer  */
#define I2C_FUNC_SMBUS_WRITE_I2C_BLOCK_2 0x20000000 /* w/ 2-byte reg. addr. */
#define I2C_FUNC_SMBUS_READ_BLOCK_DATA_PEC  0x40000000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_WRITE_BLOCK_DATA_PEC 0x80000000 /* SMBus 2.0 */

#define I2C_FUNC_SMBUS_BYTE I2C_FUNC_SMBUS_READ_BYTE | \
  I2C_FUNC_SMBUS_WRITE_BYTE
#define I2C_FUNC_SMBUS_BYTE_DATA I2C_FUNC_SMBUS_READ_BYTE_DATA | \
  I2C_FUNC_SMBUS_WRITE_BYTE_DATA
#define I2C_FUNC_SMBUS_WORD_DATA I2C_FUNC_SMBUS_READ_WORD_DATA | \
  I2C_FUNC_SMBUS_WRITE_WORD_DATA
#define I2C_FUNC_SMBUS_BLOCK_DATA I2C_FUNC_SMBUS_READ_BLOCK_DATA | \
  I2C_FUNC_SMBUS_WRITE_BLOCK_DATA
#define I2C_FUNC_SMBUS_I2C_BLOCK I2C_FUNC_SMBUS_READ_I2C_BLOCK | \
  I2C_FUNC_SMBUS_WRITE_I2C_BLOCK

#define I2C_FUNC_SMBUS_EMUL I2C_FUNC_SMBUS_QUICK | \
  I2C_FUNC_SMBUS_BYTE | \
  I2C_FUNC_SMBUS_BYTE_DATA | \
  I2C_FUNC_SMBUS_WORD_DATA | \
  I2C_FUNC_SMBUS_PROC_CALL | \
  I2C_FUNC_SMBUS_WRITE_BLOCK_DATA | \
  I2C_FUNC_SMBUS_WRITE_BLOCK_DATA_PEC | \
  I2C_FUNC_SMBUS_I2C_BLOCK

const uint32_t func = I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;

#define STATUS_IDLE     0
#define STATUS_ADDRESS_ACK 1
#define STATUS_ADDRESS_NACK 2

static unsigned char status = STATUS_IDLE;

#ifdef ARDUINO_LEONARDO
#define LED_SETUP() DDRC |= (1<<7)
#define L_LED_OFF() PORTC &= ~(1<<7)
#define L_LED_ON() PORTC |= (1<<7)
#define L_LED_TOGGLE() PORTC ^= (1<<7)
#else
// Pro micro RX led reversed
#define LED_SETUP() DDRB |= (1<<0)
#define L_LED_OFF() PORTB |= (1<<0)
#define L_LED_ON() PORTB &= ~(1<<0)
#define L_LED_TOGGLE() PORTB ^= (1<<0)
#endif

#ifdef DEBUG
#define DEBUGC Serial_SendByte
static void i2c_scan(void);
static void loghex(uint8_t v);
#else
#define DEBUGC(x)
#endif

static void usb_i2c_io(void);

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	SetupHardware();

	GlobalInterruptEnable();

	for (;;) {
	  USB_USBTask();
	}
}

/** Configures the board hardware and chip peripherals for the project's functionality. */
void SetupHardware(void)
{
#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
#endif

	Serial_Init(115200, false);
	Serial_SendString("I2C-ATmega-USB version 0.1\r\n");

    static FILE uart_str;
	Serial_CreateStream(&uart_str);
    stdout = stdin = &uart_str;

	LED_SETUP();

	twi_init();
	twi_setTimeoutInMicros(TWI_TIMEOUT, 1);

#ifdef USE_INTERNAL_PULL_UPS
	DDRD &= ~(3);
	PORTD |= (3);
#endif

	GlobalInterruptEnable();

#ifdef DEBUG
	i2c_scan();
#endif

	/* Hardware Initialization */
	USB_Init();

	Serial_SendString("READY.\n\r");
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	// All our messages are vendor type
	if (!(USB_ControlRequest.bmRequestType & REQTYPE_VENDOR)) {
		DEBUGC('#');
		return;
	}
	L_LED_ON();

	// Debug printing is too slow and causes usb errors
	/*
	DEBUGF("bmRequestType:%02x bRequest:%02x wValue:%04x wIndex:%04x wLength:%04x\n\r",
		USB_ControlRequest.bmRequestType, USB_ControlRequest.bRequest,
		USB_ControlRequest.wValue, USB_ControlRequest.wIndex,
		USB_ControlRequest.wLength);
	*/

	switch (USB_ControlRequest.bRequest) {
        case CMD_ECHO:
			Endpoint_ClearSETUP();
			Endpoint_Write_Control_Stream_LE(&USB_ControlRequest.wValue, sizeof(USB_ControlRequest.wValue));
			Endpoint_ClearOUT();
			DEBUGC('E');
			break;

        case CMD_GET_FUNC:
			// TODO if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_VENDOR| REQREC_INTERFACE))
			/* Report our capabilities */
			Endpoint_ClearSETUP();
			Endpoint_Write_Control_Stream_LE(&func, sizeof(func));
			Endpoint_ClearOUT();
			DEBUGC('F');
			break;

        case CMD_SET_DELAY:
			// TODO if (USB_ControlRequest.bmRequestType & (REQREC_INTERFACE))
			// i2c bit delay, default is 10us -> 100kHz max
			Endpoint_ClearSETUP();
			Endpoint_ClearStatusStage();
			twi_setFrequency(1000000 / USB_ControlRequest.wValue);
			DEBUGC('D');
          	break;

        case CMD_I2C_IO:
        case CMD_I2C_IO | CMD_I2C_BEGIN:
        case CMD_I2C_IO | CMD_I2C_END:
        case CMD_I2C_IO | CMD_I2C_BEGIN | CMD_I2C_END:
          	usb_i2c_io();
          	break;

        case CMD_GET_STATUS:
			// TODO if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_VENDOR| REQREC_INTERFACE))
			Endpoint_ClearSETUP();
			Endpoint_Write_Control_Stream_LE(&status, sizeof(status));
			Endpoint_ClearOUT();
			DEBUGC('S');
			DEBUGC('\n');
			DEBUGC('\r');
			break;

        default:
			break;
	}

	L_LED_OFF();
}

static void usb_i2c_io(void)
{
	uint8_t cmd = USB_ControlRequest.bRequest;
	uint8_t address = USB_ControlRequest.wIndex;
	uint8_t is_read = USB_ControlRequest.wValue & I2C_M_RD;
	uint8_t size = USB_ControlRequest.wLength;

	static uint8_t buf[64];

	// Debug printing is too slow and causes usb errors
  	//DEBUGF("i2c %s at 0x%02x, len = %d\n", (is_read)?"rd":"wr", address, size);

	if (is_read) {
		DEBUGC('R');
		DEBUGC('0' + cmd);
		DEBUGC('0' + USB_ControlRequest.wValue);
		DEBUGC('0' + size);
  		uint8_t bytes = twi_readFrom(address, buf, size, cmd & CMD_I2C_END);
		Endpoint_ClearSETUP();
		if (size) {
			// TODO errorhandling
			Endpoint_Write_Control_Stream_LE(buf, size);
		}
		Endpoint_ClearOUT();
		DEBUGC('0' + bytes);

		status = (!size || bytes) ? STATUS_ADDRESS_ACK : STATUS_ADDRESS_NACK;

	} else {
		Endpoint_ClearSETUP();
		// TODO errorhandling
		Endpoint_Read_Control_Stream_LE(buf, size);
		Endpoint_ClearIN();

		// FIXME heuristics
		uint8_t do_wait = (cmd & CMD_I2C_END) && !size;

		uint8_t ret = twi_writeTo(address, buf, size, do_wait, cmd & CMD_I2C_END);
		DEBUGC('W');
		DEBUGC('0' + cmd);
		DEBUGC('0' + USB_ControlRequest.wValue);
		DEBUGC('0' + size);
		DEBUGC('0' + ret);
		status = (!ret) ? STATUS_ADDRESS_ACK : STATUS_ADDRESS_NACK;
	}
}

#ifdef DEBUG
static void loghex(uint8_t v) {
	static char hex[] = "0123456789ABCDEF";
	DEBUGC(hex[v >> 4]);
	DEBUGC(hex[v & 0xf]);
}

static void i2c_scan(void) {
	for (int i=0; i < 128; i++) {
		if (i < 8) {
			printf("   ");
		} else {
			uint8_t status = twi_writeTo(i, 0, 0, 1, 1);
			if (!status) {
				printf ("%02x*", i);
			} else {
				printf("%2x ", status);
			}
		}
		if ((i % 16) == 15) {
			printf("\n\r");
		}
	}
}
#endif
