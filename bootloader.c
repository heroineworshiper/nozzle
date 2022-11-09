/*
 * F-35 COMMON NOZZLE BOARD
 * Copyright (C) 2020-2022 Adam Williams <broadcast at earthling dot net>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 */

// bootloader for the common motor controller

// make bootloader.hex
// make bootloader_isp

#include "avr_debug.h"
#include "nozzle.h"

volatile uint8_t tick = 0;
volatile uint8_t got_tick = 0;

ISR(TIMER0_OVF_vect)
{
    got_tick = 1;
}



void main()
{
// disable watchdog
	WDTCSR = 0;
// enable LED
    DDRB |= 1 << DDB5;
    LED_ON

// move interrupt vector to bootloader flash
//    bitSet(MCUCR, IVSEL);

	init_serial();
	print_text("\n\nWelcome to F-35 bootloader\n");


// tick clock prescaler page 108
    TCCR0B = 0b00000100;
// enable interrupt handler
    TIMSK0 = 0b00000001;

// enable interrupts
//	sei();

	while(1)
	{
		handle_serial();
		if(have_uart_in)
		{
			have_uart_in = 0;
		}

        if(got_tick)
        {
            got_tick = 0;
            tick++;
        }
    }
}





