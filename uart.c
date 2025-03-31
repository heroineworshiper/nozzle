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

// single wire UART driver


#include "uart.h"
#include "nozzle.h"

#include <stdint.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>




#define BAUD 115200L

// the UART buffer is sent at deterministic times
uint8_t uart_buffer[UART_SIZE];
uint8_t uart_used = 0;
uint8_t uart_write_ptr = 0;
uint8_t uart_read_ptr = 0;

// the debug buffer is sent in fragments between UART packets
#define DEBUG_SIZE 192
#define DEBUG_FRAGMENT 64
uint8_t debug_buffer[DEBUG_SIZE];
uint8_t debug_used = 0;
uint8_t debug_write_ptr = 0; 
uint8_t debug_read_ptr = 0;

volatile uint8_t have_uart_in = 0;
volatile uint8_t uart_in = 0;

// send data to the UART buffer
// void send_uart(uint8_t *text, uint8_t size)
// {
// 	uint8_t i;
// 	for(i = 0; uart_used < UART_SIZE && i < size; i++)
// 	{
// 		uart_buffer[uart_write_ptr++] = text[i];
// 		uart_used++;
// 		if(uart_write_ptr >= UART_SIZE) uart_write_ptr = 0;
// 	}
// }

// send data to the debug buffer
void print_text(char *text)
{
	uint8_t i;
	for(i = 0; debug_used < DEBUG_SIZE && text[i] != 0; i++)
	{
		debug_buffer[debug_write_ptr++] = text[i];
		debug_used++;
		if(debug_write_ptr >= DEBUG_SIZE) debug_write_ptr = 0;
	}
}

void print_number(int number)
{
	char string[8];
	char *ptr = string;
	if(number < 0)
	{
		*ptr++ = '-';
		number = -number;
	}

	if(number >= 10000) *ptr++ = '0' + (number / 10000);
	if(number >= 1000) *ptr++ = '0' + ((number / 1000) % 10);
	if(number >= 100) *ptr++ = '0' + ((number / 100) % 10);
	if(number >= 10) *ptr++ = '0' + ((number / 10) % 10);
	*ptr++ = '0' + (number % 10);
	*ptr++ = ' ';
	*ptr = 0;
	print_text(string);
}

void print_number_nospace(int number)
{
	char string[8];
	char *ptr = string;
	if(number < 0)
	{
		*ptr++ = '-';
		number = -number;
	}

	if(number >= 10000) *ptr++ = '0' + (number / 10000);
	if(number >= 1000) *ptr++ = '0' + ((number / 1000) % 10);
	if(number >= 100) *ptr++ = '0' + ((number / 100) % 10);
	if(number >= 10) *ptr++ = '0' + ((number / 10) % 10);
	*ptr++ = '0' + (number % 10);
	*ptr = 0;
	print_text(string);
}

void print_hex(uint8_t x)
{
    char string[4];
    const char table[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
        'a', 'b', 'c', 'd', 'e', 'f' };
    string[0] = table[(x >> 4) & 0xf];
    string[1] = table[x & 0xf];
    string[2] = 0;
    print_text(string);
}


// void handle_serial()
// {
// 	if(uart_used) 
// 	{
// // enable transmit mode.  disable receive mode
//         if(!bitRead(UCSR0B, TXEN0))
//         {
//             bitSet(UCSR0B, TXEN0);
//             bitClear(UCSR0B, RXEN0);
//         }
// 
// 
// 	    if(bitRead(UCSR0A, UDRE0)) 
// 	    {
// 			bitSet(UCSR0A, UDRE0); 
// 			UDR0 = uart_buffer[uart_read_ptr++]; 
// 			if(uart_read_ptr >= UART_SIZE) uart_read_ptr = 0; 
// 			uart_used--; 
// 	    }
// 
// 	}
//     else
//     {
// // disable transmit mode.  enable receive mode
//         if(bitRead(UCSR0B, TXEN0))
//         {
// // last byte transmitted
//             if(bitRead(UCSR0A, TXC0))
//             {
//                 bitSet(UCSR0A, TXC0);
//                 bitClear(UCSR0B, TXEN0);
//                 bitSet(UCSR0B, RXEN0);
//             }
//         }
// 
// 
//     }
// }

// send out a debug packet
void handle_debug()
{
    if(debug_used > 0 && uart_used < UART_SIZE)
    {
        uint8_t i = 0;
        while(i < DEBUG_FRAGMENT && debug_used > 0 && uart_used < UART_SIZE)
        {
            uart_buffer[uart_write_ptr++] = debug_buffer[debug_read_ptr++];
            if(uart_write_ptr >= UART_SIZE) uart_write_ptr = 0;
            if(debug_read_ptr >= DEBUG_SIZE) debug_read_ptr = 0;
            debug_used--;
            uart_used++;
            i++;
        }
    }
}

void init_serial()
{
	uint16_t baud_setting = (F_CPU / 4 / BAUD - 1) / 2;
	UBRR0H = baud_setting >> 8;
	UBRR0L = baud_setting & 0xff;
	UCSR0A = (1 << U2X0);
	UCSR0C = (1 << UCSZ01) |
		(1 << UCSZ00);
	UCSR0B = (1 << RXCIE0) |
		(1 << RXEN0);
//         |
//		(1 << TXEN0);


// enable pullup on receive pin
    bitSet(PORTD, PORTD0);
// input mode
    bitClear(DDRD, DDD0);

    uart_read_ptr = 0;
}

// receive interrupt
ISR(USART_RX_vect)
{
	uart_in = UDR0;
	have_uart_in = 1;
}





