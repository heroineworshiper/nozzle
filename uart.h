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

#ifndef UART_H
#define UART_H

#include <stdint.h>

extern uint8_t uart_used;
extern volatile uint8_t have_uart_in;
extern volatile uint8_t uart_in;
#define UART_SIZE 64
extern uint8_t uart_buffer[UART_SIZE];
extern uint8_t uart_used;
extern uint8_t uart_write_ptr;
extern uint8_t uart_read_ptr;

void send_uart(uint8_t *text, uint8_t size);
void print_text(char *text);
void print_number(int number);
void print_number_nospace(int number);
void flush_serial();
void handle_serial();
void handle_debug();
void init_serial();
void print_byte(char c);
void print_hex(uint8_t x);

#define HANDLE_SERIAL \
if(uart_used) \
{ \
/* enable transmit mode.  disable receive mode */ \
    if(!bitRead(UCSR0B, TXEN0)) \
    { \
        bitSet(UCSR0B, TXEN0); \
        bitClear(UCSR0B, RXEN0); \
    } \
 \
 \
	if(bitRead(UCSR0A, UDRE0))  \
	{ \
		bitSet(UCSR0A, UDRE0);  \
		UDR0 = uart_buffer[uart_read_ptr++];  \
		if(uart_read_ptr >= UART_SIZE) uart_read_ptr = 0;  \
		uart_used--;  \
	} \
 \
} \
else \
{ \
/* disable transmit mode.  enable receive mode */ \
    if(bitRead(UCSR0B, TXEN0)) \
    { \
/* last byte transmitted */ \
        if(bitRead(UCSR0A, TXC0)) \
        { \
            bitSet(UCSR0A, TXC0); \
            bitClear(UCSR0B, TXEN0); \
            bitSet(UCSR0B, RXEN0); \
        } \
    } \
}

#define SEND_UART(text, size) \
{ \
	uint8_t i; \
	for(i = 0; uart_used < UART_SIZE && i < size; i++) \
	{ \
		uart_buffer[uart_write_ptr++] = text[i]; \
		uart_used++; \
		if(uart_write_ptr >= UART_SIZE) uart_write_ptr = 0; \
	} \
}


#endif





