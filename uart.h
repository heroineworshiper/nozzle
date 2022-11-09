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




#endif





