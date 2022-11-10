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

#ifndef NOZZLE_H
#define NOZZLE_H

#define F_CPU 8000000L




#include <stdint.h>

// BOARD values
// 0 - inlet
// 1 - elbow 1
// 2 - elbow 3



#define HZ 125
#define LED_ON PORTB &= ~(1 << PORTB5);
#define LED_OFF PORTB |= (1 << PORTB5);

// encoder values for the home positions
// rebuild tables2.c & table.h if you change this
#define HOME0 8052
#define HOME1 4013
#define HOME2 5428

// size of the table for pitch
#define PITCH_STEPS 64



#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitToggle(value, bit) ((value) ^= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define CLAMP(x, y, z) ((x) = ((x) < (y) ? (y) : ((x) > (z) ? (z) : (x))))
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define ABS(x) ((x) < 0 ? (-(x)) : (x))





#endif

