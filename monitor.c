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

// print only nozzle debug statements from the UART
// make monitor;./monitor

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <unistd.h>
#include <linux/serial.h>



// Returns the FD of the serial port
static int init_serial(char *path, int baud, int custom_baud)
{
	struct termios term;

	printf("init_serial %d: opening %s\n", __LINE__, path);

// Initialize serial port
	int fd = open(path, O_RDWR | O_NOCTTY | O_SYNC);
	if(fd < 0)
	{
		printf("init_serial %d: path=%s: %s\n", __LINE__, path, strerror(errno));
		return -1;
	}
	
	if (tcgetattr(fd, &term))
	{
		printf("init_serial %d: path=%s %s\n", __LINE__, path, strerror(errno));
		close(fd);
		return -1;
	}


#ifndef __clang__
// Try to set kernel to custom baud and low latency
	if(custom_baud)
	{
		struct serial_struct serial_struct;
		if(ioctl(fd, TIOCGSERIAL, &serial_struct) < 0)
		{
			printf("init_serial %d: path=%s %s\n", __LINE__, path, strerror(errno));
		}

		serial_struct.flags |= ASYNC_LOW_LATENCY;
		serial_struct.flags &= ~ASYNC_SPD_CUST;
		if(custom_baud)
		{
			serial_struct.flags |= ASYNC_SPD_CUST;
			serial_struct.custom_divisor = (int)((float)serial_struct.baud_base / 
				(float)custom_baud + 0.5);
			baud = B38400;
		}  
/*
 * printf("init_serial: %d serial_struct.baud_base=%d serial_struct.custom_divisor=%d\n", 
 * __LINE__,
 * serial_struct.baud_base,
 * serial_struct.custom_divisor);
 */


// Do setserial on the command line to ensure it actually worked.
		if(ioctl(fd, TIOCSSERIAL, &serial_struct) < 0)
		{
			printf("init_serial %d: path=%s %s\n", __LINE__, path, strerror(errno));
		}
	}
#endif // !__clang__
/*
 * printf("init_serial: %d path=%s iflag=0x%08x oflag=0x%08x cflag=0x%08x\n", 
 * __LINE__, 
 * path, 
 * term.c_iflag, 
 * term.c_oflag, 
 * term.c_cflag);
 */
 
	tcflush(fd, TCIOFLUSH);
	cfsetispeed(&term, baud);
	cfsetospeed(&term, baud);
//	term.c_iflag = IGNBRK;
	term.c_iflag = 0;
	term.c_oflag = 0;
	term.c_lflag = 0;
//	term.c_cflag &= ~(PARENB | PARODD | CRTSCTS | CSTOPB | CSIZE);
//	term.c_cflag |= CS8;
//    term.c_cflag |= CRTSCTS; // flow control
	term.c_cc[VTIME] = 1;
	term.c_cc[VMIN] = 1;
/*
 * printf("init_serial: %d path=%s iflag=0x%08x oflag=0x%08x cflag=0x%08x\n", 
 * __LINE__, 
 * path, 
 * term.c_iflag, 
 * term.c_oflag, 
 * term.c_cflag);
 */
	if(tcsetattr(fd, TCSANOW, &term))
	{
		printf("init_serial %d: path=%s %s\n", __LINE__, path, strerror(errno));
		close(fd);
		return -1;
	}

	printf("init_serial %d: opened %s\n", __LINE__, path);
	return fd;
}

// Read a character
unsigned char read_char(int fd)
{
	unsigned char c;
	int result;

	do
	{
		result = read(fd, &c, 1);

		if(result <= 0)
		{
			printf("Unplugged\n");
			exit(0);
		}

	} while(result <= 0);
	return c;
}


uint8_t uart_in;
int print_text;
int skip_count;
void (*receive_state)();
void get_code1();

void skip_codes()
{
    skip_count--;
    if(!skip_count)
    {
        receive_state = get_code1;
    }
}

void get_code2()
{
// response to polling
    if((uart_in & 0x54) == 0x54)
    {
        receive_state = skip_codes;
        skip_count = 2;
    }
    else
// polling request
    if((uart_in & 0xa8) == 0xa8)
    {
        receive_state = skip_codes;
        skip_count = 1;
    }
    else
    if(uart_in == 0xff)
        receive_state = get_code2;
    else
    {
        print_text = 1;
        receive_state = get_code1;
    }
}

void get_code1()
{
    if(uart_in == 0xff)
        receive_state = get_code2;
    else
        print_text = 1;
}


void main(int argc, char *argv[])
{
	int serial_fd = -1;
	if(serial_fd < 0) serial_fd = init_serial("/dev/ttyUSB0", B115200, 0);
	if(serial_fd < 0) serial_fd = init_serial("/dev/ttyUSB1", B115200, 0);
	if(serial_fd < 0) serial_fd = init_serial("/dev/ttyUSB2", B115200, 0);
	if(serial_fd  < 0)
	{
		printf("Couldn't open the UART\n");
        return;
	}
    
    receive_state = get_code1;
    
    while(1)
    {
        uart_in = read_char(serial_fd);
        print_text = 0;
        receive_state();
        if(print_text)
            printf("%c", uart_in);
//        else
//            printf("%02x ", uart_in);
//        else
//            printf(".");
        fflush(stdout);
    }
    
    
    
}







