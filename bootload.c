/*
 * SINGLE WIRE PIC BOOTLOADER
 * Copyright (C) 2022 Adam Williams <broadcast at earthling dot net>
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

// program bootloader.X over a single wire UART
// make bootload



#include <ctype.h>
#include <termios.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <semaphore.h>
#include <linux/serial.h>

// start of the program
// don't write anything below this
#define START 0x800

// for the 18f14k50, we have
// 0x4000 bytes
// 64 byte erase blocks
// 16 byte write blocks
#define WRITE_BLOCK 16
#define FLASH_SIZE 0x4000

int serial_fd;
uint16_t data[65536];
// Top address in bytes
int max_address = 0;
int min_address = 0xffff;
int size = 0;

#define FIFO_SIZE 65536
typedef struct
{
	unsigned char data[FIFO_SIZE];
	int size;
// Next position to write
	int in_position;
// Next position to read
	int out_position;
	pthread_mutex_t lock;
	sem_t data_ready;
} fifo_t;
fifo_t fifo;

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

// control data direction with the DTR pin
// 0 enables the transmit pin
int set_dtr(int value)
{
    int command;
    const int dtr = TIOCM_DTR;
    if (value)
    {
        command = TIOCMBIC;
    }
    else
    {
        command = TIOCMBIS;
    }
    return ioctl(serial_fd, command, &dtr);
}

#define serial_rx() set_dtr(1)
#define serial_tx() set_dtr(0)

unsigned char read_serial()
{
	unsigned char c;
	int result;

	do
	{
		result = read(serial_fd, &c, 1);

		if(result <= 0)
		{
			printf("Unplugged\n");
			exit(1);
		}

	} while(result <= 0);
	return c;
}


void init_fifo(fifo_t *fifo)
{
	bzero(fifo, sizeof(fifo_t));

	pthread_mutexattr_t mutex_attr;
	pthread_mutexattr_init(&mutex_attr);
	pthread_mutex_init(&fifo->lock, &mutex_attr);
	sem_init(&fifo->data_ready, 0, 0);
}


void write_fifo(fifo_t *fifo, unsigned char c)
{
	int data_offset = 0;

	pthread_mutex_lock(&fifo->lock);
    if(fifo->size < FIFO_SIZE)
    {
    	fifo->data[fifo->in_position++] = c;
        if(fifo->in_position >= FIFO_SIZE)
            fifo->in_position = 0;
        fifo->size++;
	}
	pthread_mutex_unlock(&fifo->lock);
	
	sem_post(&fifo->data_ready);
}

uint8_t read_fifo(fifo_t *fifo)
{
    while(1)
    {
        sem_wait(&fifo->data_ready);
        pthread_mutex_lock(&fifo->lock);
        if(fifo->size > 0)
        {
            uint8_t result = fifo->data[fifo->out_position++];
            if(fifo->out_position >= FIFO_SIZE)
                fifo->out_position = 0;
            fifo->size--;
            pthread_mutex_unlock(&fifo->lock);
            return result;
        }
        pthread_mutex_unlock(&fifo->lock);
    }
    
    return 0;
}

void flush_fifo(fifo_t *fifo, int size)
{
    int i;
    for(i = 0; i < size; i++)
        read_fifo(fifo);
}



void write_serial(uint8_t *text, int size)
{
    if(size < 0)
    {
        size = strlen(text);
        printf("%s", text);
    }

    serial_tx();
    write(serial_fd, text, size);

// read back all characters
    flush_fifo(&fifo, size);

    serial_rx();
}

void wait_code(char *key)
{
    int key_size = strlen(key);
    int offset = 0;
    while(1)
    {
        char c = read_fifo(&fifo);
        printf("%c", c);
        
        if(c == key[offset])
        {
            offset++;
            if(offset >= key_size)
            {
                offset = 0;
                break;
            }
        }
        else
        if(c == key[0])
        {
            offset = 1;
        }
        else
        {
            offset = 0;
        }
    }
}


int read_hex(char *path)
{
    FILE *in = fopen(path, "r");
	if(!in)
	{
		printf("Couldn't open %s\n", path);
		exit(1);
	}

	int page = 0;
    char string[1024];
    int j;

    memset(data, 0xff, sizeof(data));
	while(fgets(string, 1024, in))
	{
// Convert string to binary
		unsigned char string2[1024];
		unsigned char *in_ptr = (unsigned char*)string + 1;
		unsigned char *out_ptr = string2;
		for(j = 0; j < strlen(string); j += 2)
		{
			int character = toupper(*in_ptr++);
			if(character >= '0' && character <= '9')
				*out_ptr = (character - '0') << 4;
			else
				*out_ptr = (10 + character - 'A') << 4;

			character = toupper(*in_ptr++);
			if(character >= '0' && character <= '9')
				*out_ptr |= character - '0';
			else
				*out_ptr |= 10 + character - 'A';
			out_ptr++;
		}


		unsigned char *ptr = string2;

// Number of bytes of data in the line
		int data_bytes = *ptr++;
		if(!data_bytes) break;

// Starting address of data
		int address = (*ptr++) << 8;
		address |= *ptr++;

		int type = *ptr++;

// Data is the number of a page
// TODO: handle this properly so it doesn't clobber the lower addresses
		if(type == 4)
		{
			page = (*ptr++) << 8;
			page |= *ptr++;
			if(page != 0)
			{
				printf("page = %d  Don't know what to do.\n", page);
			}
		}
		else
// Data is program data
		{
			if(min_address > address)
				min_address = address;
			for(j = 0; j < data_bytes; j += 2)
			{
				uint16_t word = *ptr++;
				word |= (*ptr++) << 8;
				data[address >> 1] = word;
				address += 2;
			}
			if(max_address < address)
				max_address = address;
		}

// Checksum
		ptr++;
	}

// round to the nearest write block
    if((max_address % WRITE_BLOCK) > 0)
    {
        max_address += WRITE_BLOCK - (max_address % WRITE_BLOCK);
    }
    size = max_address - START;

// Dump the data
// 	for(j = 0x0; j < max_address; j += 16)
// 	{
// 		printf("%08x: %04x %04x %04x %04x %04x %04x %04x %04x\n", 
// 			j,
// 			data[j / 2 + 0],
// 			data[j / 2 + 1],
// 			data[j / 2 + 2],
// 			data[j / 2 + 3],
// 			data[j / 2 + 4],
// 			data[j / 2 + 5],
// 			data[j / 2 + 6],
// 			data[j / 2 + 7]);
// 	}
}



void reader(void *ptr)
{
    while(1)
    {
        uint8_t c = read_serial();
        write_fifo(&fifo, c);
    }
}

void do_read(int start, int size)
{
    uint8_t command[16];
    int i;

// read it back
    printf("Reading %d bytes from 0x%x\n", size, start);
    command[0] = 'R';
    command[1] = 'E';
    command[2] = 'A';
    command[3] = 'D';
    command[4] = start & 0xff;
    command[5] = (start >> 8) & 0xff;
    command[6] = size & 0xff;
    command[7] = (size >> 8) & 0xff;
    write_serial(command, 8);

    for(i = 0; i < size; i++)
    {
        uint8_t c = read_fifo(&fifo);
        if((i % 16) == 0) printf("%04x: ", i + start);
        printf("%02x ", c);
        if((i % 16) == 15) printf("\n");
    }
    printf("\n");
}

void do_write()
{
    uint8_t command[16];
    int i;

    printf("Writing %d bytes to 0x%x ", size, START);
    command[0] = 'W';
    command[1] = 'R';
    command[2] = 'I';
    command[3] = 'T';
    command[4] = START & 0xff;
    command[5] = (START >> 8) & 0xff;
    command[6] = size & 0xff;
    command[7] = (size >> 8) & 0xff;
    write_serial(command, 8);


    for(i = 0; i < size; i += WRITE_BLOCK)
    {
        wait_code("READY.\n");
        printf("Writing %d bytes to 0x%x ", WRITE_BLOCK, START + i);
        memcpy(command, &data[START / 2 + i / 2], WRITE_BLOCK);
        write_serial(command, WRITE_BLOCK);
    }

    wait_code("READY.\n");
}

void do_quit()
{
    uint8_t command[16];

    printf("Starting program\n");

    command[0] = 'Q';
    command[1] = 'U';
    command[2] = 'I';
    command[3] = 'T';
    command[4] = 0;
    command[5] = 0;
    command[6] = 0;
    command[7] = 0;
    write_serial(command, 8);
}

void main(int argc, char *argv[])
{
    if(argc < 2)
    {
        printf("Program the PIC over a single wire bootloader\n");
        printf("Usage: \n    bootload /dev/ttyUSB1 -w <hex file>     write a program\n");
        printf("    bootload /dev/ttyUSB1 -r     read the flash\n");
        printf("    bootload /dev/ttyUSB1        dump the UART\n");
        return;
    }


    char *path = argv[1];
    char *hex_path = 0;
    int do_read_ = 0;
    int do_write_ = 0;

    if(argc > 2 && !strcmp(argv[2], "-w"))
    {
        hex_path = argv[3];
        do_write_ = 1;
    }

    if(argc > 2 && !strcmp(argv[2], "-r"))
    {
        do_read_ = 1;
    }


    if(hex_path) read_hex(hex_path);



    serial_fd = init_serial(path, B115200, 0);
    
    if(serial_fd < 0)
    {
        return;
    }

    init_fifo(&fifo);
	pthread_attr_t  attr;
	pthread_attr_init(&attr);
	pthread_t tid;
	pthread_create(&tid, 
		&attr, 
		(void*)reader, 
		0);

    

    serial_rx();

    if(do_write_ || do_read_)
    {
// wait for the start code
        wait_code(
            "    **** COMMODORE 64 BASIC V2 ****\n\n"
            " 64K RAM SYSTEM  38911 BASIC BYTES FREE\n\n"
            "READY.\n");

        write_serial("LOAD\n", -1);

        wait_code("PRESS PLAY ON TAPE\n");
    }

    if(do_write_) do_write();

    if(do_read_) do_read(0, FLASH_SIZE);
//    do_read(START, size);


    if(do_write_ || do_read_) do_quit();

    while(1)
    {
        uint8_t c = read_fifo(&fifo);
        printf("%c", c);
    }
}



