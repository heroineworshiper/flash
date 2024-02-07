/*
 * MANUAL TO ETTL FLASH CONVERSION
 * Copyright (C) 2023 Adam Williams <broadcast at earthling dot net>
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

// converts the PPM file to a C struct for the LCD
// the PPM must be ASCII WIDTHxHEIGHT 24 bit color
// the output is LSB left pixel, 1 bit per pixel

// gcc -o ppmtolcd ppmtolcd.c
// ./ppmtolcd lcd3.ppm


#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define WIDTH 216
#define HEIGHT 8
#define TEXTLEN 1024
#define MAX_DATA (WIDTH * HEIGHT * 3)

void main(int argc, char *argv[])
{

	if(argc < 2)
	{
		printf("Need a filename\n");
		exit(1);
	}


	FILE *fd = fopen(argv[1], "r");
	char string[TEXTLEN];
	int skip = 4;
	int data[MAX_DATA];
	int data_size = 0;
	while(!feof(fd))
	{
		if(!fgets(string, TEXTLEN, fd)) break;
		
		char *ptr1 = string;
		while(*ptr1 != 0)
		{
// got a comment line
			if(*ptr1 == '#')
			{
				break;
			}
			
			if(*ptr1 != '\n' && *ptr1 != ' ')
			{
// got alphanumeric data.
				if(skip > 0) skip--;
				break;
			}
			
			ptr1++;
		}
		
		if(skip == 0 && data_size < MAX_DATA)
		{
			data[data_size++] = atoi(ptr1);
//			printf("%s %d: %d\n", string, data_size, atoi(ptr1));
		}
		
		if(data_size >= MAX_DATA)
		{
			break;
		}
	}


//	printf("data_size=%d\n", data_size);

 	int i;
//     for(i = 0; i < 8 * 3; i++)
//     {
//         printf("%02x ", data[i]);
//     }
//     printf("\n");

	int bytes = WIDTH / 8 * HEIGHT;
	unsigned char output[bytes];
	int *input = data;
	for(i = 0; i < bytes; i++)
	{
		int j;
		unsigned char code = 0;
		for(j = 0; j < 8; j++)
		{
			if(*input >= 0x80)
			{
				code |= 0x1 << j;
			}
			input += 3;
		}
		output[i] = code;
	}



	for(i = 0; i < bytes; i++)
	{
		if(i > 0 && !(i % 18) && i < bytes - 1)
		{
			printf("\n");
		}
		printf("0x%02x", output[i]);
		if(i < bytes - 1)
		{
			printf(", ");
		}
	}
	printf("\n");
}



