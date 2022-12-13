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

// This uses a single wire connected to PIC TX & RX.  When connecting to radios,
// we normally want to transmit & receive by connecting TX & RX to a modulation
// pin on the radio chip, which leaves only wire for the UART.
// The bootloader splits it into TX & RX on an FTDI dongle.  
// It requires a switch to disconnect FTDI TX when receiving.
// This logic board uses a 74HCT125 to disconnect FTDI TX.
// Lowering DTR enables FTDI TX.  Raising DTR disables FTDI TX.


//                               --------- FTDI DTR               
//                              |                                 
//                              O/|                               
//                              / |                               
// PIC TX -----            ----<  |------- FTDI TX                
//             |          |     \ |                               
//             *----------*      \|                               
//             |          |                                       
// PIC RX -----            --------------- FTDI RX                


// It uses bootload.c to program the PIC.
// The XC8 linker argument --codeoffset=0x800 is required to build 
// the user program.


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <xc.h>
#include <pic18f14k50.h>



// CONFIG1H
//#pragma config FOSC = IRCCLKOUT // Oscillator Selection bits (Internal RC oscillator, CLKOUT function on OSC2)
#pragma config FOSC = IRC // Oscillator Selection bits (Internal RC oscillator, GPIO on OSC2)
#pragma config PLLEN = ON      // 4 X PLL Enable bit

// CONFIG2L
#pragma config PWRTEN = ON     // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 22        // Brown-out Reset Voltage bits (VBOR set to 2.2 V nominal)

// CONFIG2H
#pragma config WDTEN = ON       // Watchdog Timer Enable bit (WDT is always enabled. SWDTEN bit has no effect.)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config HFOFST = OFF     // HFINTOSC Fast Start-up bit (The system clock is held off until the HFINTOSC is stable.)

// CONFIG4L
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)


#define CLOCKSPEED 32000000
// a value which doesn't require reloading timer 2
// This is based on the amount of time needed to delay in serial_tx
#define HZ 122
#define BAUD 115200
#define BOOT_DELAY (HZ)
#define LED_LAT LATC7
#define LED_TRIS TRISC7

uint8_t tick = 0;
uint8_t offset = 0;
uint8_t command[8];
uint16_t address;
uint16_t size;


void test_main();

void flush_serial()
{
    while(!TXIF)
    {
    }

    while(!TRMT)
    {
    }
}

uint8_t read_serial()
{
    while(!RCIF)
    {
        ClrWdt();
    }
    
    RCIF = 0;
    uint8_t c = RCREG;
    return c;
}

void write_serial(uint8_t value)
{
    while(!TXIF)
    {
    }

    TXREG = value;    
}

void write_text(const uint8_t *text)
{
    while(*text != 0)
    {
        write_serial(*text);
        text++;
    }
}

void write_ready()
{
    write_text("READY.\n");
}

void delay(uint8_t ticks)
{
    while(ticks > 0)
    {
        ClrWdt();
        
        if(TMR2IF)
		{
            TMR2IF = 0;
            ticks--;
        }
    }
}

// serial port transmit mode
void serial_tx()
{
// wait for FTDI to change mode.  This depends on the USB dongle's latency.
    delay(HZ / 24);
    TXSTA = 0b00100100;
    RCSTA = 0b10000000;
}

// serial port receive mode
void serial_rx()
{
    flush_serial();
    TXSTA = 0b00000100;
    RCSTA = 0b10010000;
}

void do_read()
{
    uint16_t i;

    _LOAD_TBLPTR(address);
    serial_tx();
    for(i = 0; i < size; i++)
    {
// read post increment
        asm("tblrd *+");
        write_serial(TABLAT);
        ClrWdt();
    }
    serial_rx();
}

void do_write()
{
    uint16_t i;
    uint8_t j, k;

// don't overwrite the bootloader
    if(address < 0x800)
    {
        serial_tx();
        write_text("?ILLEGAL QUANTITY\n");
        serial_rx();
        
        while(1)
        {
            ClrWdt();
        }
    }

// for the 18f14k50, we have
// 64 byte erase blocks
// 16 byte write blocks
    for(i = 0; i < size; )
    {
        _LOAD_TBLPTR(address);
        EEPGD = 1; // direct access to code memory
        CFGS = 0; 
        WREN = 1; // enable write
        FREE = 1; // enable erase
        EECON2 = 0x55;
        EECON2 = 0xaa;
        EECON1bits.WR = 1; // erase it
        
//        asm("tblrd *-"); // dummy read to decriment pointer
        
        for(j = 0; j < 4 && i < size; j++)
        {
            serial_tx();
            write_ready();
            serial_rx();
// TBLPTR is getting corrupted somehow
            _LOAD_TBLPTR(address - 1);
            for(k = 0; k < 16; k++)
            {
                TABLAT = read_serial();
                asm("tblwt +*");  // table write with pre-increment
	            i++;
            }
            
//            EEPGD = 1; // direct access to code memory
//            CFGS = 0; 
//            WREN = 1; // enable write
//            FREE = 0; // disable erase
            EECON2 = 0x55;
            EECON2 = 0xaa;
            EECON1bits.WR = 1; // write it
            address += 16;
        }
    }

    WREN = 0; // disable write
    serial_tx();
    write_ready();
    serial_rx();
}

void start_program()
{
    serial_tx();
    write_text("LOAD\"*\",8,1\n");
    serial_rx();
    asm("goto 0x800");
}

void main()
{
    OSCCON = 0b11100000;
// digital mode
    ANSEL = 0b00000000;
    ANSELH = 0b00000000;

// mane timer
// 16:1 prescale 16:1 postscale
    T2CON = 0b01111111;

    serial_tx();
    BAUDCON = 0b00001000;
// baud = clockspeed / (4 * (SPBRG + 1))
    SPBRG = CLOCKSPEED / 4 / BAUD - 1;

    write_text("\n\n\n\n    **** COMMODORE 64 BASIC V2 ****\n\n"
        " 64K RAM SYSTEM  38911 BASIC BYTES FREE\n\n");
    write_ready();
    serial_rx();


// wait a while
    while(1)
    {
        ClrWdt();
        
        if(TMR2IF)
		{
            TMR2IF = 0;
            tick++;
            if(tick >= BOOT_DELAY)
            {
// jump to program
                start_program();
            }
        }
        
        if(RCIF)
        {
            const uint8_t key[] = "LOAD\n";
            uint8_t c = RCREG;
            RCIF = 0;

            if(c == key[offset])
            {
                offset++;

                if(offset >= sizeof(key) - 1)
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

    serial_tx();
    write_text("PRESS PLAY ON TAPE\n");
    serial_rx();

// process commands
    while(1)
    {
        uint8_t c = read_serial();
        command[offset++] = c;
        if(offset >= 8)
        {
// LED_LAT = 1;
// LED_TRIS = 0;
            offset = 0;
            address = command[4] | ((uint16_t)command[5] << 8);
            size = command[6] | ((uint16_t)command[7] << 8);

//             serial_tx();
//             write_text("MAIN\n");
//             serial_rx();

// use text commands to prevent an accidental write
            if(command[0] == 'R' && 
                command[1] == 'E' &&
                command[2] == 'A' &&
                command[3] == 'D')
                do_read();
            else
            if(command[0] == 'W' &&
                command[1] == 'R' &&
                command[2] == 'I' &&
                command[3] == 'T')
                do_write();
            else
            if(command[0] == 'Q' &&
                command[1] == 'U' &&
                command[2] == 'I' &&
                command[3] == 'T')
                start_program();
        }
    }
}


void __interrupt(high_priority) isr()
{
    asm("goto 0x808");
//    test_main();
}

void __interrupt(low_priority) isr1()
{
    asm("goto 0x818");
}

// test functions
// void test_main() @ 0x400
// {
//     serial_tx();
//     write_serial('T');
//     write_serial('E');
//     write_serial('S');
//     write_serial('T');
//     write_serial(' ');
//     write_serial('M');
//     write_serial('A');
//     write_serial('I');
//     write_serial('N');
//     write_serial('\n');
//     serial_rx();
//     while(1)
//     {
//         ClrWdt();
//     }
// }














