/*
 * WIRELESS ETTL FLASH CONVERSION
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

// to program it:
// ./bootload /dev/ttyUSB1 -w wireless.X/dist/default/production/wireless.X.production.hex 



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
#define HZ 122
// byte framing delay 50us
#define TIMER1_VALUE -(CLOCKSPEED / 4 / 20000)

#define LED_LAT LATC7
#define LED_TRIS TRISC7

#define BUTTON_PORT PORTA1

typedef union 
{
	struct
	{
		unsigned interrupt_complete : 1;
        unsigned got_byte : 1;
        unsigned got_clk : 1;
	};
	
	unsigned char value;
} flags_t;

flags_t flags;
uint8_t tick = 0;
uint8_t dummy = 0;

#define UART_BUFSIZE 64
uint8_t uart_buffer[UART_BUFSIZE];
uint8_t uart_size = 0;
uint8_t uart_position1 = 0;
uint8_t uart_position2 = 0;

uint8_t d1_value = 0;
uint8_t d2_value = 0;
uint8_t byte_counter = 0;

void handle_uart()
{
    if(uart_size > 0 && TXIF)
    {
        TXIF = 0;
        TXREG = uart_buffer[uart_position2++];
		uart_size--;
		if(uart_position2 >= UART_BUFSIZE)
		{
			uart_position2 = 0;
		}
    }
}

void print_byte(uint8_t c)
{
	if(uart_size < UART_BUFSIZE)
	{
		uart_buffer[uart_position1++] = c;
		uart_size++;
		if(uart_position1 >= UART_BUFSIZE)
		{
			uart_position1 = 0;
		}
	}
}

void print_text(const uint8_t *s)
{
	while(*s != 0)
	{
		print_byte(*s);
		s++;
	}
}

const uint8_t hex_table[] = 
{
	'0', '1', '2', '3', '4', '5', '6', '7', 
	'8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
};

void print_hex2(uint8_t number)
{
	print_byte(hex_table[(number >> 4) & 0xf]);
	print_byte(hex_table[number & 0xf]);
    print_byte(' ');
}

void main()
{
// keep settings from the bootloader
// LED on
    LED_LAT = 1;
    LED_TRIS = 0;
// analog mode for comparators
    ANSEL = 0b11100000;
    ANSELH = 0b00000000;

// mane timer
// 16:1 prescale 16:1 postscale
    T2CON = 0b01111111;

// byte framing timer
    T1CON = 0b10000000;

// UART transmit mode
    TXSTA = 0b00100100;
    RCSTA = 0b10000000;

// connect Vref to C2 for testing
//    REFCON1 = 0b10100000;
// Enable Vref
    REFCON1 = 0b10000000;
// Vref = 3.3 * REFCON2 / 32
    REFCON2 = 26;

// page 221
// comparator 1 CLK
    CM1CON0 = 0b10011101;
// comparator 2 D1
// page 222
    CM2CON0 = 0b10011110;
// hysteresis
    CM2CON1 = 0b00001100;
//    CM2CON1 = 0b00000000;
// Comparator interrupt doesn't work.  Might be too slow so it's always catching up.
//    C1IE = 1;


//    PEIE = 1;
//    GIE = 1;

    print_text("Welcome to wireless ETTL\n");

    while(1)
    {
        ClrWdt();
//        handle_uart();
        
//         if(TMR2IF)
//         {
//             TMR2IF = 0;
//             tick++;
//         }

// // Comparator interrupt doesn't work
//         if(C1IF)
//         {
//             C1IF = 0;
// // have to read this to reset the mismatch
//             dummy = CM1CON0;
//             LED_LAT = !LED_LAT;
//         }

//         if(flags.got_byte)
//         {
//             flags.got_byte = 0;
//             print_hex2(d1_value);
//             print_hex2(d2_value);
//             print_byte('\n');
//             d1_value = 0;
//             d2_value = 0;
//             byte_counter = 0;
//         }

//         if(C1IF)
//         {
//             C1IF = 0;
//             dummy = CM1CON0;
//             LED_LAT = !LED_LAT;
//         }

// rising CLK edge
        if(C1OUT != flags.got_clk)
        {
            flags.got_clk = C1OUT;
//            if(C1OUT)
//            {
                LED_LAT = !LED_LAT;
//            }
        }
    }
}


void __interrupt(low_priority) isr1()
{
}

void __interrupt(high_priority) isr()
{
    flags.interrupt_complete = 0;
	while(!flags.interrupt_complete)
	{
		flags.interrupt_complete = 1;


// byte framing expired
//         if(TMR1IF)
//         {
//             TMR1IF = 0;
//             TMR1IE = 0;
//             flags.interrupt_complete = 0;
//             d1_value = 0;
//             d2_value = 0;
//             byte_counter = 0;
//         }

// // Comparator interrupt is too slow
//        if(C1IF)
//        {
//            C1IF = 0;
// // have to read this to reset the mismatch
//            dummy = CM1CON0;
//            flags.interrupt_complete = 0;
// LED_LAT = !LED_LAT;
//         }
// 

//            if(C1OUT)
//            {
// rising edge
//                 d1_value <<= 1;
//                 if(C2OUT) d1_value |= 1;
// 
// // point comparator 2 to D2
//                 CM2CON0 = 0b10011111;
//                 WRITETIMER1(TIMER1_VALUE);
//                 TMR1IF = 0;
//                 TMR1IE = 1;
// 
//                 d2_value <<= 1;
//                 if(C2OUT) d2_value |= 1;
// 
// // point comparator 2 to D1
//                 CM2CON0 = 0b10011110;
//                 byte_counter++;
//                 if(byte_counter >= 8)
//                 {
//                     flags.got_byte = 1;
//                 }
//            }
//       }
    }
}










