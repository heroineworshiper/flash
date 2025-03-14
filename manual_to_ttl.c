/*
 * MANUAL TO ETTL FLASH CONVERSION
 * Copyright (C) 2023-2025 Adam Williams <broadcast at earthling dot net>
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

// Runs on an atmega328 to convert a manual flash to ETTL.
// Connects to the 6V, GND, trigger, & power of the flash.
// Receives commands from an ETTL to wireless transmitter wireless.bin.
// The radios can be replaced with a software FIFO to go directly from an ETTL
// hotshoe, but the transmitter side requires a much faster microcontroller.  

// make manual_to_ttl.hex
// make flash_fuse
// make flash_isp

#define F_CPU 8000000L

#include <inttypes.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "si4421.h"

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitToggle(value, bit) ((value) ^= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define CLAMP(x, y, z) ((x) = ((x) < (y) ? (y) : ((x) > (z) ? (z) : (x))))
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define ABS(x) ((x) < 0 ? (-(x)) : (x))

#define HZ 125
#define BAUD 200000L
#define UART_SIZE 64
uint8_t uart_buffer[UART_SIZE];
uint8_t uart_used = 0;
uint8_t uart_write_ptr = 0;
uint8_t uart_read_ptr = 0;

volatile uint8_t have_uart_in = 0;
volatile uint8_t uart_in = 0;

// handle radio recieve
void get_start1();
void (*handle_rx)() = get_start1;
uint8_t rx_counter = 0;
uint8_t packet_id = 0;
uint8_t packet_type;
// preflash power from radio packet
uint8_t preflash_power = 0;
// mane flash power from radio packet
uint8_t flash_power = 0;
// power used from 0-MAX_POWER
uint8_t power_used = 0xff;
uint8_t radio_packet[RADIO_PACKETSIZE * 2];

// RADIO_PACKETSIZE * 2
const uint8_t salt[] = 
{
    0xd0, 0xf1, 0x09, 0xa7, 0xcf, 0x89, 0x44, 0x40, 
    0xb1, 0x4c, 0x98, 0x41, 0xbc, 0xd7, 0xd1, 0x32,
    0x61, 0x8c, 0xcd, 0xa7
};

// metadata offset of offset computed from wireless.c: var_offsets
#define PREFLASH1_POWER (1 + 2)
#define MANE_FLASH1_SPEED (4 + 2)
#define MANE_FLASH1_POWER (3 + 2)


#define TOTAL_CHANNELS 4
#define MAX_POWER 7
// camera power per stop
#define CAM_PER_STOP 8
#define EXPOSURE_MIN -3
#define EXPOSURE_MAX 3
// user settings
uint8_t channel = 1;
// number of stops.  Experimentally determined by testing
// multiple apertures & ISOs
int8_t exposure_value = 2;
// 0-MAX_POWER
uint8_t manual_power = 0;
// 3 modes to work around slow recycling
#define NO_TTL 0 // manual mode
#define TTL_PREFLASH 1 // fire only preflash
#define TTL_MANEFLASH 2 // fire only mane flash using the previous preflash
// fire the preflash & the mane flash in time
#define TTL_FULL 3
uint8_t ttl_mode = TTL_PREFLASH;
//uint8_t ttl_mode = TTL_FULL;

#define RADIO_CS_GPIO PORTD
#define RADIO_CS_DDR DDRD
#define RADIO_CS_PIN 2
#define RADIO_SDO_GPIO PORTD
#define RADIO_SDO_DDR DDRD
#define RADIO_SDO_PIN 3
#define RADIO_CLK_GPIO PORTD
#define RADIO_CLK_DDR DDRD
#define RADIO_CLK_PIN 4

// AVCC must be connected for port C to work
#define LED_GPIO PORTC
#define LED_PIN 5
#define LED_DDR DDRC
#define LED2_GPIO PORTB
#define LED2_PIN 2
#define LED2_DDR DDRB

// SCR gate
#define X_PORT PORTC
#define X_PIN 1
#define X_DDR DDRC

// quench high voltage
#define POWER_PORT PORTC
#define POWER_PIN 2
#define POWER_DDR DDRC

#define LCD_CS_GPIO PORTB
#define LCD_CS_PIN 2
#define LCD_CS_DDR DDRB

#define LCD_MOSI_GPIO PORTB
#define LCD_MOSI_PIN 3
#define LCD_MOSI_DDR DDRB

#define LCD_CLK_GPIO PORTB
#define LCD_CLK_PIN 5
#define LCD_CLK_DDR DDRB

#define MODE_OUT PORTD
#define MODE_IN PIND
#define MODE_PIN 5
#define UP_OUT PORTD
#define UP_IN PIND
#define UP_PIN 6
#define DOWN_OUT PORTD
#define DOWN_IN PIND
#define DOWN_PIN 7
#define CHANNEL_OUT PORTB
#define CHANNEL_IN PINB
#define CHANNEL_PIN 7


#define TAP_TICKS (HZ / 20)
#define LONG_TICKS (HZ / 2)
#define REPEAT_TICKS (LONG_TICKS + HZ / 4)
#define BUTTON_MASK 0xf0
uint8_t button_state = 0;
uint16_t button_tick;

#define LED_TIMEOUT HZ
uint8_t led_tick = 0;

// uS values from lowest to highest power, 1 stop per entry
const int16_t delay_values[] = 
{
    0, 24, 32, 50, 70, 100, 170, 300
};

// Display driver for the LS013b7dh05
// based on 
// https://github.com/adafruit/Adafruit_SHARP_Memory_Display/blob/master/Adafruit_SharpMem.cpp
#define WIDTH 144
#define HEIGHT 168
#define WRITECMD   (0x01)
#define VCOM	   (0x02)
#define CLEAR	   (0x04)
uint8_t vcom = VCOM;


// created by ./ppmtolcd lcd.ppm
const uint8_t lcd_data[] = 
{
0x1e, 0x63, 0x0c, 0x33, 0x00, 0xbf, 0xdf, 0xe0, 0x0e, 0xbf, 0xdf, 0xe0, 0x07, 0x1e, 0x0c, 0x1e, 0x1e, 0x30, 
0x3f, 0x1e, 0x3f, 0x1e, 0x1e, 0x00, 0x00, 0x18, 0x06, 0x33, 0x77, 0x1e, 0x37, 0x00, 0x0c, 0xc6, 0x20, 0x09, 
0x0c, 0xc6, 0x20, 0x0c, 0x33, 0x0c, 0x33, 0x33, 0x38, 0x03, 0x33, 0x33, 0x33, 0x33, 0x18, 0x00, 0x0c, 0x0c, 
0x03, 0x7f, 0x33, 0x3f, 0x00, 0x0c, 0xc6, 0x20, 0x08, 0x0c, 0xc6, 0xa0, 0x09, 0x3b, 0x0e, 0x30, 0x30, 0x3c, 
0x1f, 0x03, 0x18, 0x33, 0x33, 0x18, 0x00, 0x06, 0x18, 0x03, 0x6b, 0x3f, 0x3f, 0x00, 0x0c, 0xc6, 0xa0, 0x0a, 
0x0c, 0xc6, 0x20, 0x0c, 0x37, 0x0c, 0x18, 0x1c, 0x33, 0x30, 0x1f, 0x0c, 0x1e, 0x3e, 0x7e, 0x7e, 0x06, 0x18, 
0x03, 0x63, 0x33, 0x3b, 0x00, 0x0c, 0xc6, 0xa0, 0x0b, 0x0c, 0xc6, 0xa0, 0x07, 0x33, 0x0c, 0x06, 0x30, 0x7f, 
0x30, 0x33, 0x0c, 0x33, 0x30, 0x18, 0x00, 0x06, 0x18, 0x33, 0x63, 0x33, 0x33, 0x00, 0x0c, 0xc6, 0xa0, 0x0a, 
0x0c, 0xc6, 0xa0, 0x00, 0x33, 0x0c, 0x03, 0x33, 0x30, 0x33, 0x33, 0x0c, 0x33, 0x33, 0x18, 0x00, 0x0c, 0x0c, 
0x1e, 0x63, 0x33, 0x33, 0x00, 0x0c, 0xc6, 0xaf, 0x0a, 0x0c, 0xc6, 0xaf, 0x00, 0x1e, 0x3f, 0x3f, 0x1e, 0x30, 
0x1e, 0x1e, 0x0c, 0x1e, 0x1e, 0x00, 0x00, 0x18, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x0e, 
0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
#define PPM_W 216
// input coords in lcd_data
#define CHAN_X 0
#define MANUAL_X 8
// TTL with preflash
#define TTLPRE_X 72
// TTL with mane flash
#define TTLMANE_X 40 
#define DIGIT_X 104
#define PLUS_X 184
#define MINUS_X 192
#define LPARENTH_X 200
#define RPARENTH_X 208
// output coords on LCD
// character sizes
#define QUAD_W 32
#define QUAD_H 32
#define OCTO_W 64
#define OCTO_H 64
#define MODE_CHARS 4
#define BUTTON_X (WIDTH - QUAD_W + 4)
#define C_Y 0
#define M_Y QUAD_H
#define PLUS_Y (HEIGHT - QUAD_H * 2)
#define MINUS_Y (HEIGHT - QUAD_H)
#define CURRENT_CHAN_X (QUAD_W / 2)
#define CURRENT_MODE_X 1
#define USED_X (QUAD_W / 2)
#define USED_Y (QUAD_H * 2)
#define DIGIT1_X 0
#define DIGIT1_Y (HEIGHT - OCTO_H)
#define DIGIT2_X OCTO_W
#define DIGIT2_Y (HEIGHT - OCTO_H)


// send data to the UART buffer
void send_uart(uint8_t *text, uint8_t size)
{
	uint8_t i;
	for(i = 0; uart_used < UART_SIZE && i < size; i++)
	{
		uart_buffer[uart_write_ptr++] = text[i];
		uart_used++;
		if(uart_write_ptr >= UART_SIZE) uart_write_ptr = 0;
	}
}


// send data to the debug buffer
void print_text(char *text)
{
    char *ptr = text;
    while(*ptr != 0) ptr++;
	send_uart(text, ptr - text);
}

void print_hex2(uint8_t x)
{
    char string[4];
    const char table[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
        'a', 'b', 'c', 'd', 'e', 'f' };
    string[0] = table[(x >> 4) & 0xf];
    string[1] = table[x & 0xf];
    string[2] = ' ';
    string[3] = 0;
    send_uart(string, 3);
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

void init_serial()
{
	uint16_t baud_setting = (F_CPU / 4 / BAUD - 1) / 2;
	UBRR0H = baud_setting >> 8;
	UBRR0L = baud_setting & 0xff;
	UCSR0A = (1 << U2X0);
	UCSR0C = (1 << UCSZ01) |
		(1 << UCSZ00);
	UCSR0B = (1 << RXCIE0) |
		(1 << RXEN0) |
		(1 << TXEN0);


    uart_read_ptr = 0;
}

// write radio SPI
void write_radio_spi(uint16_t data)
{
// print_text("write_spi ");
// print_hex(data);
// print_lf();
    bitClear(RADIO_CS_GPIO, RADIO_CS_PIN);

    int i;
    for(i = 0; i < 16; i++)
    {
        if(data & 0x8000)
        {
            bitSet(RADIO_SDO_GPIO, RADIO_SDO_PIN);
        }
        else
        {
            bitClear(RADIO_SDO_GPIO, RADIO_SDO_PIN);
        }
        data <<= 1;
// need the delay if the system clock is over 16Mhz
//        usleep(1);
        bitSet(RADIO_CLK_GPIO, RADIO_CLK_PIN);
//        usleep(1);
        bitClear(RADIO_CLK_GPIO, RADIO_CLK_PIN);
    }
    
    bitSet(RADIO_CS_GPIO, RADIO_CS_PIN);
}

void set_channel()
{
    uint32_t channel2 = (uint32_t)channel * (MAX_CHANNEL - MIN_CHANNEL) / 4 + MIN_CHANNEL;
    write_radio_spi(CFSREG(channel2));
}

void init_radio()
{
// enable outputs
    bitSet(RADIO_CS_DDR, RADIO_CS_PIN);
    bitSet(RADIO_CLK_DDR, RADIO_CLK_PIN);
    bitSet(RADIO_SDO_DDR, RADIO_SDO_PIN);
    

// scan for synchronous code
    write_radio_spi(FIFORSTREG);
// enable synchron latch
    write_radio_spi(FIFORSTREG | 0x0002);
    write_radio_spi(GENCREG);
    write_radio_spi(AFCCREG);
    set_channel();
    write_radio_spi(DRVSREG);
    write_radio_spi(PMCREG);
    write_radio_spi(RXCREG);
    write_radio_spi(TXCREG);
    write_radio_spi(BBFCREG);
// receiver on
    write_radio_spi(PMCREG | 0x0080);
}

#define WRITE_SPI(x) \
{ \
/* write.  page 169 */ \
    SPDR = (x); \
/* wait.  page 168 */ \
    while(!bitRead(SPSR, SPIF)) {} \
}

#define WAIT_SPI while(!bitRead(SPSR, SPIF)) {}

// receive interrupt
ISR(USART_RX_vect)
{
	uart_in = UDR0;
	have_uart_in = 1;
}



// void load_settings()
// {
// // EEPROM address
//     EEARH = 0;
//     EEARL = 0;
// // read command
//     bitSet(EECR, EERE);
//     channel = EEDR;
// 
//     EEARL = 1;
//     bitSet(EECR, EERE);
//     exposure_value = EEDR;
// 
//     EEARL = 2;
//     bitSet(EECR, EERE);
//     manual_power = EEDR;
// 
//     EEARL = 3;
//     bitSet(EECR, EERE);
//     ttl_mode = EEDR;
//     switch(ttl_mode) 
//     {
//         case NO_TTL:
//         case TTL_PREFLASH:
//             break;
//     
//         case TTL_MANEFLASH:
//             ttl_mode = TTL_PREFLASH;
//             break;
//     
//         default:
//             ttl_mode = TTL_PREFLASH;
//     }
//     
//     CLAMP(channel, 0, TOTAL_CHANNELS - 1);
//     CLAMP(exposure_value, EXPOSURE_MIN, EXPOSURE_MAX);
//     CLAMP(manual_power, 0, MAX_POWER);
// }
// 
// void write_eeprom()
// {
// // master write enable
//     bitSet(EECR, EEMPE);
// // write enable
//     bitSet(EECR, EEPE);
// // wait for it
//     while(bitRead(EECR, EEPE))
//     {
//     }
// }
// 
// void save_settings()
// {
// // EEPROM address
//     EEARH = 0;
//     EEARL = 0;
// // data
//     EEDR = channel;
//     write_eeprom();
// 
//     EEARL = 1;
// // data
//     EEDR = exposure_value;
//     write_eeprom();
// 
//     EEARL = 2;
// // data
//     EEDR = manual_power;
//     write_eeprom();
// 
//     EEARL = 3;
// // data
//     EEDR = ttl_mode;
//     write_eeprom();
// }

// void draw_screen()
// {
// // enable SPI. page 167
//     SPCR = 0b01110100;
// // double speed. page 168
//     SPSR = 0b00000001;
// 
// // active high CS
//     bitSet(LCD_CS_GPIO, LCD_CS_PIN);
//     SPDR = WRITECMD | vcom;
// 	if(vcom)
//         vcom = 0;
//     else
//     	vcom = VCOM;
// 
//     uint8_t mode_x = MANUAL_X;
//     uint8_t digit1_x = 0xff;
//     uint8_t digit2_x = 0xff;
//     if(ttl_mode) 
//     {
//         if(ttl_mode == TTL_PREFLASH || ttl_mode == TTL_FULL)
//             mode_x = TTLPRE_X;
//         else
//             mode_x = TTLMANE_X;
//         if(exposure_value > 0) 
//         {
//             digit1_x = PLUS_X;
//             digit2_x = DIGIT_X + exposure_value * 8;
//         }
//         else
//         if(exposure_value < 0)
//         {
//             digit1_x = MINUS_X;
//             digit2_x = DIGIT_X - exposure_value * 8;
//         }
//         else
//             digit2_x = DIGIT_X;
//     }
//     else
//     {
//         if(manual_power > 9)
//             digit1_x = DIGIT_X + (manual_power - 9) * 8;
//         digit2_x = DIGIT_X + (manual_power % 10) * 8;
//     }
// 
//     uint8_t i, j;
// #define FLUSH_PIXEL \
//     value >>= 1; \
//     if(!pixel) value |= 0x80; \
//     if(!(j % 8)) \
//     { \
//         WAIT_SPI \
//         SPDR = value; \
//         value = 0xff; \
//     }
// 
//     for(i = 0; i < HEIGHT; i++)
//     {
// // address of current line
//         WAIT_SPI
//         SPDR = i + 1;
//         uint8_t value = 0xff;
// 
//         if(i >= C_Y && 
//             i < C_Y + QUAD_H)
//         {
//             for(j = 1; j < BUTTON_X - 1; j++)
//             {
//                 uint8_t pixel = 0;
//                 if(j >= CURRENT_CHAN_X && j < CURRENT_CHAN_X + QUAD_W)
//                 {
//                     uint8_t in_byte = lcd_data[(i / 4) * (PPM_W / 8) + (DIGIT_X / 8) + channel + 1];
//                     uint8_t in_bit = (j - CURRENT_CHAN_X) / 4;
//                     pixel = in_byte & (1 << in_bit);
//                 }
//                 FLUSH_PIXEL
//             }
//         }
//         else
//         if(i >= M_Y && 
//             i < M_Y + QUAD_H)
//         {
//             for(j = 1; j < BUTTON_X - 1; j++)
//             {
//                 uint8_t pixel = 0;
//                 if(j >= CURRENT_MODE_X && j < CURRENT_MODE_X + QUAD_W * MODE_CHARS)
//                 {
//                     uint8_t character = (j - CURRENT_MODE_X) / QUAD_W;
//                     uint8_t in_byte = lcd_data[((i - M_Y) / 4) * (PPM_W / 8) + (mode_x / 8) + character];
//                     uint8_t in_bit = ((j - CURRENT_MODE_X) / 4) % 8;
//                     pixel = in_byte & (1 << in_bit);
//                 }
//                 FLUSH_PIXEL
//             }
//         }
//         else
//         if(i >= USED_Y && i < USED_Y + QUAD_H && power_used != 0xff)
//         {
// // power used
//             for(j = 1; j < USED_X; j++)
//             {
//                 uint8_t pixel = 0;
//                 FLUSH_PIXEL
//             }
//             uint8_t in_byte = lcd_data[((i - USED_Y) / 4) * (PPM_W / 8) + 
//                 (LPARENTH_X / 8)];
//             for( ; j < USED_X + QUAD_W; j++)
//             {
//                 uint8_t in_bit = (j - USED_X) / 4;
//                 uint8_t pixel = in_byte & (1 << in_bit);
//                 FLUSH_PIXEL
//             }
//             in_byte = lcd_data[((i - USED_Y) / 4) * (PPM_W / 8) + 
//                 (DIGIT_X / 8) + power_used];
//             for( ; j < USED_X + QUAD_W * 2; j++)
//             {
//                 uint8_t in_bit = (j - USED_X - QUAD_W) / 4;
//                 uint8_t pixel = in_byte & (1 << in_bit);
//                 FLUSH_PIXEL
//             }
//             in_byte = lcd_data[((i - USED_Y) / 4) * (PPM_W / 8) + 
//                 (RPARENTH_X / 8)];
//             for( ; j < USED_X + QUAD_W * 3; j++)
//             {
//                 uint8_t in_bit = (j - USED_X - QUAD_W * 2) / 4;
//                 uint8_t pixel = in_byte & (1 << in_bit);
//                 FLUSH_PIXEL
//             }
//             for( ; j < BUTTON_X - 1; j++)
//             {
//                 uint8_t pixel = 0;
//                 FLUSH_PIXEL
//             }
//         }
//         else
//         if(i >= DIGIT1_Y && i < DIGIT1_Y + OCTO_H)
//         {
//             for(j = 1; j < BUTTON_X - 1; j++)
//             {
//                 uint8_t pixel = 0;
//                 if(digit1_x != 0xff && j >= DIGIT1_X && j < DIGIT1_X + OCTO_W)
//                 {
// // digit 1
//                     uint8_t in_byte = lcd_data[((i - DIGIT1_Y) / 8) * (PPM_W / 8) + (digit1_x / 8)];
//                     uint8_t in_bit = (j - DIGIT1_X) / 8;
//                     pixel = in_byte & (1 << in_bit);
//                     
//                 }
//                 else
//                 if(j >= DIGIT2_X && j < DIGIT2_X + OCTO_W)
//                 {
// // digit 2
//                     uint8_t in_byte = lcd_data[((i - DIGIT1_Y) / 8) * (PPM_W / 8) + (digit2_x / 8)];
//                     uint8_t in_bit = (j - DIGIT2_X) / 8;
//                     pixel = in_byte & (1 << in_bit);
//                 }
//                 FLUSH_PIXEL
//             }
//         }
//         else
//         {
//             for(j = 1; j < BUTTON_X - 1; j++)
//             {
//                 uint8_t pixel = 0;
//                 FLUSH_PIXEL
//             }
//         }
// 
// // vertical line.  Assume not on an 8 pixel boundary
//         value >>= 1;
//         j++;
// 
// // button column
//         for( ; j < WIDTH; j++)
//         {
//             uint8_t pixel = 0;
//             uint8_t in_byte;
// // button keys are 4x size
//             if(i >= C_Y && i < C_Y + QUAD_H)
//             {
//                 in_byte = lcd_data[((i - C_Y) / 4) * (PPM_W / 8) + (CHAN_X / 8)];
//             }
//             else
//             if(i >= M_Y && i < M_Y + QUAD_H)
//             {
//                 in_byte = lcd_data[((i - M_Y) / 4) * (PPM_W / 8) + (MANUAL_X / 8)];
//             }
//             else
//             if(i >= PLUS_Y && i < PLUS_Y + QUAD_H)
//             {
//                 in_byte = lcd_data[((i - PLUS_Y) / 4) * (PPM_W / 8) + (PLUS_X / 8)];
//             } 
//             else
//             if(i >= MINUS_Y && i < MINUS_Y + QUAD_H)
//             {
//                 in_byte = lcd_data[((i - MINUS_Y) / 4) * (PPM_W / 8) + (MINUS_X / 8)];
//             } 
//             uint8_t in_bit = (j - BUTTON_X) / 4;
//             pixel = in_byte & (1 << in_bit);
// 
//             FLUSH_PIXEL
//         }
// 
// // skip last pixel
//         value >>= 1;
//         WAIT_SPI
//         SPDR = value;
// 
// // end of line code
//         WAIT_SPI
//         SPDR = 0;
//     }
// 
// // end of frame
//     WAIT_SPI
//     SPDR = 0;
//     WAIT_SPI
//     bitClear(LCD_CS_GPIO, LCD_CS_PIN);
// }

uint8_t wait_trigger(uint8_t code)
{
    uint8_t ticks = HZ + 1;
    uint8_t counter = 0;
    while(ticks > 0)
    {
        if(bitRead(TIFR0, TOV0))
        {
            bitSet(TIFR0, TOV0);
            ticks--;
        }
        
        if(have_uart_in)
        {
            have_uart_in = 0;
            if(uart_in == code)
            {
                counter++;
                if(counter >= TRIGGER_DEBOUNCE)
                    return 0;
            }
            else
            {
                counter = 0;
            }
        }
    }
    return 1;
}

void usleep(int16_t delay)
{
    if(!delay) return;
// page 133
    TCCR1B = 0b00000010;
    delay = -delay;
// clear overflow flag page 137
    bitSet(TIFR1, TOV1);
// write timer value page 116
    TCNT1 = delay;
    while(!bitRead(TIFR1, TOV1)) {}
    
}

void tick_sleep(uint8_t delay)
{
    delay++;
    while(delay > 0)
    {
        if(bitRead(TIFR0, TOV0))
        {
            bitSet(TIFR0, TOV0);
            delay--;
        }
    }
}

void fire_it(int16_t delay)
{
//      |             |             |            |
// trigger high - power high - trigger low - power low
//      -----delay-----
//      --------- SCR_DELAY ---------
//      --------------- TOTAL_DELAY --------------           
// power to fire the preflash at
#define PREFLASH_POWER 2 // 0 - MAX_POWER
//#define PREFLASH_POWER 0 // 0 - MAX_POWER

#define SCR_DELAY 250 // us
#define TOTAL_DELAY 5000 // us
    if(delay == 0)
    {
        bitSet(X_PORT, X_PIN);
        bitSet(POWER_PORT, POWER_PIN);
        usleep(SCR_DELAY);
        bitClear(X_PORT, X_PIN);
        usleep(TOTAL_DELAY - SCR_DELAY);
    }
    else
    if(delay <= SCR_DELAY)
    {
        bitSet(X_PORT, X_PIN);
        usleep(delay);
        bitSet(POWER_PORT, POWER_PIN);
        usleep(SCR_DELAY - delay);
        bitClear(X_PORT, X_PIN);
        usleep(TOTAL_DELAY - SCR_DELAY);
    }
    else
    {
        bitSet(X_PORT, X_PIN);
        usleep(SCR_DELAY);
        bitClear(X_PORT, X_PIN);
        usleep(delay - SCR_DELAY);
        bitSet(POWER_PORT, POWER_PIN);
        usleep(TOTAL_DELAY - delay);
    }
    bitClear(POWER_PORT, POWER_PIN);
}

void get_packet()
{
    radio_packet[rx_counter] = uart_in ^ salt[rx_counter];
    rx_counter++;
    if(rx_counter >= RADIO_PACKETSIZE * 2)
    {
        uint8_t i;
        uint8_t error = 0;
        handle_rx = get_start1;
        for(i = 0; i < RADIO_PACKETSIZE; i++)
        {
            if(radio_packet[i] != radio_packet[i + RADIO_PACKETSIZE])
            {
                error = 1;
                break;
            }
        }


        if(!error && radio_packet[0] != packet_id)
        {
            int offset = 0;
            packet_id = radio_packet[offset++];
            packet_type = radio_packet[offset++];

// extract metadata
//print_text("packet_type=");
//print_hex2(packet_type);
//print_text("\n");
            if(packet_type == TYPE_POWERON ||
                packet_type == TYPE_METERING1)
            {
                bitToggle(LED_GPIO, LED_PIN);
                led_tick = 0;
            }
            else
            if(packet_type == TYPE_PREFLASH1)
            {
                if(ttl_mode == TTL_PREFLASH || ttl_mode == TTL_FULL)
                {
                    preflash_power = radio_packet[PREFLASH1_POWER];
print_text("preflash_power=");
print_hex2(preflash_power);
print_text("\n");
                }
            }
            else
            if(packet_type == TYPE_MANE_FLASH1)
            {
                if(ttl_mode == TTL_PREFLASH || ttl_mode == TTL_FULL)
                {
                    flash_power = radio_packet[MANE_FLASH1_POWER];
print_text("flash_power=");
print_hex2(flash_power);
print_text("\n");
                }
            }

// handle triggering
            if(packet_type == TYPE_PREFLASH2)
            {
// CLK only
                int16_t delay = delay_values[PREFLASH_POWER];
                if(!wait_trigger(TRIGGER_CODE_CLK))
                {
                    if(ttl_mode == TTL_PREFLASH || ttl_mode == TTL_FULL)
                    {
// use hard coded power for preflash
                        fire_it(delay);
print_text("PREFLASH delay=");
print_number(delay);
print_text("\n");
                    }
                    else
                    {
print_text("PREFLASH skipped\n");
                    }
                }
            }
            else
            if((packet_type == TYPE_MANE_FLASH1 &&
                radio_packet[MANE_FLASH1_SPEED] == 0x1d) ||
                packet_type == TYPE_FAST_FLASH ||
                packet_type == TYPE_MANUAL_FLASH)
            {
                int16_t delay;
                uint8_t skip = 0;
                int8_t power = 0;
                if(ttl_mode)
                {
// flash_power & preflash_power are only updated in the TTL_PREFLASH state
                    int8_t diff = flash_power - preflash_power;
                    power = PREFLASH_POWER + 
                        diff / CAM_PER_STOP + 
                        exposure_value;
// round it
                    if(diff > 0 && (diff % CAM_PER_STOP) >= CAM_PER_STOP / 2)
                        power++;
                    else
                    if(diff < 0 && (-diff % CAM_PER_STOP) >= CAM_PER_STOP / 2)
                        power--;
                    CLAMP(power, 0, MAX_POWER);
                    power_used = power;
                }

// now firing mane flash
                if(ttl_mode == TTL_MANEFLASH || ttl_mode == TTL_FULL)
                {
                    delay = delay_values[power];
                    if(ttl_mode != TTL_FULL) ttl_mode = TTL_PREFLASH;
                    bitClear(LED2_GPIO, LED2_PIN);
                }
                else
// now firing preflash
                if(ttl_mode == TTL_PREFLASH)
                {
                    skip = 1;
                    ttl_mode = TTL_MANEFLASH;
                    bitSet(LED2_GPIO, LED2_PIN);
                }
                else
                {
                    delay = delay_values[manual_power];
                }

                if(!wait_trigger(TRIGGER_CODE_CLK))
                {
// X
                    if(!wait_trigger(TRIGGER_CODE_X))
                    {
                        if(!skip) 
                        {
                            fire_it(delay);


print_text("MANE_FLASH delay=");
print_number(delay);
print_hex2(preflash_power);
print_hex2(flash_power);
print_text("\n");
                        }
                        else
                        {
print_text("MANE_FLASH skipped\n");
                        }

//                        if(ttl_mode) draw_screen();
                    }
                }
            }
        }
    }
}

void get_start2()
{
    if(uart_in == 0xea)
    {
        handle_rx = get_packet;
        rx_counter = 0;
    }
    else
    if(uart_in != 0xff)
        handle_rx = get_start1;
// data was 0xff
}

void get_start1()
{
    if(uart_in == 0xff)
        handle_rx = get_start2;
}



void main()
{
// disable watchdog
	WDTCSR = 0;

// radio indicator
    bitSet(LED_DDR, LED_PIN);
    bitSet(LED_GPIO, LED_PIN);

// mane flash indicator when not using LCD
    bitSet(LED2_DDR, LED2_PIN);
    bitClear(LED2_GPIO, LED2_PIN);

// rely on data direction to get a 3.9V off voltage
//    bitClear(X_PORT, X_PIN);
//    bitClear(X_DDR, X_PIN);
    bitClear(X_PORT, X_PIN);
    bitSet(X_DDR, X_PIN);

    bitClear(POWER_PORT, POWER_PIN);
    bitSet(POWER_DDR, POWER_PIN);

    bitSet(MODE_OUT, MODE_PIN);
    bitSet(UP_OUT, UP_PIN);
    bitSet(DOWN_OUT, DOWN_PIN);
    bitSet(CHANNEL_OUT, CHANNEL_PIN);


// init the LCD
//     bitClear(LCD_CS_GPIO, LCD_CS_PIN);
//     bitSet(LCD_CS_DDR, LCD_CS_PIN);
//     bitClear(LCD_CLK_GPIO, LCD_CLK_PIN);
//     bitSet(LCD_CLK_DDR, LCD_CLK_PIN);
//     bitClear(LCD_MOSI_GPIO, LCD_MOSI_PIN);
//     bitSet(LCD_MOSI_DDR, LCD_MOSI_PIN);
// 

// tick clock prescaler page 108
// CLKio is 32khz
    TCCR0B = 0b00000100;

    uint8_t delay = HZ / 2;
    while(delay > 0)
    {
// handle tick
        if(bitRead(TIFR0, TOV0))
        {
            bitSet(TIFR0, TOV0);
            delay--;
        }
    }



	init_serial();
    print_text("\n\nWelcome to manual to ETTL\n");

//    load_settings();

    init_radio();

    uint16_t i = 0;

//    draw_screen();

// enable interrupts
	sei();

	while(1)
	{
// handle tick
        if(bitRead(TIFR0, TOV0))
        {
            bitSet(TIFR0, TOV0);
            led_tick++;
            if(led_tick >= LED_TIMEOUT)
            {
                led_tick = 0;
                bitSet(LED_GPIO, LED_PIN);
            }
            
            if((button_state & BUTTON_MASK) != BUTTON_MASK)
            {
                button_tick++;
                if(button_tick == TAP_TICKS ||
                    button_tick == LONG_TICKS ||
                    button_tick == REPEAT_TICKS)
                {
                    if(button_tick == REPEAT_TICKS)
                        button_tick = LONG_TICKS;
                    if(!bitRead(CHANNEL_IN, CHANNEL_PIN))
                    {
                        channel++;
                        if(channel >= TOTAL_CHANNELS)
                            channel = 0;
                        set_channel();
//                        save_settings();
//                        draw_screen();
                    }
                    else
                    if(!bitRead(MODE_IN, MODE_PIN) &&
                        button_tick != LONG_TICKS &&
                        button_tick != REPEAT_TICKS)
                    {
                        switch(ttl_mode)
                        {
                            case NO_TTL:
                                ttl_mode = TTL_PREFLASH;
                                break;
                            case TTL_PREFLASH:
                                ttl_mode = TTL_FULL;
                                break;
                            case TTL_MANEFLASH:
                                ttl_mode = TTL_FULL;
                                break;
                            case TTL_FULL:
                                ttl_mode = NO_TTL;
                                break;
                        }
//                        save_settings();
//                        draw_screen();
                    }
                    else
                    if(!bitRead(UP_IN, UP_PIN))
                    {
                        if(ttl_mode)
                        {
                            exposure_value++;
                            if(exposure_value > EXPOSURE_MAX)
                                exposure_value = EXPOSURE_MAX;
                        }
                        else
                        {
                            manual_power++;
                            if(manual_power > MAX_POWER)
                                manual_power = 0;
                        }
//                        save_settings();
//                        draw_screen();
                    }
                    else
                    if(!bitRead(DOWN_IN, DOWN_PIN))
                    {
                        if(ttl_mode)
                        {
                            exposure_value--;
                            if(exposure_value < EXPOSURE_MIN)
                                exposure_value = EXPOSURE_MIN;
                        }
                        else
                        {
                            manual_power--;
                            if(manual_power > MAX_POWER)
                                manual_power = MAX_POWER;
                        }
//                        save_settings();
//                        draw_screen();
                    }
                }
            }
        }

	    if(uart_used) 
	    {
	        if(bitRead(UCSR0A, UDRE0)) 
	        {
			    bitSet(UCSR0A, UDRE0); 
			    UDR0 = uart_buffer[uart_read_ptr++]; 
			    if(uart_read_ptr >= UART_SIZE) uart_read_ptr = 0; 
			    uart_used--; 
	        }
	    }

        if(have_uart_in)
        {
            have_uart_in = 0;
            handle_rx();
//            print_hex2(uart_in);
        }

        uint8_t prev_button_state = button_state;
        button_state = ((PIND & 0b11100000) >> 1) | (PINB & 0b10000000);
        if(prev_button_state != button_state)
        {
            button_tick = 0;
//print_text("BUTTON PRESS ");
//print_hex2(button_state);
//print_text("\n");
        }

    }
}












