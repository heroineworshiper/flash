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

// Loop that runs on the flash side
#include "wireless.h"
#include "linux.h"
#include "uart.h"
#include "misc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dac.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_iwdg.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_flash.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"


#ifdef SIM_CAM



// handle radio recieve
void get_start1();
void (*handle_rx)() = get_start1;
int rx_counter = 0;
uint8_t packet_id = 0;
// last exposure compensation code
int8_t exposure_code = 0;
// last focal length
uint8_t focal_length = 0;

// returns 1 if it timed out
int wait_trigger(uint8_t code)
{
// set up trigger timeout
    SET_BYTE_TIMER(TRIGGER_TIMEOUT);

    int counter = 0;
    while(!BYTE_TIMER_EXPIRED)
    {
        if(radio_size > 0)
        {
            radio_data = radio_buffer[radio_read_ptr++];
            if(radio_read_ptr >= RADIO_BUFSIZE)
                radio_read_ptr = 0;

            USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
            radio_size--;
            USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

            if(radio_data == code) 
            {
                counter++;

                if(counter >= TRIGGER_DEBOUNCE)
                    return 0;
            }
            else
                counter = 0;
        }
    }
    return 1;
}


void get_packet()
{
    radio_packet[rx_counter] = radio_data ^ salt[rx_counter];
    rx_counter++;
    if(rx_counter >= RADIO_PACKETSIZE * 2)
    {
        int i, j;
        int error = 0;
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
            sim_size = ref_packet_size[packet_type];
            const uint8_t *ref_packet = ref_packets[packet_type];
            
// initialize the simulated packet
            for(i = 0; i < sim_size; i++)
                toflash_data[i] = ref_packet[i * 2 + 1];

            sim_offset = 0;
// fill in variables
            const uint8_t *vars = var_offsets[packet_type];
            if(vars)
            {
                while(*vars != 0)
                {
                    toflash_data[*vars] = radio_packet[offset++];
                    vars++;
                }
            }

// apply exposure compensation
// & other hacks to flash power
            if(packet_type == TYPE_MANE_FLASH1)
            {
                int power_offset = 0;
                if(exposure_code > 0)
                {
                    power_offset = -exposure_code + 1;
                }
                else
                if(exposure_code < 0)
                {
                    power_offset = -exposure_code;
                }

//                print_text("power=");
//                print_hex2(toflash_data[15]);
//                print_text(" ");
//                print_text("power_offset=");
//                print_number(power_offset);
//                print_lf();

                int power = toflash_data[15];
                power += power_offset;
                CLAMP(power, 0, 0xff);
                toflash_data[15] = power;
            }

// send it to the flash
            for(sim_offset = 0; sim_offset < sim_size; sim_offset++)
            {
                d2_sim = toflash_data[sim_offset];
                d1_value = 0;

// wait for flash to go high with timeout
                SET_BYTE_TIMER(BYTE_TIMEOUT);
                while(d1_raw < D1_HIGH && !BYTE_TIMER_EXPIRED)
                {
                    START_ADC2
                    while((ADC2->SR & ADC_FLAG_EOC) == 0)
                        ;
                    d1_raw = ADC2->DR;
                }

// Raise D2 before the byte
                SET_D
                usleep(10);
                for(i = 0; i < 8; i++)
                {

// lower CLK
                    CLEAR_CLK
                    if((d2_sim & 0x80))
                    {
                        SET_D
                    }
                    else
                    {
                        CLEAR_D
                    }
                    d2_sim <<= 1;
                    usleep(5);
// raise CLK
                    SET_CLK
// read from flash
                    d1_raw = ADC2->DR;
                    START_ADC2
                    while((ADC2->SR & ADC_FLAG_EOC) == 0)
                        ;
                    d1_raw = ADC2->DR;
                    d1_value <<= 1;

                    if(d1_raw >= D1_HIGH)
                        d1_value |= 1;

                    usleep(5);
                }

// wait for flash to go low with timeout
                SET_BYTE_TIMER(BYTE_TIMEOUT);
                while(d1_raw >= D1_HIGH && !BYTE_TIMER_EXPIRED)
                {
                    START_ADC2
                    while((ADC2->SR & ADC_FLAG_EOC) == 0)
                        ;
                    d1_raw = ADC2->DR;
                }

                fromflash_data[sim_offset] = d1_value;

// hold D2 for a while
                usleep(10);
// set D2 to idle
                CLEAR_D
                usleep(10);

// handle metering trigger
                if(packet_type == TYPE_METERING2 &&
                    sim_offset == 1)
                {
                    CLEAR_PIN(X_GPIO, 1 << X_PIN);
                    usleep(200);
                    SET_PIN(X_GPIO, 1 << X_PIN);
                }
            }

// reset the ID timer & pin
            TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);
            SET_RADIO_TIMER(RADIO_TIMEOUT);
            ENABLE_RADIO_TIMER
            TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
            SET_PIN(ID_GPIO, 1 << ID_PIN);



            if(packet_type == TYPE_POWERON)
            {
                TOGGLE_PIN(LED_GPIO, 1 << LED_PIN);
                exposure_code = fromflash_data[15];
                focal_length = toflash_data[18];
            }
            else
// get flash configuration
            if(packet_type == TYPE_METERING1)
            {
                TOGGLE_PIN(LED_GPIO, 1 << LED_PIN);
                exposure_code = fromflash_data[12];
                focal_length = toflash_data[15];
            }
            else
// handle triggers
            if(packet_type == TYPE_PREFLASH2)
            {
// turn off ID pin to turn off focusing aid on the GODOX
                CLEAR_PIN(ID_GPIO, 1 << ID_PIN);

// CLK only
// CLK must be in GPIO mode/3V
                if(!wait_trigger(TRIGGER_CODE_CLK))
                {

                    CLEAR_PIN(CLK_GPIO, 1 << CLK_PIN);
                    usleep(5000);
                    SET_PIN(CLK_GPIO, 1 << CLK_PIN);
                }
            }
            else
            if((packet_type == TYPE_MANE_FLASH1 &&
                toflash_data[17] == 0x1d) ||
                packet_type == TYPE_FAST_FLASH ||
                packet_type == TYPE_MANUAL_FLASH)
            {
// turn off ID pin to turn off focusing aid on the GODOX
                CLEAR_PIN(ID_GPIO, 1 << ID_PIN);
                if(!wait_trigger(TRIGGER_CODE_CLK))
                {

// CLK
// CLK must be in GPIO mode/3V
                    CLEAR_PIN(CLK_GPIO, 1 << CLK_PIN);

// X
                    if(!wait_trigger(TRIGGER_CODE_X))
                    {
                        CLEAR_PIN(X_GPIO, 1 << X_PIN);
// This is supposed to be the shutter speed but is hard coded to 1/100 for now.
                        usleep(10000);
                        SET_PIN(X_GPIO, 1 << X_PIN);
                    }
                    usleep(10000);
                    SET_PIN(CLK_GPIO, 1 << CLK_PIN);
                }
            }


// debugging
            switch(packet_type)
            {
                case TYPE_POWERON:
                    print_text("POWERON\n");
                    break;
                case TYPE_METERING1:
                    print_text("METERING1 ");
                    print_text("EXPOSURE CODE=");
                    print_hex2(exposure_code);
                    print_lf();
                    break;
                case TYPE_METERING2:
                    print_text("METERING2\n");
                    break;
                case TYPE_PREFLASH1:
                    print_text("PREFLASH1\n");
                    break;
                case TYPE_PREFLASH2:
                    print_text("PREFLASH2\n");
                    break;
                case TYPE_MANE_FLASH1:
                    print_text("MANE FLASH1 ");
                    print_text("ISO response=");
                    print_hex2(fromflash_data[6]);
                    print_text(" ISO code=");
                    print_hex2(toflash_data[8]);
                    print_lf();
                    break;
                case TYPE_MANE_FLASH2:
                    print_text("MANE FLASH2\n");
                    break;
                case TYPE_FAST_FLASH:
                    print_text("FAST FLASH\n");
                    break;
            }


// print the differences
            for(i = 0; i < sim_size; i++)
            {
                print_hex2(fromflash_data[i]);
                if(ref_packet[i * 2] != fromflash_data[i])
                {
                    send_uart('*');
                }
                send_uart('/');
                print_hex2(toflash_data[i]);
                if(ref_packet[i * 2 + 1] != toflash_data[i])
                {
                    send_uart('*');
                }
                if(((i + 1) % 8) == 0)
                {
                    send_uart('\n');
                }
                else
                {
                    send_uart(' ');
                }
            }
            print_lf();
        }
    }
}

void get_start2()
{
    if(radio_data == 0xea)
    {
        handle_rx = get_packet;
        rx_counter = 0;
    }
    else
    if(radio_data != 0xff)
        handle_rx = get_start1;
// data was 0xff
}

void get_start1()
{
    if(radio_data == 0xff)
        handle_rx = get_start2;
}


void flash_loop()
{
    receiver_on();

    while(1)
    {
        HANDLE_UART_OUT


// forward from radio to the packet parser
        if(radio_size > 0)
        {
            radio_data = radio_buffer[radio_read_ptr++];
            if(radio_read_ptr >= RADIO_BUFSIZE)
                radio_read_ptr = 0;

            USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
            radio_size--;
            USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

//            print_hex2(radio_data);
//            send_uart(' ');

            handle_rx();
        }
    }
}

// radio uart received
void USART1_IRQHandler(void)
{
    uint8_t c = USART1->DR;
    if(radio_size < RADIO_BUFSIZE)
    {
        radio_buffer[radio_write_ptr++] = c;
        if(radio_write_ptr >= RADIO_BUFSIZE)
            radio_write_ptr = 0;
        radio_size++;
    }
}


// ID pin timer
void TIM5_IRQHandler()
{
    if(TIM5->SR & TIM_FLAG_Update)
	{
		TIM5->SR = ~TIM_FLAG_Update;
// disable the ID pin after 1 overflow
        DISABLE_RADIO_TIMER
        CLEAR_PIN(ID_GPIO, 1 << ID_PIN);
        SET_PIN(LED_GPIO, 1 << LED_PIN);
    }
}


#endif // SIM_CAM





