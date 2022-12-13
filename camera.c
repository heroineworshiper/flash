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


// Loop that runs on the camera side

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






#ifndef SIM_CAM



#ifdef DEBUG_TIMEOUT
uint8_t *debug_buffer;
int debug_offset = 0;
#define DEBUG_SIZE 512
#endif // DEBUG_TIMEOUT


uint8_t packet_id = 0xff;


#ifdef SIM_FLASH

void write_radio(uint8_t c)
{
    if(radio_size < RADIO_BUFSIZE)
    {
        radio_buffer[radio_write_ptr++] = c;
        if(radio_write_ptr >= RADIO_BUFSIZE)
            radio_write_ptr = 0;
        radio_size++;
    }
}

// wrap a radio packet & send it
void transmit()
{
// wait for GPIO to exit trigger mode
    write_radio(0xff);
    write_radio(0xff);
    write_radio(0xea);
    
    int i, j, offset = 0;
    for(j = 0; j < 2; j++)
    {
// send twice with different salt
        for(i = 0; i < RADIO_PACKETSIZE; i++)
        {
            write_radio(radio_packet[i] ^ salt[offset % sizeof(salt)]);
            offset++;
        }
    }
}
#endif // SIM_FLASH



// reset the packet engine
void reset_packet()
{
    capture_size = 0;
    packet_type = TYPE_NONE;
    d1_sim = 0xff;

#ifdef DEBUG_RAW
    print_text("\nCAPTURE RESTARTS\n");
#endif
}


void diff_packet()
{
#ifndef DEBUG_RAW
    int i;
    const uint8_t *ref = ref_packets[packet_type];
    int size = ref_packet_size[packet_type];

    for(i = 0; i < capture_size; i++)
    {
        print_hex2(fromflash_data[i]);
        if(ref[i * 2] != fromflash_data[i])
        {
            send_uart('*');
        }
        send_uart('/');
        print_hex2(toflash_data[i]);
        if(ref[i * 2 + 1] != toflash_data[i])
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
#endif
}



void process_byte()
{
    int time_difference = TIM2->CNT;
    int i;
    TIM2->CNT = 0;

// print raw data with time based packetization for debugging
#ifdef DEBUG_RAW
    if(time_difference > BYTE_TIMEOUT)
    {
        if((byte_counter % 8) != 0)
            send_uart('\n');
        print_text("BYTES: ");
        print_number(byte_counter);
        print_number_nospace(time_difference);
        print_text("uS\n");
        byte_counter = 0;
    }
#endif

    if(time_difference > PACKET_TIMEOUT)
    {
// time out the packet capture
        reset_packet();
    }

#ifdef DEBUG_RAW
// print raw data with time based packetization for debugging
    print_hex2(d1_value);
    send_uart('/');
    print_hex2(d2_value);
    if((byte_counter % 8) == 0)
        send_uart('\n');
    else
        send_uart(' ');
#endif

#ifdef SIM_FLASH
    d1_sim = 0x8e;
#endif

// capture the packet
    if(capture_size < PACKET_MAX)
    {
        toflash_data[capture_size] = d2_value;
#ifndef SIM_FLASH
        fromflash_data[capture_size] = d1_value;
#endif
        capture_size++;

// get packet type
        int prev_type = packet_type;
        if(packet_type == TYPE_NONE)
        {
            if(capture_size == 1)
            {
                if(toflash_data[0] == 0xa5)
                {
                    packet_type = TYPE_METERING2;
                    trigger_state = TRIGGER_IDLE;
                    trigger_code = TRIGGER_CODE_NONE;
                    print_text("\nMETERING2\n");
                }
            }
            else
            if(capture_size == 2)
            {
                if(toflash_data[1] == 0xb3)
                {
                    packet_type = TYPE_MANE_FLASH1;
                    trigger_state = TRIGGER_FLASH;
                    trigger_code = TRIGGER_CODE_NONE;
                    print_text("\nMANE FLASH1\n");
                }
                else
                if(toflash_data[0] == 0xb4 &&
// shutter speed code 2
                    (toflash_data[1] == 0x1d ||
                    toflash_data[1] == 0x05))
                {
                    packet_type = TYPE_MANE_FLASH2;
                    trigger_state = TRIGGER_IDLE;
                    trigger_code = TRIGGER_CODE_NONE;
                    print_text("\nMANE FLASH2\n");
                } 
                else
                if(toflash_data[0] == 0xb4 &&
                    toflash_data[1] == 0x25)
                {
                    packet_type = TYPE_FAST_FLASH;
                    trigger_state = TRIGGER_FLASH;
                    trigger_code = TRIGGER_CODE_NONE;
                    print_text("\nFAST FLASH\n");
                } 
                else
                if(toflash_data[1] == 0xb4)
                {
                    packet_type = TYPE_PREFLASH1;
                    trigger_state = TRIGGER_IDLE;
                    trigger_code = TRIGGER_CODE_NONE;
                    print_text("\nPREFLASH1\n");
                }
                else
                if(toflash_data[0] == 0xb4 &&
                    toflash_data[1] == 0x23)
                {
                    packet_type = TYPE_PREFLASH2;
                    trigger_state = TRIGGER_PREFLASH;
                    trigger_code = TRIGGER_CODE_NONE;
                    print_text("\nPREFLASH2\n");
                }
            }
            else
            if(capture_size == 4)
            {
                if(toflash_data[1] == 0xb5 &&
                    toflash_data[2] == 0x4c &&
                    toflash_data[3] == 0xe5)
                {
                    packet_type = TYPE_METERING1;
                    trigger_state = TRIGGER_IDLE;
                    trigger_code = TRIGGER_CODE_NONE;
                    print_text("\nMETERING1\n");
                }
                else
                if(toflash_data[1] == 0xb5 &&
                    toflash_data[2] == 0x4c &&
                    toflash_data[3] == 0xff)
                {
                    packet_type = TYPE_POWERON;
                    trigger_state = TRIGGER_IDLE;
                    trigger_code = TRIGGER_CODE_NONE;
                    print_text("\nPOWERON\n");
                }
                else
                {
                    print_text("\nUNKNOWN PACKET: ");
                    for(i = 0; i < capture_size; i++)
                    {
                        print_hex2(fromflash_data[i]);
                        send_uart('/');
                        print_hex2(toflash_data[i]);
                        send_uart(' ');
                    }
                    print_lf();
                }
            }
        }
        
        if(prev_type == TYPE_NONE &&
            packet_type != TYPE_NONE)
        {
// initialize the simulated packet
            int size = ref_packet_size[packet_type];
            const uint8_t *ref = ref_packets[packet_type];
            for(i = 0; i < size; i++)
                fromflash_data[i] = ref[i * 2];
// TODO: fill in fromflash values
        }

        if(packet_type != TYPE_NONE)
        {
            d1_sim = fromflash_data[capture_size];
        }
        else
        if(capture_size == 2)
        {
// replay simulated values without knowledge of the packet type
            if(toflash_data[1] == 0xb5)
                d1_sim = 0xde;
        }


        if(capture_size == ref_packet_size[packet_type])
        {
// the packet is complete
            int offset = 0;


#ifdef SIM_FLASH
            radio_packet[offset++] = packet_id++;
            radio_packet[offset++] = packet_type;
// pack variable data for certain packets
            const uint8_t *vars = var_offsets[packet_type];
            if(vars)
            {
                while(*vars != 0)
                {
                    radio_packet[offset++] = toflash_data[*vars];
                    vars++;
                }
            }
            if(offset > RADIO_PACKETSIZE)
            {
                print_text("\noffset > RADIO_PACKETSIZE\n");
            }

// send it to the flash
            for(i = 0; i < repeats[packet_type]; i++)
                transmit();
#endif // SIM_FLASH


// Dump the packet
            diff_packet();


            reset_packet();
        }
    }

    byte_counter++;


    bit_counter = 0;
    d1_value = 0;
    d2_value = 0;
}

void camera_loop()
{
    while(1)
    {
        HANDLE_UART_OUT


#ifdef SIM_FLASH
// transmit to real flash
        if((USART1->SR & USART_FLAG_TC) != 0)
        {
            if(radio_size > 0)
	        {
		        USART1->DR = radio_buffer[radio_read_ptr++];
                if(radio_read_ptr >= RADIO_BUFSIZE)
                    radio_read_ptr = 0;
                radio_size--;
	        }
            else
            if(trigger_state != TRIGGER_IDLE)
            {
                USART1->DR = trigger_code;
            }
        }
#endif // SIM_FLASH


// detect X transitions
        int x_value2 = PIN_IS_SET(GPIOC, GPIO_Pin_2);
        if(x_value2 && !x_value)
        {
            print_text("\nX 3.3V ");
            print_number_nospace(TIM2->CNT);
            print_text("uS\n");
        }
        else
        if(!x_value2 && x_value)
        {
            if(trigger_state == TRIGGER_FLASH)
                trigger_code = TRIGGER_CODE_X;
            
            print_text("\nX 0V ");
            print_number_nospace(TIM2->CNT);
            print_text("uS\n");
        }
        x_value = x_value2;

// read clock pin
        if((ADC1->SR & ADC_FLAG_EOC) != 0)
        {
            int clk_raw2 = ADC1->DR;
            ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART;

            if(clk_raw2 >= CLK_LOW && 
                clk_raw2 < CLK_HIGH && 
                clk_raw >= CLK_HIGH)
            {
// falling edge.  Write bit
#ifdef SIM_FLASH
                if((d1_sim & 0x80))
                {
                    SET_D
                }
                else
                {
                    CLEAR_D
                }
                d1_sim <<= 1;
#endif
            }
            else
            if(clk_raw2 < CLK_LOW && clk_raw >= CLK_LOW)
            {
                if(trigger_state != TRIGGER_IDLE)
                    trigger_code = TRIGGER_CODE_CLK;


                print_text("\nCLK 0V ");
                print_number_nospace(TIM2->CNT);
                print_text("uS\n");
            }
            else
            if(clk_raw2 >= CLK_HIGH && clk_raw < CLK_LOW)
            {
                print_text("\nCLK 3.3V ");
                print_number_nospace(TIM2->CNT);
                print_text("uS\n");
            }
            else
            if(clk_raw2 >= CLK_LOW && clk_raw < CLK_LOW)
            {
                print_text("\nCLK 2V ");
                print_number_nospace(TIM2->CNT);
                print_text("uS\n");
            }
            else
            if(clk_raw2 >= CLK_HIGH && clk_raw < CLK_HIGH)
            {
// rising edge
// read bit
                d1_value <<= 1;
                if(d1_raw >= D1_HIGH)
                    d1_value |= 1;
                d2_value <<= 1;
                if(d2_raw >= D2_HIGH)
                    d2_value |= 1;
                bit_counter++;
                if(bit_counter >= 8)
                {
                    process_byte();

#ifdef SIM_FLASH
// set up flow control
                    d1_sim_timeout = D1_SIM_TIMEOUT;
#endif
                }
                else
                {
// reset the framing timer
                    RESET_BIT_TIMER
                    ENABLE_BIT_TIMER
                }
            }

    //TOGGLE_PIN(GPIOC, GPIO_Pin_14);
            clk_raw = clk_raw2;

#ifdef SIM_FLASH
// flow control
            if(d1_sim_timeout > 0)
            {
                d1_sim_timeout--;
    // must lower a 1 bit for a while
                if(d1_sim_timeout == D1_SIM_TIMEOUT - 10)
                {
                    CLEAR_D
                }
    // must raise it after a certain amount of time to signify idle
                if(d1_sim_timeout == 0)
                {
                    SET_D
                }
            }
#endif // SIM_FLASH


#ifdef DEBUG_TIMEOUT
            debug_buffer[debug_offset++] = clk_raw2;
            debug_buffer[debug_offset++] = d2_raw;
            if(debug_offset >= DEBUG_SIZE) debug_offset = 0;
#endif

// static int debug_counter = 0;
// static int min_value = 0xffff;
// static int max_value = 0;
// if(clk_raw2 < min_value) min_value = clk_raw2;
// if(clk_raw2 > max_value) max_value = d2_raw;
// debug_counter++;
// if(debug_counter > 100000)
// {
// print_text("CLK RANGE: ");
// print_number(min_value);
// print_number(max_value);
// print_lf();
// min_value = 0xffff;
// max_value = 0;
// debug_counter = 0;
// }



        }


// only read ID/D1 for testing
#ifndef SIM_FLASH
        if((ADC2->SR & ADC_FLAG_EOC) != 0)
        {
            if(reading_id) 
            {
                int id_raw2 = ADC2->DR;
                if(id_raw2 >= ID_HIGH && id_raw < ID_HIGH)
                {
                    print_text("\nID HIGH\n");
                }
                else
                if(id_raw2 < ID_HIGH && id_raw >= ID_HIGH)
                {
                    print_text("\nID LOW\n");
                }
                id_raw = id_raw2;
            }
            else
            {
                d1_raw = ADC2->DR;
            }


// read ID instead of D1
            if(clk_raw >= CLK_HIGH)
            {
                reading_id = 1;
                ADC_RegularChannelConfig(ADC2, ADC_Channel_13, 1, ADC_SAMPLE_TIME);
            }
            else
            {
// read D1 instead of ID
                reading_id = 0;
                ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SAMPLE_TIME);
            }


            START_ADC2

// static int debug_counter = 0;
// static int min_value = 0xffff;
// static int max_value = 0;
// if(d1_raw < min_value) min_value = d1_raw;
// if(d1_raw > max_value) max_value = d1_raw;
// debug_counter++;
// if(debug_counter > 100000)
// {
// print_text("D1 RANGE: ");
// print_number(min_value);
// print_number(max_value);
// print_lf();
// min_value = 0xffff;
// max_value = 0;
// debug_counter = 0;
// }

        }
#endif // !SIM_FLASH


// data from cam
        if((ADC3->SR & ADC_FLAG_EOC) != 0)
        {
            d2_raw = ADC3->DR;
            ADC3->CR2 |= (uint32_t)ADC_CR2_SWSTART;


//static int debug_counter = 0;
//static int min_value = 0xffff;
//static int max_value = 0;
//if(d2_raw < min_value) min_value = d2_raw;
//if(d2_raw > max_value) max_value = d2_raw;
//debug_counter++;
//if(debug_counter > 100000)
//{
//print_text("ID: ");
//print_number_nospace(id_raw);
//print_text("D2 RANGE: ");
//print_number(min_value);
//print_number(max_value);
//print_lf();
//min_value = 0xffff;
//max_value = 0;
//debug_counter = 0;
//}
        }
    }
}






void TIM1_UP_TIM10_IRQHandler()
{
// byte framing expired
    if(TIM10->SR & TIM_FLAG_Update)
	{
		TIM10->SR = ~TIM_FLAG_Update;
// disable the framing timer after 1 overflow
        DISABLE_BIT_TIMER

        if(bit_counter > 0 && bit_counter < 8)
        {
// throw it out & print a debugging waveform if it was in the middle of a packet
            bit_counter = 0;
            d1_value = 0;
            d2_value = 0;
#ifdef SIM_FLASH
            SET_D
            d1_sim = 0xff;
#endif

#ifdef DEBUG_TIMEOUT
              if(byte_counter > 1)
              {
                  print_text("\nBIT TIMEOUT\n");
                  int i;
                  for(i = 0; i < DEBUG_SIZE / 2; i++)
                  {
                      print_number(debug_buffer[(debug_offset + i * 2) & 255]);
                      print_number_nospace(debug_buffer[(debug_offset + i * 2 + 1) & 255]);
                      print_lf();
                  }
              }
#else // DEBUG_TIMEOUT
            print_text("\nBIT TIMEOUT\n");
#endif

//                 print_text("\nDROPPED ");
//                 print_hex2(d1_value);
//                 send_uart('/');
//                 print_hex2(d2_value);
//                 print_lf();
        }
    }
}

#endif // !SIM_CAM


