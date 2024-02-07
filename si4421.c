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

#include "linux.h"
#include "wireless.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "misc.h"
#include "uart.h"
#include "si4421.h"

// Si4421 radio bits


uint8_t radio_state = RADIO_IDLE;

#define RADIO_CS_GPIO GPIOA
#define RADIO_CS_PIN GPIO_Pin_11
#define RADIO_SDO_GPIO GPIOA
#define RADIO_SDO_PIN GPIO_Pin_13
#define RADIO_CLK_GPIO GPIOA
#define RADIO_CLK_PIN GPIO_Pin_12



// write radio SPI
void write_spi(uint16_t data)
{
// print_text("write_spi ");
// print_hex(data);
// print_lf();
    CLEAR_PIN(RADIO_CS_GPIO, RADIO_CS_PIN);

    int i;
    for(i = 0; i < 16; i++)
    {
        if(data & 0x8000)
        {
            SET_PIN(RADIO_SDO_GPIO, RADIO_SDO_PIN);
        }
        else
        {
            CLEAR_PIN(RADIO_SDO_GPIO, RADIO_SDO_PIN);
        }
        data <<= 1;
// need the delay if the system clock is over 16Mhz
// Timer based delay seems to lock up inside an interrupt handler
//        usleep(1);
        udelay(1);
        SET_PIN(RADIO_CLK_GPIO, RADIO_CLK_PIN);
//        usleep(1);
        udelay(1);
        CLEAR_PIN(RADIO_CLK_GPIO, RADIO_CLK_PIN);
    }
    
    SET_PIN(RADIO_CS_GPIO, RADIO_CS_PIN);
}

void init_radio()
{
    GPIO_InitTypeDef GPIO_InitStructure;


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	GPIO_PinAFConfig(RADIO_UART_GPIO, RADIO_RX_PIN, GPIO_AF_USART1);
	GPIO_PinAFConfig(RADIO_UART_GPIO, RADIO_TX_PIN, GPIO_AF_USART1);

// RX enabled
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

#ifdef SIM_CAM
	GPIO_InitStructure.GPIO_Pin = 1 << RADIO_RX_PIN;
  	GPIO_Init(RADIO_UART_GPIO, &GPIO_InitStructure);
#endif


// TX enabled
#ifdef SIM_FLASH
    GPIO_InitStructure.GPIO_Pin = 1 << RADIO_TX_PIN;
    GPIO_Init(RADIO_UART_GPIO, &GPIO_InitStructure);
#endif


	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 200000;

	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
#ifdef SIM_CAM
	USART_InitStructure.USART_Mode = USART_Mode_Rx;
#endif

#ifdef SIM_FLASH
	USART_InitStructure.USART_Mode = USART_Mode_Tx;
#endif



/* USART configuration */
  	USART_Init(USART1, &USART_InitStructure);
/* Enable USART */
  	USART_Cmd(USART1, ENABLE);

/* Enable the UART Interrupt */
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
  	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

// initialize radio SPI
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Pin = RADIO_CS_PIN |
        RADIO_SDO_PIN |
        RADIO_CLK_PIN;
    GPIO_Init(RADIO_CS_GPIO, &GPIO_InitStructure);
    SET_PIN(RADIO_CS_GPIO, RADIO_CS_PIN);
    CLEAR_PIN(RADIO_SDO_GPIO, RADIO_SDO_PIN);
    CLEAR_PIN(RADIO_CLK_GPIO, RADIO_CLK_PIN);
    
    



// scan for synchronous code
    write_spi(FIFORSTREG);
// enable synchron latch
    write_spi(FIFORSTREG | 0x0002);
    write_spi(GENCREG);
    write_spi(AFCCREG);
    set_channel();
    write_spi(DRVSREG);
    write_spi(PMCREG);
    write_spi(RXCREG);
    write_spi(TXCREG);
    write_spi(BBFCREG);

}

// turn the radio off
void radio_off()
{
    write_spi(PMCREG);
}

// turn the transmitter on
// Your job: wait 5ms
void transmitter_on()
{
    write_spi(PMCREG | 0x0020);

    radio_state = RADIO_WARMUP;
    SET_RADIO_TIMER(RADIO_WARMUP_TIME)
    ENABLE_RADIO_TIMER

}

void receiver_on()
{
    write_spi(PMCREG | 0x0080);
}

void set_channel()
{
    int channel2 = channel * (MAX_CHANNEL - MIN_CHANNEL) / 4 + MIN_CHANNEL;
    write_spi(CFSREG(channel2));
}






