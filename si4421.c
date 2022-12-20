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

// Si4421 radio bits


int radio_state = RADIO_IDLE;

// RADIO_CHANNEL is from 96-3903 & set by the user
// data rate must be slow enough to service FIFOs
// kbps = 10000 / (29 * (DRVSREG<6:0> + 1) * (1 + DRPE * 7))
// RADIO_BAUD_CODE = 10000 / (29 * kbps) / (1 + DRPE * 7) - 1
// RADIO_DATA_SIZE is the amount of data to read before resetting the sync code

#define RADIO_CHANNEL 96

// scan for synchronous code
#define FIFORSTREG 0xCA81
// read continuously
//#define FIFORSTREG              (0xCA81 | 0x0004)
// 915MHz
#define FREQ_BAND 0x0030
// Center Frequency: 915.000MHz
//#define CFSREG (0xA000 | RADIO_CHANNEL)
#define CFSREG(chan) (0xA000 | (chan))
// crystal load 10pF
#define XTAL_LD_CAP 0x0003
// power management page 16
#define PMCREG 0x8201
#define GENCREG (0x8000 | XTAL_LD_CAP | FREQ_BAND)


// +3/-4 Fres
//#define AFCCREG 0xc4f7
// +15/-16 Fres
#define AFCCREG 0xc4d7

// Data Rate
// data rate must be slow enough to service FIFOs
// kbps = 10000 / (29 * (DRVSREG<6:0> + 1) * (1 + DRPE * 7))
// RADIO_BAUD_CODE = 10000 / (29 * kbps) - 1
#define RADIO_BAUD_CODE 3

// data rate prescaler.  Divides data rate by 8 if 1
//#define DRPE (1 << 7)
#define DRPE 0
#define DRVSREG (0xC600 | DRPE | RADIO_BAUD_CODE)


// Page 37 of the SI4421 datasheet gives optimum bandwidth values
// but the lowest that works is 200khz
//#define RXCREG 0x9481     // BW 200KHz, LNA gain 0dB, RSSI -97dBm
//#define RXCREG 0x9440     // BW 340KHz, LNA gain 0dB, RSSI -103dBm
#define RXCREG 0x9420       // BW 400KHz, LNA gain 0dB, RSSI -103dBm

//#define TXCREG 0x9850     // FSK shift: 90kHz
#define TXCREG 0x98f0       // FSK shift: 165kHz
#define STSREG 0x0000
#define RXFIFOREG 0xb000

// analog filter for raw mode
#define BBFCREG                 0xc23c


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
    write_spi(CFSREG(RADIO_CHANNEL));
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







