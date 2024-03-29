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

// make wireless.bin
// ../stm32stuff/uart_programmer wireless.bin


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


// parsed data
// captured byte from flash
uint8_t d1_value = 0;
// captured byte to flash
uint8_t d2_value = 0;
// generated byte for D1
uint8_t d1_sim = 0xff;
// generated byte for D2
uint8_t d2_sim = 0;
int have_d2_sim = 0;
int x_value = 0;
int id_value = 0;
int bit_counter = 0;
// analog readouts
int clk_raw = 0;
int d1_raw = 0;
int d2_raw = 0;
int id_raw = 0;
// ADC is reading ID instead of D1
int reading_id = 0;
int byte_counter = 0;




// unencoded packet
uint8_t radio_packet[RADIO_PACKETSIZE * 2];


// buffer for receive/transmit
// must buffer more than 1 packet with resends
#define RADIO_BUFSIZE 128
uint8_t radio_buffer[RADIO_BUFSIZE];
volatile int radio_size = 0;
volatile int radio_write_ptr = 0;
volatile int radio_read_ptr = 0;
uint8_t radio_data;
volatile uint8_t channel = 0;
#define TOTAL_CHANNELS 4
#define CONFIG_START 0x0800c000

// RADIO_PACKETSIZE * 2
const uint8_t salt[] = 
{
    0xd0, 0xf1, 0x09, 0xa7, 0xcf, 0x89, 0x44, 0x40, 
    0xb1, 0x4c, 0x98, 0x41, 0xbc, 0xd7, 0xd1, 0x32,
    0x61, 0x8c, 0xcd, 0xa7
};
// null for debugging
// const uint8_t salt[] = 
// {
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
//     0x00, 0x00, 0x00, 0x00
// };


// send codes for flash triggers
int trigger_state = TRIGGER_IDLE;
uint8_t trigger_code = TRIGGER_CODE_NONE;




// micro seconds
// maximum time to wait for the next bit
#define BIT_TIMEOUT 40
// when to print the byte counter for debugging
#define BYTE_TIMEOUT 20000
// when to reset the packet parser because of a glitch
#define PACKET_TIMEOUT 80000
// Wait this long for trigger codes
#define TRIGGER_TIMEOUT 1000000

#define PACKET_MAX 128
// the last captured packet
// captured or simulated data to the flash
uint8_t toflash_data[PACKET_MAX];
// captured or simulated data from the flash
uint8_t fromflash_data[PACKET_MAX];
// bytes captured
int capture_size = 0;
// offset in simulated packet
int sim_offset = 0;
// size of simulated packet
int sim_size = 0;





// reference packets are for the following settings:
// 28mm 1/100 F8.0 ISO100
// ETTL, High speed, single flash, no exposure compensation

// from flash/to flash are interleaved bytes

// power on, metering start
const uint8_t poweron_ref[] = 
{
    0xff, 0xff, 0x8e, 0xb5, 0xde, 0x4c, 0x8e, 0xff, 0x8e, 0xb5, 0xde, 0x4c, 0x8e, 0xe5, 0x0d, 0xff,
    0x8e, 0xa9, 0x91, 0x25, 0x8e, 0xa1, 0x2b, 0xf8, 0x2b, 0xa5, 0x00, 0x2c, 0x8e, 0xb9, 0x00, 0x80,
    0x8e, 0xbd, 0xaa, 0x00, 0x59, 0x1c, 0x00, 0x99, 0xff, 0xfb, 0x02, 0xff, 0x8e, 0xf9, 0x55, 0xff,
    0x8e, 0xf7, 0x7b, 0xff, 0x8e, 0xb3, 0x00, 0x12, 0x00, 0x46, 0x00, 0x2f, 0x00, 0xf5, 0x57, 0xff,
    0x49, 0xff, 0x5c, 0xff, 0x0f, 0xff, 0x00, 0xbe, 0x8e, 0x14, 0x8e, 0xbb, 0x00, 0x48, 0x00, 0xff,
    0x8e, 0xe6, 0x3f, 0xff, 0x10, 0xff, 0x00, 0xff, 0x8e, 0xb7, 0x8e, 0x38, 0x8e, 0xb8, 0x8e, 0x6d,
    0x8e, 0xb4, 0x8e, 0x1d, 0x8e, 0xa8, 0x91, 0x00
};

// metering repeat
const uint8_t metering1[] =
{
    0x8e, 0xff, 0x8e, 0xb5, 0xde, 0x4c, 0x8e, 0xe5, 0x0d, 0xff, 0x8e, 0xa9, 0x91, 0x25, 0x8e, 0xa1,
    0x2b, 0xf8, 0x2b, 0xa5, 0x00, 0x2c, 0x8e, 0xb9, 0x00, 0x80, 0x8e, 0xbd, 0x18, 0x00, 0x69, 0x1c,
    0x1c, 0x99, 0xff, 0xfb, 0x02, 0xff, 0x8e, 0xf9, 0x55, 0xff, 0x8e, 0xf7, 0x7b, 0xff, 0x8e, 0xb3,
    0x00, 0x12, 0x00, 0x46, 0x00, 0x2f, 0x00, 0xf5, 0x57, 0xff, 0x49, 0xff, 0x5c, 0xff, 0x0f, 0xff,
    0x00, 0xbe, 0x8e, 0x14, 0x8e, 0xbb, 0x00, 0x48, 0x00, 0xff, 0x8e, 0xe6, 0x3f, 0xff, 0x10, 0xff,
    0x00, 0xff, 0x8e, 0xb7, 0x8e, 0x38, 0x8e, 0xb8, 0x8e, 0x6d, 0x8e, 0xb4, 0x8e, 0x1d, 0x8e, 0xa8,
    0x91, 0x00
};

// sometimes sent after poweron & metering repeat
const uint8_t metering2_ref[] =
{
    0x8e, 0xa5, 0x00, 0x2d, // X is then triggered
    0x8e, 0xa5, 0x01, 0x2c
};


const uint8_t preflash1_ref[] =
{
    0x8e, 0xff, 0x8e, 0xb4, 0x8e, 0x1d, 0x8e, 0xf2, 0xc0, 0xff, 0x8e, 0xff, 0x8e, 0xb4, 0x8e, 0x03,
    0x8e, 0xf2, 0xa0, 0xff, 0x8e, 0xb0, 0x8e, 0x80, 0x8e, 0xb1, 0x8e, 0x04, 0x8e, 0xb3, 0x00, 0x12,
    0x00, 0x46, 0x00, 0x2f, // 32ms delay
};

const uint8_t preflash2_ref[] =
{
    0x00, 0xb4, 0x8e, 0x23  // trigger follows
};


const uint8_t mane_flash1_ref[] =
{
    0x7f, 0xff, 0x8e, 0xb3, 0x00, 0x32, 0x00, 0x46, 0x00, 0x2f, 0x00, 0xf8, 0x36, 0xff, 0x8e, 0xbb,
    0x00, 0x48, 0x00, 0xff, 0x8e, 0xb7, 0x8e, 0x38, 0x8e, 0xb8, 0x8e, 0x6d, 0x8e, 0xb0, 0x8e, 0x88,
    0x8e, 0xb4, 0x8e, 0x1d, 0x8e, 0xf2, 0xc0, 0xff, 0x8e, 0xb3, 0x00, 0x36, 0x00, 0x46, 0x00, 0x2f,
    0x00, 0xb4, 0x8e, 0x3d, // trigger follows

};

const uint8_t manual_flash_ref[] =
{
    0x8e, 0xbb, 0xfb, 0x48, 0x00, 0xff, 0x8e, 0xb7, 0x8e, 0x16, 0x8e, 0xb8, 0x8e, 0x6d, 0x8e, 0xb4, 
    0x8e, 0x1d, 0x8e, 0xb4, 0x8e, 0x3d
};

const uint8_t mane_flash2_ref[] =
{
    0x7f, 0xb4, 0x8e, 0x1d, 0x8e, 0xb3, 0x00, 0x12, 0x00, 0x46, 0x00, 0x2f, 0x00, 0xfc, 0x88, 0xff,
    0xc0, 0xff, 0x1a, 0xff, 0x2a, 0xff, 0x85, 0xff, 0x78, 0xff 
};

// Different packets when shutter speed is 1/200
const uint8_t fast_flash_ref[] =
{
    0x8e, 0xb4, 0x8e, 0x25
};


// offsets of variable bytes in each packet type, null terminated
// Must be smaller than RADIO_PACKETSIZE - 2
const uint8_t poweron_vars[] = { 18, 38, 45, 47, 0 };
const uint8_t metering1_vars[] = { 15, 35, 42, 44, 46, 0 };
const uint8_t preflash1_vars[] = { 2, 11, 0 };
const uint8_t maneflash1_vars[] = { 8, 11, 13, 15, 17, 21, 24, 25, 0 };
const uint8_t maneflash2_vars[] = { 1, 0 };

int packet_type = TYPE_NONE;

// number of times to repeat each packet
const uint8_t repeats[] = 
{ 
    0, 
    2, 
    2, 
    2, 
    4, 
    2, 
    4, 
    2, 
    4,
    0
};

// tables of variables for each packet
const uint8_t *var_offsets[] =
{
    0,
    poweron_vars,
    metering1_vars,
    0,
    preflash1_vars,
    0,
    maneflash1_vars,
    maneflash2_vars,
    0,
    0
};

const int ref_packet_size[] = 
{
    0, 
    sizeof(poweron_ref) / 2,
    sizeof(metering1) / 2,
    sizeof(metering2_ref) / 2,
    sizeof(preflash1_ref) / 2,
    sizeof(preflash2_ref) / 2,
    sizeof(mane_flash1_ref) / 2,
    sizeof(mane_flash2_ref) / 2,
    sizeof(fast_flash_ref) / 2,
    sizeof(manual_flash_ref) / 2,
};

const uint8_t* ref_packets[] = 
{
    0,
    poweron_ref,
    metering1,
    metering2_ref,
    preflash1_ref,
    preflash2_ref,
    mane_flash1_ref,
    mane_flash2_ref,
    fast_flash_ref,
    manual_flash_ref,
};



// delay with TIM2
// seems to lock up inside an interrupt handler
void usleep(int us)
{
    SET_BYTE_TIMER(us);
    while(!BYTE_TIMER_EXPIRED)
        ;
}

void main()
{
/* Enable the GPIOs */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |
			RCC_AHB1Periph_GPIOC, 
		ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 |
            RCC_APB2Periph_SYSCFG |
            RCC_APB2Periph_TIM10, 
        ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 |
            RCC_APB1Periph_TIM5 |
            RCC_APB1Periph_DAC, 
        ENABLE);


 	NVIC_InitTypeDef NVIC_InitStructure;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 	NVIC_SetVectorTable(NVIC_VectTab_FLASH, PROGRAM_START - 0x08000000);
    init_linux();
	init_uart();

    const uint32_t *buffer = (const uint32_t*)CONFIG_START;
    channel = *buffer;
    CLAMP(channel, 0, TOTAL_CHANNELS - 1);

    init_radio();

    print_text("\n\n\n\nWelcome to wireless ETTL\n");
    print_text("channel=");
    print_number(channel);
    print_lf();

#ifdef SIM_FLASH
    print_text("SIM_FLASH\n");
#endif
#ifdef SIM_CAM
    print_text("SIM_CAM\n");
#endif

// LED
    GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Pin = 1 << LED_PIN;
    GPIO_Init(LED_GPIO, &GPIO_InitStructure);
    SET_PIN(LED_GPIO, 1 << LED_PIN);

// Button
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = 1 << BUTTON_PIN;
    GPIO_Init(BUTTON_GPIO, &GPIO_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
 	NVIC_Init(&NVIC_InitStructure);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, BUTTON_PIN);

    EXTI_InitTypeDef exti_config;
    exti_config.EXTI_Line = 1 << BUTTON_PIN;
    exti_config.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_config.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    exti_config.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti_config);

// button timer runs at 4khz
    TIM_DeInit(TIM1);
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 0xffff;
// clockspeed / 4000 - 1
	TIM_TimeBaseStructure.TIM_Prescaler = 41999;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    TIM1->CNT = 0;
	TIM_Cmd(TIM1, ENABLE);

// initialize the DAC
    DAC_DeInit();
    DAC_InitTypeDef DAC_InitStruct;
    DAC_StructInit(&DAC_InitStruct);
    DAC_Init(DAC_Channel_1, &DAC_InitStruct);
    DAC_Init(DAC_Channel_2, &DAC_InitStruct);

// set the DAC up for digital 0
    DAC_SetChannel2Data(DAC_Align_8b_R, SIM_D1_LOW);
    DAC_SetChannel1Data(DAC_Align_8b_R, SIM_D2_LOW);


// initialize the ADC
    ADC_InitTypeDef       ADC_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_DeInit();




	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
//	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
//	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
// 	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_10Cycles;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_StructInit(&ADC_InitStructure);
// ADC3 is always capturing 12 bits
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_8b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;



// measure time between bytes & create timeouts
    TIM_DeInit(TIM2);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 0xffffffff;
// clockspeed / 2000000 - 1
	TIM_TimeBaseStructure.TIM_Prescaler = 83;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM2, ENABLE);

#if defined(SIM_CAM) || defined(SIM_FLASH)
// timer for radio & ID pin
    TIM_DeInit(TIM5);
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
    DISABLE_RADIO_TIMER

 	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
 	NVIC_Init(&NVIC_InitStructure);
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
#endif


#ifdef SIM_CAM
// simulate cam
// set idle values
    SET_PIN(X_GPIO, 1 << X_PIN);
    CLEAR_PIN(ID_GPIO, 1 << ID_PIN);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = (1 << X_PIN);
    GPIO_Init(X_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = (1 << ID_PIN);
    GPIO_Init(ID_GPIO, &GPIO_InitStructure);


// only capture D1/from flash for testing
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

// capture data from flash for testing
    ADC_Init(ADC2, &ADC_InitStructure);
    ADC_Cmd(ADC2, ENABLE);
    ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SAMPLE_TIME);
    ADC_SoftwareStartConv(ADC2);


// set the GPIOs for digital 1
// enabling the DAC prevents changes to the pin mode
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Pin = (1 << D_PIN);
    GPIO_Init(D_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = (1 << CLK_PIN);
    GPIO_Init(CLK_GPIO, &GPIO_InitStructure);
    SET_PIN(D_GPIO, 1 << D_PIN);
    SET_PIN(CLK_GPIO, 1 << CLK_PIN);

// disable the DAC to get digital 1
// enable the DAC to get digital 0
// D2 is digital 0 when idle
    CLEAR_D
    SET_CLK

#else // SIM_CAM

// simulate flash
// capture X
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOC, &GPIO_InitStructure);



// timeout between bits
	TIM_DeInit(TIM10);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = BIT_TIMEOUT;
// clockspeed / 2000000 - 1
	TIM_TimeBaseStructure.TIM_Prescaler = 83;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM10, ENABLE);

 	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
 	NVIC_Init(&NVIC_InitStructure);

	TIM_ITConfig(TIM10, TIM_IT_Update, ENABLE);


// capture CLK, D1 from flash, D2 to flash, ID
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Pin = 
        GPIO_Pin_3;   // ID
	GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = 
        GPIO_Pin_0 |   // CLK
        GPIO_Pin_1 |   // D1
        GPIO_Pin_2 |   // D2
        GPIO_Pin_4 |   // DAC CLK
        GPIO_Pin_5;    // DAC D1/D2
	GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_Init(ADC3, &ADC_InitStructure);
    ADC_Cmd(ADC1, ENABLE);
    ADC_Cmd(ADC3, ENABLE);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SAMPLE_TIME);
    ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 1, ADC_SAMPLE_TIME);
    ADC_SoftwareStartConv(ADC1);
    ADC_SoftwareStartConv(ADC3);


// only read ID/D1 for testing
#ifndef SIM_FLASH
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
	ADC_Init(ADC2, &ADC_InitStructure);
    ADC_Cmd(ADC2, ENABLE);
// measure ID 1st
    reading_id = 1;
//    ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SAMPLE_TIME);
    ADC_RegularChannelConfig(ADC2, ADC_Channel_13, 1, ADC_SAMPLE_TIME);
    ADC_SoftwareStartConv(ADC2);
#endif // !SIM_FLASH
#endif // !SIM_CAM





#ifdef SIM_FLASH
// set the GPIO up for digital 1
// enabling the DAC prevents changes to the pin mode
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Pin = 1 << D_PIN;
    GPIO_Init(D_GPIO, &GPIO_InitStructure);
    SET_PIN(D_GPIO, 1 << D_PIN);

// disable the DAC to get digital 1
    SET_D
#endif

#ifdef DEBUG_TIMEOUT
    debug_buffer = kmalloc(DEBUG_SIZE, 0);
#endif

#ifndef SIM_CAM
    camera_loop();
#else
    flash_loop();
#endif


}


// sleep for the button routines
void msleep(int ms)
{
    TIM1->CNT = 0;
    while(TIM1->CNT < ms * 4)
    {
    }
}


void blink_channel()
{
    
    int i;
    for(i = 0; i < channel + 1; i++)
    {
        SET_PIN(LED_GPIO, 1 << LED_PIN);
        msleep(250);
        CLEAR_PIN(LED_GPIO, 1 << LED_PIN);
        msleep(250);
    }
    
    msleep(500);
}

#define LONG_PRESS 1000
#define SHORT_PRESS 5
void EXTI15_10_IRQHandler()
{
// clear interrupt
    EXTI->PR = 1 << BUTTON_PIN;

    int prev_channel = channel;
// handle the button in a private interrupt handler loop
    int long_press = 0;
// reset the timer
    TIM1->CNT = 0;
    CLEAR_PIN(LED_GPIO, 1 << LED_PIN);
#ifdef SIM_CAM
    USART_Cmd(USART1, DISABLE);
#endif
    while(!PIN_IS_SET(BUTTON_GPIO, 1 << BUTTON_PIN))
    {
// get time in ms
        int time_difference = TIM1->CNT / 4;
        if(time_difference >= LONG_PRESS)
        {
            if(!long_press)
            {
                long_press = 1;
            }
// advance the channel
            channel++;
            if(channel >= TOTAL_CHANNELS)
                channel = 0;
// blink the current channel
            blink_channel();
            TIM1->CNT = 0;
        }
    }
    int time_difference = TIM1->CNT / 4;
// blink the current channel
    if(!long_press && time_difference >= SHORT_PRESS)
    {
        CLEAR_PIN(LED_GPIO, 1 << LED_PIN);
        msleep(500);
        blink_channel();
    }
    SET_PIN(LED_GPIO, 1 << LED_PIN);

// save it
    if(prev_channel != channel)
    {
	    FLASH_Unlock();
	    FLASH_ClearFlag(FLASH_FLAG_EOP | 
		    FLASH_FLAG_OPERR | 
		    FLASH_FLAG_WRPERR | 
    	    FLASH_FLAG_PGAERR | 
		    FLASH_FLAG_PGPERR |
		    FLASH_FLAG_PGSERR); 

        FLASH_EraseSector(FLASH_Sector_3, VoltageRange_3);
        uint32_t channel2 = channel;
        FLASH_ProgramWord(CONFIG_START, channel2);
	    FLASH_Lock(); 

        set_channel();
    }


#ifdef SIM_CAM
    USART_Cmd(USART1, ENABLE);
#endif
//     print_text("EXTI15_10_IRQHandler ");
//     print_number(PIN_IS_SET(BUTTON_GPIO, 1 << BUTTON_PIN));
//     print_number(time_difference);
//     print_lf();
}
