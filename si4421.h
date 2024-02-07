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


#define RADIO_PACKETSIZE 10
extern const uint8_t salt[RADIO_PACKETSIZE * 2];

// type of packet captured
#define TYPE_NONE 0
#define TYPE_POWERON 1
#define TYPE_METERING1 2
#define TYPE_METERING2 3
#define TYPE_PREFLASH1 4
#define TYPE_PREFLASH2 5
#define TYPE_MANE_FLASH1 6
#define TYPE_MANE_FLASH2 7
#define TYPE_FAST_FLASH 8
#define TYPE_MANUAL_FLASH 9

#define TRIGGER_CODE_CLK 0xaa // CLK 0V
#define TRIGGER_CODE_X 0x55 // CLK & X 0V
#define TRIGGER_DEBOUNCE 2

// RADIO_CHANNEL is from 96-3903 & set by the user
// data rate must be slow enough to service FIFOs
// kbps = 10000 / (29 * (DRVSREG<6:0> + 1) * (1 + DRPE * 7))
// RADIO_BAUD_CODE = 10000 / (29 * kbps) / (1 + DRPE * 7) - 1
// RADIO_DATA_SIZE is the amount of data to read before resetting the sync code

#define MIN_CHANNEL 96
#define MAX_CHANNEL 3903

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











