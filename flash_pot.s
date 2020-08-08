;
; flash_pot
; Copyright (C) 1997-2017 Adam Williams <broadcast at earthling dot net>
; 
; This program is free software; you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation; either version 2 of the License, or
; (at your option) any later version.
; 
; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU General Public License for more details.
; 
; You should have received a copy of the GNU General Public License
; along with this program; if not, write to the Free Software
; Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
; 


; LED front end for a digital pot

; gpasm -o flash_pot.hex flash_pot.s

; 18lf458
; usb_programmer -c 0x300000 0000001000000000
; Crystal
; usb_programmer -c 0x300002 0000111100001110
; brownout protection, watchdog timer
; usb_programmer -c 0x300006 0000000010000000
; low voltage programming disabled
; usb_programmer flash_pot.hex
; usb_programmer -r flash_pot.hex




PROCESSOR 18f458
#include "p18f458.inc"


#include "pic_util.inc"


#define CLOCKSPEED 3600000
; LED update time
#define LED_DELAY -(CLOCKSPEED / 4 / 120)

; button timer update time
#define BUTTON_DELAY (-CLOCKSPEED / 4 / 100)

; location in EEPROM of state
#define DATA_EE_ADDR 0x00


#define DEBOUNCE_THRESHOLD 0x10
#define DEBOUNCE_MAX 0x20
; ms of button repeat 1
#define BUTTON_REPEAT1 500
; ms of later button repeats
#define BUTTON_REPEAT2 250

; LED segments
; C2 ground1
; C5 ground2

; C0 middle
; C1 period
; D3 bottom left
; C4 bottom
; C3 top left
; D0 top
; D1 top right
; D2 bottom right

; AD5231 resistor pins
; D4 CLK
; D5 SDI
; D6 SDO
; D7 !CS

; button pins
; B5 
; B4 

#define LED_MASKC B'00011011'
#define LED_MASKD B'00001111'

#define BUTTON_REPEAT1_TOTAL (CLOCKSPEED / (-BUTTON_DELAY) / 4) * BUTTON_REPEAT1 / 1000 + 1
#define BUTTON_REPEAT2_TOTAL (CLOCKSPEED / (-BUTTON_DELAY) / 4) * BUTTON_REPEAT2 / 1000 + 1

; delay before writing to EEPROM
#define EEPROM_DELAY (CLOCKSPEED / (-BUTTON_DELAY) / 4) * 7 + 1


#define GAIN_MAX 10


; AD5231
#define CLK_LAT LATD
#define CLK_TRIS TRISD
#define CLK_PIN 4

#define SDO_LAT LATD
#define SDO_TRIS TRISD
#define SDO_PIN 5

#define CS_LAT LATD
#define CS_TRIS TRISD
#define CS_PIN 7

; pot SPI
#define SPI_DELAY_VALUE 255
#define STARTUP_DELAY_SECONDS 1
; hack this to be less than 1 second for the reset after flash
#define STARTUP_COUNTS (CLOCKSPEED * STARTUP_DELAY_SECONDS / 4 / (-BUTTON_DELAY) + 1) / 4


SEND_SPI macro BYTE1, BYTE2, BYTE3
; 24 bits
	SET_REGISTER SPI_COUNTER, 24
; buffer
	SET_REGISTER SPI_BUFFER + 0, BYTE1
; 2 most significant wiper bits
	SET_REGISTER SPI_BUFFER + 1, BYTE2
; 8 least significant wiper bits
	SET_REGISTER SPI_BUFFER + 2, BYTE3
	SET_REGISTER16 SPI_STATE, spi_start
	bsf FLAGS, FLAG_NOP
	ENDM

SEND_REG_SPI macro
; 24 bits
	SET_REGISTER SPI_COUNTER, 24
	SET_REGISTER16 SPI_STATE, spi_start
	bsf FLAGS, FLAG_NOP
	ENDM



cblock 0x00
	INTERRUPT_COMPLETE : 1
	FLAG_EEPROM_WRITE : 1
; send a NOP to the pot to put it to sleep
	FLAG_NOP : 1
; write gain to pot when ready
	FLAG_SEND_GAIN : 1
; LED is on
	FLAG_LED_ON : 1
endc


	VARSTART H'00', H'100'
	VARADD FLAGS, 1

; LED parameters
	VARADD LED_STATE, 2
; padded for pointer addition
; most significant
	VARADD DIGIT0, 2
; least significant
	VARADD DIGIT1, 2
	VARADD PERIOD_ON, 1

; the gain index
; padded for pointer addition
	VARADD GAIN, 2
	VARADD BUTTON0_DEBOUNCE, 1
	VARADD BUTTON1_DEBOUNCE, 1
	VARADD BUTTON0_COUNTER, 1
	VARADD BUTTON1_COUNTER, 1
	VARADD BUTTON0_STATE, 2
	VARADD BUTTON1_STATE, 2
; delays
    VARADD BUTTON_COUNTER, 2


	VARADD EEPROM_STATE, 2
	VARADD EEPROM_COUNTER, 2

	VARADD SPI_DELAY, 1
	VARADD SPI_STATE, 2
	VARADD SPI_BUFFER, 3
	VARADD SPI_COUNTER, 1
	VARADD STARTUP_COUNTER, 2


	VARADD TEMP, 1
	VARADD TEMP0, 1
	VARADD TEMP1, 1
	VARADD TEMP2, 1



	ORG RESETVECTOR
	goto start
	


	ORG INTVECTORHI
	goto interrupt_handler

	ORG INTVECTORLO
	goto interrupt_handler



	ORG 0x400

start:
	BANKSEL H'00'

	clrf FLAGS

	SET_REGISTER16 GAIN, 0

	call init_led


	call init_eeprom

; buttons
	bcf INTCON2, RBPU
	clrf BUTTON0_DEBOUNCE
	clrf BUTTON1_DEBOUNCE
	SET_REGISTER16 BUTTON0_STATE, button0_idle
	SET_REGISTER16 BUTTON1_STATE, button1_idle
	SET_REGISTER T1CON, B'10000001'


; initialize pot
	CLEAR_REGISTER16 STARTUP_COUNTER

	call init_spi



	call calculate_digits

	bsf INTCON, PEIE
	bsf INTCON, GIE



loop:
	clrwdt

	call handle_led
	
	btfsc PIR1, TMR1IF
	call handle_buttons

	call handle_eeprom	

	call handle_spi

	bra loop




interrupt_handler:
	retfie FAST






handle_buttons:
	bcf PIR1, TMR1IF


	SET_TIMER_LITERAL16 TMR1L, BUTTON_DELAY


; update the eeprom timer when timer1 goes off
	SKIP_LESS_LITERAL16 EEPROM_COUNTER, EEPROM_DELAY
	bra eeprom_delay2
		INC16 EEPROM_COUNTER
eeprom_delay2:




; update the startup timer
	SKIP_LESS_LITERAL16 STARTUP_COUNTER, STARTUP_COUNTS
	bra startup_delay2
		INC16 STARTUP_COUNTER
startup_delay2:



	btfss PORTB, 5
	bra button1_down

; decrease debounce
		SKIP_ZERO BUTTON1_DEBOUNCE
			decf BUTTON1_DEBOUNCE, F
			bra handle_buttons2

; increase debounce
button1_down:
		SKIP_GREATEREQUAL_LITERAL BUTTON1_DEBOUNCE, DEBOUNCE_MAX
			incf BUTTON1_DEBOUNCE, F



handle_buttons2:
	btfss PORTB, 4
	bra button0_down

; decrease debounce
		SKIP_ZERO BUTTON0_DEBOUNCE
			decf BUTTON0_DEBOUNCE, F
			bra handle_buttons3

; increase debounce
button0_down:
		SKIP_GREATEREQUAL_LITERAL BUTTON0_DEBOUNCE, DEBOUNCE_MAX
			incf BUTTON0_DEBOUNCE, F






handle_buttons3:
	SET_PC_REG BUTTON0_STATE
handle_buttons4:
	SET_PC_REG BUTTON1_STATE
handle_buttons5:
	return





button0_idle:
	SKIP_GREATER_LITERAL BUTTON0_DEBOUNCE, DEBOUNCE_THRESHOLD
	bra handle_buttons4

; button pressed
		SET_REGISTER16 BUTTON0_STATE, button0_repeat1
		SET_REGISTER BUTTON0_COUNTER, BUTTON_REPEAT1_TOTAL


		incf GAIN, F
		call update_gain
		bra handle_buttons4

button0_repeat1:
	SKIP_GREATER_LITERAL BUTTON0_DEBOUNCE, DEBOUNCE_THRESHOLD
	bra button0_reset

			decfsz BUTTON0_COUNTER, F
			bra handle_buttons4

				SET_REGISTER16 BUTTON0_STATE, button0_repeat2
				SET_REGISTER BUTTON0_COUNTER, BUTTON_REPEAT2_TOTAL
				incf GAIN, F
				call update_gain
				bra handle_buttons4


button0_repeat2:
	SKIP_GREATER_LITERAL BUTTON0_DEBOUNCE, DEBOUNCE_THRESHOLD
	bra button0_reset

			decfsz BUTTON0_COUNTER, F
			bra handle_buttons4

				SET_REGISTER BUTTON0_COUNTER, BUTTON_REPEAT2_TOTAL
				incf GAIN, F
				call update_gain
				bra handle_buttons4

button0_reset:
	SET_REGISTER16 BUTTON0_STATE, button0_idle
	bra handle_buttons4














button1_idle:
	SKIP_GREATER_LITERAL BUTTON1_DEBOUNCE, DEBOUNCE_THRESHOLD
	bra handle_buttons5  


; button pressed
		SET_REGISTER16 BUTTON1_STATE, button1_repeat1
		SET_REGISTER BUTTON1_COUNTER, BUTTON_REPEAT1_TOTAL

		decf GAIN, F
		call update_gain
		bra handle_buttons5

button1_repeat1:
	SKIP_GREATER_LITERAL BUTTON1_DEBOUNCE, DEBOUNCE_THRESHOLD
	bra button1_reset

			decfsz BUTTON1_COUNTER, F
			bra handle_buttons5

				SET_REGISTER16 BUTTON1_STATE, button1_repeat2
				SET_REGISTER BUTTON1_COUNTER, BUTTON_REPEAT2_TOTAL
				decf GAIN, F
				call update_gain
				bra handle_buttons5


button1_repeat2:
	SKIP_GREATER_LITERAL BUTTON1_DEBOUNCE, DEBOUNCE_THRESHOLD
	bra button1_reset

			decfsz BUTTON1_COUNTER, F
			bra handle_buttons5

				SET_REGISTER BUTTON1_COUNTER, BUTTON_REPEAT2_TOTAL
				decf GAIN, F
				call update_gain
				bra handle_buttons5



button1_reset:
	SET_REGISTER16 BUTTON1_STATE, button1_idle
	bra handle_buttons5





; port C, port D
led_digits:
; 0
	DB B'00011000', B'00001111'
; 1
	DB B'00000000', B'00000110'
; 2
	DB B'00010001', B'00001011'
; 3
	DB B'00010001', B'00000111'
; 4
	DB B'00001001', B'00000110'
; 5
	DB B'00011001', B'00000101'
; 6
	DB B'00011001', B'00001101'
; 7
	DB B'00000000', B'00000111'
; 8
	DB B'00011001', B'00001111'
; 9
	DB B'00001001', B'00000111'


init_led:
; LED timer
	CLEAR_TIMER_REGISTER TMR0L
	bcf INTCON, TMR0IF
	SET_REGISTER T0CON, B'10001000'

	SET_REGISTER16 LED_STATE, led_on1

; led segments off & output mode
	OR_LITERAL LATC, LED_MASKC
	OR_LITERAL LATD, LED_MASKD
	AND_LITERAL TRISC, ~LED_MASKC
	AND_LITERAL TRISD, ~LED_MASKD

; turn off LED grounds
	bsf LATC, 2
	bsf LATC, 5
	bcf TRISC, 2
	bcf TRISC, 5

	SET_REGISTER16 DIGIT0, 6
	SET_REGISTER16 DIGIT1, 7
	clrf PERIOD_ON
	return




handle_led:
	btfss INTCON, TMR0IF
	return
		bcf INTCON, TMR0IF
		SET_TIMER_LITERAL16 TMR0L, LED_DELAY

; turn off all segments
    	AND_LITERAL LATC, ~LED_MASKC
    	AND_LITERAL LATD, ~LED_MASKD

		SET_PC_REG LED_STATE

; show digit 0
led_on1:
	SET_REGISTER16 LED_STATE, led_on2

    SKIP_GREATER_LITERAL DIGIT0, 0
    bra led_on3

	    SET_FLASH_LITERAL led_digits
    ; add digit * 2
	    ADD16 TBLPTRL, DIGIT0
	    ADD16 TBLPTRL, DIGIT0

	    TBLRD*+
	    OR_REG LATC, TABLAT

	    TBLRD*+
	    OR_REG LATD, TABLAT

led_on3:
; multiplex pins
	bsf LATC, 2
	bcf LATC, 5
	return





; show digit 1
led_on2:
	SET_REGISTER16 LED_STATE, led_on1

; turn on period
	btfsc PERIOD_ON, 0
	bsf LATC, 1

	SET_FLASH_LITERAL led_digits
; add digit * 2
	ADD16 TBLPTRL, DIGIT1
	ADD16 TBLPTRL, DIGIT1

	TBLRD*+
	OR_REG LATC, TABLAT

	TBLRD*+
	OR_REG LATD, TABLAT

; multiplex pins
	bcf LATC, 2
	bsf LATC, 5
	return




; called whenever user changes gain
; writes the eeprom, pot, & updates the display
update_gain:
	bsf FLAGS, FLAG_EEPROM_WRITE
	CLEAR_REGISTER16 EEPROM_COUNTER
	bsf FLAGS, FLAG_SEND_GAIN

; convert the gain into display digits
calculate_digits:
	SKIP_EQUAL_LITERAL GAIN, 0xff
	bra calculate_digits4
		SET_REGISTER GAIN, GAIN_MAX

calculate_digits4:
	SKIP_GREATER_LITERAL GAIN, GAIN_MAX
	bra calculate_digits3
		SET_REGISTER GAIN, 0

calculate_digits3:
	clrf DIGIT0
	clrf DIGIT1
	
	COPY_REGISTER TEMP, GAIN
calculate_digits1:
	SKIP_GREATER_LITERAL TEMP, 9
	bra calculate_digits2
		SUBTRACT_LITERAL TEMP, 10
		incf DIGIT0, F
		bra calculate_digits1
		
calculate_digits2:
	COPY_REGISTER DIGIT1, TEMP
	return






init_eeprom:
; read gain from EEPROM
	SET_REGISTER EEADR, DATA_EE_ADDR
	bcf EECON1, EEPGD
	bcf EECON1, CFGS
	bsf EECON1, RD
	COPY_REGISTER GAIN, EEDATA
    
	SET_REGISTER16 EEPROM_STATE, eeprom_idle
	CLEAR_REGISTER16 EEPROM_COUNTER
	return



handle_eeprom:
	SET_PC_REG EEPROM_STATE



eeprom_idle:
; must write immediately because the flash discharge resets the micro
;	SKIP_GREATEREQUAL_LITERAL16 EEPROM_COUNTER, EEPROM_DELAY
;	return

; need EEPROM write
		btfss FLAGS, FLAG_EEPROM_WRITE
		return

; write it
			SET_REGISTER16 EEPROM_STATE, eeprom_write1
			bcf PIR2, EEIF
			bcf FLAGS, FLAG_EEPROM_WRITE
			SET_REGISTER EEADR, DATA_EE_ADDR
			COPY_REGISTER EEDATA, GAIN
			bcf EECON1, EEPGD
			bcf EECON1, CFGS
			bsf EECON1, WREN

; disable interrupts
			bcf INTCON, GIE
			SET_REGISTER EECON2, 0x55
			SET_REGISTER EECON2, 0xAA
			bsf EECON1, WR
; enable interrupts
			bsf INTCON, GIE
			return



eeprom_write1:
	btfss PIR2, EEIF
	return

;        clrf PERIOD_ON
		SET_REGISTER16 EEPROM_STATE, eeprom_idle
		CLEAR_REGISTER16 EEPROM_COUNTER
		return








init_spi:
; all pins high
	bsf CLK_LAT, CLK_PIN
	bsf CS_LAT, CS_PIN
	bsf SDO_LAT, SDO_PIN

	bcf CLK_TRIS, CLK_PIN
	bcf CS_TRIS, CS_PIN
	bcf SDO_TRIS, SDO_PIN
; send initial configuration to the pot when startup timer expires
	SET_REGISTER16 SPI_STATE, init_pot
	return



handle_spi:
	decfsz SPI_DELAY, F
	return

		SET_REGISTER SPI_DELAY, SPI_DELAY_VALUE
		SET_PC_REG SPI_STATE


; send initial configuration to the pot when startup timer expires
init_pot:
	SKIP_EQUAL_LITERAL16 STARTUP_COUNTER, STARTUP_COUNTS
	return
		bra send_gain

spi_idle:
; need to send gain?
	btfsc FLAGS, FLAG_SEND_GAIN
		bra send_gain
	return

spi_start:
; all pins high
	bsf CLK_LAT, CLK_PIN
	bsf CS_LAT, CS_PIN
	bsf SDO_LAT, SDO_PIN
	SET_REGISTER16 SPI_STATE, spi_start2
	return

spi_start2:
; cs low
	bcf CS_LAT, CS_PIN
	SET_REGISTER16 SPI_STATE, spi_bit1
	return

spi_bit1:
	SKIP_NONZERO SPI_COUNTER
	bra spi_done

; clk low
	bcf CLK_LAT, CLK_PIN
; shift in bit
    COPY_BIT SDO_LAT, SDO_PIN, SPI_BUFFER + 0, 7
	
	rlcf SPI_BUFFER + 2, F
	rlcf SPI_BUFFER + 1, F
	rlcf SPI_BUFFER + 0, F

; decrease counter
		decf SPI_COUNTER, F
		SET_REGISTER16 SPI_STATE, spi_bit2
		return


spi_bit2:
; raise clock
	bsf CLK_LAT, CLK_PIN
	SET_REGISTER16 SPI_STATE, spi_bit1
	return

spi_done:
; all pins high
	bsf CLK_LAT, CLK_PIN
	bsf CS_LAT, CS_PIN
	bsf SDO_LAT, SDO_PIN

	SET_REGISTER16 SPI_STATE, spi_done2
	btfsc FLAGS, FLAG_NOP
		bra spi_nop
		return

spi_done2:
; all pins low
	bcf CLK_LAT, CLK_PIN
	bcf CS_LAT, CS_PIN
	bcf SDO_LAT, SDO_PIN
; need to send gain?
	SET_REGISTER16 SPI_STATE, spi_idle
	btfsc FLAGS, FLAG_SEND_GAIN
		bra send_gain

    clrf PERIOD_ON
	return

spi_nop:
	SEND_SPI 0x00, 0x00, 0x00
	bcf FLAGS, FLAG_NOP
	return

; resistor values for each gain
; resistance is 60 + DW * 40
gain_table:
    DW 1   ; gain 0
    DW 2   ; gain 1
    DW 4   ; gain 2
    DW 6   ; gain 3
    DW 8   ; gain 4
    DW 11   ; gain 5
    DW 15   ; gain 6
    DW 20   ; gain 7
    DW 30   ; gain 8
    DW 50   ; gain 9
    DW 1023      ; gain 10


send_gain:
	bcf FLAGS, FLAG_SEND_GAIN
    setf PERIOD_ON

; look up resistor value based on gain index
    SET_FLASH_LITERAL gain_table
; add gain * 2
	ADD16 TBLPTRL, GAIN
	ADD16 TBLPTRL, GAIN

; set data bytes to table value
    TBLRD*+
	COPY_REGISTER SPI_BUFFER + 2, TABLAT
    TBLRD*+
	COPY_REGISTER SPI_BUFFER + 1, TABLAT

; set command byte to 11, page 17
	SET_REGISTER SPI_BUFFER + 0, B'10110000'
	SEND_REG_SPI
	return



END









