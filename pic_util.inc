#ifndef PIC_UTIL_INC
#define PIC_UTIL_INC

;#include "bootloader.h"

; all numbers default to base 10
	RADIX DEC

#define RESETVECTOR	0x0000
#define INTVECTORHI	0x0008
#define INTVECTORLO	0x0018

; the highest byte the bootloader can access.  The top of the "access bank".
ifdef __18F2450
BOOTLOADER_BASE = H'5f'
endif
ifdef __18F458
BOOTLOADER_BASE = H'5f'
endif
ifdef __18F4580
BOOTLOADER_BASE = H'5f'
endif
ifdef __18F1320
BOOTLOADER_BASE = H'7f'
endif

; the pins change if PORTB is used for sonar
; the pin used for the program clock
#define BOOTLOADER_PGC PORTB, D'6'
; the pin used for the program data
#define BOOTLOADER_PGD PORTB, D'7'
; the pin used for the program data direction
#define BOOTLOADER_PGD_DIR TRISB, D'7'


#define POINTER0 FSR0L
#define POINTER1 FSR1L
#define POINTER2 FSR2L



VARSTART macro MSTART, MEND
VARCURRENT SET MSTART
VAREND SET MEND
	ENDM

VARADD macro VARNAME, SIZE
	IF (VARCURRENT + SIZE > VAREND)
  		ERROR "Too many variables for allocated file registers"
	ENDIF
VARNAME EQU VARCURRENT
VARCURRENT SET VARCURRENT + SIZE
	ENDM




; set flash pointer to literal
SET_FLASH_LITERAL macro reg
	movlw LOW(reg)
	movwf TBLPTRL
	movlw HIGH(reg)
	movwf TBLPTRH
	clrf TBLPTRU
	ENDM

; set flash pointer to reg
SET_FLASH_REG macro reg
	movf reg, W
	movwf TBLPTRL
	movf reg + 1, W
	movwf TBLPTRH
	clrf TBLPTRU
	ENDM

; copy the 16 bit register to the program counter
SET_PC_REG macro reg
	movff reg + 1, PCLATH
	movf reg, W
	movwf PCL
	ENDM

; set pointer0 to the register address
SET_POINTER0_LITERAL macro reg
	lfsr 0, reg
	ENDM

; set pointer0 to the contents of the register
SET_POINTER0_REG macro reg
	movff reg, FSR0L
	movff reg + 1, FSR0H
	ENDM

; set pointer0 to the contents of the register
SET_POINTER1_REG macro reg
	movff reg, FSR1L
	movff reg + 1, FSR1H
	ENDM

; set pointer2 to the contents of the register
SET_POINTER2_REG macro reg
	movff reg, FSR2L
	movff reg + 1, FSR2H
	ENDM

; set pointer1 to point to the register
SET_POINTER1_LITERAL macro reg
	lfsr 1, reg
	ENDM

; set pointer1 to point to the register
SET_POINTER2_LITERAL macro reg
	lfsr 2, reg
	ENDM

; data for the 18f1220 stores a 16 bit value
DATA_BE macro word1, word2
	DATA word1
	DATA word2
	ENDM

SET_REGISTER macro reg, literal
	movlw literal
	movwf reg
	ENDM

SET_REGISTER16 macro reglo, literal
	movlw LOW(literal)
	movwf reglo
	movlw HIGH(literal)
	movwf reglo + 1
	ENDM

SET_POSTINC16 macro ptr, literal
	movlw LOW(literal)
	movwf ptr
	movlw HIGH(literal)
	movwf ptr
	ENDM

; timer must be written high byte first or the high byte will be discarded
SET_TIMER_LITERAL16 macro reglo, literal
	movlw HIGH(literal)
	movwf reglo + 1
	movlw LOW(literal)
	movwf reglo
	ENDM

; timer must be written high byte first or the high byte will be discarded
SET_TIMER_REG16 macro dst, src
	movff src + 1, dst + 1
	movff src, dst
	ENDM

; the assembler doesn't seem to take 32 bit literals so we
; split the literal into shorts for this one
SET_REGISTER24 macro reglo, literal23_16, literal15_0
	movlw LOW(literal15_0)
	movwf reglo
	movlw HIGH(literal15_0)
	movwf reglo + 1
	movlw literal23_16
	movwf reglo + 2
	ENDM

; the assembler doesn't seem to take 32 bit literals so we
; split the literal into shorts for this one
SET_REGISTER32 macro reglo, literal31_16, literal15_0
	movlw LOW(literal15_0)
	movwf reglo
	movlw HIGH(literal15_0)
	movwf reglo + 1
	movlw LOW(literal31_16)
	movwf reglo + 2
	movlw HIGH(literal31_16)
	movwf reglo + 3
	ENDM

; the assembler doesn't seem to take 32 bit literals so we
; split the literal into shorts for this one
SET_REGISTER48 macro reglo, literal47_32, literal31_16, literal15_0
	movlw HIGH(literal47_32)
	movwf reglo + D'5'
	movlw LOW(literal47_32)
	movwf reglo + 4
	movlw HIGH(literal31_16)
	movwf reglo + 3
	movlw LOW(literal31_16)
	movwf reglo + 2
	movlw HIGH(literal15_0)
	movwf reglo + 1
	movlw LOW(literal15_0)
	movwf reglo
	ENDM

; the assembler doesn't seem to take 32 bit literals so we
; split the literal into shorts for this one
SET_REGISTER64 macro reglo, literal63_48, literal47_32, literal31_16, literal15_0
	movlw LOW(literal15_0)
	movwf reglo
	movlw HIGH(literal15_0)
	movwf reglo + 1
	movlw LOW(literal31_16)
	movwf reglo + 2
	movlw HIGH(literal31_16)
	movwf reglo + 3
	movlw LOW(literal47_32)
	movwf reglo + 4
	movlw HIGH(literal47_32)
	movwf reglo + D'5'
	movlw LOW(literal63_48)
	movwf reglo + D'6'
	movlw HIGH(literal63_48)
	movwf reglo + D'7'
	ENDM

CLEAR_REGISTER48 macro reglo
	clrf reglo
	clrf reglo + 1
	clrf reglo + 2
	clrf reglo + 3
	clrf reglo + 4
	clrf reglo + D'5'
	ENDM

CLEAR_REGISTER32 macro reglo
	clrf reglo
	clrf reglo + 1
	clrf reglo + 2
	clrf reglo + 3
	ENDM

CLEAR_REGISTER24 macro reglo
	clrf reglo
	clrf reglo + 1
	clrf reglo + 2
	ENDM

CLEAR_REGISTER16 macro reglo
	clrf reglo
	clrf reglo + 1
	ENDM

CLEAR_POINTER16 macro reglo
	clrf reglo
	clrf reglo
	ENDM

CLEAR_POSTINC32 macro reglo
	clrf reglo
	clrf reglo
	clrf reglo
	clrf reglo
	ENDM

; clear a timer register
CLEAR_TIMER_REGISTER macro reglo
	clrf reglo + 1
	clrf reglo
	ENDM

; set to full positive
MAX_REGISTER16 macro reglo
	setf reglo
	setf reglo + 1
	ENDM

; increment register by 1 without changing status bits
INC_REGISTER macro reg
	movf STATUS, W        ; store carry bit
	incf reg, F
	movwf STATUS          ; retrieve carry bit
	ENDM

; these may not be used with POSTINC*
; copy value of src register to dst register
COPY_REGISTER64 macro dst, src
	movff src, dst
	movff src + 1, dst + 1
	movff src + 2, dst + 2
	movff src + 3, dst + 3
	movff src + 4, dst + 4
	movff src + D'5', dst + D'5'
	movff src + D'6', dst + D'6'
	movff src + D'7', dst + D'7'
	ENDM

COPY_REGISTER32 macro dst, src
	movff src, dst
	movff src + 1, dst + 1
	movff src + 2, dst + 2
	movff src + 3, dst + 3
	ENDM

; copy value of src register to dst register
COPY_REGISTER16 macro dst, src
	movff src, dst
	movff src + 1, dst + 1
	ENDM;

; copy value of src postinc pointer to dst postinc pointer
COPY_POINTER16 macro dst, src
	movff src, dst
	movff src, dst
	ENDM;

; copy value of postinc pointer to dst
COPY_REG16_POINTER16 macro dst, ptr
	movff ptr, dst
	movff ptr, dst + 1
	ENDM;

; copy value of postinc pointer to dst
COPY_REG16_POSTINC16 macro dst, ptr
	movff ptr, dst
	movff ptr, dst + 1
	ENDM;

; copy value of postinc pointer to dst
COPY_REG16_PTR16 macro dst, ptr
	movff ptr, dst
	movff ptr, dst + 1
	ENDM;

; copy value of postinc pointer to dst
COPY_REG24_PTR24 macro dst, ptr
	movff ptr, dst
	movff ptr, dst + 1
	movff ptr, dst + 2
	ENDM

; copy value of src to dst postinc pointer
COPY_POSTINC16_REG16 macro ptr, src
	movff src, ptr
	movff src + 1, ptr
	ENDM

; copy value of src to dst postinc pointer
COPY_POSTINC32_REG32 macro ptr, src
	movff src, ptr
	movff src + 1, ptr
	movff src + 2, ptr
	movff src + 3, ptr
	ENDM

; copy value of postinc pointer to dst 
COPY_REG32_POSTINC32 macro dst, ptr
	movff ptr, dst
	movff ptr, dst + 1
	movff ptr, dst + 2
	movff ptr, dst + 3
	ENDM

; copy value of src register to dst register
COPY_REGISTER macro dst, src
	movff src, dst
	ENDM

; movff doesn't work with SSPBUF
COPY_REGISTER_SSP macro dst, src
	movf src, W
	movwf dst
	ENDM

; want only 1 write to the destination register
COPY_BIT macro dst_reg, dst_bit, src_reg, src_bit
	btfsc src_reg, src_bit
	bsf dst_reg, dst_bit
	btfss src_reg, src_bit
	bcf dst_reg, dst_bit
	ENDM


; copy value of src register to dst register starting with the high byte
; required when setting timers
; reading timers requires copying the low byte first
COPY_TO_TIMER macro dst, src
	movff src + 1, dst + 1
	movff src, dst
	ENDM

POSTDEC_TO_TIMER macro dst, src
	movff src, dst + 1
	movff src, dst
	ENDM

; multiply 8 bit register by 8 bit literal and store low 8 bits in reg
MULTIPLY_DST8_LITERAL macro reg, literal
	movlw literal
	mulwf reg
	COPY_REGISTER reg, PRODL
	ENDM

; multiply 16 bit POSTINC pointer by 8 bit literal and store in 24 bit dst
MULTIPLY_PTR16_LITERAL8 macro dst, ptr, literal
	clrf dst + 2
	movlw literal
	mulwf ptr
	COPY_REGISTER16 dst, PRODL
	movlw literal
	mulwf ptr
	ADD16 dst + 1, PRODL
	ENDM

; multiply 16 bit register by 8 bit literal and store in 24 bit dst
MULTIPLY_REG16_LITERAL8 macro dst, reg, literal
	clrf dst + 2
	movlw literal
	mulwf reg
	COPY_REGISTER16 dst, PRODL
	movlw literal
	mulwf reg + 1
	ADD16 dst + 1, PRODL
	ENDM


; multiply 16 bit literal by 8 bit reg and store in 24 bit dst
MULTIPLY_REG8_LITERAL16 macro dst, reg, literal
	clrf dst + 2
; low byte
	movlw LOW(literal)
	mulwf reg
	COPY_REGISTER16 dst, PRODL
; high byte
	movlw HIGH(literal)
	mulwf reg
	ADD16 dst + 1, PRODL
	ENDM



; multiply 16 bit unsigned reg1 by 8 bit reg2 and store in 24 bit dst
MULTIPLY_REG16_REG8 macro dst, reg1, reg2
	clrf dst + 2

	movf reg2, W
	mulwf reg1
	COPY_REGISTER16 dst, PRODL

	mulwf reg1 + 1
	ADD16 dst + 1, PRODL
	ENDM

; multiply signed 16 bit register by 8 bit literal and store in 24 bit dst
; overwrites TEMP5
MULTIPLY_SIGNED_REG16_LITERAL8 macro dst, reg, literal
	clrf dst + 2
	movlw literal
	mulwf reg
	COPY_REGISTER16 dst, PRODL
	movlw literal
	mulwf reg + 1
	ADD16 dst + 1, PRODL

; carry sign to extra byte.  Use PRODL as temporary input.
	clrf TEMP5
	btfsc reg + 1, 7
	setf TEMP5
	movlw literal
	mulwf TEMP5
	ADD dst + 2, PRODL
	ENDM


; Multiply 24 bit register by 8 bit literal and store in 32 bit dst
MULTIPLY_REG24_LITERAL8 macro dst, reg, literal
	clrf dst + 2
	clrf dst + 3
	movlw literal
	mulwf reg
	COPY_REGISTER16 dst, PRODL
	movlw literal
	mulwf reg + 1
	ADD16 dst + 1, PRODL
	movlw literal
	mulwf reg + 2
	ADD16 dst + 2, PRODL
	ENDM

; multiply 32 bit POSTINC pointer by 8 bit literal and store in 40 bit dst
MULTIPLY_PTR32_LITERAL8 macro dst, ptr, literal
; speed up calculation
	clrf dst + 2
	clrf dst + 3
	clrf dst + 4
; multiply byte 0
	movlw literal
	mulwf ptr
	COPY_REGISTER16 dst, PRODL
; multiply byte 1
	movlw literal
	mulwf ptr
	ADD16 dst + 1, PRODL
; multiply byte 2
	movlw literal
	mulwf ptr
	ADD16 dst + 2, PRODL
; multiply byte 3
	movlw literal
	mulwf ptr
	ADD16 dst + 3, PRODL
	ENDM

; multiply 32 bit POSTINC pointer by 8 bit reg and store in 40 bit dst
MULTIPLY_PTR32_REG8 macro dst, ptr, reg
; speed up calculation
	clrf dst + 2
	clrf dst + 3
	clrf dst + 4
; multiply byte 0
	movf reg, W
	mulwf ptr
	COPY_REGISTER16 dst, PRODL
; multiply byte 1
	movf reg, W
	mulwf ptr
	ADD16 dst + 1, PRODL
; multiply byte 2
	movf reg, W
	mulwf ptr
	ADD16 dst + 2, PRODL
; multiply byte 3
	movf reg, W
	mulwf ptr
	ADD16 dst + 3, PRODL
	ENDM

; multiply 32 bit register by 8 bit literal and store in 40 bit dst
MULTIPLY_REG32_LITERAL8 macro dst, reg, literal
	clrf dst + 2
	clrf dst + 3
	clrf dst + 4
; multiply byte 0
	movlw literal
	mulwf reg
	COPY_REGISTER16 dst, PRODL
; multiply byte 1
	movlw literal
	mulwf reg + 1
	ADD16 dst + 1, PRODL
; multiply byte 2
	movlw literal
	mulwf reg + 2
	ADD16 dst + 2, PRODL
; multiply byte 3
	movlw literal
	mulwf reg + 3
	ADD16 dst + 3, PRODL
	ENDM



AND_LITERAL macro reg, literal
	movlw literal
	andwf reg, F
	ENDM

AND_REG macro reg1, reg2
	movf reg2, W
	andwf reg1, F
	ENDM



OR_LITERAL macro reg, literal
	movlw literal
	iorwf reg, F
	ENDM

XOR_LITERAL macro reg, literal
	movlw literal
	xorwf reg, F
	ENDM

XOR_REG macro reg1, reg2
	movf reg2, W
	xorwf reg1, F
	ENDM

OR_REG macro reg1, reg2
	movf reg2, W
	iorwf reg1, F
	ENDM


; adds the src bit to the destination bit
; in other words, if the src bit is 1, the destination bit is toggled
ADD_BIT macro dst_reg, dst_bit, src_reg, src_bit
	btfsc src_reg, src_bit
	btg dst_reg, dst_bit
	ENDM

; add literal to register and store in register
ADD_LITERAL macro reg, literal
	movlw literal
	addwf reg, F
	ENDM

; add reg1 to reg2 and store in reg1
ADD macro reg1, reg2
	movf reg2, W
	addwf reg1, F
	ENDM

; add reg1 to reg2 and store in dst
ADD_DST macro dst, reg1, reg2
	movf reg2, W
	addwf reg1, W
	movwf dst
	ENDM

; add reg1 to literal and store in dst
ADD_DST_LITERAL macro dst, reg1, literal
	movlw literal
	addwf reg1, W
	movwf dst
	ENDM

; add 16 bit registers
ADD16 macro dstlo, srclo
	movf srclo, W
	addwf dstlo, F
	movf srclo + 1, W
	addwfc dstlo + 1, F
	ENDM

; add signed reg8 to reg16
ADD_REG16_REG8 macro reg16, reg8
	movf reg8, W
	addwf reg16, F
; carry sign bit
	movlw 0
	btfsc reg8, 7
		movlw H'ff'
	addwfc reg16 + 1, F
	ENDM

; add unsigned reg8 to reg16
ADD_REG16_UNSIGNEDREG8 macro reg16, reg8
	movf reg8, W
	addwf reg16, F
; carry sign bit
	movlw 0
	addwfc reg16 + 1, F
	ENDM

; add 16 bit postinc pointer to register
ADD16_POSTINC macro dstlo, ptr
	movf ptr, W
	addwf dstlo, F
	movf ptr, W
	addwfc dstlo + 1, F
	ENDM

; add 16 bit registers & store in dst
ADD_DST16_REG16_REG16 macro dst, src1, src2
	movf src2, W
	addwf src1, W
	movwf dst
	movf src2 + 1, W
	addwfc src1 + 1, W
	movwf dst + 1
	ENDM

; add 16 bit register, 16 bit literal & store in dst
ADD_DST16_REG16_LITERAL16 macro dst, src1, literal
	movlw LOW(literal)
	addwf src1, W
	movwf dst
	movlw HIGH(literal)
	addwfc src1 + 1, W
	movwf dst + 1
	ENDM

; add 24 bit registers
ADD24 macro dstlo, srclo
	movf srclo, W
	addwf dstlo, F

	movf srclo + 1, W
	addwfc dstlo + 1, F

	movf srclo + 2, W
	addwfc dstlo + 2, F
	ENDM

; increment 16 bit register
INC16 macro reg
	infsnz reg, F
	incf reg + 1, F
	ENDM

; add 16 bit register to literal
ADD_LITERAL16 macro dstlo, literal
	movlw LOW(literal)
	addwf dstlo, F
	movlw HIGH(literal)
	addwfc dstlo + 1, F
	ENDM

; add 24 bit literal to 24 bit register
ADD_LITERAL24 macro reg, literalhi, literallo
	movlw LOW(literallo)
	addwf reg, F
	movlw HIGH(literallo)
	addwfc reg + 1, F
	movlw LOW(literalhi)
	addwfc reg + 2, F
	movlw HIGH(literalhi)
	addwfc reg + 3, F
	ENDM


; add 32 bit literal to 32 bit register
ADD_LITERAL32 macro reg, literalhi, literallo
	movlw LOW(literallo)
	addwf reg, F
	movlw HIGH(literallo)
	addwfc reg + 1, F
	movlw LOW(literalhi)
	addwfc reg + 2, F
	movlw HIGH(literalhi)
	addwfc reg + 3, F
	ENDM


; add 32 bit register to 32 bit register
ADD32 macro dstlo, srclo
	movf srclo, W
	addwf dstlo, F
	movf srclo + 1, W
	addwfc dstlo + 1, F
	movf srclo + 2, W
	addwfc dstlo + 2, F
	movf srclo + 3, W
	addwfc dstlo + 3, F
	ENDM

; add signed 16 bit register to 32 bit register
ADD_REG32_REG16 macro dstlo, srclo
	movf srclo, W
	addwf dstlo, F
	movf srclo + 1, W
	addwfc dstlo + 1, F
; carry sign bit
	movlw 0
	btfsc srclo + 1, 7
		movlw H'ff'
	addwfc dstlo + 2, F
	addwfc dstlo + 3, F
	ENDM

; add signed 16 bit register to 24 bit register
ADD_REG24_REG16 macro dstlo, srclo
	movf srclo, W
	addwf dstlo, F
	movf srclo + 1, W
	addwfc dstlo + 1, F
; carry sign bit
	movlw 0
	btfsc srclo + 1, 7
		movlw H'ff'
	addwfc dstlo + 2, F
	ENDM

ADD_REG24_REG24_REG24 macro DST, SRC1, SRC2
	movf SRC2, W
	addwf SRC1, W
	movwf DST
	
	movf SRC2 + 1, W
	addwfc SRC1 + 1, W
	movwf DST + 1

	movf SRC2 + 2, W
	addwfc SRC1 + 2, W
	movwf DST + 2
	ENDM


; add unsigned reg8 to reg24
ADD_REG24_UNSIGNEDREG8 macro reg24, reg8
	movf reg8, W
	addwf reg24, F
; carry sign bit
	movlw 0
	addwfc reg24 + 1, F
	addwfc reg24 + 2, F
	ENDM

; add 40 bit register to 40 bit register
ADD40 macro dstlo, srclo
	movf srclo, W
	addwf dstlo, F
	movf srclo + 1, W
	addwfc dstlo + 1, F
	movf srclo + 2, W
	addwfc dstlo + 2, F
	movf srclo + 3, W
	addwfc dstlo + 3, F
	movf srclo + 4, W
	addwfc dstlo + 4, F
	ENDM




; subtract reg2 from reg1 and store result in reg1
SUBTRACT macro reg1, reg2
	movf reg2, W
	subwf reg1, F
	ENDM

; subtract reg2 from reg1 and store result in reg1
SUBTRACT_REGISTER macro reg1, reg2
	movf reg2, W
	subwf reg1, F
	ENDM

; dst = reg1 - reg2
SUBTRACT_DST_REG macro dst, reg1, reg2
	movf reg2, W
	subwf reg1, W
	movwf dst
	ENDM

; subtract literal from register and store result in register
SUBTRACT_LITERAL macro reg, literal
	movlw literal
	subwf reg, F
	ENDM

; subtract literal from register and store result in register
SUBTRACT_DST_LITERAL macro dst, reg, literal
	movlw literal
	subwf reg, W
	movwf dst
	ENDM

; subtract 16 bit register srclo from dstlo and store result in dstlo
SUBTRACT16 macro dstlo, srclo
	movf srclo, W
	subwf dstlo, F
	movf srclo + 1, W
	subwfb dstlo + 1, F
	ENDM

; subtract literal from register and store result in register
SUBTRACT_DST16_LITERAL16 macro dst, reg, literal
	movlw LOW(literal)
	subwf reg, W
	movwf dst
	movlw HIGH(literal)
	subwfb reg + 1, W
	movwf dst + 1
	ENDM

; subtract signed reg8 from reg16
SUBTRACT_REG16_REG8 macro reg16, reg8
	movf reg8, W
	subwf reg16, F
; carry sign bit
	movlw 0
	btfsc reg8, 7
		movlw H'ff'
	subwfb reg16 + 1, F
	ENDM

; subtract 16 bit register src2 from src1 and store result in dst
SUBTRACT_DST16_REG16_REG16 macro dst, src1, src2
	movf src2, W
	subwf src1, W
	movwf dst
	movf src2 + 1, W
	subwfb src1 + 1, W
	movwf dst + 1
	ENDM

; subtract 16 bit pointer srclo from register dstlo and store result in dstlo
SUBTRACT_REG_PTR16 macro dstlo, srclo
	movf srclo, W
	subwf dstlo, F
	movf srclo, W
	subwfb dstlo + 1, F
	ENDM

; subtract 16 bit reg srclo from pointer dstlo
SUBTRACT_PTR_REG16 macro dstlo, srclo
	movf srclo, W
	subwf dstlo, F
	movf srclo + 1, W
	subwfb dstlo, F
	ENDM

; subtract 24 bit reg srclo from pointer dstlo
SUBTRACT_PTR_REG24 macro dstlo, srclo
	movf srclo, W
	subwf dstlo, F
	movf srclo + 1, W
	subwfb dstlo, F
	movf srclo + 2, W
	subwfb dstlo, F
	ENDM

; subtract 16 bit pointer src2 from register src1 and store result in dst
SUBTRACT_DST_REG_PTR16 macro dst, src1, src2
	movf src2, W
	subwf src1, W
	movwf dst
	movf src2, W
	subwfb src1 + 1, W
	movwf dst + 1
	ENDM

; subtract 16 bit register src2 from register src1 and store result in pointer
SUBTRACT_POINTER16_REG16_REG16 macro dst, src1, src2
	movf src2, W
	subwf src1, W
	movwf dst
	movf src2 + 1, W
	subwfb src1 + 1, W
	movwf dst
	ENDM

; subtract 24 bit register src2 from register src1 and store result in dst
SUBTRACT_DST24_REG24_REG24 macro dst, src1, src2

	movf src2, W
	subwf src1, W
	movwf dst

	movf src2 + 1, W
	subwfb src1 + 1, W
	movwf dst + 1

	movf src2 + 2, W
	subwfb src1 + 2, W
	movwf dst + 2
	ENDM

; subtract 16 bit register src2 from 24 bit register src1 and store 
; result in 24 bit dst register
SUBTRACT_DST24_REG24_REG16 macro dst, src1, src2
	movf src2, W
	subwf src1, W
	movwf dst
	movf src2 + 1, W
	subwfb src1 + 1, W
	movwf dst + 1
	movlw 0
	subwfb src1 + 2, W
	movwf dst + 2
	ENDM

; subtract 16 bit register src2 from 16 bit register src1 and store 
; result in 24 bit dst register
SUBTRACT_DST24_REG16_REG16 macro dst, src1, src2
	movf src2, W
	subwf src1, W
	movwf dst

	movf src2 + 1, W
	subwfb src1 + 1, W
	movwf dst + 1

	movlw 0
	clrf dst + 2
	subwfb dst + 2, F
	ENDM

; subtract 16 bit register src2 from 16 bit pointer ptr1 and store 
; result in 24 bit dst register
SUBTRACT_DST24_PTR16_REG16 macro dst, ptr1, src2
	movf src2, W
	subwf ptr1, W
	movwf dst

	movf src2 + 1, W
	subwfb ptr1, W
	movwf dst + 1

	movlw 0
	clrf dst + 2
	subwfb dst + 2, F
	ENDM

; subtract 16 bit postinc pointer srclo from dstlo and store result in dstlo
SUBTRACT_POINTER16 macro dstlo, srclo
	movf srclo, W
	subwf dstlo, F
	movf srclo, W
	subwfb dstlo, F
	ENDM

; subtract literal from reg
SUBTRACT_LITERAL16 macro dstlo, literal
	movlw LOW(literal)
	subwf dstlo, F
	movlw HIGH(literal)
	subwfb dstlo + 1, F
	ENDM

; subtract literal from POSTINC
SUBTRACT_POSTINC_LITERAL16 macro dstlo, literal
	movlw LOW(literal)
	subwf dstlo, F
	movlw HIGH(literal)
	subwfb dstlo, F
	ENDM

; subtract reg from literal and place in reg
SUBTRACT_LITERAL_REG16 macro literal, dstlo
	bsf STATUS, C
	movlw LOW(literal)
	subfwb dstlo, F
	movlw HIGH(literal)
	subfwb dstlo + 1, F
	ENDM

; subtract srclo from dstlo and store result in dstlo
SUBTRACT32 macro dstlo, srclo
	movf srclo, W
	subwf dstlo, F
	movf srclo + 1, W
	subwfb dstlo + 1, F
	movf srclo + 2, W
	subwfb dstlo + 2, F
	movf srclo + 3, W
	subwfb dstlo + 3, F
	ENDM


; subtract srclo from dstlo and store result in dstlo
SUBTRACT24 macro dstlo, srclo
	movf srclo, W
	subwf dstlo, F
	movf srclo + 1, W
	subwfb dstlo + 1, F
	movf srclo + 2, W
	subwfb dstlo + 2, F
	ENDM


; subtract 32 bit literal from 32 bit register
SUBTRACT_LITERAL32 macro reg, literalhi, literallo
	movlw LOW(literallo)
	subwf reg, F
	movlw HIGH(literallo)
	subwfb reg + 1, F
	movlw LOW(literalhi)
	subwfb reg + 2, F
	movlw HIGH(literalhi)
	subwfb reg + 3, F
	ENDM



; skip if the contents of the registers are equal
SKIP_EQUAL macro reg1, reg2
	movf reg1, W
	cpfseq reg2
	ENDM

; skip if the contents of the registers are not equal
SKIP_NOTEQUAL macro reg1, reg2
	movf reg1, W
	subwf reg2, W
	btfsc STATUS, Z
	ENDM

; skip if the contents of the registers are not equal
SKIP_NOTEQUAL_LITERAL16 macro reg, literal
	movlw LOW(literal)
; skip if 1st bytes equal
	cpfseq reg
	bra $ + 10
	movlw HIGH(literal)
	subwf reg + 1, W
; status Z is 0 if not equal
	btfsc STATUS, Z
	ENDM

; skip if the contents of the registers are not equal
SKIP_NOTEQUAL16 macro reg1, reg2
	movf reg1, W
; skip if 1st bytes equal
	cpfseq reg2
	bra $ + 10
	movf reg1 + 1, W
	subwf reg2 + 1, W
; status Z is 0 if not equal
	btfsc STATUS, Z
	ENDM

; skip if the register equals the literal
SKIP_EQUAL_LITERAL macro reg1, literal
	movlw literal
	cpfseq reg1
	ENDM

; skip if the 16 bit registers are equal
SKIP_EQUAL16 macro reg1, reg2
	movf reg2, W
	cpfseq reg1
	bra $ + 6
	movf reg2 + 1, W
	cpfseq reg1 + 1
	ENDM

; skip if the 16 bit registers are equal
SKIP_EQUAL32 macro reg1, reg2
	movf reg2, W
	cpfseq reg1
	bra $ + 18
	movf reg2 + 1, W
	cpfseq reg1 + 1
	bra $ + 12
	movf reg2 + 2, W
	cpfseq reg1 + 2
	bra $ + 6
	movf reg2 + 3, W
	cpfseq reg1 + 3
	ENDM

; skip if the register equals the literal
SKIP_EQUAL_LITERAL16 macro reg1, literal
	movlw LOW(literal)
	cpfseq reg1
	bra $ + 6
	movlw HIGH(literal)
	cpfseq reg1 + 1
	ENDM


; skip if the register is not equal to the literal
SKIP_NOTEQUAL_LITERAL macro reg1, literal
	movlw literal
	subwf reg1, W
	btfsc STATUS, Z
	ENDM

SKIP_ZERO macro reg
	movf reg, W
	btfss STATUS, Z
	ENDM

SKIP_ZERO16 macro reg
	movf reg, W
; if it's zero, test the next byte
	btfss STATUS, Z
; if it's nonzero, don't skip now
	bra $ + 6
	movf reg + 1, W
	btfss STATUS, Z
	ENDM

SKIP_NONZERO macro reg
	movf reg, W
	btfsc STATUS, Z
	ENDM

SKIP_NONZERO16 macro reg
	movf reg, W
; if it's zero, test the next byte
	btfss STATUS, Z
; if it's nonzero, skip now
	bra $ + 8
	movf reg + 1, W
	btfsc STATUS, Z
	ENDM

SKIP_NEGATIVE32 macro reg
	btfss reg + 3, D'7'
	ENDM

GOTO_IF_ZERO macro reg, target
	movf reg, W
	bz target
	ENDM

GOTO_IF_NOT_ZERO macro reg, target
	movf reg, W
	bnz target
	ENDM

; skip if the bits are equal
SKIP_BITS_EQUAL macro reg1, bit1, reg2, bit2
	bcf STATUS, C
	btfsc reg1, bit1
	btg STATUS, C
	btfsc reg2, bit2
	btg STATUS, C
	btfsc STATUS, C
	ENDM

; skip if the bits are not equal
SKIP_BITS_NOTEQUAL macro reg1, bit1, reg2, bit2
	bcf STATUS, C
	btfsc reg1, bit1
	btg STATUS, C
	btfsc reg2, bit2
	btg STATUS, C
	btfss STATUS, C
	ENDM

; skip if reg1 > reg2
SKIP_GREATER macro reg1, reg2
	movf reg2, W
	cpfsgt reg1
	ENDM


; skip if the register contents are > literal
SKIP_GREATER_LITERAL macro reg, literal
	movlw literal
	cpfsgt reg
	ENDM

SKIP_GREATEREQUAL_LITERAL macro reg, literal
	movlw literal - 1
	cpfsgt reg
	ENDM

SKIP_GREATEREQUAL macro reg1, reg2
	movf reg2, W
	subwf reg1, W
	btfss STATUS, C
	ENDM


; skip if reglo < literal
SKIP_LESS_LITERAL16 macro reglo, literal
	movlw LOW(literal)
	subwf reglo, W
	movlw HIGH(literal)
	subwfb reglo + 1, W
; if literal > reglo then skip
	btfsc STATUS, C
	ENDM

; skip if register contents > literal
SKIP_GREATER_LITERAL16 macro reglo, literal
; subtract register low from literal low and discard result
	movf reglo, W
	sublw LOW(literal)
; subtract register high from literal high and discard result
	movlw HIGH(literal)
	subfwb reglo + 1, W
; negative if register > literal
; testing for N didn't handle all cases
	btfsc STATUS, C
	ENDM

; skip if postinc pointer contents > literal
SKIP_POINTER_GREATER_LITERAL16 macro ptr, literal
	movf ptr, W
	sublw LOW(literal)
	movlw HIGH(literal)
	subfwb ptr, W
	btfsc STATUS, C
	ENDM

; skip if register contents >= literal
SKIP_GREATEREQUAL_LITERAL16 macro reglo, literal
	movlw LOW(literal)
	subwf reglo, W
	movlw HIGH(literal)
	subwfb reglo + 1, W
; carry = 1 if reglo >= literal
	btfss STATUS, C
	ENDM


; skip if register1 contents > register2 contents
SKIP_GREATER16 macro dstlo, srclo
; subtract register1 low from register2 low
	movf dstlo, W
	subwf srclo, W
; subtract register1 high from register2 high
	movf dstlo + 1, W
	subwfb srclo + 1, W
; negative if register1 > register2
; testing for N didn't handle all cases
	btfsc STATUS, C
	ENDM


; skip if register reg1 >= reg2
SKIP_GREATEREQUAL16 macro reg1, reg2
	movf reg2, W
	subwf reg1, W
	movf reg2 + 1, W
	subwfb reg1 + 1, W
; carry = 1 if reglo >= literal
	btfss STATUS, C
	ENDM

; skip if register1 contents > register2 contents
SKIP_GREATER32 macro dstlo, srclo
; subtract register1 low from register2 low
	movf dstlo, W
	subwf srclo, W

; subtract register1 high from register2 high
	movf dstlo + 1, W
	subwfb srclo + 1, W

	movf dstlo + 2, W
	subwfb srclo + 2, W

	movf dstlo + 3, W
	subwfb srclo + 3, W

; negative if register1 > register2
; testing for N didn't handle all cases
	btfsc STATUS, C
	ENDM

; skip if 32 bit reg1 >= 32 bit reg2
SKIP_GREATEREQUAL32 macro dstlo, srclo
	movf srclo, W
	subwf dstlo, W

	movf srclo + 1, W
	subwfb dstlo + 1, W

	movf srclo + 2, W
	subwfb dstlo + 2, W

	movf srclo + 3, W
	subwfb dstlo + 3, W

; carry = 1 if reg1 >= reg2
	btfss STATUS, C
	ENDM

; skip if 32 bit register >= 32 bit literal
SKIP_GREATEREQUAL_LITERAL32 macro reglo, literalhi16, literallo16
	movlw LOW(literallo16)
	subwf reglo, W
	movlw HIGH(literallo16)
	subwfb reglo + 1, W
	movlw LOW(literalhi16)
	subwfb reglo + 2, W
	movlw HIGH(literalhi16)
	subwfb reglo + 3, W
; carry = 1 if register >= literal
	btfss STATUS, C
	ENDM



; skip if reg1 < literal
; reg2 is 2 16bit constants
SKIP_LESS_LITERAL32 macro reg1, reg2h, reg2l
	movlw LOW(reg2l)
	subwf reg1 + 0, W
	movlw HIGH(reg2l)
	subwfb reg1 + 1, W
	movlw LOW(reg2h)
	subwfb reg1 + 2, W
	movlw HIGH(reg2h)
	subwfb reg1 + 3, W
	btfsc STATUS, C
	ENDM

; skip if register < literal
SKIP_LESS_LITERAL macro reg, literal
	movlw literal
	cpfslt reg
	ENDM

; skip if register <= literal
SKIP_LESSEQUAL_LITERAL macro reg, literal
	movlw literal + 1
	cpfslt reg
	ENDM

; skip if reg1 < reg2
; temp must point to a 32 bit register
SKIP_LESS32 macro temp, reg1, reg2
	COPY_REGISTER32 temp, reg1
	SUBTRACT32 temp, reg2
	btfss temp + 3, D'7' ; bit 31 is 1 if literal was > reg
	ENDM


; skip if register1 < register2
SKIP_LESS16 macro reg1, reg2
	movf reg2 + 0, W
	subwf reg1 + 0, W
	movf reg2 + 1, W
	subwfb reg1 + 1, W
	btfsc STATUS, C
	ENDM


; skip if register1 < register2
SKIP_LESS macro reg1, reg2
	movf reg2, W
	cpfslt reg1
	ENDM

; skip if register1 <= register2
SKIP_LESSEQUAL macro reg1, reg2
	movf reg1, W
	subwf reg2, W
	btfss STATUS, C
	ENDM

; skip if register1 <= register2
SKIP_LESSEQUAL16 macro reg1, reg2
	movf reg1, W
	subwf reg2, W
	movf reg1 + 1, W
	subwfb reg2 + 1, W
	btfss STATUS, C
	ENDM



COMMON_VARS macro
; need timer wraparounds for LED flashes
; set when timer 1 wrapped
; bit 0 - main loop sets
; bit 1 - interrupt handler sets
	VARADD TIMER1_WRAP, 1
	VARADD TEMP0, 2
	VARADD TEMP1, 2
	VARADD TEMP2, 2
	VARADD TEMP3, 2
; temp3 must be 5 bytes for analog to work
	VARADD TEMP4, 2
	VARADD TEMP5, 2

; temporaries to use in the interrupt handlers
	VARADD INT_TEMP0, 2
	VARADD INT_TEMP1, 2
; use pointer2 in interrupt handlers
	ENDM




#endif ; PIC_UTIL_INC




