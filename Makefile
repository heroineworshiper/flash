# float, no USB
GCC_ARM := /opt/gcc-arm-none-eabi-4_6-2012q2/bin/arm-none-eabi-gcc
OBJCOPY := /opt/gcc-arm-none-eabi-4_6-2012q2/bin/arm-none-eabi-objcopy
ARM_CFLAGS := \
	-O2 \
	-c \
	-mcpu=cortex-m4 \
	-mthumb \
	-march=armv7e-m \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard \
	-mlittle-endian \
	-ffreestanding \
	-I. \
	-I../stm32stuff


#ARM_CFLAGS += -DENABLE_PRINT


ARM_LIBM := $(shell $(GCC_ARM) $(ARM_CFLAGS) -print-file-name=libm.a)
ARM_LIBC := $(shell $(GCC_ARM) $(ARM_CFLAGS) -print-libgcc-file-name)
ARM_LFLAGS := -mcpu=cortex-m4 \
	-mthumb \
	-march=armv7e-m \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard \
	-mlittle-endian \
	-ffreestanding \
	-nostdlib \
	-nostdinc \
	$(ARM_LIBM) $(ARM_LIBC)


AVR_DIR := /root/arduino-1.8.15/hardware/tools/avr/bin/
AVR_GCC := $(AVR_DIR)avr-gcc
AVR_OBJCOPY := $(AVR_DIR)avr-objcopy -j .text -j .data -O ihex
AVR_DUDE := avrdude -v -patmega328p -cstk500v1 -P/dev/ttyACM0 -b19200
AVR_CFLAGS := -O2 -mmcu=atmega328p
AVR_LFLAGS := -O2 -mmcu=atmega328p -Wl,--section-start=.text=0x0000 -nostdlib


manual_to_ttl.hex: manual_to_ttl.c si4421.h
	$(AVR_GCC) $(AVR_CFLAGS) -o manual_to_ttl.o manual_to_ttl.c
	$(AVR_GCC) $(AVR_LFLAGS) -o manual_to_ttl.elf manual_to_ttl.o
	$(AVR_OBJCOPY) manual_to_ttl.elf manual_to_ttl.hex

# program atmega328 fuse.  page 283
# brownout is failing high for some reason
flash_fuse:
	$(AVR_DUDE) -e -Ulock:w:0x3F:m -Uefuse:w:0x06:m -Uhfuse:w:0xD3:m -Ulfuse:w:0xE2:m 
      

flash_isp: manual_to_ttl.hex
	$(AVR_DUDE) -Uflash:w:manual_to_ttl.hex:i -Ulock:w:0x0F:m





WIRELESS_OBJS := \
        si4421.o \
        camera.o \
        flash.o \
        wireless.o
        

ARM_OBJS := \
	../stm32stuff/startup_main.o \
	../stm32stuff/arm_math2.o \
	uart.o \
	../stm32stuff/linux.o \
	../stm32stuff/misc.o \
	../stm32stuff/stm32f4xx_rcc.o \
	../stm32stuff/stm32f4xx_usart.o \
	../stm32stuff/stm32f4xx_gpio.o \
	../stm32stuff/stm32f4xx_dac.o \
	../stm32stuff/stm32f4xx_dcmi.o \
	../stm32stuff/stm32f4xx_dma.o \
	../stm32stuff/stm32f4xx_exti.o \
	../stm32stuff/stm32f4xx_i2c.o \
	../stm32stuff/stm32f4xx_iwdg.o \
	../stm32stuff/stm32f4xx_tim.o \
	../stm32stuff/stm32f4xx_adc.o \
	../stm32stuff/stm32f4xx_flash.o \
	../stm32stuff/stm32f4xx_syscfg.o \
        ../stm32stuff/system_stm32f4xx.o




$(shell echo $(GCC_ARM) $(ARM_CFLAGS) > arm_gcc )



flash_pot.hex:
	gpasm -o flash_pot.hex flash_pot.s

wireless.bin: $(WIRELESS_OBJS) $(ARM_OBJS)
	$(GCC_ARM) -o wireless.elf \
		$(WIRELESS_OBJS) \
		$(ARM_OBJS) \
		$(ARM_LFLAGS) \
		-T../stm32stuff/main.ld
	$(OBJCOPY) -O binary wireless.elf wireless.bin



usb_programmer: usb_programmer.c
	$(GCC) -o usb_programmer usb_programmer.c $(USB_LIB) -I$(USB_DIR) -lpthread -lrt

bootload: bootload.c
	gcc -o bootload bootload.c -lpthread

clean:
	rm -f *.o *.bin *.hex flash_pot.lst flash_pot.cod usb_programmer

$(WIRELESS_OBJS) $(ARM_OBJS):
	`cat arm_gcc` -c $< -o $*.o

../stm32stuff/startup_main.o:     ../stm32stuff/startup_main.s
../stm32stuff/arm_math2.o:        ../stm32stuff/arm_math2.c
uart.o:             ../stm32stuff/uart.c
../stm32stuff/linux.o:            ../stm32stuff/linux.c
../stm32stuff/misc.o:             ../stm32stuff/misc.c
../stm32stuff/stm32f4xx_rcc.o:    ../stm32stuff/stm32f4xx_rcc.c
../stm32stuff/stm32f4xx_usart.o:  ../stm32stuff/stm32f4xx_usart.c
../stm32stuff/stm32f4xx_gpio.o:   ../stm32stuff/stm32f4xx_gpio.c
../stm32stuff/stm32f4xx_dac.o:   ../stm32stuff/stm32f4xx_dac.c
../stm32stuff/stm32f4xx_dcmi.o:   ../stm32stuff/stm32f4xx_dcmi.c
../stm32stuff/stm32f4xx_dma.o:    ../stm32stuff/stm32f4xx_dma.c
../stm32stuff/stm32f4xx_exti.o:    ../stm32stuff/stm32f4xx_exti.c
../stm32stuff/stm32f4xx_i2c.o:    ../stm32stuff/stm32f4xx_i2c.c
../stm32stuff/stm32f4xx_iwdg.o:   ../stm32stuff/stm32f4xx_iwdg.c
../stm32stuff/stm32f4xx_tim.o:    ../stm32stuff/stm32f4xx_tim.c
../stm32stuff/stm32f4xx_adc.o:    ../stm32stuff/stm32f4xx_adc.c
../stm32stuff/stm32f4xx_flash.o:  ../stm32stuff/stm32f4xx_flash.c
../stm32stuff/stm32f4xx_syscfg.o: ../stm32stuff/stm32f4xx_syscfg.c
../stm32stuff/system_stm32f4xx.o: ../stm32stuff/system_stm32f4xx.c
wireless.o: wireless.c wireless.h
si4421.o: si4421.c wireless.h
camera.o: camera.c wireless.h
flash.o: flash.c wireless.h





























