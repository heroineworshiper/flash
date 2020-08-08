flash_pot.hex:
	gpasm -o flash_pot.hex flash_pot.s

usb_programmer: usb_programmer.c
	$(GCC) -o usb_programmer usb_programmer.c $(USB_LIB) -I$(USB_DIR) -lpthread -lrt

clean:
	rm -f flash_pot.lst flash_pot.hex flash_pot.cod usb_programmer




