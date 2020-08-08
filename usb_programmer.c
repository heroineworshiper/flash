
//
// This programmer supports a bootloader.
// Usage:
//
// First the bootloader must be loaded.
// Run programmer without the bootloader.
//     programmer bootloader.hex
//
// The CPU can be verified without the bootloader
//     programmer -r bootloader.hex
//
// The CPU can be incrementally programmed without the bootloader.
//     programmer -i bootloader.hex
//
// PGD, PGC, and MCLR must be patched to the 
// programming hardness.  GND must also be connected if the power supplies
// are different.
// It is programmed in the circuit by invoking the bootloader.
//     programmer -b program.hex
//
// The bootloader can also write programs incrementally.
//     programmer -b -i program.hex
//
// The bootloader can verify the program in memory.
//     programmer -b -r program.hex

#include "bootloader.h"
#include "parapin.h"
#ifdef IS_GLIDER
#include "pic_glider.h"
#endif
#include <errno.h>
#include <fcntl.h>
#include <sched.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include "libusb.h"



// New pins
// LP_PIN01     voltage on MCLR
// LP_PIN02     PGD output
// LP_PIN04     enable MCLR output
// LP_PIN15     PGM input
// LP_PIN17     PGC output

// Programming board header from left to right
// PGD
// MCLR
// GND
// PGM input
// PGC

// Registers
#define PORTA 0x80
#define TRISA 0x92
#define EECON1 0xa6
#define EECON2 0xa7
#define ADCON1 0xc1
#define TBLPTRL 0xf6
#define TBLPTRH 0xf7
#define TBLPTRU 0xf8

// Special instructions
#define NOP 0x0
// Table write, no increment
#define TBLWT 0xc
// Table write, post increment by 2
#define TBLWT_INC 0xd
// Table write, start programming
#define TBLWT_PROGRAM 0xf
#define TBLRD 0x9





// Debugger state
// Wait for 0xf
#define DEBUG_WAIT 0
#define DEBUG_READ_SIZE1 1
// Read value size
#define DEBUG_READ_SIZE2 2
// Read value
#define DEBUG_READ_VALUE 3



#define PIC18F1220 0
#define PIC18F458 1
#define PIC18F14K50 2
#define PIC18F26K20 3
#define PIC18F2450 4











// Parport mode
// delay in microseconds for programming bootloader
#define DELAY1 300
// read delay is much shorter
#define READ_DELAY1 300
// delay for using bootloader
#define DELAY2 0
// delay for USB reads
// might need changes depending on the layout
//#define USB_READ_DELAY 0xffff
#define USB_READ_DELAY 0xff00
// delay for USB writes
//#define USB_WRITE_DELAY 0xffff
#define USB_WRITE_DELAY 0xff00
// delay for programming
#define USB_PROGRAMMING_DELAY 0xc000
// delay for programming configuration
#define USB_CONFIG_DELAY 0x0
// read delay is much faster
#define READ_DELAY2 100
// read delay for dumping PGD must be large to ensure a slow program loop
#define READ_DELAY3 100000
#define RESET_DELAY 100000

#define ERASE_DELAY 100
#define COMMAND_DELAY 0
//#define BAUD B57600
#define BAUD B9600

#define MAX(x, y) ((x) > (y) ? (x) : (y))
#define MIN(x, y) ((x) < (y) ? (x) : (y))

#define SET_REGISTER(reg, literal) \
/* movlw literal */ \
send_command(NOP, (0x0e << 8) | literal, 0, 0, 0); \
/* movwf reg */ \
send_command(NOP, (0x6e << 8) | reg, 0, 0, 0);

#define SET_POINTER(address) \
	send_command(0x0, 0x0e00 | ((address & 0xff0000) >> 16), 0, 0, 0); \
	send_command(0x0, 0x6ef8, 0, 0, 0); \
	send_command(0x0, 0x0e00 | ((address & 0xff00) >> 8), 0, 0, 0); \
	send_command(0x0, 0x6ef7, 0, 0, 0); \
	send_command(0x0, 0x0e00 | (address & 0xff), 0, 0, 0); \
	send_command(0x0, 0x6ef6, 0, 0, 0);




// Print bit activity
int debug = 0;
int use_bootloader = 0;
int use_serial = 0;
int use_usb = 1;
int do_debugger = 0;
// Parse output from Vicaglider
int do_parser = 0;
//int do_program = 1;
int max_byte;
int min_address;
//int start_address = PROGRAM_START;
int start_address = 0;
unsigned char *data_diff64;
unsigned char *data_bytes;
uint16_t *data;
int delay;
int read_delay;
FILE *debug_file = 0;
FILE *debug_txt = 0;
// serial port for bootloader
char *serial_path = 0;
int serial_fd = -1;
int pic_model = PIC18F458;
int do_configuration = 0;








// USB mode

#define VENDOR_ID 0x04d8
#define PRODUCT_ID 0x000c

// Timer value in write_usb_image
int usb_delay = 0;
// Programming delay
int usb_delay2 = 0;
#define PACKET_SIZE 64
#define IMAGE_SIZE 0x800000
struct libusb_device_handle *devh;
struct libusb_transfer *out_urb;
struct libusb_transfer *in_urb;
unsigned char in_urb_data[PACKET_SIZE];
unsigned char out_urb_data[PACKET_SIZE];
pthread_mutex_t usb_send_lock;
pthread_mutex_t usb_recv_lock;
// Bit image for USB bit banger
unsigned char tris_image[IMAGE_SIZE];
unsigned char write_image[IMAGE_SIZE];
uint16_t timer_image[IMAGE_SIZE];
unsigned char read_image[IMAGE_SIZE];
unsigned char read_samples[IMAGE_SIZE];
unsigned char *tris_ptr = tris_image;
unsigned char *write_ptr = write_image;
uint16_t *timer_ptr = timer_image;
unsigned char *read_ptr = read_image;
// Current status of tris & mask
unsigned char current_tris = 0xff;
unsigned char current_mask = 0;
uint16_t current_delay = 0;
// Pins for USB PIC programming
// 12V enable when high (digital 2)
#define USB_MCLR2 (1 << 0)
// 0V enable when high (digital 3)
#define USB_MCLR (1 << 1)
// PGC (digital 4)
#define USB_PGC (1 << 2)
// PGD (digital 5)
#define USB_PGD (1 << 3)















int init_serial()
{
	struct termios term;

// Initialize serial port
	serial_fd = open(serial_path, O_RDWR | O_NOCTTY | O_SYNC);
	if(serial_fd < 0)
	{
		printf("%s: %s\n", serial_path, strerror(errno));
		return 1;
	}
	
	if (tcgetattr(serial_fd, &term))
	{
		printf("tcgetattr: %s\n", strerror(errno));
		close(serial_fd);
		return 1;
	}

	tcflush(serial_fd, TCIOFLUSH);
	cfsetispeed(&term, BAUD);
	cfsetospeed(&term, BAUD);
	term.c_iflag = IGNBRK;
	term.c_oflag = 0;
	term.c_lflag = 0;
	term.c_cflag |= HUPCL;
	if(tcsetattr(serial_fd, TCSADRAIN, &term))
	{
		printf("tcsetattr: %s\n", strerror(errno));
		close(serial_fd);
		return 1;
	}
	return 0;
}

int write_serial(unsigned char c)
{
	int result;
	do
	{
		result = write(serial_fd, &c, 1);
	} while(!result);
	return result;
}

unsigned char read_serial()
{
	unsigned char c;
	int result;
	do
	{
		result = read(serial_fd, &c, 1);
	} while(!result);

	return c;
}



static void urb_callback(struct libusb_transfer *urb)
{
	if(urb->status != LIBUSB_TRANSFER_COMPLETED)
	{
		libusb_submit_transfer(urb);
		return;
	}


/*
 * 	if(urb == out_urb)
 * 	{
 * // Release buffer
 * 		pthread_mutex_unlock(&usb_send_lock);
 * 	}
 * 	else
 * 	if(urb == in_urb)
 * 	{
 * // release buffer
 * 		pthread_mutex_unlock(&usb_recv_lock);
 * 	}
 * 
 */


}

// Write an URB & get the return packet
void write_usb(int want_input)
{
	const int debug = 0;
	bzero(in_urb_data, PACKET_SIZE);
	libusb_submit_transfer(out_urb);

	if(debug)
	{
		int i;
		printf("write_usb %d data:  ", __LINE__);
		for(i = 1; i < PACKET_SIZE; i += 4)
		{
			printf("%d", out_urb_data[i] & USB_PGD ? 1 : 0);
			//printf("%02x ", out_urb_data[i]);
		}
		printf("\n");
		printf("write_usb %d clock: ", __LINE__);
		for(i = 1; i < PACKET_SIZE; i += 4)
		{
			printf("%d", out_urb_data[i] & USB_PGC ? 1 : 0);
			//printf("%02x ", out_urb_data[i]);
		}
		printf("\n");
		printf("write_usb %d tris:  ", __LINE__);
		for(i = 0; i < PACKET_SIZE; i += 4)
		{
			printf("%d", out_urb_data[i] & USB_PGD ? 1 : 0);
			//printf("%02x ", out_urb_data[i]);
		}
		printf("\n");
		printf("write_usb %d time:  ", __LINE__);
		for(i = 2; i < PACKET_SIZE; i += 4)
		{
			printf("0x%04x ", out_urb_data[i] | (out_urb_data[i] << 8));
			//printf("%02x ", out_urb_data[i]);
		}
		printf("\n");
	}

	struct timeval tv;
	tv.tv_sec = 10;
	tv.tv_usec = 0;
	if(debug) printf("write_usb %d\n", __LINE__);
	int result = libusb_handle_events_timeout(0, &tv);
	if(debug) printf("write_usb %d\n", __LINE__);


	libusb_submit_transfer(in_urb);
	if(debug) printf("write_usb %d\n", __LINE__);

	tv.tv_sec = 10;
	tv.tv_usec = 0;
	result = libusb_handle_events_timeout(0, &tv);
	if(debug) printf("write_usb %d\n", __LINE__);

// Return values need 2 URBs.
	if(want_input)
	{
		libusb_submit_transfer(in_urb);
		tv.tv_sec = 10;
		tv.tv_usec = 0;
		result = libusb_handle_events_timeout(0, &tv);
	}

	if(debug)
	{
		int i;
		printf("write_usb %d input: ", __LINE__);
		for(i = 0; i < PACKET_SIZE / 4; i++)
		{
			//printf("%02x ", in_urb_data[i]);
			printf("%d", in_urb_data[i] & USB_PGD ? 1 : 0);
		}
		printf("\n");
	}



/*
 * 	libusb_submit_transfer(out_urb);
 * 	libusb_submit_transfer(in_urb);
 * 	pthread_mutex_lock(&usb_send_lock);
 * 	pthread_mutex_lock(&usb_recv_lock);
 */
}

// Write 1 value for pins & send to device
// Return the read values
void write_usb_single()
{
	int i;

// Delay
	for(i = 0; i < PACKET_SIZE; )
	{
		out_urb_data[i++] = current_tris;
		out_urb_data[i++] = current_mask;
		out_urb_data[i++] = current_delay & 0xff;
		out_urb_data[i++] = current_delay >> 8;
	}

	write_usb(0);
}

// Write 1 value to the pin image
void write_usb_mask()
{
	*tris_ptr++ = current_tris;
	*write_ptr++ = current_mask;
	*timer_ptr++ = current_delay;
}

// Send the USB images & reset the pointers
void write_usb_image()
{
	int i, j;
	int image_size = write_ptr - write_image;
	const int debug = 0;
	int prev_offset = 0;

	read_ptr = read_image;
	write_ptr = write_image;
	tris_ptr = tris_image;
	for(i = 0; i < image_size; )
	{
// Delay
		if(debug) 
		{
			printf("write_usb_image %d pointer=%d\n", __LINE__, i);
		}
		else
		{
			if(i - prev_offset > 1000)
			{
				printf("write_usb_image %d: %d/%d bits\r",
					__LINE__,
					i,
					image_size);
				fflush(stdout);
				prev_offset = i;
			}
		}
// Masks
		for(j = 0; j < PACKET_SIZE; )
		{
			if(i <= image_size - 1)
			{
				out_urb_data[j++] = tris_image[i];
				out_urb_data[j++] = write_image[i];
				out_urb_data[j++] = timer_image[i] & 0xff;
				out_urb_data[j++] = timer_image[i] >> 8;
			}
			else
// Freeze last bits
			{
				out_urb_data[j++] = tris_image[image_size - 1];
				out_urb_data[j++] = write_image[image_size - 1];
				out_urb_data[j++] = timer_image[image_size - 1] & 0xff;
				out_urb_data[j++] = timer_image[image_size - 1] >> 8;
			}
			i++;
		}

// Write URB
		write_usb(1);

// Get return values
		memcpy(read_ptr, in_urb_data, PACKET_SIZE / 4);
		read_ptr += PACKET_SIZE / 4;
	}
	printf("\n");

	read_ptr = read_image;
}

/*
 * void* urb_thread()
 * {
 * 	while(1)
 * 	{
 * 		libusb_handle_events(0);
 * 	}
 * }
 */


int init_usb()
{
	int result = 0;

	result = libusb_init(0);
	if(result < 0)
	{
		printf("init_usb: Couldn't initialize libusb\n");
		return 1;
	}

	devh = libusb_open_device_with_vid_pid(0, VENDOR_ID, PRODUCT_ID);
	if(!devh)
	{
		printf("init_usb: Couldn't find ICSP bit banger\n");
		return 1;
	}


	result = libusb_claim_interface(devh, 0);
	if(result < 0)
	{
		printf("init_usb: bit banger in use\n");
		return 1;
	}


	bzero(tris_image, IMAGE_SIZE);
	bzero(write_image, IMAGE_SIZE);
	bzero(read_image, IMAGE_SIZE);


	in_urb = libusb_alloc_transfer(0);
	libusb_fill_bulk_transfer(in_urb,
		devh, 
		1 | LIBUSB_ENDPOINT_IN,
		in_urb_data, 
		PACKET_SIZE, 
		urb_callback,
		in_urb, 
		1000);

	out_urb = libusb_alloc_transfer(0);
	libusb_fill_bulk_transfer(out_urb,
		devh, 
		1 | LIBUSB_ENDPOINT_OUT,
		out_urb_data, 
		PACKET_SIZE, 
		urb_callback,
		out_urb, 
		1000);


/*
 * 	pthread_mutexattr_t mutex_attr;
 * 	pthread_mutexattr_init(&mutex_attr);
 * 	pthread_mutex_init(&usb_send_lock, &mutex_attr);
 * 	pthread_mutex_lock(&usb_send_lock);
 * 	pthread_mutex_init(&usb_recv_lock, &mutex_attr);
 * 	pthread_mutex_lock(&usb_recv_lock);
 * 
 * 	pthread_t tid;
 * 	pthread_attr_t  attr;
 * 	pthread_attr_init(&attr);
 * 	pthread_create(&tid, &attr, urb_thread, 0);
 */
 	return 0;
}

void init_pins()
{
	if(!use_usb)
	{
//		pin_init_user(LPT1);
//		pin_output_mode(LP_DATA_PINS | LP_SWITCHABLE_PINS);
	}
}

void test_pins()
{
	printf("test_pins: Testing pins\n");
#if 0
	current_mask = 0xff;
	current_tris = 0x0;
	write_usb_single();

	printf("Hit enter.\n");
	fgetc(stdin);
#endif

#if 0
	printf("Capturing PGC\n");
#define BUFFER_SIZE 0x100000
	unsigned char *buffer = malloc(BUFFER_SIZE);
	unsigned char *ptr = buffer;
	unsigned char *end = buffer + BUFFER_SIZE;
	while(ptr < end)
	{
		if(pin_is_set(LP_PIN15)) 
			*ptr++ = 0xff;
		else
			*ptr++ = 0x0;
/*
 * 		if(pin_is_set(LP_PIN15)) 
 * 			printf("1");
 * 		else
 * 			printf("0");
 * 		fflush(stdout);
 */
	}
	printf("Writing PGC to test.hex\n");
	FILE *out = fopen("test.hex", "w");
	fwrite(buffer, 1, BUFFER_SIZE, out);
	fclose(out);
	exit(0);
#endif


#if 0

	printf("test output pin\n");
	while(1)
	{
		set_pin(LP_PIN07);
		sleep(1);
		clear_pin(LP_PIN07);
		sleep(1);
	}
#endif

#if 0
	printf("test input pin\n");
	while(1)
	{
		printf("%d ", pin_is_set(LP_PIN12) ? 1 : 0);
		fflush(stdout);
		sleep(1);
	}
#endif

#if 0
// MCLR test
	while(1)
	{
		printf("MCLR 0V\n");
		clear_pin(LP_PIN01);
		set_pin(LP_PIN04);
		sleep(5);
		printf("MCLR max\n");
		set_pin(LP_PIN04);
		set_pin(LP_PIN01);
 		sleep(5);
		printf("MCLR doubleing\n");
		clear_pin(LP_PIN01);
		clear_pin(LP_PIN04);
		sleep(5);
	}
#endif

#if 0
// PGD
	while(1)
	{
		printf("PGD on\n");
		set_pin(LP_PIN02);
		sleep(5);
		printf("PGD off\n");
		clear_pin(LP_PIN02);
		sleep(5);
	}
#endif


#if 0
// PGC
	while(1)
	{
		printf("PGC on\n");
		set_pin(LP_PIN17);
		sleep(5);
		printf("PGC off\n");
		clear_pin(LP_PIN17);
		sleep(5);
	}
#endif

#if 0
// PGM
	printf("PGM value:\n");
	while(1)
	{
		printf("%d ", pin_is_set(LP_PIN15) ? 1 : 0);
		fflush(stdout);
		usleep(100000);
	}
#endif

}

void reset_pins()
{
	const int debug = 1;
// Reset the chip
	printf("reset_pins: Resetting chip.\n");
//	usleep(100000);
	if(use_usb)
	{
		int i;

// Flush USB endpoints if interrupt
/*
 * 		for(i = 0; i < 2; i++)
 * 		{
 * 			if(debug) printf("reset_pins %d\n", __LINE__);
 * 			struct timeval tv;
 * 			tv.tv_sec = 1;
 * 			tv.tv_usec = 0;
 * 			int result = libusb_handle_events_timeout(0, &tv);
 * 			printf("reset_pins: result=%d\n", result);
 * 		}
 * 		if(debug) printf("reset_pins %d\n", __LINE__);
 */

//		test_pins();

// Lower MCLR
		current_mask |= USB_MCLR;
		current_mask &= ~USB_MCLR2;
		current_tris = 0;
		current_delay = 0;
		write_usb_single();
		usleep(RESET_DELAY);
		if(debug) printf("reset_pins %d\n", __LINE__);

// Disable pins
		current_tris = 0xff;
		write_usb_single();
		usleep(RESET_DELAY);
		if(debug) printf("reset_pins %d\n", __LINE__);
	}

#ifdef HAVE_PARPORT
	else
	{
// Lower MCLR
		clear_pin(LP_PIN01 | 
			LP_PIN02 | 
			LP_PIN04 | 
			LP_PIN17);

		set_pin(LP_PIN04);
		usleep(RESET_DELAY);
// Float MCLR
		clear_pin(LP_PIN04);
	}
#endif




	if(debug_file) fclose(debug_file);
	if(debug_txt) fclose(debug_txt);
	debug_file = 0;
	debug_txt = 0;
}

void signal_entry()
{
	printf("\nGot interrupted.\n");
	reset_pins();
	exit(0);
}

// Generate clock signal to read bits
void read_bits_usb(int total)
{
	int i;
	if(use_bootloader)
	{
// Set data to input mode & raise clock
		current_mask |= USB_PGC;
		current_mask &= ~USB_PGD;
		current_tris |= USB_PGD;
		current_delay = usb_delay;
		write_usb_mask();

		for(i = 0; i < total; i += 2)
		{
// Delay to read bit
			write_usb_mask();
// Lower clock
			current_mask &= ~USB_PGC;
			write_usb_mask();
// Delay to read bit
			write_usb_mask();
// Raise clock
			current_mask |= USB_PGC;
			write_usb_mask();
		}
	}
	else
	{
// Set data to input mode
		current_tris |= USB_PGD;
		current_delay = usb_delay;

		for(i = 0; i < total; i++)
		{
// Raise clock
			current_mask |= USB_PGC;
			write_usb_mask();

// Lower clock & get bit
			current_mask &= ~USB_PGC;
			write_usb_mask();
		}
	}
}

// reading pins with the parallel port is faster than writing
void read_bits(uint16_t *bits, int total)
{
	int i;
	unsigned int bit;
	const int debug = 0;
	unsigned int newbit = 1 << (total - 1);

	*bits = 0;

	if(use_bootloader)
	{
		if(debug) 
		{
			printf("Read bits: ");
			fflush(stdout);
		}

// Set data to input mode & raise clock
		if(use_usb)
		{
			read_ptr++;
		}

		for(i = 0; i < total; i += 2)
		{
			*bits >>= 1;
			
			if(use_usb)
			{
// Delay
				read_ptr++;
				bit = (*read_ptr & USB_PGD) ? 1 : 0;
				if(bit) *bits |= newbit;
				*bits >>= 1;
// Lower clock
				read_ptr++;
// Delay
				read_ptr++;
				bit = (*read_ptr & USB_PGD) ? 1 : 0;
				if(bit) *bits |= newbit;
// Raise clock
				read_ptr++;
			}

#ifdef HAVE_PARPORT
			else
			{
				usleep(read_delay / 2);
				bit = pin_is_set(LP_PIN15) ? 1 : 0;

				if(debug) 
				{
					printf("%d", bit); 
					fflush(stdout); 
				}
				if(bit) *bits |= newbit;

// Lower clock
				clear_pin(LP_PIN17);
				usleep(read_delay / 2);
				*bits >>= 1;
				bit = pin_is_set(LP_PIN15) ? 1 : 0;
				if(debug) 
				{
					printf("%d", bit); 
					fflush(stdout); 
				}
				if(bit) *bits |= newbit;

// Raise clock
				set_pin(LP_PIN17);
			}
#endif

		}

		if(debug)
			printf("\n");
	}
	else
	{
		if(use_usb)
		{
// Scan tris buffer for next input mode
			while(!(*tris_ptr & USB_PGD)) 
			{
				tris_ptr++;
				read_ptr++;
			}

if(debug) printf("read_bits %d pointer=%d\n", __LINE__, (int)(tris_ptr - tris_image));
// Lower clock & set data to input mode
//			read_ptr++;
//			tris_ptr++;
		}
#ifdef HAVE_PARPORT
		else
		{
// Lower clock
			clear_pin(LP_PIN17);
// Lower PGD
			clear_pin(LP_PIN02);
			usleep(delay * 4);
		}
#endif


		for(i = 0; i < total; i++)
		{
			*bits >>= 1;

			if(i == 8)
			{
//				*bits = 0;

// Delay for direction change
				if(!use_usb) usleep(delay * 4);
			}


			if(use_usb)
			{
// Raise clock
				read_ptr++;
				tris_ptr++;
// Lower clock & get bit
				bit = (*read_ptr++ & USB_PGD) ? 1 : 0;
				tris_ptr++;
			}
#ifdef HAVE_PARPORT
			else
			{
// Raise clock
				set_pin(LP_PIN17);
				usleep(delay * 4);

// Lower clock
				clear_pin(LP_PIN17);
// Get bit
				bit = pin_is_set(LP_PIN15) ? 1 : 0;
			}
#endif

			if(bit) *bits |= newbit;

			if(debug)
			{
				printf("%d", bit ? 1 : 0);
				fflush(stdout);
			}
			
			if(!use_usb) usleep(delay * 4);

//sleep(1);
		}
		
		if(debug) printf("\n");
	}
}


// Send bits, least significant first.
void write_bits(uint16_t bits, int total, int programming_time)
{
	int i;
	int bit;
	const int debug = 0;

	if(debug)
	{
		printf("write_bits: ");
		fflush(stdout);
	}

	for(i = 0; i < total; i++)
	{
// Clock up.  Data to output mode
		if(use_usb)
		{
			current_tris &= ~USB_PGD;
			current_mask |= USB_PGC;
			current_delay = usb_delay;
		}
#ifdef HAVE_PARPORT
		else
		{
			set_pin(LP_PIN17);
			usleep(delay / 2);
		}
#endif


// Set data
		bit = bits & 0x1;
		if(bit)
		{
			if(use_usb)
			{
				current_mask |= USB_PGD;
				write_usb_mask();
			}
#ifdef HAVE_PARPORT
			else
			{
				set_pin(LP_PIN02);
			}
#endif

			if(debug) 
			{
				printf("1");
				fflush(stdout);
			}
		}
		else
		{
			if(use_usb)
			{
				current_mask &= ~USB_PGD;
				write_usb_mask();
			}
#ifdef HAVE_PARPORT
			else
			{
				clear_pin(LP_PIN02);
			}
#endif

			if(debug)
			{
				printf("0");
				fflush(stdout);
			}
		}



// Clock down to latch bit
		if(use_usb)
		{
// Hold PGC high for programming time P9
			if(programming_time && i == (total - 1))
			{
				current_delay = usb_delay2;
				write_usb_mask();
				current_delay = usb_delay;
			}
// TODO: should be P10 if programming time
			current_mask &= ~USB_PGC;
			write_usb_mask();
		}
#ifdef HAVE_PARPORT
		else
		{
			if(programming_time && i == (total - 1))
			{
				usleep(5000);
			}
			clear_pin(LP_PIN17);
			usleep(delay / 2);
		}
#endif




// Send next bit
		if(use_bootloader)
		{
			bits >>= 1;
			i++;

// Set data
			bit = bits & 0x1;
			if(bit)
			{
				if(use_usb)
				{
					current_mask |= USB_PGD;
					write_usb_mask();
				}
#ifdef HAVE_PARPORT
				else
				{
					set_pin(LP_PIN02);
				}
#endif

				if(debug) 
				{
					printf("1");
					fflush(stdout);
				}
			}
			else
			{
				if(use_usb)
				{
					current_mask &= ~USB_PGD;
					write_usb_mask();
				}
#ifdef HAVE_PARPORT
				else
				{
					clear_pin(LP_PIN02);
				}
#endif
				if(debug)
				{
					printf("0");
					fflush(stdout);
				}
			}
			
			if(!use_usb)
			{
				usleep(delay / 2);
			}

// Clock up to latch bit
			if(use_usb)
			{
				current_mask |= USB_PGC;
				write_usb_mask();
			}
#ifdef HAVE_PARPORT
			else
			{
				set_pin(LP_PIN17);
				usleep(delay / 2);
			}
#endif

		}

		bits >>= 1;
	}
	if(debug) printf("\n");
}


void send_command(unsigned char command, 
	uint16_t output, 
	uint16_t *input,
	int erase_delay,
	int program_delay)
{
	int i;
	static int last_command = 0;

// Clock in command
	if(debug)
	{
		printf("send_command %02x ", command);
		fflush(stdout);
	}

// Send in command
	write_bits(command, 
		4, 
		(program_delay || last_command == TBLWT) ? 1 : 0);
	if(erase_delay)
	{
		if(use_usb)
		{
			current_delay = usb_delay2;
			write_usb_mask();
			current_delay = usb_delay;
/*
 * 			write_usb_mask();
 * 			write_usb_mask();
 * 			write_usb_mask();
 * 			write_usb_mask();
 * 			write_usb_mask();
 * 			write_usb_mask();
 * 			write_usb_mask();
 * 			write_usb_mask();
 */
		}
		else
		{
			printf("Bulk erase delay.\n");
			usleep(10000);
		}
	}

// Send in data
	if(command == TBLRD)
	{
		if(debug)
			printf(" ");

		if(use_usb)
		{
// Send clock signals to get padding
			read_bits_usb(8);
// Delay for direction change
			write_usb_mask();
			write_usb_mask();
			write_usb_mask();
			write_usb_mask();
// Send clock signals to get output
			read_bits_usb(8);
// Delay before next command
			current_tris &= ~USB_PGD;
			write_usb_mask();
			write_usb_mask();
			write_usb_mask();
			write_usb_mask();
		}
		else
		{
			read_bits(input, 16);
		}

		if(debug)
		{
			printf(" %04x ", *input);
			fflush(stdout);
		}
	}
	else
	{
		if(debug)
		{
			printf(" %04x ", output);
			fflush(stdout);
		}

		write_bits(output, 16, 0);
	}


	if(debug) printf("\n");
	if(!use_usb) usleep(COMMAND_DELAY);
	last_command = command;
}

// In USB mode, read the next input from the image
void read_command(uint16_t *input)
{
	read_bits(input, 8);
	read_ptr += 4;
	tris_ptr += 4;
	read_bits(input, 8);
	read_ptr += 4;
	tris_ptr += 4;
}


void start_programming_mode()
{
/*
 * printf("start_programming_mode %d use_bootloader=%d use_usb=%d\n",
 * __LINE__,
 * use_bootloader,
 * use_usb);
 */
	if(use_bootloader)
	{
		if(use_usb)
		{
// Ground MCLR, raise PGC & PGD
			current_tris &= ~(USB_MCLR | USB_MCLR2 | USB_PGC | USB_PGD);
			current_mask = USB_MCLR | USB_PGC | USB_PGD;
			current_delay = usb_delay;
			write_usb_single();

			usleep(RESET_DELAY);


// Raise MCLR to 5V
			current_mask &= ~USB_MCLR;
			write_usb_single();
		}
#ifdef HAVE_PARPORT
		else
		{
// Ground MCLR
			printf("Lowering MCLR\n");
			set_pin(LP_PIN04);
			clear_pin(LP_PIN01);
// Raise PGC to trigger bootloader
			set_pin(LP_PIN17);
// Raise PGD to trigger bootloader
			set_pin(LP_PIN02);
			usleep(RESET_DELAY);


// Raise MCLR to 5V by floating it
			printf("Raising MCLR\n");
			clear_pin(LP_PIN04);
			clear_pin(LP_PIN01);
		}
#endif

	}
	else
	{

		if(use_usb)
		{
// Ground MCLR
			

			printf("start_programming_mode %d: Lowering MCLR\n", __LINE__);
			current_tris = ~(USB_MCLR | USB_MCLR2 | USB_PGC | USB_PGD);
			current_mask = USB_MCLR;
			current_delay = usb_delay;
			write_usb_single();
			write_usb_single();
			usleep(RESET_DELAY);
// Debug
//printf("start_programming_mode %d: press Enter to raise MCLR\n", __LINE__);
//fgetc(stdin);

// Raise MCLR to 12V
			printf("start_programming_mode %d: Raising MCLR\n", __LINE__);
			current_mask &= ~USB_MCLR;
			current_mask |= USB_MCLR2;
			write_usb_single();
		}
#ifdef HAVE_PARPORT
		else
		{
// Ground MCLR
			printf("Lowering MCLR\n");
			clear_pin(LP_PIN01);
			set_pin(LP_PIN04);


// Lower PGC and PGD
			clear_pin(LP_PIN02 | LP_PIN17);


//		printf("Power cycle board and press Enter.\n");
//		getc(stdin);
			usleep(RESET_DELAY);

// Raise MCLR to programming voltage
			printf("Raising MCLR\n");
			set_pin(LP_PIN01 | LP_PIN04);
		}

#endif
	}

// Don't need to wait for the debouncing circuit.
	usleep(RESET_DELAY);
}



void verify(int max_byte)
{
	int total_errors = 0;
	int j, i;
	int first_bad = -1;
	int column = 0;
	printf("Reading %d bytes.  Start = 0x%x\n", 
		max_byte - start_address,
		start_address);

	if(use_bootloader)
	{
		start_programming_mode();

// Read out data
		for(j = start_address; j < max_byte; j += 64)
		{
			if(use_serial)
			{
				write_serial(0xff);
				write_serial('R');
				write_serial(j & 0xff);
				write_serial((j >> 8) & 0xff);

				for(i = 0; i < 64 && i + j < max_byte; i++)
				{
					uint16_t temp;
					if(!column) printf("0x%04x: ", j + i);
					temp = read_serial();
					if(debug) printf(" ");
					printf("%02x ", temp);
					fflush(stdout);

					column++;
					if(temp != data_bytes[j + i])
					{
						if(first_bad < 0) first_bad = j + i;
						printf("*** Error.  Orig = %02x\n", data_bytes[j + i]);
	// Overwrite reference
						data_bytes[j + i] = temp;
						total_errors++;
						column = 0;
					}
					else
					{
						if(column >= 8) 
						{
							printf("\n");
							column = 0;
						}
					}
				}
			}
			else
			{
// Send command
				write_bits(BOOTLOADER_READ, 8, 0);
// Send address
				write_bits(j, 16, 0);

				if(use_usb) 
				{
					for(i = 0; i < 64 && i + j < max_byte; i++)
					{
// Generate clock signals for reading bits
						read_bits_usb(8);
					}
					write_usb_image();
				}

				for(i = 0; i < 64 && i + j < max_byte; i++)
				{
					uint16_t temp;
					if(!column) printf("0x%04x: ", j + i);
					read_bits(&temp, 8);
					if(debug) printf(" ");
					printf("%02x ", temp);
					fflush(stdout);

					column++;
					if(temp != data_bytes[j + i])
					{
						if(first_bad < 0) first_bad = j + i;
						printf("*** Error.  Orig = %02x\n", data_bytes[j + i]);
// Overwrite reference
						data_bytes[j + i] = temp;
						total_errors++;
						column = 0;
					}
					else
					{
						if(column >= 8) 
						{
							printf("\n");
							column = 0;
						}
					}
				}
			}
		}
	}
	else
	{
		start_programming_mode();

// Set table pointer
		SET_POINTER(start_address);

		for(j = start_address; j < max_byte; j++)
		{
			uint16_t temp;
			if(!column && !use_usb) printf("0x%04x: ", j);
			send_command(TBLRD, 0, &temp, 0, 0);

			if(!use_usb)
			{
				if(!debug)
				{
					printf("%02x ", temp & 0xff);
					fflush(stdout);
					column++;
				}

				if((temp & 0xff) != data_bytes[j])
				{
					if(first_bad < 0) first_bad = j;
					printf("*** Error.  Orig = %02x\n", data_bytes[j]);
// Overwrite reference
					data_bytes[j] = temp;
					total_errors++;
					column = 0;
				}
				else
				if(column >= 8)
				{
					printf("\n");
					column = 0;
				}
			}
		}
		
		if(use_usb) 
		{
			write_usb_image();

// Read the bits from the image
			column = 0;
			for(j = start_address; j < max_byte; j++)
			{
				if(!column) printf("0x%04x: ", j);
				uint16_t temp;
// Discard 8 bits
				read_bits(&temp, 8);
// Skip delay
				read_ptr += 4;
				tris_ptr += 4;
// Get data return
				read_bits(&temp, 8);
// Skip delay
				read_ptr += 4;
				tris_ptr += 4;

				printf("%02x ", temp);
				fflush(stdout);
				column++;
				if(temp != data_bytes[j])
				{
					if(first_bad < 0) first_bad = j;
					printf("*** Error.  Orig = %02x\n", data_bytes[j]);
// Overwrite reference
					data_bytes[j] = temp;
					total_errors++;
					column = 0;
				}
				else
				if(column >= 8)
				{
					printf("\n");
					column = 0;
				}
			}
		}
	}

	if(total_errors)
		printf("\nGot %d errors.  First error at 0x%04x\n", 
			total_errors,
			first_bad);
	else
		printf("\nGot no errors.\n");
}











void write_incremental(char *orig_path)
{
	int i, j, k;
	printf("Reading previous program from %s\n", orig_path);

	FILE *orig = fopen(orig_path, "r");
	int program_start = 0;
	int total_sectors = 0;
	int address;

	/* if(use_bootloader) */ program_start = start_address;


	if(!orig)
	{
		printf("No previous program found.\n");
		for(i = program_start; i < max_byte; i += 64)
		{
			data_diff64[i / 64] = 1;
			total_sectors++;
		}
	}
	else
	{
		fseek(orig, program_start, SEEK_SET);
		for(i = program_start; i < max_byte; i++)
		{
			int end_of_file = feof(orig);
			unsigned char prev_byte = fgetc(orig);


// Bytes not equivalent or current byte is beyond end of previous file
			if(prev_byte != data_bytes[i] || end_of_file)
			{
				data_diff64[i / 64] = 1;
			}
		}
		fclose(orig);


		for(i = program_start; i < max_byte; i += 64)
		{
			if(data_diff64[i / 64])
			{
				total_sectors++;
			}
		}
	}

	printf("write_incremental: Writing %d bytes\n", MIN(total_sectors * 64, max_byte - program_start));

	if(total_sectors)
	{
		start_programming_mode();

		if(!use_bootloader)
		{
			if(pic_model == PIC18F458)
			{
// Direct access to config memory.
				send_command(0x0, 0x8ea6, 0, 0, 0);
				send_command(0x0, 0x8ca6, 0, 0, 0);
// Write 00h to 3C0006h to enable single-panel writes.
				SET_POINTER(0x3c0006)
				send_command(TBLWT, 0x0000, 0, 0, 0);

			}

// Direct access to code memory.
 			send_command(0x0, 0x8ea6, 0, 0, 0);
			send_command(0x0, 0x9ca6, 0, 0, 0);
		}
	}

	for(i = program_start; i < max_byte; i += 64)
	{
		if(data_diff64[i / 64])
		{
			address = i;




			if(use_bootloader)
			{
				if(use_serial)
				{
					write_serial(0xff);
					write_serial('W');
					write_serial(address & 0xff);
					write_serial((address >> 8) & 0xff);

		// Must write 64 bytes per write command to get a confirmation byte
					for(j = i; j < i + 64; j += 8)
					{
		// Must write 8 bytes to cause a write operation
						printf("0x%04x: ", j);
						for(k = j; k < j + 8; k++)
						{
							printf("%02x ", data_bytes[k]);
							fflush(stdout);
							write_serial(data_bytes[k]);
						}
						printf("\n");
					}

// Wait for completion
					while(read_serial() != 'D')
						;
				}
				else
				{
					write_bits(BOOTLOADER_WRITE, 8, 0);
					write_bits(address, 16, 0);

// Write 64 bytes per write command
					for(j = i; j < i + 64 && j < max_byte; j += 8)
					{
// Must write 8 bytes to cause a write operation
						printf("0x%04x: ", j);
						for(k = j; k < j + 8; k++)
						{
							printf("%02x ", data_bytes[k]);
							fflush(stdout);
							write_bits(data_bytes[k], 8, 0);
						}
						printf("\n");
					}

					if(use_usb) write_usb_image();
				}
			}
			else
			{




// Enable writing program memory from documentation

// Set write pointer
				if(!use_usb) printf("write_incremental: Erasing 0x%04x\n", address);

				if(pic_model == PIC18F2450)
				{
// Erase 64 bytes
					send_command(0x0, 0x8ea6, 0, 0, 0);
					send_command(0x0, 0x9ca6, 0, 0, 0);
					send_command(0x0, 0x84a6, 0, 0, 0);
					SET_POINTER(address)
					send_command(0x0, 0x88a6, 0, 0, 0);
					send_command(0x0, 0x82a6, 0, 0, 0);
					send_command(0x0, 0x0000, 0, 0, 1);
				}
				else
				{
					SET_POINTER(address)
// Setup erase
					send_command(0x0, 0x84a6, 0, 0, 0);
					send_command(0x0, 0x88a6, 0, 0, 0);

// Unlock flash
					send_command(0x0, 0x0e55, 0, 0, 0);
					send_command(0x0, 0x6ea7, 0, 0, 0);
					send_command(0x0, 0x0eaa, 0, 0, 0);
					send_command(0x0, 0x6ea7, 0, 0, 0);

// Start erase
					send_command(0x0, 0x82a6, 0, 0, 0);
					send_command(0x0, 0x0000, 0, 0, 1);
					if(use_usb)
					{
						write_usb_mask();
						write_usb_mask();
						write_usb_mask();
						write_usb_mask();
					}
					else
					{
						usleep(ERASE_DELAY);
					}
				}


// Write 64 bytes
				int write_buffer = 8;
				if(pic_model == PIC18F14K50 ||
					pic_model == PIC18F26K20 ||
					pic_model == PIC18F2450) write_buffer = 16;
				for(j = 0; j < 64 && address < max_byte; j += write_buffer)
				{
// Disable writes
					if(pic_model == PIC18F2450)
					{
						send_command(0x0, 0x8ea6, 0, 0, 0);
						send_command(0x0, 0x9ca6, 0, 0, 0);
					}
					else
					{
						send_command(0x0, 0x94a6, 0, 0, 0);
					}


//					if(use_usb)
					{
						printf("Writing 0x%04x: \n", address);
						if(!debug)
						{
							for(k = 0; k < write_buffer; k++)
							{
								printf("%02x ", data_bytes[address + k]);
							}
							printf("\n");
						}
					}
// Set pointer
					SET_POINTER(address)

// Write write_buffer bytes
					for(k = 0; k < write_buffer - 2; k += 2)
					{
						send_command(TBLWT_INC, data[address / 2], 0, 0, 0);
						address += 2;
					}

// Write final 2 bytes
					send_command(TBLWT_PROGRAM, data[address / 2], 0, 0, 0);
					address += 2;
					send_command(0x0, 0, 0, 0, 1);
				}

			}
		}
	}

	if(use_usb) write_usb_image();

// Write new program
	printf("Writing original to %s\n", orig_path);
	orig = fopen(orig_path, "w");
	if(orig)
	{
		fwrite(data, 1, max_byte, orig);
		fclose(orig);
	}

}















// The debug routine drives PGC and waits for 0xff to come from PGD.
// This aligns the serial signal.
// Every 0xff is expected to proceed with a single byte of information.
void dump_pgd()
{
	printf("Dumping PGD\n");
	read_delay = READ_DELAY3;

// Enter executable
	reset_pins();



	debug_file = fopen("/tmp/debug", "w");
	debug_txt = fopen("/tmp/debug.txt", "w");
	

	int state = DEBUG_WAIT;
	int counter = 0;
	uint64_t value = 0;
	int size = 0;
	int is_double = 0;
	int i, j;
	int parser_state = 0;
	int parser_count = 0;
	int parser_code = 0;
	unsigned char parser_data[1024];

// Parser states
	enum
	{
// Scanning for start code
		PARSER_START_CODE,
// Reading data
		PARSER_DATA
	};

	if(use_serial)
	{
		while(1)
		{
			unsigned char value = read_serial();
			int got_it = 0;
			if(do_parser)
			{
				switch(parser_state)
				{
					case PARSER_START_CODE:
						if(value & 0x80)
						{
							parser_state = PARSER_DATA;
							parser_count = 0;
							parser_code = value;
							got_it = 1;
						}
						break;
					case PARSER_DATA:
						got_it = 1;
						parser_data[parser_count++] = value;
#ifdef IS_GLIDER
						switch(parser_code)
						{
							case HEADING_CODE:
								if(parser_count >= 4)
								{
									parser_state = PARSER_START_CODE;
									printf("Head: %f ", 
										(float)(*(int32_t*)parser_data) / 100000);
								}
								break;
							case LONGITUDE_CODE:
								if(parser_count >= 4)
								{
									parser_state = PARSER_START_CODE;
									printf("Lon: %f ", 
										(float)(*(int32_t*)parser_data) / 10000000);
								}
								break;
							case LATITUDE_CODE:
								if(parser_count >= 4)
								{
									parser_state = PARSER_START_CODE;
									printf("Lat: %f ", 
										(float)(*(int32_t*)parser_data) / 10000000);
								}
								break;
							case ALTITUDE_CODE:
								if(parser_count >= 4)
								{
									parser_state = PARSER_START_CODE;
									printf("Alt: %f ", 
										(float)(*(int32_t*)parser_data) / 1000);
								}
								break;
							case GROUNDSPEED_CODE:
								if(parser_count >= 4)
								{
									parser_state = PARSER_START_CODE;
									printf("Speed: %f ", 
										(float)(*(int32_t*)parser_data) / 100);
								}
								break;
							case CLIMBRATE_CODE:
								if(parser_count >= 4)
								{
									parser_state = PARSER_START_CODE;
									printf("Climb rate: %f\n", 
										(float)(*(int32_t*)parser_data) / 100);
								}
								break;
							case WAYPOINT_HEADING_CODE:
								if(parser_count >= 4)
								{
									parser_state = PARSER_START_CODE;
									printf("Waypoint heading: %f\n", 
										(float)(*(int32_t*)parser_data) / 100000);
								}
								break;
							case WAYPOINT_DISTANCE_CODE:
								if(parser_count >= 4)
								{
									parser_state = PARSER_START_CODE;
									printf("Waypoint distance: %f\n", 
										(float)(*(int32_t*)parser_data) / 10000000);
								}
								break;
						}
#endif // IS_GLIDER
						break;
				}
				
			}

/*
 * 			if(!got_it)
 * 			{
 * 				if(value >= 0x20 && value <= 0x7e ||
 * 					value == 0xa ||
 * 					value == 0xd ||
 * 					!isatty(1))
 * 					printf("%c", value);
 * 				else
 * 					printf(".");
 * 
 * 				fflush(stdout);
 * 			}
 */

			printf("%02x ", value);
			fflush(stdout);

			if(debug_file)
			{
				fprintf(debug_txt, "%02x ", value);
				fflush(debug_txt);
				fputc(value, debug_file);
				fflush(debug_file);
			}
		}
	}
	else
	while(1)
	{
		uint16_t byte;
		uint16_t bit;
		uint16_t type;

		read_bits(&byte, 8);

		for(j = 0; j < 8; j++)
		{
			bit = byte & 0x1;
			byte >>= 1;

			switch(state)
			{
				case DEBUG_WAIT:
					is_double = 0;
					if(bit == 0)
					{
						counter = 0;
					}
					else
						counter++;

					if(counter == 4)
					{
						state = DEBUG_READ_SIZE1;
					}
					break;

				case DEBUG_READ_SIZE1:
					if(!bit)
					{
						state = DEBUG_READ_SIZE2;
						counter = 0;
						size = 0;
					}
					else
// keep reading until first 0 bit
					{
						counter = 3;
						state = DEBUG_WAIT;
					}
					break;

				case DEBUG_READ_SIZE2:
					size >>= 1;
					size |= (bit ? 0x04 : 0x00);
					counter++;
					if(counter == 3)
					{
						if(size > 0)
						{
// special case for fixed
							if(size == 5) 
							{
								size = 4;
								is_double = 1;
							}
							else
// special case for 64 bits
							if(size == 7) 
							{
								size = 8;
							}
							state = DEBUG_READ_VALUE;
						}
						else
						{
							state = DEBUG_WAIT;
						}
						counter = 0;
						value = 0;
						if(debug) printf("\n");
					}
					break;

				case DEBUG_READ_VALUE:
					value >>= 1;
					value |= ((uint64_t)bit) << (size * 8 - 1);
					counter++;
					if(counter == size * 8)
					{
						state = DEBUG_WAIT;
						counter = 0;
						if(debug) printf("\n");
						printf("Got ");

// print binary result
						if(0)
							for(i = size * 8 - 1; i >= 0; i--)
							{
								uint64_t mask = ((uint64_t)1) << i;
								if(value & mask) 
									printf("1");
								else
									printf("0");
							}

						switch(size)
						{
							case 1:
								printf(" 0x%llx\n", (long long)value);
								break;

							case 2:
								printf(" 0x%04llx\n", (long long)value);
								break;

							case 4:
								if(is_double)
									printf(" 0x%08x %f\n", 
										(unsigned int)value, 
										*(double*)&value);
								else
									printf(" 0x%08llx\n", (long long)value);
								break;

							case 8:
								printf(" 0x%16llx\n", (long long)value);
								break;

							default:
								printf(" 0x%llx\n", (long long)value);
								break;
						}


						if(debug_file)
						{
							fprintf(debug_txt, "0x%llx\n", (long long)value);
							fflush(debug_txt);
							for(i = 0; i < size; i++)
							{
								fputc(value & 0xff, debug_file);
								value >>= 8;
								fflush(debug_file);
							}
						}
					}
					else
					if(!(counter % 8) && debug) 
						printf(" ");
					break;
			}
		}
	}
}



void chip_erase()
{
	printf("chip_erase: performing bulk erase\n");
	start_programming_mode();
	if(pic_model == PIC18F2450)
	{
		SET_POINTER(0x3c0005);
		send_command(TBLWT, 0x3f3f, 0, 0, 0);
		SET_POINTER(0x3c0004);
		send_command(TBLWT, 0x8f8f, 0, 0, 0);
	}
	else
	if(pic_model == PIC18F14K50 ||
		pic_model == PIC18F26K20)
	{
		SET_POINTER(0x3c0005);
		send_command(TBLWT, 0x0f0f, 0, 0, 0);
		SET_POINTER(0x3c0004);
		send_command(TBLWT, 0x8f8f, 0, 0, 0);
	}
	else
	{
		SET_POINTER(0x3c0004);
		send_command(TBLWT, 0x0080, 0, 0, 0);
	}

	send_command(NOP, 0x0000, 0, 0, 0);
	send_command(NOP, 0x0000, 0, 0, 0);
	if(use_usb) write_usb_image();

}


// Program 2 bytes of configuration memory starting on an even address
void program_configuration(int address, uint16_t data, int verify_only)
{
	uint16_t temp;
	if(!verify_only)
	{
		printf("program_configuration: programming 0x%x with %d%d%d%d%d%d%d%d\n", 
			address,
			(data & 0x0080) ? 1 : 0,
			(data & 0x0040) ? 1 : 0,
			(data & 0x0020) ? 1 : 0,
			(data & 0x0010) ? 1 : 0,
			(data & 0x0008) ? 1 : 0,
			(data & 0x0004) ? 1 : 0,
			(data & 0x0002) ? 1 : 0,
			(data & 0x0001) ? 1 : 0);
		printf("program_configuration: programming 0x%x with %d%d%d%d%d%d%d%d\n", 
			address + 1,
			(data & 0x8000) ? 1 : 0,
			(data & 0x4000) ? 1 : 0,
			(data & 0x2000) ? 1 : 0,
			(data & 0x1000) ? 1 : 0,
			(data & 0x0800) ? 1 : 0,
			(data & 0x0400) ? 1 : 0,
			(data & 0x0200) ? 1 : 0,
			(data & 0x0100) ? 1 : 0);

		start_programming_mode();


// Direct access to configuration memory
		send_command(0x0, 0x8ea6, 0, 0, 0);
		send_command(0x0, 0x8ca6, 0, 0, 0);
// bsf EECON1, WREN
		if(pic_model == PIC18F14K50 ||
			pic_model == PIC18F26K20) send_command(0x0, 0x84a6, 0, 0, 0);
// Position program counter with a goto command
		if(pic_model != PIC18F26K20 &&
			pic_model != PIC18F2450)
		{
			send_command(0x0, 0xef00, 0, 0, 0);
			send_command(0x0, 0xf800, 0, 0, 0);
		}
// Set table pointer to configuration register low byte
		SET_POINTER(address)
// Program low byte
		send_command(TBLWT_PROGRAM, (data & 0xff), 0, 0, 0);
// nop
		send_command(0x0, 0x0000, 0, 0, 1);
// Increase pointer
		send_command(0x0, 0x2af6, 0, 0, 0);
// Program high byte
		send_command(TBLWT_PROGRAM, (data & 0xff00), 0, 0, 0);
		send_command(0x0, 0x0000, 0, 0, 1);
		if(use_usb) write_usb_image();
	}


#if 1
	reset_pins();
	init_pins();
// Verify
	printf("program_configuration: verifying\n");
	start_programming_mode();

// Direct access to configuration memory
	send_command(0x0, 0x8ea6, 0, 0, 0);
	send_command(0x0, 0x8ca6, 0, 0, 0);
// Position program counter with a goto command
	send_command(0x0, 0xef00, 0, 0, 0);
	send_command(0x0, 0xf800, 0, 0, 0);
	SET_POINTER(address);

	if(use_usb)
	{
// Clock in read commands
		send_command(TBLRD, 0, &temp, 0, 0);
		send_command(TBLRD, 0, &temp, 0, 0);
		write_usb_image();

// Read bits
		read_command(&temp);
		printf("program_configuration %d: 0x%x: %d%d%d%d%d%d%d%d\n", 
			__LINE__,
			address, 
			(temp & 0x80) ? 1 : 0,
			(temp & 0x40) ? 1 : 0,
			(temp & 0x20) ? 1 : 0,
			(temp & 0x10) ? 1 : 0,
			(temp & 0x8) ? 1 : 0,
			(temp & 0x4) ? 1 : 0,
			(temp & 0x2) ? 1 : 0,
			(temp & 0x1) ? 1 : 0);
		read_command(&temp);
		printf("program_configuration %d: 0x%x: %d%d%d%d%d%d%d%d\n", 
			__LINE__,
			address + 1, 
			(temp & 0x80) ? 1 : 0,
			(temp & 0x40) ? 1 : 0,
			(temp & 0x20) ? 1 : 0,
			(temp & 0x10) ? 1 : 0,
			(temp & 0x8) ? 1 : 0,
			(temp & 0x4) ? 1 : 0,
			(temp & 0x2) ? 1 : 0,
			(temp & 0x1) ? 1 : 0);
	}
	else
	{

// read low byte
		send_command(TBLRD, 0, &temp, 0, 0);
		printf("0x%x: %d%d%d%d%d%d%d%d\n", 
			address, 
			(temp & 0x80) ? 1 : 0,
			(temp & 0x40) ? 1 : 0,
			(temp & 0x20) ? 1 : 0,
			(temp & 0x10) ? 1 : 0,
			(temp & 0x8) ? 1 : 0,
			(temp & 0x4) ? 1 : 0,
			(temp & 0x2) ? 1 : 0,
			(temp & 0x1) ? 1 : 0);
		send_command(TBLRD, 0, &temp, 0, 0);
		printf("0x%x: %d%d%d%d%d%d%d%d\n", 
			address + 1, 
			(temp & 0x80) ? 1 : 0,
			(temp & 0x40) ? 1 : 0,
			(temp & 0x20) ? 1 : 0,
			(temp & 0x10) ? 1 : 0,
			(temp & 0x8) ? 1 : 0,
			(temp & 0x4) ? 1 : 0,
			(temp & 0x2) ? 1 : 0,
			(temp & 0x1) ? 1 : 0);
	}
#endif

}








int main(int argc, char *argv[])
{
	int i, j, k;
	uint16_t configuration_bits = 0;
	int configuration_address = 0;
	data = (uint16_t*)calloc(sizeof(uint16_t), 131072);
	data_diff64 = (unsigned char*)calloc(1, 131072);
	data_bytes = (unsigned char*)data;
	char string[1024] = { 0 };
	char program_path[1024] = { 0 };
	char orig_path[1024] = { 0 };
	int page;
// Write a program
	int do_incremental = 1;
	int do_read = 0;
	int do_erase = 0;
	int user_delay = USB_READ_DELAY;

	if(argc < 2)
	{
		printf(" -e - chip erase\n");
		printf(" -p - pic model number\n");
		printf(" -r <file> - verify against original\n");
		printf(" -i <file> - program only changes\n");
		printf(" -d [serial path] - enter debugging mode (bootloader only)\n");
		printf(" -b [serial path] <-rid> [file] - use the bootloader to perform the operations instead of the programmer\n");
//		printf(" -u <-rid> [file] - use the USB big banger & bootloader to perform the operations\n");
		printf(" -c <address> <bits> -  program 2 bytes of configuration data\n");
		printf(" -s <address> - start at address in hex\n");
		printf(" -z <delay> - bit banging delay\n");
		printf("Examples:\n");
		printf("Program the configuration register at 0x300000 with 00000000 and\n");
		printf("0x300001 with 11001111 using the programming harness.\n");
		printf("    programmer -c 0x300000 1100111100000000\n");
		printf("Upload the program using the programming harness.\n");
		printf("    programmer trapezoidian.hex\n");
		printf("Upload the program using the bootloader.\n");
		printf("    programmer -b trapezoidian.hex\n");
		printf("Upload the program incrementally on the programming harness.\n");
		printf("    programmer -i trapezoidian.hex\n");
		printf("Upload the program incrementally on the bootloader.\n");
		printf("    programmer -b -i trapezoidian.hex\n");
		printf("Verify the program on the programming harness.\n");
		printf("    programmer -r trapezoidian.hex\n");
		printf("Verify the program on the bootloader.\n");
		printf("    programmer -b -r trapezoidian.hex\n");
		printf("Verify the program starting from the address.\n");
		printf("    programmer -b -r -s 0x700 trapezoidian.hex\n");
		printf("Run the program and dump the PGD output.\n");
		printf("    programmer -d\n");
		printf("Upload the program using serial I/O.\n");
		printf("    programmer -b /dev/ttyUSB0 trapezoidian.hex\n");
		printf("Verify the program using serial I/O.\n");
		printf("    programmer -b /dev/ttyUSB0 -r trapezoidian.hex\n");
		printf("Run the program and dump the serial I/O.\n");
		printf("    programmer -d /dev/ttyUSB0\n");
		exit(1);
	}

	delay = DELAY1;
	read_delay = READ_DELAY1;

	for(i = 1; i < argc; i++)
	{
		if(!strcmp(argv[i], "-p"))
		{
			if(i + 1 >= argc)
			{
				printf("Need a PIC model for -p\n");
				exit(1);
			}

			i++;
			if(!strcmp(argv[i], "18f1320"))
				pic_model = PIC18F1220;
			else
			if(!strcmp(argv[i], "18f1220"))
				pic_model = PIC18F1220;
			else
			if(!strcmp(argv[i], "18f458"))
				pic_model = PIC18F458;
			else
			if(!strcmp(argv[i], "18f14k50"))
				pic_model = PIC18F14K50;
			else
			if(!strcmp(argv[i], "18f26k20"))
				pic_model = PIC18F26K20;
			else
			if(!strcmp(argv[i], "18f2450"))
				pic_model = PIC18F2450;
			else
			{
				printf("Unknown PIC model.\n");
				exit(1);
			}
		}
		else
		if(!strcmp(argv[i], "-e"))
		{
			do_erase = 1;
			do_incremental = 0;
			break;
		}
		else
		if(!strcmp(argv[i], "-z"))
		{
			if(i + 1 >= argc)
			{
				printf("Need a delay\n");
				exit(1);
			}
			user_delay = 0xffff - atoi(argv[i + 1]);
			i++;
		}
		else
		if(!strcmp(argv[i], "-c"))
		{
			do_configuration = 1;
			do_incremental = 0;
			if(i + 2 >= argc)
			{
				printf("Need address and bits for configuration programming.\n");
				exit(1);
			}
			if(strlen(argv[i + 2]) != 16)
			{
				printf("Configuration bits must be 16 bits\n");
				exit(1);
			}
			sscanf(argv[i + 1], "%x", &configuration_address);
			for(j = 0, k = 0x8000; j < 16; j++, k >>= 1)
			{
				if(argv[i + 2][j] == '1') configuration_bits |= k;
			}
			i += 2;
		}
		else
		if(!strcmp(argv[i], "-s"))
		{
			if(i + 2 >= argc)
			{
				printf("Need address for start address.\n");
				exit(1);
			}
			sscanf(argv[i + 1], "%x", &start_address);
			i++;
		}
		else
		if(!strcmp(argv[i], "-r"))
		{
			do_read = 1;
			do_incremental = 0;
		}
		else
// 		if(!strcmp(argv[i], "-u"))
// 		{
// 			use_usb = 1;
// 		}
// 		else
		if(!strcmp(argv[i], "-b"))
		{
			use_bootloader = 1;
			delay = DELAY2;
			read_delay = READ_DELAY2;
			if(i < argc)
			{
				if(!strncmp(argv[i + 1], "/dev/tty", 8))
				{
					serial_path = argv[++i];
					use_serial = 1;
					use_usb = 0;
				}
			}
			else
			{
				printf("-b needs either a .hex path or serial path.\n");
			}
		}
		else
		if(!strcmp(argv[i], "-i"))
		{
			do_incremental = 1;
		}
		else
		if(!strcmp(argv[i], "-d"))
		{
			use_bootloader = 1;
			do_debugger = 1;
			do_incremental = 0;
			if(i < argc - 1)
			{
				if(!strncmp(argv[i + 1], "/dev/tty", 8))
				{
					serial_path = argv[++i];
					use_serial = 1;
				}
			}
		}
		else
		{
			strcpy(program_path, argv[i]);
			FILE *in = fopen(program_path, "r");
			if(!in)
			{
				perror("fopen");
				exit(1);
			}

			page = 0;
			
// Top address in bytes
			max_byte = 0;
// Bootloader addresses from after the bootloader
			if(use_bootloader)
				min_address = 0xffff;
			else
// Boot ROM addresses from 0
				min_address = 0x0;

			while(fgets(string, 1024, in))
			{
// Convert string to binary
				unsigned char string2[1024];
				unsigned char *in_ptr = (unsigned char*)string + 1;
				unsigned char *out_ptr = string2;
				for(j = 0; j < strlen(string); j += 2)
				{
					int character = toupper(*in_ptr++);
					if(character >= '0' && character <= '9')
						*out_ptr = (character - '0') << 4;
					else
						*out_ptr = (10 + character - 'A') << 4;

					character = toupper(*in_ptr++);
					if(character >= '0' && character <= '9')
						*out_ptr |= character - '0';
					else
						*out_ptr |= 10 + character - 'A';
					out_ptr++;
				}


				unsigned char *ptr = string2;

// Number of bytes of data in the line
				int data_bytes = *ptr++;
				if(!data_bytes) break;

// Starting address of data
				int address = (*ptr++) << 8;
				address |= *ptr++;

				int type = *ptr++;

// Data is the number of a page
// TODO: handle this properly so it doesn't clobber the lower addresses
				if(type == 4)
				{
					page = (*ptr++) << 8;
					page |= *ptr++;
					if(page != 0)
					{
						printf("page = %d  Don't know what to do.\n", page);
					}
				}
				else
// Data is program data
				{
					if(min_address > address)
						min_address = address;
					for(j = 0; j < data_bytes; j += 2)
					{
						uint16_t word = *ptr++;
						word |= (*ptr++) << 8;
						data[address >> 1] = word;
						address += 2;
					}
					if(max_byte < address)
						max_byte = address;
				}

// Checksum
				ptr++;
			}
			
// Dump data
			for(j = 0x0; j < max_byte; j += 16)
			{
				printf("%08x: %04x %04x %04x %04x %04x %04x %04x %04x\n", 
					j,
					data[j / 2 + 0],
					data[j / 2 + 1],
					data[j / 2 + 2],
					data[j / 2 + 3],
					data[j / 2 + 4],
					data[j / 2 + 5],
					data[j / 2 + 6],
					data[j / 2 + 7]);
			}
			
		}
	}





	int total_bytes = max_byte - min_address;
	int current_bytes = 0;

	if(program_path[0])
		sprintf(orig_path, "%s.orig", program_path);
	else
		do_incremental = 0;

// Set delays
//	if(do_read)
//		usb_delay = USB_READ_DELAY;
//	else
//		usb_delay = USB_WRITE_DELAY;
	usb_delay = user_delay;
	usb_delay2 = USB_PROGRAMMING_DELAY;
// Doesn't need delay for the GPS speedometer
	switch(pic_model)
	{
// Uses the USB pins for programming
		case PIC18F14K50:
 		case PIC18F26K20:
 			if(do_configuration)
			{
				usb_delay = 0xb000;
				usb_delay2 = USB_CONFIG_DELAY;
			}
			break;
	}

	char *address_changed = calloc(1, total_bytes / 2);

//	signal(SIGINT, signal_entry);

// Initialize communications
	if(!use_usb)
	{
		struct sched_param params;
		params.sched_priority = 1;
		sched_setscheduler(0, SCHED_RR, &params);

		init_pins();
	
//		test_pins();
	}





	if(use_serial)
	{
		if(init_serial())
		{
			reset_pins();
			exit(1);
		}
	}
	
	if(use_usb)
	{
		if(init_usb()) exit(1);
	}

// Do the operation
	if(do_erase)
	{
		chip_erase();
	}
	else
	if(do_configuration)
	{
		program_configuration(configuration_address, configuration_bits, do_read);
	}
	else
	if(do_read && orig_path[0])
	{
		verify(max_byte);
// Write original copy
		if(do_read)
			printf("Writing chip contents to %s\n", orig_path);
		else
			printf("Writing original to %s\n", orig_path);


		FILE *orig = fopen(orig_path, "w");
		fwrite(data, 1, max_byte, orig);
		fclose(orig);
	}
	else
	if(do_incremental)
	{
/*
 * 		if(!use_bootloader && 
 * 			!strstr(orig_path, "pic_bootloader.hex") &&
 * 			!strstr(orig_path, "pic_ground_bootloader.hex") &&
 * 			!strstr(orig_path, "pic_test_bootloader.hex") &&
 * 			!strstr(orig_path, "test.hex") &&
 * 			!strstr(orig_path, "pic_protect.hex"))
 * 		{
 * 			printf("Can't write anything but bootloader with direct access.\n");
 * 			exit(1);
 * 		}
 */

		write_incremental(orig_path);
	}
	else
	if(do_debugger)
	{
		dump_pgd();
	}

	reset_pins();
}

