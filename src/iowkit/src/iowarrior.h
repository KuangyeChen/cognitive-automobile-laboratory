#ifndef _IOWARRIOR_H_
#define _IOWARRIOR_H_

/* Version Information */
#define DRIVER_VERSION "v0.5.0"
#define DRIVER_AUTHOR "Christian Lucht <lucht@codemercs.com>"
#define DRIVER_DESC "USB IO-Warrior driver (Linux 2.6.x)"
#define DRIVER_SUPPORTED_DEVICES "IOWarrior USB Devices"

#define USB_VENDOR_ID_CODEMERCS	1984
/* low speed iowarrior */
#define USB_DEVICE_ID_CODEMERCS_IOW40	0x1500
#define USB_DEVICE_ID_CODEMERCS_IOW24	0x1501
#define USB_DEVICE_ID_CODEMERCS_IOWPV1	0x1511
#define USB_DEVICE_ID_CODEMERCS_IOWPV2	0x1512
/* full speed iowarrior */
#define USB_DEVICE_ID_CODEMERCS_IOW56	0x1503

#define CODEMERCS_MAGIC_NUMBER	0xC0	// like COde Mercenaries

/* Define the ioctl commands for reading and writing data */
#define IOW_WRITE	_IOW(CODEMERCS_MAGIC_NUMBER, 1, long)
#define IOW_READ	_IOW(CODEMERCS_MAGIC_NUMBER, 2, long)

/* Define an ioctl command for getting more info on the device */
 
/* A struct for available device info which is used */
/* in the new ioctl IOW_GETINFO                     */
struct iowarrior_info {
	int vendor;			// vendor id : supposed to be USB_VENDOR_ID_CODEMERCS in all cases
	int product;			// product id : depends on type of chip (USB_DEVICE_ID_CODEMERCS_XXXXX)
	char serial[9];			// the serial number of our chip (if a serial-number is not available this is empty string)
	int revision;			// revision number of the chip
	int speed;			// USB-speed of the device (0=UNKNOWN, 1=LOW, 2=FULL 3=HIGH)
	int power;			// power consumption of the device in mA
	int if_num;			// the number of the endpoint
	unsigned int packet_size;	// size of the data-packets on this interface
};

/*
  Get some device-information (product-id , serial-number etc.)
  in order to identify a chip.
*/
#define IOW_GETINFO _IOR(CODEMERCS_MAGIC_NUMBER, 3, struct iowarrior_info)

/* Get a minor range for your devices from the usb maintainer */
#ifdef CONFIG_USB_DYNAMIC_MINORS
#define IOWARRIOR_MINOR_BASE	0
#else
// SKELETON_MINOR_BASE 192 + 16, not offical yet
#define IOWARRIOR_MINOR_BASE	208
#endif

/* interrupt input queue sizes */
#define MAX_INTERRUPT_BUFFER_IN 16
#define MAX_INTERRUPT_BUFFER_OUT 16

#endif  // _IOWARRIOR_H_
