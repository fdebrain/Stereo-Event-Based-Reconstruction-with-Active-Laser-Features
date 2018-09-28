#ifndef EBV_UTILSUSB_H
#define EBV_UTILSUSB_H

#include <libusb-1.0/libusb.h>

#include <stdio.h>
#include <sys/types.h>
#include <errno.h>
#include <unistd.h>
//#include <cstring>

void print_devs(char* output,libusb_device** devs);
void print_dev_desc(char* output, libusb_device_handle* devh, libusb_device_descriptor* desc);
void print_endp_desc(char* output, const libusb_endpoint_descriptor *desc);
void print_interface_desc(char* output, libusb_device_handle* devh, const libusb_interface_descriptor* desc);
void print_config_desc(char* output, libusb_device_handle* devh, const libusb_config_descriptor* desc);

#endif // EBV_UTILSUSB_H
