#ifndef AUDIO_H
#define AUDIO_H

#include <libusb.h>

int init_audio(libusb_device_handle *devh);

int close_audio(libusb_device_handle *devh);

#endif
