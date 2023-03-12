#include <libusb.h>
#include <SDL.h>

#define EP_ISO_IN 0x85
#define AUDIO_INTERFACE 4

#define NUM_TRANSFERS 10
#define PACKET_SIZE 180
#define NUM_PACKETS 64

static struct libusb_transfer *transfers[NUM_TRANSFERS];

static SDL_AudioDeviceID sdl_audio_device_id = 0;

static void transfer_callback(struct libusb_transfer *transfer) {
  unsigned int i;

  for (i = 0; i < transfer->num_iso_packets; i++) {
    struct libusb_iso_packet_descriptor *pack = &transfer->iso_packet_desc[i];

    if (pack->status != LIBUSB_TRANSFER_COMPLETED) {
      SDL_Log("XFR callback error (status %d: %s)", pack->status,
              libusb_error_name(pack->status));
      /* This doesn't happen, so bail out if it does. */
      return;
    }

    const Uint8 *data = libusb_get_iso_packet_buffer_simple(transfer, i);

    if (sdl_audio_device_id != 0) {
      SDL_QueueAudio(sdl_audio_device_id, data, pack->actual_length);
    }
  }

  if (libusb_submit_transfer(transfer) < 0) {
    SDL_Log("error re-submitting URB\n");
  }
}

static int start_audio_capture(libusb_device_handle *devh) {
  static Uint8 buffer[PACKET_SIZE * NUM_PACKETS];

  for (int i = 0; i < NUM_TRANSFERS; i++) {
    transfers[i] = libusb_alloc_transfer(NUM_PACKETS);
    if (!transfers[i]) {
      SDL_Log("Could not allocate transfer");
      return -12;
    }

    libusb_fill_iso_transfer(transfers[i], devh, EP_ISO_IN, buffer,
                             sizeof(buffer), NUM_PACKETS, transfer_callback, NULL, 0);
    libusb_set_iso_packet_lengths(transfers[i], sizeof(buffer) / NUM_PACKETS);

    libusb_submit_transfer(transfers[i]);
  }

  return 0;
}

int audio_device_id = 0;

static int claim_audio_interface(libusb_device_handle *devh) {
  int rc = libusb_kernel_driver_active(devh, AUDIO_INTERFACE);
  if (rc == 1) {
    SDL_Log("Detaching kernel driver");
    rc = libusb_detach_kernel_driver(devh, AUDIO_INTERFACE);
    if (rc < 0) {
      SDL_Log("Could not detach kernel driver: %s\n",
              libusb_error_name(rc));
      return rc;
    }
  }
  rc = libusb_claim_interface(devh, AUDIO_INTERFACE);
  if (rc < 0) {
    SDL_Log("Error claiming interface: %s\n", libusb_error_name(rc));
    return rc;
  }
  return rc;
}

int init_audio(libusb_device_handle *devh) {

  SDL_Log("USB audio setup");

  int rc;

  rc = claim_audio_interface(devh);
  if (rc < 0) {
    return rc;
  }

  rc = libusb_set_interface_alt_setting(devh, AUDIO_INTERFACE, 1);
  if (rc < 0) {
    SDL_Log("Error setting alt setting: %s\n", libusb_error_name(rc));
    return rc;
  }

  if (!SDL_WasInit(SDL_INIT_AUDIO)) {
    if (SDL_InitSubSystem(SDL_INIT_AUDIO) < 0) {
      SDL_Log("Init audio failed %s", SDL_GetError());
      return -1;
    }
  } else {
    SDL_Log("Audio was already initialised");
  }

  static SDL_AudioSpec audio_spec;
  audio_spec.format = AUDIO_S16;
  audio_spec.channels = 2;
  audio_spec.freq = 44100;

  SDL_AudioSpec _obtained;
  SDL_zero(_obtained);


  SDL_Log("Current audio driver is %s and device %d", SDL_GetCurrentAudioDriver(),
          audio_device_id);

  sdl_audio_device_id = SDL_OpenAudioDevice(NULL, 0, &audio_spec, &_obtained, SDL_FALSE);

  SDL_Log("Obtained audio spec. Sample rate: %d, channels: %d, samples: %d, size: %d",
          _obtained.freq,
          _obtained.channels,
          _obtained.samples, _obtained.size);

  SDL_PauseAudioDevice(sdl_audio_device_id, SDL_FALSE);

  SDL_Log("Starting capture");
  if ((rc = start_audio_capture(devh)) < 0) {
    SDL_Log("Capture failed to start: %d", rc);
    return rc;
  }

  SDL_Log("Successful init");
  return 0;
}

int close_audio(libusb_device_handle *devh) {
  SDL_Log("Closing audio");

  int rc, i;

  for (i = 0; i < NUM_TRANSFERS; i++) {
    rc = libusb_cancel_transfer(transfers[i]);
    if (rc < 0) {
      SDL_Log("Error cancelling transfer: %s\n", libusb_error_name(rc));
      return rc;
    }
  }

  if (sdl_audio_device_id != 0) {
    SDL_Log("Closing audio device %d", sdl_audio_device_id);
    SDL_AudioDeviceID device = sdl_audio_device_id;
    sdl_audio_device_id = 0;
    SDL_CloseAudioDevice(device);
  }

  SDL_Log("Releasing interface %d", AUDIO_INTERFACE);

  rc = libusb_release_interface(devh, AUDIO_INTERFACE);
  if (rc < 0) {
    SDL_Log("Error releasing interface: %s\n", libusb_error_name(rc));
    return rc;
  }

  SDL_Log("Audio closed");

  return 0;
}

