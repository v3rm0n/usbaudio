#include <libusb.h>
#include <SDL.h>
#include "libmaru.h"

libusb_device_handle *devh = NULL;

#define EP_ISO_IN 0x85
#define AUDIO_INTERFACE 4

#define NUM_TRANSFERS 1
#define PACKET_SIZE 180
#define NUM_PACKETS 16

static SDL_AudioDeviceID sdl_audio_device_id = 0;


int audio_device_id = 0;
maru_context *ctx = NULL;

int init_audio() {

  SDL_Log("USB audio setup");


  struct maru_stream_desc desc = {
          .sample_rate=44100,
          .channels = 2,
          .bits=16
  };

  maru_create_context_from_vid_pid(&ctx, 0x16c0, 0x048a, &desc);

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
  audio_spec.samples = PACKET_SIZE * NUM_PACKETS;

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

  SDL_Log("Successful init");
  return 0;
}

int close_audio() {
  SDL_Log("Closing audio");

  maru_destroy_context(ctx);

  int rc;

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


int main() {

  init_audio();

  // Enter the SDL event loop
  SDL_Event event;
  while (1) {
    SDL_PollEvent(&event);
    if (event.type == SDL_QUIT) {
      break;
    }
  }

  close_audio();

  libusb_close(devh);

  libusb_exit(NULL);

  return 0;
}
