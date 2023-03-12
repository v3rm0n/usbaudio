#include <libusb.h>
#include <SDL.h>
#include "audio.h"

int do_main_loop = 1;

static libusb_device_handle *devh = NULL;
static libusb_context *ctx = NULL;

void intHandler() {
  do_main_loop = 0;
}

int main() {

  signal(SIGINT, intHandler);
  signal(SIGTERM, intHandler);
#ifdef SIGQUIT
  signal(SIGQUIT, intHandler);
#endif

  int r;
  r = libusb_init(&ctx);
  if (r < 0) {
    SDL_Log("libusb_init failed: %s", libusb_error_name(r));
    return 0;
  }
  devh = libusb_open_device_with_vid_pid(ctx, 0x16c0, 0x048a);
  if (devh == NULL) {
    SDL_Log("libusb_open_device_with_vid_pid returned invalid handle");
    return 0;
  }
  SDL_Log("USB device init success");

  init_audio(devh);

  while (do_main_loop) {
    int rc = libusb_handle_events(ctx);
    if (rc != LIBUSB_SUCCESS) {
      SDL_Log("Audio loop error: %s\n", libusb_error_name(rc));
      break;
    }
  }

  close_audio(devh);

  libusb_close(devh);

  libusb_exit(ctx);

  return 0;
}
