/* libmaru - Userspace USB audio class driver.
 * Copyright (C) 2012 - Hans-Kristian Arntzen
 * Copyright (C) 2012 - Agnes Heyer
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "libmaru.h"
#include "fifo.h"
#include <libusb.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <sys/event.h>
#include <time.h>
#include <SDL_log.h>

/** \ingroup lib
 * \brief Struct holding information needed for a libusb transfer. */
struct maru_transfer {
  /** Underlying libusb transfer struct */
  struct libusb_transfer *trans;

  /** Associated context */
  maru_context *ctx;

  /** Associated stream */
  struct maru_stream_internal *stream;

  /** Associated fifo region */
  struct maru_fifo_locked_region region;

  /** Set if the transfer is currently queued up in libusb's event system. */
  bool active;
  /** Set if transfer should not be queued up again.
   * Used mostly for feedback endpoint. */
  bool block;

  /** Capacity of embedded_data */
  size_t embedded_data_capacity;
  /** Embeddable structure for use to transfer data that
   * cannot be transfered contigously in region. */
  uint8_t embedded_data[];
};

/** \ingroup lib
 * \brief Struct holding a list of active and vacant transfers for a stream. */
struct transfer_list {
  /** Vector of allocated transfers. */
  struct maru_transfer **transfers;
  /** Capacity of transfers. */
  size_t capacity;
  /** Size of transfers. */
  size_t size;
};

/** \ingroup lib
 * \brief Struct holding information needed for a single stream. */
struct maru_stream_internal {
  /** The associated fifo of a stream. Set to NULL if a stream is not claimed. */
  maru_fifo *fifo;
  /** Associated streaming endpoint of the stream. */
  unsigned stream_ep;
  /** Associated feedback endpoint of the stream. */
  unsigned feedback_ep;

  /** Interface index for audio streaming interface */
  unsigned stream_interface;
  /** Altsetting for streaming interface */
  unsigned stream_altsetting;

  /** Fixed point transfer speed in audio frames / USB frame (16.16). */
  uint32_t transfer_speed;
  /** Fraction that keeps track of when to send extra frames to keep up with transfer_speed. */
  uint32_t transfer_speed_fraction;
  /** Multiplier for transfer_speed to convert frames to bytes. */
  unsigned transfer_speed_mult;

  /** Optional callback to notify write avail. */
  maru_notification_cb read_cb;
  /** Userdata for write_cb */
  void *read_userdata;
  /** Optional callbak to notify errors in stream. */
  maru_notification_cb error_cb;
  /** Userdata for error_cb */
  void *error_userdata;

  /** pipe to synchronize tear-down of a stream. */
  int sync_pipe[2];

  /** Transfer list. */
  struct transfer_list trans;
  /** Transfers currently queued up. */
  unsigned trans_count;
  /** Maximum number of transfer allowed to be queued up. */
  unsigned enqueue_count;

  struct {
    /** Set if timer is started, and start_time and offset have valid times. */
    bool started;
    /** Time of timer start. */
    maru_usec start_time;
    /** Timer offset. Used to combat clock skew. */
    maru_usec offset;
    /** Total write count, used along with start_time to calculate latency. */
    uint64_t read_cnt;
  } timer;

};


/** \ingroup lib
 * \brief Struct holding information in the libmaru context. */
struct maru_context {
  /** Underlying libusb context */
  libusb_context *ctx;
  /** libusb device handle to the audio card */
  libusb_device_handle *handle;
  /** Cached configuration descriptor for audio card */
  struct libusb_config_descriptor *conf;

  /** List of allocated streams */
  struct maru_stream_internal *streams;
  /** Number of allocated hardware streams */
  unsigned num_streams;

  /** File descriptor that thread polls for to tear down cleanly in maru_destroy_context(). */
  int quit_pipe[2];

  /** kqueue descriptor that thread polls on. */
  int kqfd;

  /** Context-global lock */
  pthread_mutex_t lock;
  /** Thread */
  pthread_t thread;
  /** Set to true if thread has died prematurely */
  bool thread_dead;
};

// USB audio spec structures.
struct usb_uas_format_descriptor {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bDescriptorSubtype;
  uint8_t bFormatType;
  uint8_t bNrChannels;
  uint8_t nSubFrameSize;
  uint8_t nBitResolution;
  uint8_t bSamFreqType;
  uint8_t tSamFreq[];
} __attribute__((packed));

#define USB_CLASS_AUDIO                1
#define USB_SUBCLASS_AUDIO_STREAMING   2

#define USB_ENDPOINT_ISOCHRONOUS       0x01
#define USB_ENDPOINT_ADAPTIVE          0x08

#define USB_CLASS_DESCRIPTOR           0x20
#define USB_INTERFACE_DESCRIPTOR_TYPE  0x04
#define USB_FORMAT_DESCRIPTOR_SUBTYPE  0x02
#define USB_FORMAT_TYPE_I              0x01

#define USB_AUDIO_FEEDBACK_SIZE        3

static bool poll_list_add(int kqfd, int fd, short events) {
  struct kevent event[2];
  int n = 0;

  if (events & POLLIN) {
    EV_SET(&event[n++], fd, EVFILT_READ, EV_ADD, 0, 0, 0);
  }

  if (events & POLLOUT) {
    EV_SET(&event[n++], fd, EVFILT_WRITE, EV_ADD, 0, 0, 0);
  }

  return kevent(kqfd, event, n, NULL, 0, NULL) == 0;
}

static bool poll_list_remove(int kqfd, int fd) {
  struct kevent event[2];
  int n = 0;

  EV_SET(&event[n++], fd, EVFILT_READ, EV_DELETE, 0, 0, 0);
  EV_SET(&event[n++], fd, EVFILT_WRITE, EV_DELETE, 0, 0, 0);

  return kevent(kqfd, event, n, NULL, 0, NULL) == 0;
}

static void poll_list_unblock(int kqfd, int fd, short events) {
  struct kevent event[2];
  int n = 0;

  if (events & POLLIN) {
    EV_SET(&event[n++], fd, EVFILT_READ, EV_ENABLE, 0, 0, 0);
  }

  if (events & POLLOUT) {
    EV_SET(&event[n++], fd, EVFILT_WRITE, EV_ENABLE, 0, 0, 0);
  }

  if (kevent(kqfd, event, n, NULL, 0, NULL) < 0) {
    fprintf(stderr, "poll_list_unblock() failed!\n");
    perror("kevent");
  }
}

static void poll_list_block(int kqfd, int fd) {
  struct kevent event[2];
  int n = 0;

  EV_SET(&event[n++], fd, EVFILT_READ, EV_DISABLE, 0, 0, 0);
  EV_SET(&event[n++], fd, EVFILT_WRITE, EV_DISABLE, 0, 0, 0);

  if (kevent(kqfd, event, n, NULL, 0, NULL) < 0) {
    fprintf(stderr, "poll_list_block() failed!\n");
    perror("kevent");
  }
}

static inline void ctx_lock(maru_context *ctx) {
  pthread_mutex_lock(&ctx->lock);
}

static inline void ctx_unlock(maru_context *ctx) {
  pthread_mutex_unlock(&ctx->lock);
}

static void poll_added_cb(int fd, short events, void *userdata) {
  maru_context *ctx = userdata;
  poll_list_add(ctx->kqfd, fd, events);
}

static void poll_removed_cb(int fd, void *userdata) {
  maru_context *ctx = userdata;
  poll_list_remove(ctx->kqfd, fd);
}

static bool poll_list_init(maru_context *ctx) {
  bool ret = true;

  const struct libusb_pollfd **list = libusb_get_pollfds(ctx->ctx);
  if (!list)
    return false;

  const struct libusb_pollfd **tmp = list;
  while (*tmp) {
    const struct libusb_pollfd *fd = *tmp;
    if (!poll_list_add(ctx->kqfd, fd->fd, fd->events)) {
      ret = false;
      goto end;
    }
    tmp++;
  }

  if (!poll_list_add(ctx->kqfd, ctx->quit_pipe[0], POLLIN)) {
    ret = false;
    goto end;
  }

  libusb_set_pollfd_notifiers(ctx->ctx, poll_added_cb, poll_removed_cb,
                              ctx);

  end:
  free(list);
  return ret;
}

static void poll_list_deinit(maru_context *ctx) {
  libusb_set_pollfd_notifiers(ctx->ctx, NULL, NULL, NULL);

  if (ctx->kqfd >= 0) {
    close(ctx->kqfd);
    ctx->kqfd = -1;
  }
}

static struct maru_stream_internal *fd_to_stream(maru_context *ctx, int fd) {
  struct maru_stream_internal *ret = NULL;

  for (unsigned i = 0; i < ctx->num_streams; i++) {
    if (ctx->streams[i].fifo &&
        fd == maru_fifo_read_notify_fd(ctx->streams[i].fifo)) {
      ret = &ctx->streams[i];
      break;
    }
  }

  return ret;
}

static void free_transfers_stream(maru_context *ctx,
                                  struct maru_stream_internal *stream);

static struct maru_transfer *find_vacant_transfer(struct maru_transfer **transfers,
                                                  size_t length, size_t required_buffer) {
  for (size_t i = 0; i < length; i++) {
    if (!transfers[i]->active && transfers[i]->embedded_data_capacity >= required_buffer)
      return transfers[i];
  }

  return NULL;
}

static bool append_transfer(struct transfer_list *list, struct maru_transfer *trans) {
  if (list->size >= list->capacity) {
    size_t new_capacity = list->capacity * 2 + 1;
    struct maru_transfer **new_trans = realloc(list->transfers, new_capacity * sizeof(trans));
    if (!new_trans)
      return false;

    list->capacity = new_capacity;
    list->transfers = new_trans;
  }

  list->transfers[list->size++] = trans;
  return true;
}

#define LIBMARU_MAX_ENQUEUE_COUNT 32
#define LIBMARU_MAX_ENQUEUE_TRANSFERS 4

static bool parse_audio_format(const uint8_t *data, size_t size, struct maru_stream_desc *desc);

static bool format_matches(const struct libusb_interface_descriptor *iface,
                           const struct maru_stream_desc *desc) {
  struct maru_stream_desc format_desc;

  if (!parse_audio_format(iface->extra, iface->extra_length, &format_desc))
    return false;

  if (!desc)
    return true;

  if (desc->sample_rate) {
    if (format_desc.sample_rate && desc->sample_rate != format_desc.sample_rate)
      return false;
    else if (!format_desc.sample_rate &&
             (desc->sample_rate < format_desc.sample_rate_min ||
              desc->sample_rate > format_desc.sample_rate_max))
      return false;
  }

  if (desc->channels && desc->channels != format_desc.channels)
    return false;
  if (desc->bits && desc->bits != format_desc.bits)
    return false;

  return true;
}

static struct maru_transfer *create_transfer(struct transfer_list *list, size_t required_buffer) {
  struct maru_transfer *trans = calloc(1, sizeof(*trans) + required_buffer);
  if (!trans)
    return NULL;

  trans->embedded_data_capacity = required_buffer;
  trans->trans = libusb_alloc_transfer(LIBMARU_MAX_ENQUEUE_COUNT);
  if (!trans->trans)
    goto error;

  if (!append_transfer(list, trans))
    goto error;

  return trans;

  error:
  free(trans);
  return NULL;
}

static void transfer_stream_cb(struct libusb_transfer *trans) {
  struct maru_transfer *transfer = trans->user_data;
  transfer->active = false;

  transfer->stream->trans_count--;

  // If we are deiniting, we will die before this can be used.
  if (!transfer->block) {
    poll_list_unblock(transfer->ctx->kqfd,
                      maru_fifo_read_notify_fd(transfer->stream->fifo),
                      POLLIN);

    if (maru_fifo_read_unlock(transfer->stream->fifo, &transfer->region) != LIBMARU_SUCCESS)
      fprintf(stderr, "Error occured during read unlock!\n");
  }

  if (trans->status == LIBUSB_TRANSFER_CANCELLED)
    return;

  maru_notification_cb cb = transfer->stream->read_cb;
  void *userdata = transfer->stream->read_userdata;
  if (cb)
    cb(userdata);

  for (int i = 0; i < trans->num_iso_packets; i++) {
    if (trans->iso_packet_desc[i].length != trans->iso_packet_desc[i].actual_length)
      fprintf(stderr, "Actual length differs from sent length! (Actual: %d, Requested: %d)\n",
              trans->iso_packet_desc[i].actual_length,
              trans->iso_packet_desc[i].length);
  }

  if (trans->status != LIBUSB_TRANSFER_COMPLETED)
    fprintf(stderr, "Stream callback: Failed transfer ... (status: %d)\n", trans->status);
}

static void transfer_feedback_cb(struct libusb_transfer *trans) {
  struct maru_transfer *transfer = trans->user_data;

  if (trans->status == LIBUSB_TRANSFER_CANCELLED)
    transfer->active = false;
  else if (transfer->block) {
    transfer->active = false;
    transfer->block = false;
  }

  if (!transfer->active)
    return;

  if (trans->status == LIBUSB_TRANSFER_COMPLETED) {
    uint32_t fraction =
            (trans->buffer[0] << 0) |
            (trans->buffer[1] << 8) |
            (trans->buffer[2] << 16);

    fraction <<= 2;

    transfer->stream->transfer_speed_fraction =
    transfer->stream->transfer_speed = fraction;
    //////////
  }

  if (libusb_submit_transfer(trans) < 0)
    fprintf(stderr, "Resubmitting feedback transfer failed ...\n");
}

static void fill_transfer(maru_context *ctx,
                          struct maru_transfer *trans, const struct maru_fifo_locked_region *region,
                          const unsigned *packet_len, unsigned packets) {
  libusb_fill_iso_transfer(trans->trans,
                           ctx->handle,
                           trans->stream->stream_ep,

          // If we're contigous in ring buffer, we can just read directly from it.
                           region->second ? trans->embedded_data : region->first,

                           region->first_size + region->second_size,
                           packets,
                           transfer_stream_cb,
                           trans,
                           1000);

  for (unsigned i = 0; i < packets; i++)
    trans->trans->iso_packet_desc[i].length = packet_len[i];

  if (region->second) {
    memcpy(trans->embedded_data, region->first, region->first_size);
    memcpy(trans->embedded_data + region->first_size, region->second, region->second_size);
  }

  trans->region = *region;
}

static bool enqueue_transfer(maru_context *ctx, struct maru_stream_internal *stream,
                             const struct maru_fifo_locked_region *region, const unsigned *packet_len,
                             unsigned packets) {
  // If our region is split, we have to make a copy to get a contigous transfer.
  size_t required_buffer = region->second_size ? region->first_size + region->second_size : 0;

  // If we can reap old, used transfers, we'll reuse them as-is, no need to reallocate
  // transfers all the time. Eventually, given a fixed sized fifo buffer,
  // no new transfer will have to be
  // allocated, as there is always a vacant one.
  struct maru_transfer *transfer = find_vacant_transfer(stream->trans.transfers,
                                                        stream->trans.size, required_buffer);

  if (!transfer)
    transfer = create_transfer(&stream->trans, required_buffer);

  if (!transfer)
    return false;

  transfer->stream = stream;
  transfer->ctx = ctx;
  transfer->active = true;

  fill_transfer(ctx, transfer, region, packet_len, packets);

  if (libusb_submit_transfer(transfer->trans) < 0) {
    transfer->active = false;
    return false;
  }

  stream->trans_count++;
  if (stream->trans_count >= LIBMARU_MAX_ENQUEUE_TRANSFERS && stream->fifo)
    poll_list_block(ctx->kqfd, maru_fifo_read_notify_fd(stream->fifo));

  return true;
}

static bool enqueue_feedback_transfer(maru_context *ctx, struct maru_stream_internal *stream) {
  struct maru_transfer *trans = calloc(1, sizeof(*trans) + USB_AUDIO_FEEDBACK_SIZE);
  if (!trans)
    return NULL;

  trans->embedded_data_capacity = USB_AUDIO_FEEDBACK_SIZE;

  trans->trans = libusb_alloc_transfer(1);
  if (!trans->trans)
    goto error;

  libusb_fill_iso_transfer(trans->trans,
                           ctx->handle,
                           stream->feedback_ep,
                           trans->embedded_data,
                           USB_AUDIO_FEEDBACK_SIZE,
                           1, transfer_feedback_cb, trans, 1000);

  libusb_set_iso_packet_lengths(trans->trans, USB_AUDIO_FEEDBACK_SIZE);

  trans->stream = stream;
  trans->active = true;

  if (!append_transfer(&stream->trans, trans))
    goto error;

  if (libusb_submit_transfer(trans->trans) < 0) {
    trans->active = false;
    return false;
  }

  return true;

  error:
  if (trans) {
    libusb_free_transfer(trans->trans);
    free(trans);
  }
  return false;
}

static size_t stream_chunk_size(struct maru_stream_internal *stream) {
  size_t new_fraction = stream->transfer_speed_fraction + (stream->transfer_speed & 0xffff);

  size_t to_write = new_fraction >> 16;
  to_write *= stream->transfer_speed_mult;

  return to_write;
}

static void stream_chunk_size_finalize(struct maru_stream_internal *stream) {
  // Calculate fractional speeds (async isochronous).
  stream->transfer_speed_fraction += stream->transfer_speed & 0xffff;

  // Wrap-around.
  stream->transfer_speed_fraction = (UINT32_C(0xffff0000) & stream->transfer_speed)
                                    | (stream->transfer_speed_fraction & 0xffff);
}

static void handle_stream(maru_context *ctx, struct maru_stream_internal *stream) {
  size_t avail = maru_fifo_read_avail(stream->fifo);

  unsigned packet_len[LIBMARU_MAX_ENQUEUE_COUNT];
  unsigned packets = 0;
  size_t total_write = 0;

  size_t to_write = stream_chunk_size(stream);
  while (avail >= to_write && packets < stream->enqueue_count) {
    total_write += to_write;
    packet_len[packets++] = to_write;
    avail -= to_write;
    stream_chunk_size_finalize(stream);
    to_write = stream_chunk_size(stream);
  }

  if (packets) {
    struct maru_fifo_locked_region region;
    maru_fifo_read_lock(stream->fifo, total_write,
                        &region);

    if (!enqueue_transfer(ctx, stream, &region, packet_len, packets))
      fprintf(stderr, "Enqueue transfer failed!\n");
  }

  // We are being killed, kill all transfers and tell other thread it's safe to deinit.
  if (maru_fifo_read_notify_ack(stream->fifo) != LIBMARU_SUCCESS) {
    free_transfers_stream(ctx, stream);

    struct kevent event[2];
    int n = 0;

    EV_SET(&event[n++], maru_fifo_read_notify_fd(stream->fifo), EVFILT_READ, EV_DELETE, 0, 0, 0);

    kevent(ctx->kqfd, event, n, NULL, 0, NULL);

    write(stream->sync_pipe[1], "1", 1);
  }
}

static void free_transfers_stream(maru_context *ctx,
                                  struct maru_stream_internal *stream) {
  struct transfer_list *list = &stream->trans;

  for (unsigned trans = 0; trans < list->size; trans++) {
    struct maru_transfer *transfer = list->transfers[trans];

    // We have to cancel the stream, and wait for it to complete.
    // Cancellation is async as well.
    if (transfer->active) {
      transfer->block = true;
      libusb_cancel_transfer(transfer->trans);
      while (transfer->active)
        libusb_handle_events(ctx->ctx);
    }

    libusb_free_transfer(transfer->trans);
    free(transfer);
  }

  free(list->transfers);
  memset(list, 0, sizeof(*list));
}

static void free_transfers(maru_context *ctx) {
  for (unsigned str = 0; str < ctx->num_streams; str++)
    free_transfers_stream(ctx, &ctx->streams[str]);
}

static void kill_write_notifications(maru_context *ctx) {
  ctx_lock(ctx);

  for (unsigned i = 0; i < ctx->num_streams; i++) {
    if (ctx->streams[i].fifo)
      maru_fifo_kill_notification(ctx->streams[i].fifo);
  }

  // We are now officially dead, and no more streams or writes can be performed.
  ctx->thread_dead = true;
  ctx_unlock(ctx);
}

static void *thread_entry(void *data) {
  maru_context *ctx = data;

  bool alive = true;

#define MAX_EVENTS 16
  while (alive) {
    struct kevent events[MAX_EVENTS];
    int num_events;

    if ((num_events = kevent(ctx->kqfd, NULL, 0, events, MAX_EVENTS, NULL)) < 0) {
      if (errno == EINTR)
        continue;

      perror("kevent");
      break;
    }

    bool libusb_event = false;

    for (size_t i = 0; i < num_events; i++) {
      int fd = events[i].ident;
      struct maru_stream_internal *stream = NULL;

      if ((stream = fd_to_stream(ctx, fd)))
        handle_stream(ctx, stream);
      else if (fd == ctx->quit_pipe[0])
        alive = false;
      else
        libusb_event = true;
    }

    if (libusb_event) {
      if (libusb_handle_events_timeout(ctx->ctx, &(struct timeval) {0}) < 0) {
        fprintf(stderr, "libusb_handle_events_timeout() failed!\n");
        alive = false;
      }
    }
  }

  free_transfers(ctx);
  kill_write_notifications(ctx);
  return NULL;
}

static bool add_stream(maru_context *ctx,
                       unsigned interface, unsigned altsetting,
                       unsigned stream_ep, unsigned feedback_ep) {
  struct maru_stream_internal *new_streams = realloc(ctx->streams, (ctx->num_streams + 1) * sizeof(*ctx->streams));
  if (!new_streams)
    return false;

  ctx->streams = new_streams;

  ctx->streams[ctx->num_streams] = (struct maru_stream_internal) {
          .stream_ep = stream_ep,
          .feedback_ep = feedback_ep,
          .stream_interface = interface,
          .stream_altsetting = altsetting,
          .sync_pipe = {-1, -1},
  };

  ctx->num_streams++;
  return true;
}

static bool init_stream_nolock(maru_context *ctx,
                               maru_stream stream,
                               const struct maru_stream_desc *desc) {
  struct maru_stream_internal *str = &ctx->streams[stream];

  const struct libusb_interface_descriptor *iface =
          &ctx->conf->interface[str->stream_interface].altsetting[str->stream_altsetting];

  pipe(str->sync_pipe);
  if (str->sync_pipe[0] < 0)
    return false;

  if (str->feedback_ep && !enqueue_feedback_transfer(ctx, str))
    return false;

  size_t buffer_size = desc->buffer_size;
  if (!buffer_size)
    buffer_size = 1024 * 32;

  // Set fragment size.
  size_t frag_size = desc->fragment_size;
  if (!frag_size)
    frag_size = buffer_size >> 2;

  size_t frame_size = desc->sample_rate * desc->channels * desc->bits / 8;
  frame_size /= 1000;

  str->enqueue_count = frag_size / frame_size + 1;
  if (str->enqueue_count > LIBMARU_MAX_ENQUEUE_COUNT)
    str->enqueue_count = LIBMARU_MAX_ENQUEUE_COUNT;

  str->fifo = maru_fifo_new(buffer_size);
  if (!str->fifo)
    return false;

  if (maru_fifo_set_read_trigger(str->fifo,
                                 frag_size) < 0) {
    maru_fifo_free(str->fifo);
    str->fifo = NULL;
    return false;
  }

  if (maru_fifo_set_write_trigger(str->fifo,
                                  frag_size) < 0) {
    maru_fifo_free(str->fifo);
    str->fifo = NULL;
    return false;
  }

  poll_list_add(ctx->kqfd,
                maru_fifo_read_notify_fd(str->fifo), POLLIN);

  str->transfer_speed_mult = desc->channels * desc->bits / 8;

  str->transfer_speed_fraction = desc->sample_rate;
  str->transfer_speed_fraction <<= 16;

  str->transfer_speed_fraction /= 1000;

  str->transfer_speed = str->transfer_speed_fraction;
  str->trans_count = 0;

  str->timer.started = false;

  return true;
}

static void deinit_stream_nolock(maru_context *ctx, maru_stream stream) {
  struct maru_stream_internal *str = &ctx->streams[stream];

  if (str->sync_pipe[0] >= 0) {
    close(str->sync_pipe[0]);
    close(str->sync_pipe[1]);
    str->sync_pipe[0] = -1;
    str->sync_pipe[1] = -1;
  }

  if (str->fifo) {
    maru_fifo_free(str->fifo);
    str->fifo = NULL;
  }
}

static void deinit_stream(maru_context *ctx, maru_stream stream) {
  ctx_lock(ctx);
  deinit_stream_nolock(ctx, stream);
  ctx_unlock(ctx);
}

static bool find_interface_endpoints(maru_context *ctx,
                                     unsigned interface,
                                     unsigned altsetting,
                                     const struct libusb_interface_descriptor *iface) {
  for (unsigned i = 0; i < iface->bNumEndpoints; i++) {
    const struct libusb_endpoint_descriptor *endp =
            &iface->endpoint[i];

    if (endp->bEndpointAddress < 0x80 &&
        endp->bmAttributes & USB_ENDPOINT_ISOCHRONOUS) {
      if (endp->bmAttributes & USB_ENDPOINT_ADAPTIVE)
        SDL_Log("Adaptive");

      return add_stream(ctx,
                        interface, altsetting,
                        endp->bEndpointAddress,
                        endp->bSynchAddress);
    }
  }

  return true;
}

static bool enumerate_stream_interfaces(maru_context *ctx,
                                        const struct maru_stream_desc *desc) {
  for (unsigned i = 0; i < ctx->conf->bNumInterfaces; i++) {
    unsigned altsettings =
            ctx->conf->interface[i].num_altsetting;

    for (unsigned j = 0; j < altsettings; j++) {
      const struct libusb_interface_descriptor *iface =
              &ctx->conf->interface[i].altsetting[j];

      if (iface->bInterfaceClass == USB_CLASS_AUDIO &&
          iface->bInterfaceSubClass == USB_SUBCLASS_AUDIO_STREAMING &&
          iface->bNumEndpoints >= 1 &&
          format_matches(iface, desc)) {
        if (!find_interface_endpoints(ctx, i, j, iface))
          return false;

        break;
      }
    }
  }

  return true;
}

static bool enumerate_streams(maru_context *ctx,
                              const struct maru_stream_desc *desc) {
  struct libusb_config_descriptor *conf = ctx->conf;

  if (!enumerate_stream_interfaces(ctx, desc))
    return false;

  for (unsigned i = 0; i < ctx->num_streams; i++) {
    unsigned iface = ctx->streams[i].stream_interface;
    unsigned altsetting = ctx->streams[i].stream_altsetting;

    if (libusb_kernel_driver_active(ctx->handle, iface) && libusb_detach_kernel_driver(ctx->handle, iface) < 0)
      return false;

    if (libusb_claim_interface(ctx->handle, iface) < 0)
      return false;

    if (libusb_set_interface_alt_setting(ctx->handle, iface, altsetting) < 0)
      return false;
  }

  return true;
}

maru_error maru_create_context_from_vid_pid(maru_context **ctx,
                                            uint16_t vid, uint16_t pid,
                                            const struct maru_stream_desc *desc) {
  maru_context *context = calloc(1, sizeof(*context));
  if (!context)
    return LIBMARU_ERROR_MEMORY;

  pipe(context->quit_pipe);
  context->kqfd = kqueue();

  if (context->quit_pipe[0] < 0 ||
      context->kqfd < 0)
    goto error;

  if (libusb_init(&context->ctx) < 0)
    goto error;

  context->handle = libusb_open_device_with_vid_pid(context->ctx, vid, pid);
  if (!context->handle)
    goto error;

  if (libusb_get_active_config_descriptor(libusb_get_device(context->handle), &context->conf) < 0) {
    context->conf = NULL;
    goto error;
  }

  if (!enumerate_streams(context, desc))
    goto error;

  if (!poll_list_init(context))
    goto error;

  if (pthread_mutex_init(&context->lock, NULL) < 0)
    goto error;

  if (pthread_create(&context->thread, NULL, thread_entry, context) < 0) {
    context->thread = 0;
    goto error;
  }

  *ctx = context;
  return LIBMARU_SUCCESS;

  error:
  maru_destroy_context(context);
  return LIBMARU_ERROR_GENERIC;
}

void maru_destroy_context(maru_context *ctx) {
  if (!ctx)
    return;

  if (ctx->quit_pipe[1] >= 0) {
    write(ctx->quit_pipe[1], "1", 1);
    if (ctx->thread)
      pthread_join(ctx->thread, NULL);
    close(ctx->quit_pipe[0]);
    close(ctx->quit_pipe[1]);
  }

  poll_list_deinit(ctx);

  for (unsigned i = 0; i < ctx->num_streams; i++)
    deinit_stream(ctx, i);
  free(ctx->streams);

  pthread_mutex_destroy(&ctx->lock);

  if (ctx->conf)
    libusb_free_config_descriptor(ctx->conf);

  if (ctx->handle) {
    for (unsigned i = 0; i < ctx->num_streams; i++) {
      libusb_release_interface(ctx->handle, ctx->streams[i].stream_interface);
      libusb_attach_kernel_driver(ctx->handle, ctx->streams[i].stream_interface);
    }

    libusb_close(ctx->handle);
  }

  if (ctx->ctx)
    libusb_exit(ctx->ctx);

  free(ctx);
}

int maru_get_num_streams(maru_context *ctx) {
  return ctx->num_streams;
}

static bool parse_audio_format(const uint8_t *data, size_t length,
                               struct maru_stream_desc *desc) {
  const struct usb_uas_format_descriptor *header = NULL;
  for (size_t i = 0; i < length; i += header->bLength) {
    header = (const struct usb_uas_format_descriptor *) &data[i];

    if (header->bLength < sizeof(*header))
      continue;

    if (header->bDescriptorType != (USB_CLASS_DESCRIPTOR | USB_INTERFACE_DESCRIPTOR_TYPE) ||
        header->bDescriptorSubtype != USB_FORMAT_DESCRIPTOR_SUBTYPE ||
        header->bFormatType != USB_FORMAT_TYPE_I)
      continue;

    desc->channels = header->bNrChannels;
    desc->bits = header->nBitResolution;

    if (header->bSamFreqType == 0) // Continous
    {
      desc->sample_rate = 0;

      desc->sample_rate_min =
              (header->tSamFreq[0] << 0) |
              (header->tSamFreq[1] << 8) |
              (header->tSamFreq[2] << 16);

      desc->sample_rate_max =
              (header->tSamFreq[3] << 0) |
              (header->tSamFreq[4] << 8) |
              (header->tSamFreq[5] << 16);
    } else {
      // FIXME: Parse all sample rates correctly as separate stream descriptions.
      // Use last format in list (somewhat hacky, will do for now ...)
      unsigned rate_start = 3 * (header->bSamFreqType - 1);

      desc->sample_rate =
              (header->tSamFreq[rate_start + 0] << 0) |
              (header->tSamFreq[rate_start + 1] << 8) |
              (header->tSamFreq[rate_start + 2] << 16);

      desc->sample_rate_min = desc->sample_rate_max = 0;
    }

    return true;
  }

  return false;
}

static int maru_is_stream_available_nolock(maru_context *ctx,
                                           maru_stream stream) {
  return stream < ctx->num_streams ?
         !ctx->streams[stream].fifo :
         LIBMARU_ERROR_INVALID;
}

int maru_is_stream_available(maru_context *ctx, maru_stream stream) {
  ctx_lock(ctx);
  int ret = maru_is_stream_available_nolock(ctx, stream);
  ctx_unlock(ctx);
  return ret;
}

int maru_find_available_stream(maru_context *ctx) {
  for (unsigned i = 0; i < ctx->num_streams; i++)
    if (maru_is_stream_available(ctx, i))
      return i;

  return LIBMARU_ERROR_BUSY;
}

void maru_stream_set_write_notification(maru_context *ctx,
                                        maru_stream stream,
                                        maru_notification_cb callback, void *userdata) {
  if (stream >= ctx->num_streams)
    return;

  ctx_lock(ctx);
  ctx->streams[stream].write_cb = callback;
  ctx->streams[stream].write_userdata = userdata;
  ctx_unlock(ctx);
}

void maru_stream_set_error_notification(maru_context *ctx,
                                        maru_stream stream,
                                        maru_notification_cb callback, void *userdata) {
  if (stream >= ctx->num_streams)
    return;

  ctx_lock(ctx);
  ctx->streams[stream].error_cb = callback;
  ctx->streams[stream].error_userdata = userdata;
  ctx_unlock(ctx);
}

maru_error maru_stream_open(maru_context *ctx,
                            maru_stream stream,
                            const struct maru_stream_desc *desc) {
  maru_error ret = LIBMARU_SUCCESS;
  ctx_lock(ctx);

  if (ctx->thread_dead) {
    ret = LIBMARU_ERROR_BUSY;
    goto end;
  }

  if (stream >= ctx->num_streams) {
    ret = LIBMARU_ERROR_INVALID;
    goto end;
  }

  if (maru_is_stream_available_nolock(ctx, stream) == 0) {
    ret = LIBMARU_ERROR_BUSY;
    goto end;
  }

  if (!init_stream_nolock(ctx, stream, desc)) {
    ret = LIBMARU_ERROR_GENERIC;
    goto end;
  }

  end:
  ctx_unlock(ctx);
  return ret;
}

maru_error maru_stream_close(maru_context *ctx,
                             maru_stream stream) {
  if (maru_is_stream_available(ctx, stream) != 0)
    return LIBMARU_ERROR_INVALID;

  // Unblock so we make sure epoll_wait() catches our notification kill.
  poll_list_unblock(ctx->kqfd,
                    maru_fifo_read_notify_fd(ctx->streams[stream].fifo),
                    POLLIN);

  // Wait till thread has acknowledged our close.
  maru_fifo_kill_notification(ctx->streams[stream].fifo);
  uint64_t dummy;
  read(ctx->streams[stream].sync_pipe[0], &dummy, sizeof(dummy));

  deinit_stream_nolock(ctx, stream);

  return LIBMARU_SUCCESS;
}

static maru_usec current_time(void) {
  struct timespec tv;
  clock_gettime(CLOCK_MONOTONIC, &tv);

  maru_usec time = tv.tv_sec * INT64_C(1000000);
  time += tv.tv_nsec / 1000;
  return time;
}

static void init_timer(struct maru_stream_internal *str) {
  str->timer.started = true;

  str->timer.start_time = current_time();
  str->timer.read_cnt = 0;
  str->timer.offset = 0;
}

size_t maru_stream_read(maru_context *ctx, maru_stream stream,
                        void *data, size_t size) {
  if (stream >= ctx->num_streams)
    return 0;

  struct maru_stream_internal *str = &ctx->streams[stream];

  maru_fifo *fifo = str->fifo;
  if (!fifo) {
    fprintf(stderr, "Stream has no fifo!\n");
    return 0;
  }

  if (!str->timer.started)
    init_timer(str);

  size_t ret = maru_fifo_blocking_read(fifo, data, size);
  str->timer.read_cnt += ret;
  return ret;
}

int maru_stream_notification_fd(maru_context *ctx,
                                maru_stream stream) {
  if (stream >= ctx->num_streams)
    return LIBMARU_ERROR_INVALID;

  if (!ctx->streams[stream].fifo)
    return LIBMARU_ERROR_INVALID;

  return maru_fifo_write_notify_fd(ctx->streams[stream].fifo);
}

size_t maru_stream_write_avail(maru_context *ctx, maru_stream stream) {
  if (stream >= ctx->num_streams)
    return 0;

  maru_fifo *fifo = ctx->streams[stream].fifo;
  if (!fifo)
    return 0;

  return maru_fifo_write_avail(fifo);
}

const char *maru_error_string(maru_error error) {
  switch (error) {
    case LIBMARU_SUCCESS:
      return "No error";
    case LIBMARU_ERROR_GENERIC:
      return "Generic error";
    case LIBMARU_ERROR_IO:
      return "I/O error";
    case LIBMARU_ERROR_BUSY:
      return "Device is busy";
    case LIBMARU_ERROR_ACCESS:
      return "Permissions error";
    case LIBMARU_ERROR_INVALID:
      return "Invalid argument";
    case LIBMARU_ERROR_MEMORY:
      return "Allocation error";
    case LIBMARU_ERROR_DEAD:
      return "Data structure is dead";
    case LIBMARU_ERROR_TIMEOUT:
      return "Timeout";
    case LIBMARU_ERROR_UNKNOWN:
    default:
      return "Unknown error";
  }
}
