#include "term_comp.h"
#include "commands.h"
#include "hal.h"
#include "defines.h"
#include <stdio.h>
#include <string.h>
#include "usbd_cdc_if.h"
#include "angle.h"

#define TERM_NUM_WAVES 8
// rt fills this every send_step ticks and nrt drains it: 64 covers 12.8 ms
// of main loop at send_step 1. The f3 Makefile keeps 8, its HAL_MAX_CTX is
// 1024 bytes for all comps.
#ifndef TERM_BUF_SIZE
#define TERM_BUF_SIZE 64
#endif

/**
* ## Brief
* `term` is the serial terminal of a board. It runs the command line (every received line goes to `hal_parse`, so HAL commands and pin settings can be typed) and streams up to 8 pins as scope waves to the host (the Servoterm oscilloscope). It is loaded by default as `term0` on both the F4 and the F3. On the F4 it talks over USB. On the F3 (HV board) there is no USB: its terminal bytes travel one per packet over the link to the F4 (`ls` on the F3, `hv` on the F4), and main.c sets `term0.send_step = 0` there, so no wave data is sent. Typical use on the F4 is a config line like `term0.wave0 = pid0.pos_error` with `term0.gain0` to scale it.
*
* ## Component Explanation
*
* 1. **Sampling** (rt):
* - Every `send_step` rt ticks the 8 `wave` pins are stored into a ring buffer (64 entries on the F4, 8 on the F3 because of its small HAL memory). When the buffer is full, new samples are dropped.
*
* 2. **Sending** (nrt):
* - Each buffered sample is sent as 9 bytes: a 255 sync byte, then for each wave `CLAMP((wave + offset) * gain + 128, 1, 254)`. So a wave value of 0 is the middle of the scope, and with the default `gain` of 10 the visible range is about +-12.6.
* - Nothing is sent while `send_step` is 0 or no terminal is connected; the samples are still taken from the buffer and dropped.
*
* 3. **Receiving** (nrt):
* - `con` is 1 while a terminal is connected (always 1 on the F3).
* - Each complete received line (up to 64 characters) is passed to `hal_parse`.
*
* Defaults from nrt_init: `send_step` 50, all `gain` 10, `offset` 0.
*/

HAL_COMP(term);

HAL_PINA(wave, 8);    // *input*, Signals shown on the scope, 8 channels
HAL_PINA(offset, 8);  // *parameter*, Offset added to each wave before scaling, default 0
HAL_PINA(gain, 8);    // *parameter*, Scale of each wave on the scope, 1 unit = gain steps of the 1..254 scope range, default 10
HAL_PIN(send_step);   // *parameter*, Send a sample every send_step rt ticks, 0 = off, default 50
HAL_PIN(con);         // *output*, 1 while a terminal is connected

struct term_ctx_t {
  float wave_buf[TERM_BUF_SIZE][TERM_NUM_WAVES];
  uint32_t send_counter;
  uint32_t write_pos;
  uint32_t read_pos;
};


static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct term_ctx_t * ctx = (struct sim_ctx_t *)ctx_ptr;
  struct term_pin_ctx_t *pins = (struct term_pin_ctx_t *)pin_ptr;

  PIN(send_step) = 50;
  for(int i = 0; i < TERM_NUM_WAVES; i++) {
    PINA(gain, i) = 10;
  }
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct term_ctx_t *ctx      = (struct term_ctx_t *)ctx_ptr;
  struct term_pin_ctx_t *pins = (struct term_pin_ctx_t *)pin_ptr;

  if(ctx->send_counter++ >= PIN(send_step) - 1) {
    ctx->send_counter = 0;
    // full: drop this sample instead of lapping the reader
    uint32_t next = (ctx->write_pos + 1) % TERM_BUF_SIZE;
    if(next != ctx->read_pos) {
      for(int i = 0; i < TERM_NUM_WAVES; i++) {
        ctx->wave_buf[ctx->write_pos][i] = PINA(wave, i);
      }
      ctx->write_pos = next;
    }
  }
}

static void nrt_func(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct term_ctx_t *ctx      = (struct term_ctx_t *)ctx_ptr;
  struct term_pin_ctx_t *pins = (struct term_pin_ctx_t *)pin_ptr;

  int tmp = 0;
  unsigned char buf[TERM_NUM_WAVES + 2];

  buf[0]                  = 255;
  buf[TERM_NUM_WAVES + 1] = 0;

  unsigned int wp = ctx->write_pos;
  unsigned int bc = 0;

  while(ctx->read_pos != wp) {
    bc++;

    for(int i = 0; i < TERM_NUM_WAVES; i++) {
      tmp        = (int)((ctx->wave_buf[ctx->read_pos][i] + PINA(offset, i)) * PINA(gain, i) + 128);
      buf[i + 1] = CLAMP(tmp, 1, 254);
    }

    ctx->read_pos++;
    ctx->read_pos %= TERM_BUF_SIZE;

    buf[TERM_NUM_WAVES + 1] = 0;

    if(cdc_is_connected() && PIN(send_step) > 0) {
      cdc_tx(buf, TERM_NUM_WAVES + 1);
    }
  }

  if(cdc_is_connected()) {
    PIN(con) = 1.0;
  } else {
    PIN(con) = 0.0;
  }

  if(cdc_is_connected()) {
    char rx_buf[64];
    if(cdc_getline(rx_buf, sizeof(rx_buf))) {
      hal_parse(rx_buf);
    }
  }
}

hal_comp_t term_comp_struct = {
    .name      = "term",
    .nrt       = nrt_func,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct term_ctx_t),
    .pin_count = sizeof(struct term_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
