#include "encf_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "stm32f4xx_conf.h"
#include "dma_util.h"
#include "hw/hw.h"
#include <string.h>

/**
* ## Brief
* `encf` reads Fanuc serial absolute encoders (tested with the Aa64 type, A860-360) on the FB0 connector of the F4 board. It is loaded by `conf/template/fanuc_fb0.txt` (used e.g. by `conf/fanuc_a6-2000.txt`), which links `encf0.pos`/`abs_pos`/`state` to `fb_switch0.mot_*` and `encf0.com_pos` to `fb_switch0.com_pos`/`com_abs_pos`.
*
* ## Component Explanation
*
* 1. **Hardware (nrt_init)**:
* - The request pulse is generated with SPI3 (MOSI on PC12, prescaler 32, 16 bit word): every rt cycle the word `req_len` is written to SPI3 (see 2), which gives a request pulse of about 8.4 us. The FB0 line driver enable PD15 is switched on.
*
* 2. **Request timing (rt)**:
* - The encoder latches its position on the request, so the request goes out at a fixed point: first in rt, the capture buffer is copied out, TIM4 and the DMA are restarted, then the request is armed. The copy is decoded after that. Sent at the end of rt, after the decode, the latch time wandered with the frame's bit pattern (Y, 15 Oct: about 1 us rms, 0.24 rad/s of `vel1` noise and 0.8 A of iq noise at 126 rad/s).
* - With `req_sync` = 1 the rt only arms the word; the frt timer interrupt (TIM5, priority 0, `src/main.c`) writes it to SPI3 first thing on its next update. TIM5 and the ADC that paces the rt both count TIM3, so this is always the same update, a fixed time after the timer edge. `req_phase` shows the TIM5 count at arming (33 of 59 on Y, about 22 us before the update). On Y this lowered `vel1` noise above 200 Hz from 0.14 to 0.105 rad/s and iq noise from 0.45 to 0.37 A; `pos_advance` stayed the same. `req_sync` = 0 writes SPI3 from the rt code.
* - The encoder answer is received on FB0 A (PD12): TIM4 channel 1 captures every edge (both polarities) and DMA1 stream 0 (channel 2) stores up to 160 capture times.
*
* 3. **Bit decoding (rt)**:
* - TIM4 counts at 84 MHz, so the default `freq` of 1.024 MHz gives 82.03 ticks per bit. A bench sweep decoded CRC-clean frames for `freq` from about 986k to 1060k, so the default sits in the middle.
* - The time between two edges is multiplied by `1 / bit_ticks` and rounded to get the number of equal bits; every even-numbered interval is written as 1s. The frame is built in three 32 bit words and the write is bounded to the 80 bit frame buffer, so a noisy frame with too many bits is cut off instead of overrunning memory. Bits after the last edge up to bit 76 are set to 1 (idle line).
* - If fewer than 51 bits were decoded: `error = 1`, `state = 1` and the position pins keep their last value.
*
* 4. **Frame content and CRC (rt)**:
* - A 5 bit CRC (ITU CRC-5, from the Mesa `fabsread` notes) over bits 76..1, MSB first and computed four bits at a time with a nibble table, must be 0. On success `crc_ok` is incremented, otherwise `crc_er` is incremented and `error = 1`, `state = 1` (the position pins keep their last value).
* - Fields: battery fail bit -> `batt`, un-indexed bit -> `index` (1 = not yet indexed), 22 bit single-turn position (6 low bits + 16 high bits), 16 bit turn counter (used for `turns`, see 5), 10 bit commutation track -> `com_pos`.
* - `abs_pos = pos22 * 2 * pi / 2^22` and `com_pos = com * 2 * pi / 1024` (rad), wrapped to [-pi, pi) by sign-extending the integer count (same result as `mod()` without the `fmodf`). According to the Mesa notes the commutation track has four 0..1023 cycles per turn and is always absolute.
*
* 5. **Position, turns and state (rt, on every valid frame)**:
* - As in the Fanuc servo DSP, the encoder's 16 bit turn count and 22 bit count are read once, `acc = turns * 2^22 + count`: at start and after more than 10 frames without a valid one. On every other valid frame only the change of the count, wrapped to +-half a turn, is added, so `pos` and `turns` are continuous and the encoder's own turn step (a few counts off its count wrap) never matters.
* - `pos` = `acc` wrapped to [-pi, pi), `turns` steps exactly where `pos` wraps and is wrapped to 16 bits like the encoder's count, so after a reset the same position is read back. An encoder that is indexed from power up gives `pos = abs_pos`.
* - A change of more than 1/32 turn per frame (about 9000 rpm at 5 kHz) is not motion: the encoder re-references its count and turns at the first index after a battery loss (it clears the un-indexed bit one frame before the count jumps). That frame counts as moving like the one before, so the position loop never sees the jump, and `pos`/`turns` keep the old reference until the next reset. The turn count is lost with the battery anyway: re-zero in LinuxCNC. `abs_pos` and `com_pos` are always the encoder's own values, for commutation.
* - `state = 1` while the encoder reports un-indexed and for 3 frames after the un-indexed bit changes, else 3. An encoder that is indexed at power up reads 3 from the first frame; after the first index crossing `state` reads 3 only after `abs_pos` has jumped, so `fb_switch` does not switch commutation to `abs_pos` from the old count.
* - Until then `fb_switch` commutates from the commutation track (`com_pos` + `conf0.com_fb_offset`, measured by `id_pmsm`), see `fanuc_fb0.txt`. For a multiturn position link `linrev0.abs_rev = encf0.turns`, `linrev0.abs_pos = encf0.pos`, `linrev0.abs_state = encf0.state`.
*
* 6. **Debug output (nrt)**:
* - The raw 76 bit frame is printed as a string of 0/1.
*
* {{% hint warning %}}
* The capture buffers are file-static and the TX enable (PD15), SPI3 and TIM4 pins are hard coded for the V4 board. The frame layout was only verified for Aa64 encoders; for Aa1000 (A860-370) the extra low resolution bits are not used.
* {{% /hint %}}
*/

HAL_COMP(encf);

HAL_PIN(error);       // *output*, 1 = no valid frame (too short or CRC error)
HAL_PIN(dma);         // *output*, Number of captured edges in the last frame

HAL_PIN(pos);         // *output*, Continuous position (rad, +-pi), wraps where turns steps
HAL_PIN(abs_pos);     // *output*, Raw single-turn position from the encoder (rad, +-pi)
HAL_PIN(state);       // *output*, 1 = not indexed, just indexed or error, 3 = indexed and valid
HAL_PIN(turns);       // *output*, Multiturn counter, steps where pos wraps, 16 bit like the encoder (-32768..32767)
HAL_PIN(com_pos);     // *output*, Commutation track position (rad, +-pi per 1024 counts)
HAL_PIN(index);       // *output*, Un-indexed bit, 1 = encoder not yet indexed
HAL_PIN(batt);        // *output*, Battery fail bit
HAL_PIN(req_len);     // *parameter*, 16 bit SPI word used as request pulse (default 2046)
HAL_PIN(req_sync);    // *parameter*, 1 = the request goes out at the start of the next frt timer interrupt (TIM5, default), 0 = from the rt code
HAL_PIN(req_phase);   // *output*, debug, TIM5 count when the rt armed the request (0..ARR, one frt period)

HAL_PIN(send_step);   // *parameter*, Print the raw frame every send_step nrt calls (off below 50)
HAL_PIN(crc_ok);      // *output*, Counter of frames with correct CRC
HAL_PIN(crc_er);      // *output*, Counter of frames with CRC error

HAL_PIN(freq);        // *parameter*, Encoder bit rate (Hz, default 1024000)
HAL_PIN(bit_ticks);   // *output*, Timer ticks per bit (84e6 / freq)

static volatile uint32_t sendf;
static uint32_t send_counterf;
static volatile uint16_t tim_data[160];
static uint16_t cap[160];  // rt copy of tim_data, decoded after the next request is out
extern volatile uint16_t fb0_req_word;  // main.c, sent by the frt timer interrupt (req_sync)

#pragma pack(push, 1)
typedef struct {
  uint32_t flag0 : 6;  // 0101
  uint32_t bat : 1;
  uint32_t flag1 : 2;  // 101
  uint32_t no_index : 1;
  uint32_t flag2 : 1;  // 0
  uint32_t pos_lo : 6;
  uint32_t flag3 : 2;  // 10
  uint32_t pos_hi : 16;
  uint32_t flag4 : 2;  // 10
  uint32_t turns : 16;
  uint32_t flag5 : 2;  // 10
  uint32_t com_pos : 10;
  uint32_t flag6 : 7;  // 0000011
  uint32_t crc : 5;
  uint32_t flag7 : 3;  // 000
} fanuc_t;
#pragma pack(pop)
/*
http://freeby.mesanet.com/regmap

Fanuc data format (76 bits): So far just for Aa64 (860-360-TXXX)
bits 0..4 	constant : = 0b00101
bit  5     	1=battery fail
bits 6,7	unknown = 0b10,a860-360 0b00,a860-370 
bit  8		1=un-indexed
bits 9..17	unknown, perhaps for higher res encoders
bits 18..33	16 bit absolute encoder data (0..65535 for one turn)
bits 34..35     unknown = 0b01
bits 36..51	16 bit absolute turns count
bits 52,53	unknown = 0b01
bits 54..63	10 bit absolute commutation encoder (four 0.1023 cycles per turn)
bits 64..70	unknown = 0b1100000
bits 71..75	ITU CRC-5 (calculated MSB first)

Note: Aa1000 (860-370-TXXX) is similar with bits 10..15 being 
additional lower order count bits (only 12..15 being of much use)
extending the 16 bit count from bits 18..33 with bits 12..15  
gives a resolution of 1048576 counts/turn

These encoders are absolute if the battery backup is maintained since the 
last homing. This can be determined by the status of the un-indexed bit, 
at least until the encoder crosses index. 

Surprisingly (well it surprised me anyway) the encoders maintain position 
and count with battery power alone. It appears that they keep the LEDS/
processor alive with a few 10s of uA of battery power (probably low duty 
cycle LED pulsing/analog circuits power cycling). 

This can be verified by reading the battery current and then moving the 
encoder, it goes from a few 10s of uA to many mA with a slow encoder move.

The commutation track is always absolute so can be used for commutation 
data regardless of the index status. Note that the commutation track
seems to be interpolated so is not better than maybe 1% accuracy
*/

static union {
  uint8_t enc_data[10];
  fanuc_t fanuc;
} data;
static uint8_t print_buf[10];

struct encf_ctx_t {
  int64_t acc;      // position as one count (turns * 2^22 + count), continuous
  uint32_t last;    // 22 bit count of the last good frame
  int32_t step;     // its motion per frame
  uint32_t age;     // frames since the last good one
  uint32_t settle;  // frames of state 1 left after the un-indexed bit changed
  uint32_t last_no_index;
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct encf_ctx_t *ctx = (struct encf_ctx_t *)ctx_ptr;
  struct encf_pin_ctx_t *pins = (struct encf_pin_ctx_t *)pin_ptr;
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  LL_GPIO_StructInit(&GPIO_InitStruct);

  //TX enable
  GPIO_InitStruct.Pin   = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode  = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull  = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  //TIM CH1 A
  GPIO_InitStruct.Pin   = FB0_A_PIN;
  GPIO_InitStruct.Mode  = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull  = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = FB0_ENC_TIM_AF;
  LL_GPIO_Init(FB0_A_PORT, &GPIO_InitStruct);

  //TIM rx dma
  //fb0 rx auf 12: tim4_ch1 dma1_0_2
  //fb1 rx auf 12: tim1_ch1 dma2_3_6
  LL_DMA_InitTypeDef DMA_InitStruct;
  LL_DMA_StructInit(&DMA_InitStruct);
  DMA_InitStruct.Channel                = LL_DMA_CHANNEL_2;
  DMA_InitStruct.PeriphOrM2MSrcAddress  = (uint32_t)&FB0_ENC_TIM->CCR1;
  DMA_InitStruct.MemoryOrM2MDstAddress  = (uint32_t)&tim_data;
  DMA_InitStruct.Direction              = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  DMA_InitStruct.NbData                 = ARRAY_SIZE(tim_data);
  DMA_InitStruct.PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStruct.MemoryOrM2MDstIncMode  = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStruct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD;
  DMA_InitStruct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD;
  DMA_InitStruct.Mode                   = LL_DMA_MODE_NORMAL;
  DMA_InitStruct.Priority               = LL_DMA_PRIORITY_VERYHIGH;
  DMA_InitStruct.FIFOMode               = LL_DMA_FIFOMODE_DISABLE;
  DMA_InitStruct.FIFOThreshold          = LL_DMA_FIFOTHRESHOLD_1_2;
  DMA_InitStruct.MemBurst               = LL_DMA_MBURST_SINGLE;
  DMA_InitStruct.PeriphBurst            = LL_DMA_PBURST_SINGLE;
  dma_stream_stop(DMA1_Stream0);
  LL_DMA_DeInit(DMA1, LL_DMA_STREAM_0);
  LL_DMA_Init(DMA1, LL_DMA_STREAM_0, &DMA_InitStruct);

  //timer setup
  LL_APB1_GRP1_EnableClock(FB0_ENC_TIM_RCC);
  FB0_ENC_TIM->CR1 &= ~TIM_CR1_CEN;
  FB0_ENC_TIM->CCMR1 = TIM_CCMR1_CC1S_0;                                // cc1 input ti1
  FB0_ENC_TIM->CCER  = TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP;  // cc1 en, rising edge, falling edge
  FB0_ENC_TIM->ARR   = 65535;
  FB0_ENC_TIM->DIER  = TIM_DIER_CC1DE;  // enable cc1 dma reeuest

  //SPI is used to generate request
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
  LL_SPI_InitTypeDef SPI_InitTypeDefStruct;
  LL_SPI_StructInit(&SPI_InitTypeDefStruct);
  SPI_InitTypeDefStruct.BaudRate          = LL_SPI_BAUDRATEPRESCALER_DIV32;
  SPI_InitTypeDefStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitTypeDefStruct.Mode              = LL_SPI_MODE_MASTER;
  SPI_InitTypeDefStruct.DataWidth         = LL_SPI_DATAWIDTH_16BIT;
  SPI_InitTypeDefStruct.NSS               = LL_SPI_NSS_SOFT;
  SPI_InitTypeDefStruct.BitOrder          = LL_SPI_MSB_FIRST;
  SPI_InitTypeDefStruct.ClockPolarity     = LL_SPI_POLARITY_HIGH;
  SPI_InitTypeDefStruct.ClockPhase        = LL_SPI_PHASE_2EDGE;
  LL_SPI_Init(SPI3, &SPI_InitTypeDefStruct);

  GPIO_InitStruct.Pin   = LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode  = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull  = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  LL_SPI_Enable(SPI3);

  LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_15);  //tx enable

  PIN(req_len)       = 2046;
  PIN(req_sync)      = 1.0;
  PIN(freq)          = 1024000;
  ctx->acc           = 0;
  ctx->last          = 0;
  ctx->step          = 0;
  ctx->age           = 1000;
  ctx->settle        = 0;
  ctx->last_no_index = 0;
}

// crc5 of four bits (MSB first) from a zero register
static const uint8_t crc5_nib[16] = {0x00, 0x15, 0x1f, 0x0a, 0x0b, 0x1e, 0x14, 0x01, 0x16, 0x03, 0x09, 0x1c, 0x1d, 0x08, 0x02, 0x17};

// feed 32 bits into the crc5, MSB first
static inline uint32_t crc5_word(uint32_t crc, uint32_t x) {
  for(int j = 28; j >= 0; j -= 4) {
    crc = ((crc << 4) & 0x1f) ^ crc5_nib[((crc >> 1) ^ (x >> j)) & 0xf];
  }
  return crc;
}

// angle of a count on an n bit circle, in [-pi, pi). Wrapping the integer
// replaces mod()'s fmodf and is exact; same result as mod() to within one
// float rounding.
static inline float wrap_bits(uint32_t count, int n) {
  int32_t c = (int32_t)(count << (32 - n)) >> (32 - n);
  return (float)c * (2.0 * M_PI / (float)(1 << n));
}

// set bits [a, b) of the frame words w
static inline void set_bits(uint32_t *w, int a, int b) {
  while(a < b) {
    int n      = MIN(b - a, 32 - (a & 31));
    uint32_t m = (n == 32) ? 0xffffffff : (((1u << n) - 1) << (a & 31));
    w[a >> 5] |= m;
    a += n;
  }
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct encf_ctx_t *ctx      = (struct encf_ctx_t *)ctx_ptr;
  struct encf_pin_ctx_t *pins = (struct encf_pin_ctx_t *)pin_ptr;

  uint32_t count = ARRAY_SIZE(tim_data) - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_0);
  PIN(dma)       = count;

  // Request the next frame first, at a fixed point after the rt starts: the
  // encoder latches its position on the request, and sending it after the
  // decode below let the latch time wander with the frame's bit pattern
  // (Y, 15 Oct: ~1 us rms, 0.24 rad/s of vel1 noise and 0.8 A of iq noise
  // at 126 rad/s). The capture buffer is copied out before the DMA restarts.
  memcpy(cap, (void *)tim_data, MIN(count, ARRAY_SIZE(cap)) * sizeof(cap[0]));
  FB0_ENC_TIM->CNT  = 0;
  FB0_ENC_TIM->CCR1 = 0;
  FB0_ENC_TIM->CR1 |= TIM_CR1_CEN;
  dma_stream_stop(DMA1_Stream0);
  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);
  PIN(req_phase) = TIM_SLAVE->CNT;
  if(PIN(req_sync) > 0.0) {
    // sent by the frt timer interrupt (main.c) at the next TIM5 update: the
    // rt starts at a fixed TIM5 count (both count TIM3), so it is always the
    // same update, a fixed time after the rt start (req_phase shows where)
    fb0_req_word = PIN(req_len);
  } else {
    SPI3->DR = PIN(req_len);  // 1/(42e6/32)*11 = 8.4 us request pulse
  }

  // TIM4 counts at 84 MHz (APB1 42 MHz x2): 1 bit = 82.03 ticks at 1.024 Mbit/s.
  // 82 MHz here would put the default freq 1.2% from the CRC-clean edge
  // (bench sweep on X: clean for freq 962k-1036k, centre ~999k).
  PIN(bit_ticks)      = 84000000 / PIN(freq);
  const float per_bit = 1.0 / PIN(bit_ticks);

  // The frame is built a word at a time: a run of ones is one or two ORs
  // instead of a byte and shift per bit. Little endian, so bit j of w is bit
  // j % 8 of enc_data[j / 8], the layout fanuc_t reads.
  uint32_t w[3]      = {0, 0, 0};
  const int max_bits = sizeof(data.enc_data) * 8;
  int bits_sum       = 0;
  uint16_t prev      = cap[0];
  for(uint32_t i = 1; i < MIN(count, ARRAY_SIZE(cap)); i++) {  //each capture form dma
    //time between edges, rounded to a number of bits
    uint16_t t = cap[i];
    int bits   = (float)(uint16_t)(t - prev) * per_bit + 0.5;
    prev       = t;
    int end    = MIN(bits_sum + bits, max_bits);
    if((i & 1) == 0) {  //line starts high, set every even numbered captures to 1
      set_bits(w, bits_sum, end);
    }
    bits_sum = end;
  }
  //set remaining bits to 1
  set_bits(w, bits_sum, 77);
  memcpy((void *)data.enc_data, w, sizeof(data.enc_data));

  if(!sendf) {
    memcpy((void *)print_buf, (void *)data.enc_data, 10);
    sendf = 1;
  }
  if(bits_sum > 50) {
    //check crc, MSB first: http://freeby.mesanet.com/fabsread.pas
    //bit k of crc is shift register stage k; feedback taps are bits 0, 2 and 4.
    //Four bits per step: bits 76..65, then 64..33 and 32..1 as words.
    uint32_t crc = 0;
    for(int j = 9; j >= 1; j -= 4) {
      crc = ((crc << 4) & 0x1f) ^ crc5_nib[((crc >> 1) ^ (w[2] >> j)) & 0xf];
    }
    crc = crc5_word(crc, (w[2] << 31) | (w[1] >> 1));
    crc = crc5_word(crc, (w[1] << 31) | (w[0] >> 1));
    if(crc == 0) {
      PIN(crc_ok)
      ++;
      uint32_t pos = data.fanuc.pos_lo + (data.fanuc.pos_hi << 6);
      PIN(index)   = data.fanuc.no_index;
      PIN(batt)    = data.fanuc.bat;

      PIN(abs_pos) = wrap_bits(pos, 22);

      // Like the Fanuc servo DSP: the encoder's turns and count are read
      // once, at start and after more than 10 frames without a good one;
      // after that only the wrapped 22 bit change of the count is added, so
      // pos and turns are continuous. A change of more than 1/32 turn per
      // frame (about 9000 rpm at 5 kHz) is not motion but the encoder
      // re-referencing at its first index after a battery loss: the frame
      // counts as moving like the one before, so the position loop never
      // sees the jump. The new reference shows after a reset.
      uint32_t gap = ctx->age + 1;
      if(ctx->age > 10) {
        ctx->acc  = (int64_t)(int16_t)data.fanuc.turns * (1 << 22) + pos;
        ctx->step = 0;
      } else {
        int32_t d = (int32_t)((pos - ctx->last) << 10) >> 10;  // -2^21..2^21
        if(d > (1 << 17) * (int32_t)gap || d < -(1 << 17) * (int32_t)gap) {
          d = ctx->step * (int32_t)gap;
        } else {
          ctx->step = d / (int32_t)gap;
        }
        ctx->acc += d;
      }
      ctx->last = pos;
      ctx->age  = 0;

      // pos wraps at +-pi, turns steps there; turns wraps at 16 bits like
      // the encoder's own count, so a reset reads the same position back
      PIN(pos)   = wrap_bits((uint32_t)ctx->acc, 22);
      PIN(turns) = (int16_t)(uint16_t)((ctx->acc + (1 << 21)) >> 22);

      // abs_pos jumps a frame after the un-indexed bit clears, and fb_switch
      // commutates from abs_pos once this reads 3: hold 1 for 3 frames
      if(data.fanuc.no_index != ctx->last_no_index) {
        ctx->settle = 3;
      }
      ctx->last_no_index = data.fanuc.no_index;
      PIN(state)         = (data.fanuc.no_index || ctx->settle) ? 1 : 3;
      if(ctx->settle) {
        ctx->settle--;
      }
      pos          = data.fanuc.com_pos;
      PIN(com_pos) = wrap_bits(pos, 10);
      PIN(error)   = 0;
    } else {
      ctx->age += ctx->age < 1000;
      PIN(crc_er)
      ++;
      PIN(state) = 1;
      PIN(error) = 1;
    }
  } else {
    ctx->age += ctx->age < 1000;
    PIN(error) = 1;
    PIN(state) = 1;
  }
}

static void nrt_func(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct encf_ctx_t *ctx = (struct encf_ctx_t *)ctx_ptr;
  struct encf_pin_ctx_t *pins = (struct encf_pin_ctx_t *)pin_ptr;

  if(sendf == 1 && send_counterf++ >= PIN(send_step) - 1 && PIN(send_step) >= 50) {
    send_counterf = 0;
    for(int i = 1; i < 77; i++) {
      if(print_buf[i / 8] & (1 << i % 8)) {
        printf("1");
      } else {
        printf("0");
      }
    }
    printf("\n");
    sendf = 0;
  }
}

hal_comp_t encf_comp_struct = {
    .name      = "encf",
    .nrt       = nrt_func,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct encf_ctx_t),
    .pin_count = sizeof(struct encf_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
