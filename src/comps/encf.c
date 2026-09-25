#include "encf_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "stm32f4xx_conf.h"
#include "hw/hw.h"
#include <string.h>

HAL_COMP(encf);

HAL_PIN(error);
HAL_PIN(dma);  //dma transfers

HAL_PIN(pos);
HAL_PIN(abs_pos);
HAL_PIN(state);
HAL_PIN(turns);
HAL_PIN(com_pos);
HAL_PIN(index);
HAL_PIN(batt);
HAL_PIN(req_len);

HAL_PIN(pos_offset);

HAL_PIN(send_step);
HAL_PIN(crc_ok);
HAL_PIN(crc_er);

HAL_PIN(freq);
HAL_PIN(bit_ticks);

static volatile uint32_t sendf;
static uint32_t send_counterf;
static volatile uint16_t tim_data[160];

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

static int32_t pos_offset;
static uint32_t state_counter;

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct encf_ctx_t *ctx = (struct encf_ctx_t *)ctx_ptr;
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
  gpio_set_af(FB0_A_PORT, FB0_A_PIN_SOURCE, FB0_ENC_TIM_AF);

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
  dma_disable(DMA1_Stream0);
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

  gpio_set_af(GPIOC, 12, LL_GPIO_AF_6);
  GPIO_InitStruct.Pin   = LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode  = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull  = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  LL_SPI_Enable(SPI3);

  LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_15);  //tx enable

  pos_offset    = 0;
  PIN(pos_offset) = 0;
  PIN(req_len)  = 2046;
  state_counter = 0;
  PIN(freq)     = 1024000;
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
  // struct encf_ctx_t *ctx      = (struct encf_ctx_t *)ctx_ptr;
  struct encf_pin_ctx_t *pins = (struct encf_pin_ctx_t *)pin_ptr;

  uint32_t count = ARRAY_SIZE(tim_data) - DMA1_Stream0->NDTR;
  PIN(dma)       = count;

  //1 bit = 80 ticks 82e6/1.024e6
  PIN(bit_ticks)      = 82000000 / PIN(freq);
  const float per_bit = 1.0 / PIN(bit_ticks);

  // The frame is built a word at a time: a run of ones is one or two ORs
  // instead of a byte and shift per bit. Little endian, so bit j of w is bit
  // j % 8 of enc_data[j / 8], the layout fanuc_t reads.
  uint32_t w[3]      = {0, 0, 0};
  const int max_bits = sizeof(data.enc_data) * 8;
  int bits_sum       = 0;
  uint16_t prev      = tim_data[0];
  for(uint32_t i = 1; i < count; i++) {  //each capture form dma
    //time between edges, rounded to a number of bits
    uint16_t t = tim_data[i];
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
    //bit k of crc is the old crc[k]; feedback taps are bits 0, 2 and 4.
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
      int32_t pos = data.fanuc.pos_lo + (data.fanuc.pos_hi << 6);
      PIN(index)  = data.fanuc.no_index;
      PIN(batt)   = data.fanuc.bat;

      PIN(abs_pos) = wrap_bits(pos, 22);

      if(PIN(index) > 0.0) {
        pos_offset    = pos;
        PIN(pos)      = PIN(abs_pos);
        PIN(state)    = 1;
        state_counter = 1;
      } else if(state_counter == 1) {
        state_counter = 2;
        pos_offset    = pos;
        PIN(pos)      = PIN(abs_pos);
      } else {
        state_counter = 3;
        PIN(pos)      = wrap_bits(pos + pos_offset + ((uint32_t)PIN(pos_offset) << 6), 22);
        PIN(state)    = 3;
      }

      if (data.fanuc.turns > 32767) {
        PIN(turns) = (int32_t)data.fanuc.turns % 32768 - 32768;
      } else {
        PIN(turns) = data.fanuc.turns;
      }

      pos          = data.fanuc.com_pos;
      PIN(com_pos) = wrap_bits(pos, 10);
      PIN(error)   = 0;
    } else {
      PIN(crc_er)
      ++;
      PIN(state) = 1;
      PIN(error) = 1;
    }
  } else {
    PIN(error)    = 1;
    PIN(state)    = 1;
    state_counter = 0;
  }
  //reset timer
  FB0_ENC_TIM->CNT  = 0;
  FB0_ENC_TIM->CCR1 = 0;
  FB0_ENC_TIM->CR1 |= TIM_CR1_CEN;  // enable tim

  //send request, 1/(42e6/32)*11 = 8.4uS
  SPI3->DR = PIN(req_len);
  //start DMA
  DMA1_Stream0->CR &= ~DMA_SxCR_EN;
  LL_DMA_ClearFlag_TC0(DMA1);
  dma_enable(DMA1_Stream0);
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
    .ctx_size  = 0,
    .pin_count = sizeof(struct encf_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
