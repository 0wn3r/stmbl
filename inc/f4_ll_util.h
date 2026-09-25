#pragma once

// Small helpers for things STM32CubeF4 LL does not cover the way the old
// StdPeriph code used them: DMA streams addressed by pointer, and flash.

#include "stm32f4xx.h"
#include "stm32f4xx_ll_gpio.h"

// DMA streams are 0x18 apart, starting at DMAx_BASE + 0x10.
static inline DMA_TypeDef *dma_of_stream(DMA_Stream_TypeDef *s) {
  return ((uint32_t)s >= DMA2_Stream0_BASE) ? DMA2 : DMA1;
}

static inline uint32_t dma_stream_idx(DMA_Stream_TypeDef *s) {
  return (((uint32_t)s & 0xFFU) - 0x10U) / 0x18U;
}

static inline uint32_t dma_stream_shift(DMA_Stream_TypeDef *s) {
  static const uint8_t shift[4] = {0, 6, 16, 22};
  return shift[dma_stream_idx(s) & 3];
}

// Flag bits relative to the stream's shift: FEIF=0x01 DMEIF=0x04 TEIF=0x08 HTIF=0x10 TCIF=0x20
#define DMA_STREAM_FLAG_TC 0x20U
#define DMA_STREAM_FLAG_ALL 0x3DU

static inline void dma_clear_flags(DMA_Stream_TypeDef *s, uint32_t flags) {
  DMA_TypeDef *d = dma_of_stream(s);
  if(dma_stream_idx(s) < 4) {
    d->LIFCR = flags << dma_stream_shift(s);
  } else {
    d->HIFCR = flags << dma_stream_shift(s);
  }
}

static inline uint32_t dma_get_flags(DMA_Stream_TypeDef *s, uint32_t flags) {
  DMA_TypeDef *d = dma_of_stream(s);
  uint32_t isr   = (dma_stream_idx(s) < 4) ? d->LISR : d->HISR;
  return (isr >> dma_stream_shift(s)) & flags;
}

static inline void dma_disable(DMA_Stream_TypeDef *s) {
  s->CR &= ~DMA_SxCR_EN;
  while(s->CR & DMA_SxCR_EN) {
  }
}

static inline void dma_enable(DMA_Stream_TypeDef *s) {
  s->CR |= DMA_SxCR_EN;
}

// Flash (replaces stm32f4xx_flash.c), 2.7-3.6V range, byte/word programming
int flash_unlock(void);
void flash_lock(void);
int flash_erase_sector(uint32_t sector);
int flash_program_byte(uint32_t addr, uint8_t data);

// CRC over 32-bit words (replaces CRC_CalcBlockCRC, does not reset)
static inline uint32_t crc_calc_block(const uint32_t *buf, uint32_t len) {
  for(uint32_t i = 0; i < len; i++) {
    CRC->DR = buf[i];
  }
  return CRC->DR;
}

// Alternate function by pin number (replaces GPIO_PinAFConfig)
static inline void gpio_set_af(GPIO_TypeDef *port, uint32_t pin_source, uint32_t af) {
  uint32_t shift = (pin_source & 7U) * 4U;
  MODIFY_REG(port->AFR[pin_source >> 3], 0xFU << shift, af << shift);
}

// Clear EN without waiting for the stream to stop (old DMA_Cmd(s, DISABLE))
static inline void dma_stop(DMA_Stream_TypeDef *s) {
  s->CR &= ~DMA_SxCR_EN;
}

// Register sequence of the old StdPeriph TIM_EncoderInterfaceConfig().
// sms: LL_TIM_ENCODERMODE_*, pol: LL_TIM_IC_POLARITY_* (same bits as TIM_ICPolarity_*)
static inline void tim_encoder_config(TIM_TypeDef *tim, uint32_t sms, uint32_t pol1, uint32_t pol2) {
  tim->SMCR  = (tim->SMCR & ~TIM_SMCR_SMS) | sms;
  tim->CCMR1 = (tim->CCMR1 & ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S)) | TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
  tim->CCER  = (tim->CCER & ~(TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC1NP | TIM_CCER_CC2NP)) | pol1 | (pol2 << 4);
}

// Register sequence of the old StdPeriph TIM_ICInit() for channel 1..4.
// sel is the raw CCxS value (1 direct, 2 indirect, 3 TRC), psc is ORed in
// unshifted like TIM_SetICxPrescaler() did, so raw values keep their old effect.
static inline void tim_ic_init(TIM_TypeDef *tim, int ch, uint32_t pol, uint32_t sel, uint32_t psc, uint32_t filter) {
  uint32_t n     = (uint32_t)(ch - 1);
  uint32_t ccer  = 4U * n;                      // CCxE at bit 4*(ch-1)
  uint32_t ccm   = (n & 1U) * 8U;               // offset inside CCMR1/CCMR2
  __IO uint32_t *ccmr = (n < 2) ? &tim->CCMR1 : &tim->CCMR2;
  tim->CCER &= ~(TIM_CCER_CC1E << ccer);
  *ccmr = (*ccmr & ~((TIM_CCMR1_CC1S | TIM_CCMR1_IC1F) << ccm)) | ((sel | (filter << 4)) << ccm);
  tim->CCER = (tim->CCER & ~((TIM_CCER_CC1P | TIM_CCER_CC1NP) << ccer)) | ((pol | TIM_CCER_CC1E) << ccer);
  *ccmr = (*ccmr & ~(TIM_CCMR1_IC1PSC << ccm)) | (psc << ccm);
}

// Pin setup like the old StdPeriph GPIO_Init(): leaves the alternate function
// register alone, so gpio_set_af() may be called before or after it.
static inline void gpio_init(GPIO_TypeDef *port, const LL_GPIO_InitTypeDef *init) {
  for(uint32_t pos = 0; pos < 16; pos++) {
    uint32_t pin = init->Pin & (1U << pos);
    if(!pin) {
      continue;
    }
    if(init->Mode == LL_GPIO_MODE_OUTPUT || init->Mode == LL_GPIO_MODE_ALTERNATE) {
      LL_GPIO_SetPinSpeed(port, pin, init->Speed);
      LL_GPIO_SetPinOutputType(port, pin, init->OutputType);
    }
    LL_GPIO_SetPinMode(port, pin, init->Mode);
    LL_GPIO_SetPinPull(port, pin, init->Pull);
  }
}
