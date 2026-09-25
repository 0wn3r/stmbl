#pragma once

// Small helpers for things STM32CubeF4 LL does not cover the way the old
// StdPeriph code used them: DMA streams addressed by pointer, and flash.

#include "stm32f4xx.h"

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
