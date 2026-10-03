#pragma once

// Small helpers for things STM32CubeF4 LL does not cover.

#include "stm32f4xx.h"

// CRC over 32-bit words (replaces CRC_CalcBlockCRC, does not reset)
static inline uint32_t crc_calc_block(const uint32_t *buf, uint32_t len) {
  for(uint32_t i = 0; i < len; i++) {
    CRC->DR = buf[i];
  }
  return CRC->DR;
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
