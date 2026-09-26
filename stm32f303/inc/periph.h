#pragma once

// F3 peripheral setup on LL and plain registers. Each function replays the
// register writes the Cube HAL init code used to do (see periph.c).

#include "stm32f3xx.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_gpio.h"
#include "stm32f3xx_ll_usart.h"
#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_system.h"
#include "stm32f3xx_ll_pwr.h"
#include "stm32f3xx_ll_tim.h"
#include "stm32f3xx_ll_adc.h"
#include "stm32f3xx_ll_dac.h"
#include "stm32f3xx_ll_opamp.h"

void delay_ms(uint32_t ms);

void clock_init(void);
void tim8_init(void);
void tim8_start(void);
void adc_init(void);
void adc_calibrate(void);
void adc_start(void);
void dac_init(void);
void dac_start(void);
void opamp_init(void);
void opamp_calibrate(void);
void opamp_start(void);

// CRC over 32-bit words from the reset value (replaces HAL_CRC_Calculate)
static inline uint32_t crc_calc(const uint32_t *buf, uint32_t len) {
  CRC->CR |= CRC_CR_RESET;
  for(uint32_t i = 0; i < len; i++) {
    CRC->DR = buf[i];
  }
  return CRC->DR;
}
