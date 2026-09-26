// F3 peripheral setup without the Cube HAL. Every function replays the
// register writes the HAL (V1.5.8) init code made with the old CubeMX style
// settings, in the same order, so the hardware ends up configured the same.

#include "periph.h"
#include "f3hw.h"

extern volatile uint64_t systime;

// HAL_Delay() semantics: at least ms full SysTick periods
void delay_ms(uint32_t ms) {
  uint64_t start = systime;
  while(systime - start < (uint64_t)ms + 1) {
  }
}

static void gpio_analog(GPIO_TypeDef *port, uint32_t pins) {
  LL_GPIO_Init(port, &(LL_GPIO_InitTypeDef){.Pin = pins, .Mode = LL_GPIO_MODE_ANALOG, .Pull = LL_GPIO_PULL_NO});
}

static void gpio_af(GPIO_TypeDef *port, uint32_t pins, uint32_t af) {
  LL_GPIO_Init(port, &(LL_GPIO_InitTypeDef){.Pin = pins, .Mode = LL_GPIO_MODE_ALTERNATE, .Speed = LL_GPIO_SPEED_FREQ_LOW, .OutputType = LL_GPIO_OUTPUT_PUSHPULL, .Pull = LL_GPIO_PULL_NO, .Alternate = af});
}

// HSE 8 MHz * 9 = 72 MHz SYSCLK/HCLK, APB1 36 MHz, APB2 72 MHz,
// TIM8 and ADCs from the PLL (TIM8 144 MHz), USART3 from SYSCLK, RTC from LSI.
// SysTick 1 kHz, priority 0. Replaces HAL_Init() + SystemClock_Config().
void clock_init(void) {
  // HAL_Init()
  LL_FLASH_EnablePrefetch();
  NVIC_SetPriorityGrouping(3);  // NVIC_PRIORITYGROUP_4
  // HAL_MspInit()
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  NVIC_SetPriority(MemoryManagement_IRQn, 0);
  NVIC_SetPriority(BusFault_IRQn, 0);
  NVIC_SetPriority(UsageFault_IRQn, 0);
  NVIC_SetPriority(SVCall_IRQn, 0);
  NVIC_SetPriority(DebugMonitor_IRQn, 0);
  NVIC_SetPriority(PendSV_IRQn, 0);

  // oscillators
  LL_RCC_HSE_Enable();
  while(!LL_RCC_HSE_IsReady()) {
  }
  LL_RCC_HSI_Enable();
  while(!LL_RCC_HSI_IsReady()) {
  }
  LL_RCC_LSI_Enable();
  while(!LL_RCC_LSI_IsReady()) {
  }
  LL_RCC_PLL_Disable();
  while(LL_RCC_PLL_IsReady()) {
  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();
  while(!LL_RCC_PLL_IsReady()) {
  }

  // bus clocks, the HAL_RCC_ClockConfig() order
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_16);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
  }
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  SystemCoreClockUpdate();

  // peripheral clocks, HAL_RCCEx_PeriphCLKConfig()
  uint32_t pwr_was_off = !LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_EnableBkUpAccess();  // stays set, RTC->BKP0R is written by the bootloader command
  while(!LL_PWR_IsEnabledBkUpAccess()) {
  }
  uint32_t rtcsel = LL_RCC_GetRTCClockSource();
  if(rtcsel != LL_RCC_RTC_CLKSOURCE_NONE && rtcsel != LL_RCC_RTC_CLKSOURCE_LSI) {
    uint32_t bdcr = RCC->BDCR & ~RCC_BDCR_RTCSEL;
    LL_RCC_ForceBackupDomainReset();
    LL_RCC_ReleaseBackupDomainReset();
    RCC->BDCR = bdcr;
  }
  LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);
  if(pwr_was_off) {
    LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_PWR);
  }
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK2);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART3_CLKSOURCE_SYSCLK);
  LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSRC_PLL_DIV_1);
  LL_RCC_SetADCClockSource(LL_RCC_ADC34_CLKSRC_PLL_DIV_1);
  LL_RCC_SetUSBClockSource(LL_RCC_USB_CLKSOURCE_PLL_DIV_1_5);
  LL_RCC_SetTIMClockSource(LL_RCC_TIM8_CLKSOURCE_PLL);

  SysTick_Config(SystemCoreClock / 1000);  // HCLK source
  NVIC_SetPriority(SysTick_IRQn, 0);
}

// Center aligned PWM on TIM8 CH1-3 with complementary outputs, dead time and
// both break inputs, TRGO on update for the ADCs. Outputs stay off (MOE and
// CCxE clear) until tim8_start(). Replaces MX_TIM8_Init().
void tim8_init(void) {
  // HAL_TIM_Base_MspInit()
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM8);
  NVIC_SetPriority(TIM8_UP_IRQn, 0);
  NVIC_EnableIRQ(TIM8_UP_IRQn);

  // time base, center aligned mode 3, no ARR preload. Like TIM_Base_SetConfig()
  // the update event loads PSC/RCR with URS set, so no UIF, and CR1 is
  // written afterwards.
  uint32_t cr1 = TIM8->CR1;
  MODIFY_REG(cr1, TIM_CR1_DIR | TIM_CR1_CMS | TIM_CR1_CKD | TIM_CR1_ARPE, TIM_CR1_CMS);
  LL_TIM_SetAutoReload(TIM8, PWM_RES);
  LL_TIM_SetPrescaler(TIM8, 0);
#ifdef PWM_INVERT
  LL_TIM_SetRepetitionCounter(TIM8, 1);
#else
  LL_TIM_SetRepetitionCounter(TIM8, 0);
#endif
  LL_TIM_SetUpdateSource(TIM8, LL_TIM_UPDATESOURCE_COUNTER);
  LL_TIM_GenerateEvent_UPDATE(TIM8);
  TIM8->CR1 = cr1;

  // internal clock, TRGO = update, no master/slave
  TIM8->SMCR &= ~(TIM_SMCR_SMS | TIM_SMCR_TS | TIM_SMCR_ETF | TIM_SMCR_ETPS | TIM_SMCR_ECE | TIM_SMCR_ETP | TIM_SMCR_MSM);
  LL_TIM_SetTriggerOutput2(TIM8, LL_TIM_TRGO2_RESET);
  LL_TIM_SetTriggerOutput(TIM8, LL_TIM_TRGO_UPDATE);

  // CH1-3 PWM mode 1 with preload, active high, idle low, compare 0
  TIM8->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P |
                  TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE | TIM_CCER_CC1NP | TIM_CCER_CC2NP | TIM_CCER_CC3NP);
  TIM8->CR2 &= ~(TIM_CR2_OIS1 | TIM_CR2_OIS1N | TIM_CR2_OIS2 | TIM_CR2_OIS2N | TIM_CR2_OIS3 | TIM_CR2_OIS3N);
  MODIFY_REG(TIM8->CCMR1, TIM_CCMR1_OC1M | TIM_CCMR1_CC1S | TIM_CCMR1_OC1FE | TIM_CCMR1_OC2M | TIM_CCMR1_CC2S | TIM_CCMR1_OC2FE,
             TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE);
  MODIFY_REG(TIM8->CCMR2, TIM_CCMR2_OC3M | TIM_CCMR2_CC3S | TIM_CCMR2_OC3FE,
             TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE);
  LL_TIM_OC_SetCompareCH1(TIM8, 0);
  LL_TIM_OC_SetCompareCH2(TIM8, 0);
  LL_TIM_OC_SetCompareCH3(TIM8, 0);

  // dead time, BRK and BRK2 active high with a 0.55us filter, no automatic output
  TIM8->BDTR = PWM_DEADTIME | TIM_BDTR_BKE | TIM_BDTR_BKP | (0xAU << TIM_BDTR_BKF_Pos) |
               TIM_BDTR_BK2E | TIM_BDTR_BK2P | (0xAU << TIM_BDTR_BK2F_Pos);

  // PB3 CH1N, PB4 CH2N, PB5 CH3N, PB6 CH1, PB8 CH2, PB9 CH3
  gpio_af(GPIOB, LL_GPIO_PIN_3 | LL_GPIO_PIN_4, LL_GPIO_AF_4);
  gpio_af(GPIOB, LL_GPIO_PIN_5, LL_GPIO_AF_3);
  gpio_af(GPIOB, LL_GPIO_PIN_6, LL_GPIO_AF_5);
  gpio_af(GPIOB, LL_GPIO_PIN_8 | LL_GPIO_PIN_9, LL_GPIO_AF_10);
}

// Update interrupt and counter first, then all six outputs, like
// HAL_TIM_Base_Start_IT() and HAL_TIM(Ex)_PWM(N)_Start() did.
void tim8_start(void) {
  LL_TIM_EnableIT_UPDATE(TIM8);
  LL_TIM_EnableCounter(TIM8);
#ifndef PWM_INVERT
  LL_TIM_SetRepetitionCounter(TIM8, 1);  //uptate event foo
#endif
  LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH1);
  LL_TIM_EnableAllOutputs(TIM8);
  LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH3);
  LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH1N);
  LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH2N);
  LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH3N);
}

static const uint32_t adc_ranks[] = {0, LL_ADC_REG_RANK_1, LL_ADC_REG_RANK_2, LL_ADC_REG_RANK_3, LL_ADC_REG_RANK_4, LL_ADC_REG_RANK_5, LL_ADC_REG_RANK_6};

// HAL_ADC_ConfigChannel(): rank, sampling time, single ended
__attribute__((noinline)) static void adc_rank(ADC_TypeDef *adc, uint32_t rank, uint32_t chan, uint32_t smp) {
  LL_ADC_REG_SetSequencerRanks(adc, adc_ranks[rank], chan);
  LL_ADC_SetChannelSamplingTime(adc, chan, smp);
  LL_ADC_SetChannelSingleDiff(adc, chan, LL_ADC_SINGLE_ENDED);
}

// Regulator on, 12 bit right aligned, overrun overwrites, 6 conversion scan
// started by TIM8 TRGO (rising edge), as HAL_ADC_Init() set it up.
static void adc_setup(ADC_TypeDef *adc, ADC_Common_TypeDef *common, uint32_t extsel) {
  if(!(adc->CR & ADC_CR_ADVREGEN_0)) {
    LL_ADC_EnableInternalRegulator(adc);
    for(__IO uint32_t wait = 10U * (SystemCoreClock / 1000000U); wait; wait--) {
    }
  }
  LL_ADC_SetCommonClock(common, LL_ADC_CLOCK_ASYNC_DIV1);  // from the PLL, RCC prescaler 1
  MODIFY_REG(adc->CFGR,
             ADC_CFGR_DISCNUM | ADC_CFGR_DISCEN | ADC_CFGR_CONT | ADC_CFGR_OVRMOD | ADC_CFGR_EXTSEL | ADC_CFGR_EXTEN | ADC_CFGR_ALIGN | ADC_CFGR_RES,
             ADC_CFGR_OVRMOD | extsel | ADC_CFGR_EXTEN_0);
  adc->CFGR &= ~(ADC_CFGR_AUTDLY | ADC_CFGR_DMACFG);
  LL_ADC_REG_SetSequencerLength(adc, LL_ADC_REG_SEQ_SCAN_ENABLE_6RANKS);
}

// ADC12 EXTSEL 0111 and ADC34 EXTSEL 0100 are both TIM8 TRGO
#define EXTSEL_ADC12_T8_TRGO (ADC_CFGR_EXTSEL_2 | ADC_CFGR_EXTSEL_1 | ADC_CFGR_EXTSEL_0)
#define EXTSEL_ADC34_T8_TRGO (ADC_CFGR_EXTSEL_2)

// Replaces MX_ADC1_Init() .. MX_ADC4_Init() and their MSP init.
void adc_init(void) {
  // ADC1: iw (ch3, opamp1) x5, uw (ch4)
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ADC12);
  gpio_analog(GPIOA, LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3);
  adc_setup(ADC1, ADC12_COMMON, EXTSEL_ADC12_T8_TRGO);
  for(uint32_t r = 1; r <= 5; r++) {
    adc_rank(ADC1, r, LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_19CYCLES_5);
  }
  adc_rank(ADC1, 6, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_181CYCLES_5);

  // ADC2: iu (ch3, opamp2) x5, uv (ch2)
  gpio_analog(GPIOA, LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7);
  adc_setup(ADC2, ADC12_COMMON, EXTSEL_ADC12_T8_TRGO);
  for(uint32_t r = 1; r <= 5; r++) {
    adc_rank(ADC2, r, LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_19CYCLES_5);
  }
  adc_rank(ADC2, 6, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_181CYCLES_5);

  // ADC3: iv (ch1, opamp3) x5, uu (ch5)
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ADC34);
  gpio_analog(GPIOB, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_13);
  adc_setup(ADC3, ADC34_COMMON, EXTSEL_ADC34_T8_TRGO);
  for(uint32_t r = 1; r <= 5; r++) {
    adc_rank(ADC3, r, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_19CYCLES_5);
  }
  adc_rank(ADC3, 6, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_181CYCLES_5);

  // ADC4: hv_temp (ch4) x3, mot_temp (ch5) x2, hv (ch3)
  gpio_analog(GPIOB, LL_GPIO_PIN_12 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15);
  adc_setup(ADC4, ADC34_COMMON, EXTSEL_ADC34_T8_TRGO);
  for(uint32_t r = 1; r <= 3; r++) {
    adc_rank(ADC4, r, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_19CYCLES_5);
  }
  adc_rank(ADC4, 4, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_19CYCLES_5);
  adc_rank(ADC4, 5, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_19CYCLES_5);
  adc_rank(ADC4, 6, LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_181CYCLES_5);
}

// Single ended calibration, ADC disabled (HAL_ADCEx_Calibration_Start())
void adc_calibrate(void) {
  ADC_TypeDef *const adcs[] = {ADC1, ADC2, ADC3, ADC4};
  for(int i = 0; i < 4; i++) {
    LL_ADC_StartCalibration(adcs[i], LL_ADC_SINGLE_ENDED);
    while(LL_ADC_IsCalibrationOnGoing(adcs[i])) {
    }
  }
}

// Enable, then arm the regular group for the TIM8 trigger (HAL_ADC_Start())
void adc_start(void) {
  ADC_TypeDef *const adcs[] = {ADC1, ADC2, ADC3, ADC4};
  for(int i = 0; i < 4; i++) {
    if(!LL_ADC_IsEnabled(adcs[i])) {
      LL_ADC_Enable(adcs[i]);
      while(!LL_ADC_IsActiveFlag_ADRDY(adcs[i])) {
      }
    }
    adcs[i]->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;
    LL_ADC_REG_StartConversion(adcs[i]);
  }
}

// DAC1 CH1 on PA4, output buffer on, no trigger (MX_DAC_Init())
void dac_init(void) {
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DAC1);
  gpio_analog(GPIOA, LL_GPIO_PIN_4);
  DAC1->CR &= ~(DAC_CR_MAMP1 | DAC_CR_WAVE1 | DAC_CR_TSEL1 | DAC_CR_TEN1 | DAC_CR_BOFF1);
}

void dac_start(void) {
  LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1);
  LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, 0);
}

static OPAMP_TypeDef *const opamps[] = {OPAMP1, OPAMP2, OPAMP3};

// PGA gain 16 on VP0, factory trim, disabled (MX_OPAMPx_Init())
void opamp_init(void) {
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  gpio_analog(GPIOA, LL_GPIO_PIN_1 | LL_GPIO_PIN_2);  // OPAMP1 VINP, VOUT
  gpio_analog(GPIOA, LL_GPIO_PIN_6 | LL_GPIO_PIN_7);  // OPAMP2 VOUT, VINP
  gpio_analog(GPIOB, LL_GPIO_PIN_0 | LL_GPIO_PIN_1);  // OPAMP3 VINP, VOUT
  for(int i = 0; i < 3; i++) {
    MODIFY_REG(opamps[i]->CSR,
               OPAMP_CSR_TRIMOFFSETN | OPAMP_CSR_TRIMOFFSETP | OPAMP_CSR_USERTRIM | OPAMP_CSR_PGGAIN | OPAMP_CSR_VPSSEL |
                   OPAMP_CSR_VMSSEL | OPAMP_CSR_TCMEN | OPAMP_CSR_VPSEL | OPAMP_CSR_VMSEL | OPAMP_CSR_FORCEVP,
               OPAMP_CSR_VMSEL_1 | OPAMP_CSR_VPSEL | OPAMP_CSR_PGGAIN_0 | OPAMP_CSR_PGGAIN_1);
  }
}

// Binary search of one trim field against OUTCAL, HAL_OPAMP_SelfCalibrate()
static uint32_t opamp_trim(OPAMP_TypeDef *op, uint32_t mask, uint32_t pos) {
  uint32_t trim  = 16U;
  uint32_t delta = 8U;
  while(delta != 0U) {
    MODIFY_REG(op->CSR, mask, trim << pos);
    delay_ms(2);
    if(op->CSR & OPAMP_CSR_OUTCAL) {
      trim += delta;
    } else {
      trim -= delta;
    }
    delta >>= 1U;
  }
  MODIFY_REG(op->CSR, mask, trim << pos);
  delay_ms(2);
  if(op->CSR & OPAMP_CSR_OUTCAL) {
    trim++;
    MODIFY_REG(op->CSR, mask, trim << pos);
  }
  return trim;
}

// User trim from self calibration, OPAMPs left disabled
void opamp_calibrate(void) {
  for(int i = 0; i < 3; i++) {
    OPAMP_TypeDef *op = opamps[i];
    op->CSR |= OPAMP_CSR_FORCEVP;
    op->CSR |= OPAMP_CSR_USERTRIM;
    op->CSR |= OPAMP_CSR_CALON;
    MODIFY_REG(op->CSR, OPAMP_CSR_CALSEL, OPAMP_CSR_CALSEL);  // 90% VDDA, NMOS
    LL_OPAMP_Enable(op);
    uint32_t n = opamp_trim(op, OPAMP_CSR_TRIMOFFSETN, OPAMP_CSR_TRIMOFFSETN_Pos);
    MODIFY_REG(op->CSR, OPAMP_CSR_CALSEL, OPAMP_CSR_CALSEL_0);  // 10% VDDA, PMOS
    uint32_t p = opamp_trim(op, OPAMP_CSR_TRIMOFFSETP, OPAMP_CSR_TRIMOFFSETP_Pos);
    op->CSR &= ~OPAMP_CSR_CALON;
    LL_OPAMP_Disable(op);
    op->CSR &= ~OPAMP_CSR_FORCEVP;
    MODIFY_REG(op->CSR, OPAMP_CSR_TRIMOFFSETP, p << OPAMP_CSR_TRIMOFFSETP_Pos);
    MODIFY_REG(op->CSR, OPAMP_CSR_TRIMOFFSETN, n << OPAMP_CSR_TRIMOFFSETN_Pos);
  }
}

void opamp_start(void) {
  for(int i = 0; i < 3; i++) {
    LL_OPAMP_Enable(opamps[i]);
  }
}
