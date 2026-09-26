/*
* This file is part of the stmbl project.
*
* Copyright (C) 2013-2017 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2013-2017 Nico Stute <crinq@crinq.de>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "main.h"
#include "stm32f3xx.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_gpio.h"
#include "stm32f3xx_ll_usart.h"
#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_system.h"
#include "stm32f3xx_ll_pwr.h"
#include "stm32f3xx_ll_tim.h"
#include "stm32f3xx_ll_rtc.h"
#include "version.h"
#include "common.h"
#include "f3hw.h"

uint32_t systick_freq;
volatile uint64_t systime = 0;

volatile packet_bootloader_t rx_buf;
volatile packet_bootloader_t tx_buf;


void SystemClock_Config(void);
void Error_Handler(void);

void SysTick_Handler(void) {
  systime++;
}

// HAL_Delay() semantics: at least ms full SysTick periods
static void delay_ms(uint32_t ms) {
  uint64_t start = systime;
  while(systime - start < (uint64_t)ms + 1) {
  }
}

// CRC-32 over words from the reset value, CRC unit in its reset configuration
static uint32_t crc_calc(const uint32_t *buf, uint32_t len) {
  CRC->CR |= CRC_CR_RESET;
  for(uint32_t i = 0; i < len; i++) {
    CRC->DR = buf[i];
  }
  return CRC->DR;
}

// Flash, the register sequences of HAL_FLASH_Program() and HAL_FLASHEx_Erase()
#define FLASH_TIMEOUT_MS 50000U
#define FLASH_PAGE_SIZE 0x800U  // 2 KB pages on the F303xC

static int flash_wait(void) {
  uint64_t start = systime;
  while(FLASH->SR & FLASH_SR_BSY) {
    if(systime - start > FLASH_TIMEOUT_MS) {
      return -1;
    }
  }
  if(FLASH->SR & FLASH_SR_EOP) {
    FLASH->SR = FLASH_SR_EOP;
  }
  if(FLASH->SR & (FLASH_SR_WRPERR | FLASH_SR_PGERR)) {
    FLASH->SR = FLASH_SR_WRPERR | FLASH_SR_PGERR;
    return -1;
  }
  return 0;
}

static void flash_unlock(void) {
  if(FLASH->CR & FLASH_CR_LOCK) {
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
  }
}

static void flash_lock(void) {
  FLASH->CR |= FLASH_CR_LOCK;
}

static int flash_program_word(uint32_t addr, uint32_t data) {
  if(flash_wait()) {
    return -1;
  }
  for(uint32_t i = 0; i < 2; i++) {
    FLASH->CR |= FLASH_CR_PG;
    *(__IO uint16_t *)(addr + 2U * i) = (uint16_t)(data >> (16U * i));
    int ret = flash_wait();
    FLASH->CR &= ~FLASH_CR_PG;
    if(ret) {
      return -1;
    }
  }
  return 0;
}

static int flash_erase_pages(uint32_t addr, uint32_t end) {
  if(flash_wait()) {
    return -1;
  }
  for(; addr < end; addr += FLASH_PAGE_SIZE) {
    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR = addr;
    FLASH->CR |= FLASH_CR_STRT;
    int ret = flash_wait();
    FLASH->CR &= ~FLASH_CR_PER;
    if(ret) {
      return -1;
    }
  }
  return 0;
}

#define APP_START 0x08004000
#define APP_END 0x08020000
#define APP_RANGE_VALID(a, s) (!(((a) | (s)) & 3) && (a) >= APP_START && ((a) + (s)) <= APP_END)
#define VERSION_INFO_OFFSET 0x188
static volatile const version_info_t *app_info = (void *)(APP_START + VERSION_INFO_OFFSET);

static int app_ok(void) {
  if(!APP_RANGE_VALID(APP_START, app_info->image_size)) {
    return 0;
  }
  uint32_t crc = crc_calc((uint32_t *)APP_START, app_info->image_size / 4);

  if(crc != 0) {
    return 0;
  }
  return 1;
}

void TIM8_UP_IRQHandler() {
  static uint32_t last_dma_count = 0;

  uint32_t dma_count = DMA1_Channel3->CNDTR;
  // if(USART3->ISR & USART_ISR_RTOF) {                                    // idle line
  //   USART3->ICR |= USART_ICR_RTOCF | USART_ICR_FECF | USART_ICR_ORECF;  // timeout clear flag
  if(dma_count == last_dma_count) {  // framing
    last_dma_count = 0;

    // start rx DMA
    DMA1_Channel3->CCR &= (uint16_t)(~DMA_CCR_EN);
    DMA1_Channel3->CNDTR = sizeof(packet_bootloader_t);
    DMA1_Channel3->CCR |= DMA_CCR_EN;
  } else {
    last_dma_count = dma_count;
  }

  if(dma_count == 0) {
    if(rx_buf.header.slave_addr == 255 && rx_buf.header.len == (sizeof(packet_bootloader_t) - sizeof(stmbl_talk_header_t)) / 4 && rx_buf.header.crc == crc_calc((uint32_t *)&(rx_buf.header.slave_addr), sizeof(packet_bootloader_t) / 4 - 1)) {
      //do stuff
      //tx_buf.state = do_stuff();

      switch(rx_buf.header.flags.cmd) {
        case NO_CMD:
          break;
        case WRITE_CONF:
          break;
        case READ_CONF:
          break;
        case DO_RESET:
          flash_lock();
          NVIC_SystemReset();
          break;
        case BOOTLOADER:
          break;
      }

      int status = 0;
      switch(rx_buf.cmd) {
        case BOOTLOADER_OPCODE_NOP:
          break;
        case BOOTLOADER_OPCODE_READ:
          tx_buf.value = *(uint32_t *)rx_buf.addr;
          tx_buf.addr  = rx_buf.addr;
          break;

        case BOOTLOADER_OPCODE_WRITE:
          if(*(uint32_t *)rx_buf.addr != rx_buf.value) {
            status = flash_program_word(rx_buf.addr, rx_buf.value);
          }
          if(*(uint32_t *)rx_buf.addr != rx_buf.value) {
            status = -1;
          }
          tx_buf.value = *(uint32_t *)rx_buf.addr;
          tx_buf.addr  = rx_buf.addr;
          break;

        case BOOTLOADER_OPCODE_PAGEERASE:
          flash_unlock();
          status = flash_erase_pages(APP_START, APP_END);
          break;

        case BOOTLOADER_OPCODE_CRCCHECK:
          status = app_ok() ? 0 : -1;
          break;
      }

      if(status != 0) {
        tx_buf.state = BOOTLOADER_STATE_NAK;
      } else {
        tx_buf.state = BOOTLOADER_STATE_OK;
      }

      tx_buf.cmd                  = rx_buf.cmd;
      tx_buf.header.flags.counter = rx_buf.header.flags.counter;
      tx_buf.header.slave_addr    = 255;
      tx_buf.header.len           = (sizeof(packet_bootloader_t) - sizeof(stmbl_talk_header_t)) / 4;
      tx_buf.header.conf_addr     = 0;
      tx_buf.header.config.u32    = 0;
      tx_buf.header.flags.cmd     = NO_CMD;
      tx_buf.header.crc           = crc_calc((uint32_t *)&(tx_buf.header.slave_addr), sizeof(packet_bootloader_t) / 4 - 1);

      rx_buf.header.crc = 0;

      // start tx DMA
      DMA1_Channel2->CCR &= (uint16_t)(~DMA_CCR_EN);
      DMA1_Channel2->CNDTR = sizeof(packet_bootloader_t);
      DMA1_Channel2->CCR |= DMA_CCR_EN;
    }
  }

  TIM8->SR = ~TIM_SR_UIF;
}

void uart_init() {
  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

  // 8N1, 8x oversampling, clocked from SYSCLK; DMA requests and overrun
  // disable are set first and kept by the init, as with HAL_UART_Init()
  USART3->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR | USART_CR3_OVRDIS;
  LL_USART_Disable(USART3);
  LL_USART_Init(USART3, &(LL_USART_InitTypeDef){
                            .BaudRate            = DATABAUD,
                            .DataWidth           = LL_USART_DATAWIDTH_8B,
                            .StopBits            = LL_USART_STOPBITS_1,
                            .Parity              = LL_USART_PARITY_NONE,
                            .TransferDirection   = LL_USART_DIRECTION_TX_RX,
                            .HardwareFlowControl = LL_USART_HWCONTROL_NONE,
                            .OverSampling        = LL_USART_OVERSAMPLING_8,
                        });
  LL_USART_DisableOneBitSamp(USART3);
  USART3->CR2 &= ~(USART_CR2_LINEN | USART_CR2_CLKEN);
  USART3->CR3 &= ~(USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN);
  LL_USART_Enable(USART3);
  while(!LL_USART_IsActiveFlag_TEACK(USART3) || !LL_USART_IsActiveFlag_REACK(USART3)) {
  }

  /**USART3 GPIO Configuration    
   PB10     ------> USART3_TX
   PB11     ------> USART3_RX 
   */
  LL_GPIO_Init(GPIOB, &(LL_GPIO_InitTypeDef){
                          .Pin        = LL_GPIO_PIN_10 | LL_GPIO_PIN_11,
                          .Mode       = LL_GPIO_MODE_ALTERNATE,
                          .Speed      = LL_GPIO_SPEED_FREQ_HIGH,
                          .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
                          .Pull       = LL_GPIO_PULL_UP,
                          .Alternate  = LL_GPIO_AF_7,
                      });

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  //TX DMA
  DMA1_Channel2->CCR &= (uint16_t)(~DMA_CCR_EN);
  DMA1_Channel2->CPAR  = (uint32_t) & (USART3->TDR);
  DMA1_Channel2->CMAR  = (uint32_t)&tx_buf;
  DMA1_Channel2->CNDTR = sizeof(packet_bootloader_t);
  DMA1_Channel2->CCR   = DMA_CCR_MINC | DMA_CCR_DIR;  // | DMA_CCR_PL_0 | DMA_CCR_PL_1
  DMA1->IFCR           = DMA_IFCR_CTCIF2 | DMA_IFCR_CHTIF2 | DMA_IFCR_CGIF2;

  //RX DMA
  DMA1_Channel3->CCR &= (uint16_t)(~DMA_CCR_EN);
  DMA1_Channel3->CPAR  = (uint32_t) & (USART3->RDR);
  DMA1_Channel3->CMAR  = (uint32_t)&rx_buf;
  DMA1_Channel3->CNDTR = sizeof(packet_bootloader_t);
  DMA1_Channel3->CCR   = DMA_CCR_MINC;  // | DMA_CCR_PL_0 | DMA_CCR_PL_1
  DMA1->IFCR           = DMA_IFCR_CTCIF3 | DMA_IFCR_CHTIF3 | DMA_IFCR_CGIF3;
  DMA1_Channel3->CCR |= DMA_CCR_EN;

  USART3->RTOR = 16;               // 16 bits timeout
  USART3->CR2 |= USART_CR2_RTOEN;  // timeout en
  USART3->ICR |= USART_ICR_RTOCF;  // timeout clear flag
}

/* RTC init function: 24h, prescalers 127/255, no output, only if the calendar
   was never set up (HAL_RTC_Init()) */
static void rtc_init(void) {
  if(LL_RTC_IsActiveFlag_INITS(RTC)) {
    return;
  }
  LL_RTC_DisableWriteProtection(RTC);
  if(!(RTC->ISR & RTC_ISR_INITF)) {
    RTC->ISR |= RTC_ISR_INIT;
    uint64_t start = systime;
    while(!(RTC->ISR & RTC_ISR_INITF)) {
      if(systime - start > 1000) {
        LL_RTC_EnableWriteProtection(RTC);
        Error_Handler();
      }
    }
  }
  RTC->CR &= ~(RTC_CR_FMT | RTC_CR_OSEL | RTC_CR_POL);  // 24h, no output, high polarity
  LL_RTC_SetSynchPrescaler(RTC, 255);
  LL_RTC_SetAsynchPrescaler(RTC, 127);
  RTC->ISR &= ~RTC_ISR_INIT;
  if(!(RTC->CR & RTC_CR_BYPSHAD)) {
    RTC->ISR &= ~(RTC_ISR_INIT | RTC_ISR_RSF);
    uint64_t start = systime;
    while(!(RTC->ISR & RTC_ISR_RSF)) {
      if(systime - start > 1000) {
        LL_RTC_EnableWriteProtection(RTC);
        Error_Handler();
      }
    }
  }
  LL_RTC_SetAlarmOutputType(RTC, LL_RTC_ALARM_OUTPUTTYPE_OPENDRAIN);
  LL_RTC_EnableWriteProtection(RTC);
}

// TIM8 only times the rx framing here: 144 MHz / (2 * PWM_RES / 2), no outputs,
// no break, update interrupt at priority 15 (MX_TIM8_Init())
static void tim8_init(void) {
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM8);
  NVIC_SetPriority(TIM8_UP_IRQn, 15);
  NVIC_EnableIRQ(TIM8_UP_IRQn);

  uint32_t cr1 = TIM8->CR1;
  MODIFY_REG(cr1, TIM_CR1_DIR | TIM_CR1_CMS | TIM_CR1_CKD | TIM_CR1_ARPE, TIM_CR1_CMS);
  LL_TIM_SetAutoReload(TIM8, PWM_RES / 2);
  LL_TIM_SetPrescaler(TIM8, 0);
#ifdef PWM_INVERT
  LL_TIM_SetRepetitionCounter(TIM8, 1);
#else
  LL_TIM_SetRepetitionCounter(TIM8, 0);
#endif
  LL_TIM_SetUpdateSource(TIM8, LL_TIM_UPDATESOURCE_COUNTER);  // load PSC/RCR without setting UIF
  LL_TIM_GenerateEvent_UPDATE(TIM8);
  TIM8->CR1 = cr1;

  TIM8->SMCR &= ~(TIM_SMCR_SMS | TIM_SMCR_TS | TIM_SMCR_ETF | TIM_SMCR_ETPS | TIM_SMCR_ECE | TIM_SMCR_ETP | TIM_SMCR_MSM);
  LL_TIM_SetTriggerOutput2(TIM8, LL_TIM_TRGO2_RESET);
  LL_TIM_SetTriggerOutput(TIM8, LL_TIM_TRGO_UPDATE);
  TIM8->BDTR = PWM_DEADTIME | TIM_BDTR_BKP | TIM_BDTR_BK2P;  // breaks disabled
}

int main(void) {
  extern void *g_pfnVectors;
  SCB->VTOR = (uint32_t)&g_pfnVectors;

  SystemClock_Config();
  systick_freq = SystemCoreClock;

  /* Initialize all configured peripherals */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA | LL_AHB1_GRP1_PERIPH_GPIOB | LL_AHB1_GRP1_PERIPH_GPIOC | LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1 | LL_AHB1_GRP1_PERIPH_DMA2);
  LL_RCC_EnableRTC();

  /*Configure GPIO pin Output Level */
  LL_GPIO_ResetOutputPin(LED_PORT, LED_PIN);
  LL_GPIO_Init(LED_PORT, &(LL_GPIO_InitTypeDef){.Pin = LED_PIN, .Mode = LL_GPIO_MODE_OUTPUT, .Speed = LL_GPIO_SPEED_FREQ_LOW, .OutputType = LL_GPIO_OUTPUT_PUSHPULL, .Pull = LL_GPIO_PULL_NO});

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);  // reset config: CRC-32, init 0xFFFFFFFF, no reversal

  rtc_init();

  if(app_ok() && RTC->BKP0R == 0x00000000) {
    /* Jump to user application */
    void (*JumpToApplication)(void);
    uint32_t JumpAddress = *(__IO uint32_t *)(APP_START + 4);
    JumpToApplication    = (void *)JumpAddress;

    /* Initialize user application's Stack Pointer */
    __set_MSP(*(__IO uint32_t *)APP_START);
    JumpToApplication();
    while(1) {
    }
  } else {
    uart_init();

    tx_buf.header.slave_addr = 255;
    tx_buf.header.len        = (sizeof(packet_bootloader_t) - sizeof(stmbl_talk_header_t)) / 4;
    tx_buf.header.conf_addr  = 0;
    tx_buf.header.config.u32 = 0;
    tx_buf.header.flags.cmd  = NO_CMD;

    // start rx DMA
    DMA1_Channel3->CCR &= (uint16_t)(~DMA_CCR_EN);
    DMA1_Channel3->CNDTR = sizeof(packet_bootloader_t);
    DMA1_Channel3->CCR |= DMA_CCR_EN;

    tim8_init();
    LL_TIM_EnableIT_UPDATE(TIM8);
    LL_TIM_EnableCounter(TIM8);
  }
  RTC->BKP0R = 0x00000000;

  while(1) {
    LL_GPIO_SetOutputPin(LED_PORT, LED_PIN);
    delay_ms(50);
    LL_GPIO_ResetOutputPin(LED_PORT, LED_PIN);
    delay_ms(50);
  }
}

// HSE 8 MHz * 9 = 72 MHz, APB1 36 MHz, APB2 72 MHz, TIM8 from the PLL,
// USART3 from SYSCLK, RTC from LSI, SysTick 1 kHz at priority 0.
// Replaces HAL_Init() + the CubeMX SystemClock_Config().
void SystemClock_Config(void) {
  LL_FLASH_EnablePrefetch();
  NVIC_SetPriorityGrouping(3);  // NVIC_PRIORITYGROUP_4
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  NVIC_SetPriority(MemoryManagement_IRQn, 0);
  NVIC_SetPriority(BusFault_IRQn, 0);
  NVIC_SetPriority(UsageFault_IRQn, 0);
  NVIC_SetPriority(SVCall_IRQn, 0);
  NVIC_SetPriority(DebugMonitor_IRQn, 0);
  NVIC_SetPriority(PendSV_IRQn, 0);

  LL_RCC_HSE_Enable();
  while(!LL_RCC_HSE_IsReady()) {
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

  // RTC clock from LSI; the backup domain stays writable for RTC->BKP0R
  uint32_t pwr_was_off = !LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_EnableBkUpAccess();
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
  LL_RCC_SetUSARTClockSource(LL_RCC_USART3_CLKSOURCE_SYSCLK);
  LL_RCC_SetTIMClockSource(LL_RCC_TIM8_CLKSOURCE_PLL);

  SysTick_Config(SystemCoreClock / 1000);  // HCLK source
  NVIC_SetPriority(SysTick_IRQn, 0);
}

//Delay implementation for hal_term.c
void Wait(uint32_t ms) {
  delay_ms(ms);
}

void Error_Handler(void) {
  while(1) {
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8);
  }
}
