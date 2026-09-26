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
#include "periph.h"

#include <math.h>
#include "defines.h"
#include "hal.h"
#include "angle.h"

#include "version.h"
#include "common.h"
#include "commands.h"
#include "f3hw.h"

#include "usbd_cdc_if.h"
#ifdef USB_TERM
#include "usb_device.h"
#endif

volatile uint64_t systime = 0;

void SysTick_Handler(void) {
  /* USER CODE BEGIN SysTick_IRQn 0 */
  systime++;
  /* USER CODE END SysTick_IRQn 0 */
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

uint32_t systick_freq;

uint32_t hal_get_systick_value() {
  return (SysTick->VAL);
}

uint32_t hal_get_systick_reload() {
  return (SysTick->LOAD);
}

uint32_t hal_get_systick_freq() {
  return (systick_freq);
}

void Error_Handler(void);

// Switch the bridge off in hardware. TIM8 keeps running on its own, so once
// the rt stops -- an overrun here or in hal_run_rt, a MISC_ERROR, a stop
// command -- the last compare values stay latched and the bridge goes on
// applying them with no current loop until the break comparator trips. On X
// the f3 went silent under a 19 A burst and the bridge was found shorted.
// Nothing in the rt path reliably calls rt_stop (hal_run_rt's own overrun
// branch does not), so check the state after every tick instead.
static void bridge_off(void) {
  TIM8->BDTR &= ~TIM_BDTR_MOE;
#ifdef HV_EN_PIN
  LL_GPIO_SetOutputPin(HV_EN_PORT, HV_EN_PIN);
#endif
}

// Independent watchdog, from its own 40 kHz LSI. hal.c kicks it at the end of
// every rt, frt and nrt run (HAL_WATCHDOG, set in stm32f303/Makefile), so it
// only fires when the f3 hangs outright -- stuck in the rt interrupt, which
// also starves nrt, or locked up. A clean rt stop is bridge_off()'s job. After
// the reset the pwm pins are inputs, and PA15 (HV_EN) comes up with its JTDI
// pull-up on the IPM's ITRIP, which holds all six gates off. A system reset
// stops the IWDG, so the bootloader (ls.c resets into it) is not affected.
void hal_init_watchdog(float time) {
  IWDG->KR  = 0xCCCC;  // start; from here only a reset stops it
  IWDG->KR  = 0x5555;  // unlock PR and RLR
  IWDG->PR  = 0;       // LSI / 4: 0.1 ms per count
  IWDG->RLR = (uint32_t)CLAMP(time * 10000.0, 1.0, 4095.0);
  while(IWDG->SR) {
  }
  IWDG->KR = 0xAAAA;
}

void hal_reset_watchdog() {
  IWDG->KR = 0xAAAA;
}

void TIM8_UP_IRQHandler() {
  GPIOA->BSRR |= LL_GPIO_PIN_9;
  TIM8->SR = ~TIM_SR_UIF;
  hal_run_rt();
  if(TIM8->SR & TIM_SR_UIF) {
    hal_stop();
    hal.hal_state = RT_TOO_LONG;
  }
  if(hal.rt_state == RT_STOP) {
    bridge_off();
  }
  GPIOA->BSRR |= LL_GPIO_PIN_9 << 16;
}

void about(char *ptr) {
  printf("######## software info ########\n");
  printf(
      "%s v%i.%i.%i %s\n",
      version_info.product_name,
      version_info.major,
      version_info.minor,
      version_info.patch,
      version_info.git_version);
  printf("Branch %s\n", version_info.git_branch);
  printf("Compiled %s %s ", version_info.build_date, version_info.build_time);
  printf("by %s on %s\n", version_info.build_user, version_info.build_host);
  printf("GCC        %s\n", __VERSION__);
  printf("newlib     %s\n", _NEWLIB_VERSION);
#ifdef __CM4_CMSIS_VERSION
  printf("CMSIS      %i.%i\n", __CM4_CMSIS_VERSION_MAIN, __CM4_CMSIS_VERSION_SUB);
#endif
#ifdef __STM32F4XX_STDPERIPH_VERSION
  printf("StdPeriph  %i.%i.%i\n", __STM32F4XX_STDPERIPH_VERSION_MAIN, __STM32F4XX_STDPERIPH_VERSION_SUB1, __STM32F4XX_STDPERIPH_VERSION_SUB2);
#endif
#ifdef __STM32F3xx_CMSIS_VERSION
  printf("F3 CMSIS   %i.%i.%i\n", __STM32F3xx_CMSIS_VERSION_MAIN, __STM32F3xx_CMSIS_VERSION_SUB1, __STM32F3xx_CMSIS_VERSION_SUB2);
#endif
}

COMMAND("about", about, "show system infos");

void bootloader(char *ptr) {
#ifdef USB_DISCONNECT_PIN
  LL_GPIO_SetOutputPin(USB_DISCONNECT_PORT, USB_DISCONNECT_PIN);
  delay_ms(100);
#endif
  RTC->BKP0R = 0xDEADBEEF;
  NVIC_SystemReset();
}

COMMAND("bootloader", bootloader, "enter bootloader");

void reset(char *ptr) {
  NVIC_SystemReset();
}
COMMAND("reset", reset, "reset STMBL");

int main(void) {
  // Copy the rt path into CCM RAM (see the .ccmram section in the linker
  // script). The startup code only copies .data. Nothing that runs before
  // this point lives in CCM.
  extern uint32_t _siccmram, _sccmram, _eccmram;
  for(uint32_t *src = &_siccmram, *dst = &_sccmram; dst < &_eccmram;) {
    *dst++ = *src++;
  }

  // Relocate interrupt vectors
  extern void *g_pfnVectors;
  SCB->VTOR = (uint32_t)&g_pfnVectors;

  clock_init();
  systick_freq = SystemCoreClock;

  /* Initialize all configured peripherals */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA | LL_AHB1_GRP1_PERIPH_GPIOB | LL_AHB1_GRP1_PERIPH_GPIOC | LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  LL_GPIO_StructInit(&GPIO_InitStruct);
#ifdef USB_DISCONNECT_PIN
  GPIO_InitStruct.Pin        = USB_DISCONNECT_PIN;
  GPIO_InitStruct.Mode       = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
  LL_GPIO_Init(USB_DISCONNECT_PORT, &GPIO_InitStruct);
  LL_GPIO_ResetOutputPin(USB_DISCONNECT_PORT, USB_DISCONNECT_PIN);
#endif

  tim8_init();
  adc_init();
  dac_init();

  //COMP1 in+ pa1(ADC1_IN2)  in- pa4(dac1_ch1) out TIM8 BRK2
  COMP1->CSR = COMP_CSR_COMPxINSEL_2 | COMP1_CSR_COMP1OUTSEL_2 | COMP_CSR_COMPxEN;
  //COMP2 in+ pa7(ADC2_IN4)  in- pa4(dac1_ch1) out TIM8 BRK_ACTH COMP_CSR_COMPxNONINSEL
  COMP2->CSR = COMP_CSR_COMPxINSEL_2 | COMP2_CSR_COMP2OUTSEL_0 | COMP2_CSR_COMP2OUTSEL_1 | COMP_CSR_COMPxEN;
  //COMP4 in+ pb0(ADC3_IN12) in- pa4(dac1_ch1)  out TIM8 BRK
  COMP4->CSR = COMP_CSR_COMPxINSEL_2 | COMP4_CSR_COMP4OUTSEL_0 | COMP4_CSR_COMP4OUTSEL_1 | COMP_CSR_COMPxEN;

  opamp_init();

#ifdef USB_CONNECT_PIN
  GPIO_InitStruct.Pin        = USB_CONNECT_PIN;
  GPIO_InitStruct.Mode       = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
  LL_GPIO_Init(USB_CONNECT_PORT, &GPIO_InitStruct);
  LL_GPIO_SetOutputPin(USB_CONNECT_PORT, USB_CONNECT_PIN);
#endif

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1 | LL_AHB1_GRP1_PERIPH_DMA2);
  LL_RCC_EnableRTC();

  adc_calibrate();
  opamp_calibrate();

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);  // reset config: CRC-32, init 0xFFFFFFFF, no reversal

  //IO pins
  GPIO_InitStruct.Pin        = LL_GPIO_PIN_9 | LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode       = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  opamp_start();

  LL_TIM_OC_SetCompareCH1(TIM8, 0);
  LL_TIM_OC_SetCompareCH2(TIM8, 0);
  LL_TIM_OC_SetCompareCH3(TIM8, 0);

  adc_start();
  dac_start();
  tim8_start();

  hal_init(1.0 / 15000.0, 0.0);
  // hal load comps
  hal_parse("debug_level 1");
  hal_parse("load term");
  // load_comp(comp_by_name("sim"));
  hal_parse("load io");
  hal_parse("load ls");
  hal_parse("load dq");
  hal_parse("load idq");
  hal_parse("load svm");
  hal_parse("load hv");
  hal_parse("load curpid");

  hal_parse("ls0.rt_prio = 0.6");
  hal_parse("io0.rt_prio = 1.0");
  hal_parse("dq0.rt_prio = 2.0");
  hal_parse("curpid0.rt_prio = 3.0");
  hal_parse("idq0.rt_prio = 4.0");
  hal_parse("svm0.rt_prio = 5.0");
  hal_parse("hv0.rt_prio = 6.0");

  hal_parse("term0.send_step = 0.0");

  //link LS
  hal_parse("ls0.mot_temp = io0.mot_temp");
  hal_parse("ls0.dc_volt = io0.udc");
  hal_parse("ls0.hv_temp = io0.hv_temp");
  hal_parse("ls0.fault_in = io0.fault");
  hal_parse("io0.led = ls0.fault");
  hal_parse("curpid0.id_cmd = ls0.d_cmd");
  hal_parse("curpid0.iq_cmd = ls0.q_cmd");
  hal_parse("idq0.pos = ls0.pos");
  hal_parse("idq0.mode = ls0.phase_mode");
  hal_parse("idq0.si = dq0.si");  // dq0 runs first on the same ls0.pos
  hal_parse("idq0.co = dq0.co");
  hal_parse("idq0.ext_sc = 1");
  hal_parse("dq0.pos = ls0.pos");
  hal_parse("dq0.mode = ls0.phase_mode");
  hal_parse("io0.hv_en = ls0.en");
  hal_parse("io0.dac = ls0.dac");

  //ADC TEST
  hal_parse("hv0.udc = io0.udc");
  hal_parse("dq0.u = io0.iu");
  hal_parse("dq0.v = io0.iv");
  hal_parse("dq0.w = io0.iw");

  hal_parse("svm0.u = idq0.u");
  hal_parse("svm0.v = idq0.v");
  hal_parse("svm0.w = idq0.w");
  hal_parse("hv0.u = svm0.su");
  hal_parse("hv0.v = svm0.sv");
  hal_parse("hv0.w = svm0.sw");
  hal_parse("svm0.udc = io0.udc");
  hal_parse("curpid0.id_fb = dq0.d");
  hal_parse("curpid0.iq_fb = dq0.q");
  hal_parse("ls0.id_fb = dq0.d");
  hal_parse("ls0.iq_fb = dq0.q");
  hal_parse("ls0.ud_fb = curpid0.ud");
  hal_parse("ls0.uq_fb = curpid0.uq");
  hal_parse("ls0.y = dq0.y");
  hal_parse("ls0.u_fb = io0.u");
  hal_parse("ls0.v_fb = io0.v");
  hal_parse("ls0.w_fb = io0.w");
  hal_parse("idq0.d = curpid0.ud");
  hal_parse("idq0.q = curpid0.uq");
  hal_parse("curpid0.r = ls0.r");
  hal_parse("curpid0.ld = ls0.l");
  hal_parse("curpid0.lq = ls0.lq");  // ls0.lq falls back to ls0.l
  hal_parse("curpid0.psi = ls0.psi");
  hal_parse("curpid0.cur_bw = ls0.cur_bw");
  hal_parse("curpid0.ff = ls0.cur_ff");
  hal_parse("curpid0.kind = ls0.cur_ind");
  hal_parse("curpid0.max_cur = ls0.max_cur");
  hal_parse("curpid0.pwm_volt = ls0.pwm_volt");
  hal_parse("curpid0.vel = ls0.vel");
  hal_parse("curpid0.en = ls0.en");
  hal_parse("curpid0.cmd_mode = ls0.cmd_mode");
  hal_parse("hv0.arr = ls0.arr");
  hal_parse("hv0.drop = ls0.drop");
  hal_parse("hv0.drop_k = ls0.drop_k");
  hal_parse("io0.ignore_fault_pin = ls0.ignore_fault_pin");
  // hal_parse("load sensorless");
  // hal_parse("sensorless0.rt_prio = 7");
  // hal_parse("sensorless0.r = ls0.r");
  // hal_parse("sensorless0.l = ls0.l");
  // hal_parse("sensorless0.id = dq0.d");
  // hal_parse("sensorless0.iq = dq0.q");
  // hal_parse("sensorless0.ud = curpid0.ud");
  // hal_parse("sensorless0.uq = curpid0.uq");
  // The dead time compensation's sign comes from the COMMAND, not from io0.
  // Keyed on measured current it is a feedback path: near zero the fixed
  // compensation exceeds the real drop and drives the current it reads, which
  // split the r test's dwells and tripped the next run on the bench. hv0 builds
  // the reference phase currents from the f4's d/q command and dq0's sin/cos,
  // so no second transform runs in the rt (an idq1 here is the suspect for the
  // f3 images that never came up). Volt mode switches it off (see hv.c).
  hal_parse("hv0.d_cmd = ls0.d_cmd");
  hal_parse("hv0.q_cmd = ls0.q_cmd");
  hal_parse("hv0.si = dq0.si");
  hal_parse("hv0.co = dq0.co");
  hal_parse("hv0.cmd_mode = ls0.cmd_mode");
  hal_parse("hv0.phase_mode = ls0.phase_mode");

  hal_parse("debug_level 0");

  // hal parse config
  // hal_init_nrt();
  // error foo
  hal_start();
  hal_init_watchdog(0.005);

  while(1) {
    hal_run_nrt();
    cdc_poll();
    delay_ms(1);
  }
}

/** System Clock Configuration
*/
void Error_Handler(void) {
  /* User can add his own implementation to report the HAL error return state */
  while(1) {
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8);
  }
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}

#endif
