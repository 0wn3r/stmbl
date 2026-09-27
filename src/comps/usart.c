#include "usart_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "stm32f4xx_conf.h"
#include "dma_util.h"
#include "hw/hw.h"

/**
* ## Brief
* `usart` is an experimental test component for half-duplex serial absolute encoders on the fb0 connector of the F4 logic board: every rt period it sends a request byte and receives the answer by DMA, extracts a position from the answer and can print the raw bytes. No template loads it.
*
* ## Component Explanation
*
* 1. **Hardware (hw_init)**:
* - USART6 in half-duplex mode on PC6 (the fb0 UART TX line), PD15 switches the fb0 RS485 transmitter (FB0_Z_TXEN), DMA2 stream 1 receives up to 16 bytes.
*
* 2. **Request (rt)**:
* - Each rt period the USART is re-initialised with baud rate `freq` (default 2.5 Mbaud), the transmitter is enabled, the byte `req` (default 0x2A) is sent, `rt` busy-waits until it is out, the transmitter is disabled and the RX DMA restarted. The answer is evaluated in the next rt period. `dma` shows how many of the 16 bytes were not received.
*
* 3. **Position (rt)**:
* - Received bytes starting at byte `pos_offset` (default 2) are copied into a 64 bit little endian integer, masked and scaled to `pos` in rad, wrapped to -pi..pi:
* ```c
* pos = raw * 2 * pi / 2^pos_len - pi;
* ```
*
* 4. **Debug print (rt/nrt)**:
* - Setting `print` > 0 captures the 16 receive bytes (and clears the buffer), resets `print` to 0 and makes `nrt` print the request byte, the DMA count and all bytes in binary, LSB first. The next capture is possible once the print is done.
*
* {{% hint danger %}}
* Experimental and broken as it is: `pos_len` (default 20) is used as a byte count for the copy loop but as a bit count for the mask. With the default the loop writes 20 bytes into an 8 byte variable and reads past the 16 byte receive buffer (stack corruption), and the mask `(2 << pos_len) - 1` keeps one bit more than `pos_len`. It only runs without memory corruption for `pos_len` <= 8, and then the position has at most 9 bits.
* {{% /hint %}}
*/

HAL_COMP(usart);

HAL_PIN(req);        // *parameter*, request byte sent every rt period, default 0x2A
HAL_PIN(print);      // *input/output*, set > 0 to print one received packet, reset to 0 by the component
HAL_PIN(freq);       // *parameter*, baud rate, default 2500000
HAL_PIN(dma);        // *output*, dma transfers left (bytes not received of 16)

HAL_PIN(pos_offset); // *parameter*, index of the first position byte in the answer, default 2
HAL_PIN(pos_len);    // *parameter*, position length, default 20, see warning
HAL_PIN(pos);        // *output*, position (rad), -pi..pi

struct usart_ctx_t {
  uint8_t rxbuf[16];
  uint8_t rxbuf2[16];
  int8_t print;
  uint8_t dma;
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct usart_pin_ctx_t *pins = (struct usart_pin_ctx_t *)pin_ptr;
  PIN(freq)                    = 2500000;
  PIN(req)                     = 0x2A;  // 0x32
  PIN(pos_offset)              = 2;
  PIN(pos_len)                 = 20;
}


static void hw_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct usart_ctx_t *ctx = (struct usart_ctx_t *)ctx_ptr;
  // struct usart_pin_ctx_t * pins = (struct usart_pin_ctx_t *)pin_ptr;
  GPIO_InitTypeDef GPIO_InitStruct;
  DMA_InitTypeDef DMA_InitStructure;

  //TX enable
  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_15;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStruct);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

  //USART TX
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_6;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStruct);

  // DMA-Disable
  dma_stream_stop(DMA2_Stream1);
  DMA_DeInit(DMA2_Stream1);

  // DMA2-Config
  DMA_InitStructure.DMA_Channel            = DMA_Channel_5;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART6->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)&ctx->rxbuf;
  DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize         = ARRAY_SIZE(ctx->rxbuf);
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream1, &DMA_InitStructure);

  USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct usart_ctx_t *ctx      = (struct usart_ctx_t *)ctx_ptr;
  struct usart_pin_ctx_t *pins = (struct usart_pin_ctx_t *)pin_ptr;

  USART_InitTypeDef USART_InitStruct;

  PIN(dma) = DMA_GetCurrDataCounter(DMA2_Stream1);

  if(PIN(print) <= 1 && ctx->print == -1) {
    ctx->print = 0;
  }

  union {
    uint64_t pos;
    uint8_t data[8];
  } foo;

  for(int i = 0; i < PIN(pos_len); i++) {
    foo.data[i] = ctx->rxbuf[i + (int)PIN(pos_offset)];
  }

  foo.pos &= (2 << (int)PIN(pos_len)) - 1;

  PIN(pos) = foo.pos * 2.0 * M_PI / pow(2, PIN(pos_len)) - M_PI;

  if(PIN(print) > 0 && ctx->print == 0) {
    for(int i = 0; i < ARRAY_SIZE(ctx->rxbuf); i++) {
      ctx->rxbuf2[i] = ctx->rxbuf[i];
      ctx->rxbuf[i]  = 0;
    }
    ctx->dma   = PIN(dma);
    ctx->print = 1;
    PIN(print) = 0;
  }

  USART_Cmd(USART6, DISABLE);

  USART_InitStruct.USART_BaudRate            = PIN(freq);
  USART_InitStruct.USART_WordLength          = USART_WordLength_8b;
  USART_InitStruct.USART_StopBits            = USART_StopBits_1;
  USART_InitStruct.USART_Parity              = USART_Parity_No;
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStruct.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART6, &USART_InitStruct);

  /* Enable the USART */
  USART_Cmd(USART6, ENABLE);
  USART_HalfDuplexCmd(USART6, ENABLE);


  //TODO: irq here will cause problems
  GPIO_SetBits(GPIOD, GPIO_Pin_15);  //tx enable
  USART_SendData(USART6, (uint8_t)PIN(req));
  while(USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET)
    ;
  GPIO_ResetBits(GPIOD, GPIO_Pin_15);  //tx disable

  //start rx dma
  dma_stream_stop(DMA2_Stream1);
  DMA_Cmd(DMA2_Stream1, ENABLE);
}


static void nrt_func(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct usart_ctx_t *ctx      = (struct usart_ctx_t *)ctx_ptr;
  struct usart_pin_ctx_t *pins = (struct usart_pin_ctx_t *)pin_ptr;

  if(ctx->print > 0) {
    ctx->print = -1;
    printf("req: 0x%02X: rx: %u : ", (uint8_t)PIN(req), ctx->dma);
    for(int i = 0; i < ARRAY_SIZE(ctx->rxbuf); i++) {
      for(int j = 0; j < 8; j++) {
        if(ctx->rxbuf2[i] & (1 << j % 8)) {
          printf("1");
        } else {
          printf("0");
        }
      }
      printf(" ");
      //printf("%02X ", ctx->rxbuf2[i]);
    }
    printf("\n");
  }
}

hal_comp_t usart_comp_struct = {
    .name      = "usart",
    .nrt       = nrt_func,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .hw_init   = hw_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct usart_ctx_t),
    .pin_count = sizeof(struct usart_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
