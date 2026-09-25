#include "f4_ll_util.h"

#define FLASH_KEY1 0x45670123U
#define FLASH_KEY2 0xCDEF89ABU
#define FLASH_SR_ERR (FLASH_SR_OPERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR)

static int flash_wait(void) {
  while(FLASH->SR & FLASH_SR_BSY) {
  }
  if(FLASH->SR & FLASH_SR_ERR) {
    FLASH->SR = FLASH_SR_ERR;  // write 1 to clear
    return -1;
  }
  return 0;
}

int flash_unlock(void) {
  if(FLASH->CR & FLASH_CR_LOCK) {
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
  }
  return (FLASH->CR & FLASH_CR_LOCK) ? -1 : 0;
}

void flash_lock(void) {
  FLASH->CR |= FLASH_CR_LOCK;
}

int flash_erase_sector(uint32_t sector) {
  if(flash_wait()) {
    return -1;
  }
  FLASH->CR &= ~(FLASH_CR_PSIZE | FLASH_CR_SNB);
  FLASH->CR |= FLASH_CR_PSIZE_1 | (sector << FLASH_CR_SNB_Pos) | FLASH_CR_SER;  // x32 parallelism
  FLASH->CR |= FLASH_CR_STRT;
  int ret = flash_wait();
  FLASH->CR &= ~(FLASH_CR_SER | FLASH_CR_SNB);
  return ret;
}

int flash_program_byte(uint32_t addr, uint8_t data) {
  if(flash_wait()) {
    return -1;
  }
  FLASH->CR &= ~FLASH_CR_PSIZE;  // x8
  FLASH->CR |= FLASH_CR_PG;
  *(__IO uint8_t *)addr = data;
  int ret = flash_wait();
  FLASH->CR &= ~FLASH_CR_PG;
  return ret;
}
