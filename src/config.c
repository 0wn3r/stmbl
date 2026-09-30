/*
* This file is part of the stmbl project.
*
* Copyright (C) 2013-2015 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2013-2015 Nico Stute <crinq@crinq.de>
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

#include <stdio.h>
#include <string.h>
#include "hal.h"
#include "commands.h"
#include "stm32f4xx_conf.h"
#include "main.h"

char config[15 * 1024];
const char *config_ro = (char *)0x08008000;

void confcrc(char *ptr) {
  uint32_t len = strnlen(config, sizeof(config) - 1);
  CRC_ResetDR();
  uint32_t crc = CRC_CalcBlockCRC((uint32_t *)config, len / 4);
  for(int i = 0; i < len; i++) {
    printf("%x ", config[i]);
  }
  printf("\n");
  printf("size: %lu words: %lu crc:%lx\n", len, len / 4, crc);
}
COMMAND("confcrc", confcrc, "foo");

void flashloadconf(char *ptr) {
  strncpy(config, config_ro, sizeof(config));
}
COMMAND("flashloadconf", flashloadconf, "load config from flash");

void flashsaveconf(char *ptr) {
  printf("erasing flash page...\n");
  FLASH_Unlock();
  // a stale error flag from an earlier access fails every program below
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
  if(FLASH_EraseSector(FLASH_Sector_2, VoltageRange_3) != FLASH_COMPLETE) {
    printf("error!\n");
    FLASH_Lock();
    return;
  }
  printf("saving conf\n");
  int i   = 0;
  int ret = 0;
  do {
    ret = FLASH_ProgramByte((uint32_t)(config_ro + i), config[i]) != FLASH_COMPLETE;
    if(ret) {
      printf("error writing %i\n", ret);
      break;
    }
  } while(config[i++] != 0);
  printf("OK %i bytes written\n", i);
  FLASH_Lock();
}
COMMAND("flashsaveconf", flashsaveconf, "save config to flash");

void loadconf(char *ptr) {
  hal_parse(config);
}
COMMAND("loadconf", loadconf, "parse config");

void showconf(char *ptr) {
  printf("%s", config_ro);
}
COMMAND("showconf", showconf, "show config");

void appendconf(char *ptr) {
  printf("adding %s\n", ptr);
  strncat(config, ptr, sizeof(config) - 1);
  strncat(config, "\n", sizeof(config) - 1);
}
COMMAND("appendconf", appendconf, "append string to config");

void deleteconf(char *ptr) {
  config[0] = '\0';
}
COMMAND("deleteconf", deleteconf, "delete config");

// Erasing sector 4 took the vector table (and maybe this code) with it while
// interrupts were live: the next interrupt fetched 0xFFFFFFFF and the core
// locked up instead of resetting. Clearing one word of the image is enough:
// the bootloader's CRC check fails, so the next reset starts the ROM DFU.
// RM0090 3.6.4 allows programming 1 bits to 0 without an erase. The ROM is
// entered right away through the app's jump (main.c), because the F4
// bootloader on field boards leaves the PLL running when it jumps there and
// the ROM DFU then hangs; only the vector table's reset entry is cleared, so
// interrupts still work until the jump.
void hardboot(char *ptr) {
  printf("clearing reset vector, calling bootloader\n");
  Wait(10);  // let the text go out
  hal_stop();
  __disable_irq();
  FLASH_Unlock();
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
  FLASH_ProgramWord(0x08010004, 0);
  FLASH_Lock();
  __enable_irq();
  bootloader(0);
}
COMMAND("hardboot", hardboot, "destroy firmware to force bootloader");