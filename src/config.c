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
#include "stm32f4xx_hal.h"

char config[15 * 1024];
const char *config_ro = (char *)0x08008000;

void confcrc(char *ptr) {
  uint32_t len = strnlen(config, sizeof(config) - 1);
  CRC->CR = CRC_CR_RESET;
  uint32_t crc = crc_calc_block((uint32_t *)config, len / 4);
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
  HAL_FLASH_Unlock();
  uint32_t sector_error;
  FLASH_EraseInitTypeDef erase = {.TypeErase = FLASH_TYPEERASE_SECTORS, .Sector = FLASH_SECTOR_2, .NbSectors = 1, .VoltageRange = FLASH_VOLTAGE_RANGE_3};
  if(HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK) {
    printf("error!\n");
    HAL_FLASH_Lock();
    return;
  }
  printf("saving conf\n");
  int i   = 0;
  int ret = 0;
  do {
    ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uint32_t)(config_ro + i), config[i]) != HAL_OK;
    if(ret) {
      printf("error writing %i\n", ret);
      break;
    }
  } while(config[i++] != 0);
  printf("OK %i bytes written\n", i);
  HAL_FLASH_Lock();
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

void hardboot(char *ptr) {
  printf("erasing flash page...\n");
  HAL_FLASH_Unlock();
  uint32_t sector_error;
  FLASH_EraseInitTypeDef erase = {.TypeErase = FLASH_TYPEERASE_SECTORS, .Sector = FLASH_SECTOR_4, .NbSectors = 1, .VoltageRange = FLASH_VOLTAGE_RANGE_3};
  if(HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK) {
    printf("error!\n");
    HAL_FLASH_Lock();
    return;
  }
  printf("OK, call bootloader\n");
  HAL_FLASH_Lock();
  NVIC_SystemReset();
}
COMMAND("hardboot", hardboot, "destroy firmware to force bootloader");