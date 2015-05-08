/* CMSIS-DAP Interface Firmware
 * Copyright (c) 2009-2014 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "flash_hal.h"        // FlashOS Structures
#include "stm32f10x.h" 

#define M8(adr)  (*((vu8  *) (adr)))
#define M16(adr) (*((vu16 *) (adr)))
#define M32(adr) (*((vu32 *) (adr)))


// Flash Keys
#define RDPRT_KEY       0x5AA5
#define FLASH_KEY1      0x45670123
#define FLASH_KEY2      0xCDEF89AB

// Flash Control Register definitions
#define FLASH_PG        0x00000001
#define FLASH_PER       0x00000002
#define FLASH_MER       0x00000004
#define FLASH_OPTPG     0x00000010
#define FLASH_OPTER     0x00000020
#define FLASH_STRT      0x00000040
#define FLASH_LOCK      0x00000080
#define FLASH_OPTWRE    0x00000100

// Flash Status Register definitions
#define FLASH_BSY       0x00000001
#define FLASH_PGERR     0x00000004
#define FLASH_WRPRTERR  0x00000010
#define FLASH_EOP       0x00000020
// FLASH BANK size
#define BANK1_SIZE      0x00080000      // Bank1 Size = 512kB

int flash_hal_init(uint32_t clk)
{

  // Zero Wait State
  FLASH->ACR  = 0x00000000;

  // Unlock Flash    
  FLASH->KEYR  = FLASH_KEY1;
  FLASH->KEYR  = FLASH_KEY2;
#ifdef STM32F10X_XL
  FLASH->KEYR2 = FLASH_KEY1;                    // Flash bank 2
  FLASH->KEYR2 = FLASH_KEY2;
#endif

  // Test if IWDG is running (IWDG in HW mode)
  if ((FLASH->OBR & 0x04) == 0x00) {
    // Set IWDG time out to ~32.768 second
    IWDG->KR  = 0x5555;                         // Enable write access to IWDG_PR and IWDG_RLR     
    IWDG->PR  = 0x06;                           // Set prescaler to 256  
    IWDG->RLR = 4095;                           // Set reload value to 4095
  }

    return 0;
}

int flash_hal_uninit(uint32_t fnc)
{
	   // Lock Flash
  FLASH->CR  |=  FLASH_LOCK;
#ifdef STM32F10X_XL
  FLASH->CR2 |=  FLASH_LOCK;                    // Flash bank 2
#endif
    return 0;
}

int flash_hal_erase_chip(void)
{
  FLASH->CR  |=  FLASH_MER;                     // Mass Erase Enabled
  FLASH->CR  |=  FLASH_STRT;                    // Start Erase

  while (FLASH->SR  & FLASH_BSY) {
    IWDG->KR = 0xAAAA;                          // Reload IWDG
  }

  FLASH->CR  &= ~FLASH_MER;                     // Mass Erase Disabled

#ifdef STM32F10X_XL                             // Flash bank 2
  FLASH->CR2 |=  FLASH_MER;
  FLASH->CR2 |=  FLASH_STRT;

  while (FLASH->SR2 & FLASH_BSY) {
    IWDG->KR = 0xAAAA;
  }

  FLASH->CR2 &= ~FLASH_MER;
#endif
	
    return 0;
}

int flash_hal_erase_sector(uint32_t adr)
{
#ifdef STM32F10X_XL
  if (adr < FLASH_BASE_ADDR + BANK1_SIZE) {     // Flash bank 2
#endif
    FLASH->CR  |=  FLASH_PER;                   // Page Erase Enabled 
    FLASH->AR   =  adr;                         // Page Address
    FLASH->CR  |=  FLASH_STRT;                  // Start Erase

    while (FLASH->SR  & FLASH_BSY) {
      IWDG->KR = 0xAAAA;                        // Reload IWDG
    }

    FLASH->CR  &= ~FLASH_PER;                   // Page Erase Disabled 
#ifdef STM32F10X_XL
  }
  else {                                        // Flash bank 2
    FLASH->CR2 |=  FLASH_PER;
    FLASH->AR2  =  adr;
    FLASH->CR2 |=  FLASH_STRT;

    while (FLASH->SR2 & FLASH_BSY) {
      IWDG->KR = 0xAAAA;
    }

    FLASH->CR2 &= ~FLASH_PER;
  }
#endif
	
    return 0;
}

int flash_hal_program_page(uint32_t adr, uint32_t sz, unsigned char *buf)
{
   sz = (sz + 1) & ~1;                           // Adjust size for Half Words
  
#ifdef STM32F10X_XL
  if (adr < (FLASH_BASE_ADDR + BANK1_SIZE)) {    // Flash bank 2
#endif
    while (sz) {

      FLASH->CR  |=  FLASH_PG;                  // Programming Enabled

      M16(adr) = *((unsigned short *)buf);      // Program Half Word
      while (FLASH->SR  & FLASH_BSY);

      FLASH->CR  &= ~FLASH_PG;                  // Programming Disabled

      // Check for Errors
      if (FLASH->SR  & (FLASH_PGERR | FLASH_WRPRTERR)) {
        FLASH->SR  |= FLASH_PGERR | FLASH_WRPRTERR;
        return (1);                             // Failed
      }

      // Go to next Half Word
      adr += 2;
      buf += 2;
      sz  -= 2;
    }
#ifdef STM32F10X_XL
  }
  else {                                        // Flash bank 2
    while (sz) {

      FLASH->CR2 |=  FLASH_PG;

      M16(adr) = *((unsigned short *)buf);
      while (FLASH->SR2 & FLASH_BSY);

      FLASH->CR2 &= ~FLASH_PG;

      // Check for Errors
      if (FLASH->SR2 & (FLASH_PGERR | FLASH_WRPRTERR)) {
        FLASH->SR2 |= FLASH_PGERR | FLASH_WRPRTERR;
        return (1);
      }

      // Go to next Half Word
      adr += 2;
      buf += 2;
      sz  -= 2;
    }
  }
#endif
    return 0;
}
