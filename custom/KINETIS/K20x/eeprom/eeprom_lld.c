/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2013 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    eeprom_lld.c
 * @brief   KINETIS EEPROM subsystem low level driver source.
 *
 * @addtogroup ADC
 * @{
 */

#include "hal.h"

#if HAL_USE_EEPROM || defined(__DOXYGEN__)

#include "eeprom.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/
EEPROMDriver EEPROM;

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level EEPROM driver initialization.
 *
 * @notapi
 */
void eeprom_lld_init(void) {
  /* Driver initialization.*/
  eepromObjectInit(&EEPROM);
}

/**
 * @brief   Configures and activates the EEPROM peripheral.
 *
 * @param[in] eepp      pointer to the @p EEPROMDriver object
 *
 * @notapi
 */
void eeprom_lld_start(EEPROMDriver *eepp) {
  eepp->errors = EEPROM_ERR_NO_ERROR;
  if (eepp->state == EEPROM_STOP) {
    /* Enables the peripheral.*/
    //~ if (&EEPROM == eepp) {
    /*
    void do_flash_cmd(volatile uint8_t *fstat)
    {
      *fstat = 0x80;
      while ((*fstat & 0x80) == 0) ; // wait
    }
    00000000 <do_flash_cmd>:
       0:	f06f 037f 	mvn.w	r3, #127	; 0x7f
       4:	7003      	strb	r3, [r0, #0]
       6:	7803      	ldrb	r3, [r0, #0]
       8:	f013 0f80 	tst.w	r3, #128	; 0x80
       c:	d0fb      	beq.n	6 <do_flash_cmd+0x6>
       e:	4770      	bx	lr
    */
    uint16_t do_flash_cmd[] = {
      0xf06f, 0x037f, 0x7003, 0x7803,
      0xf013, 0x0f80, 0xd0fb, 0x4770};
    uint8_t status;

    uint8_t eesize=0x0F;  // Default to 0
      // The EEPROM is really RAM with a hardware-based backup system to
      // flash memory.  Selecting a smaller size EEPROM allows more wear
      // leveling, for higher write endurance.  If you edit this file,
      // set this to the smallest size your application can use.  Also,
      // due to Freescale's implementation, writing 16 or 32 bit words
      // (aligned to 2 or 4 byte boundaries) has twice the endurance
      // compared to writing 8 bit bytes.
                                  // Minimum EEPROM Endurance
                                  // ------------------------
    switch(eepp->config->size)
    {
      case 2048:
        eesize = 0x03;            // 35000 writes/byte or 70000 writes/word
        break;
      case 1024:
        eesize = 0x04;            // 155000 writes/byte or 310000 writes/word
        break;
      case 512:
        eesize = 0x05;	          // 155000 writes/byte or 310000 writes/word
        break;
      case 256:
        eesize = 0x06;            // 315000 writes/byte or 630000 writes/word
        break;
      case 128:
        eesize = 0x07;	          // 635000 writes/byte or 1270000 writes/word
        break;
      case 64:
        eesize = 0x08;	         // 1275000 writes/byte or 2550000 writes/word
        break;
      case 32:
        eesize = 0x09;	         // 2555000 writes/byte or 5110000 writes/word
        break;
      default:
        break;
    }
      // FlexRAM is configured as traditional RAM
      // We need to reconfigure for EEPROM usage
    if (FTFL->FCNFG & FTFL_FCNFG_RAMRDY) {

      FTFL->FCCOB0 = 0x80;    // PGMPART = Program Partition Command
      FTFL->FCCOB4 = 0x30|eesize; // EEPROM Size
      // 0K for Dataflash, 32K for EEPROM backup when using EEPROM
      // 32K for Dataflash, 0K for EEPROM backup when not using EEPROM
      FTFL->FCCOB5 = eesize < 0xF ? 0x03 : 0x00;
      __disable_irq();
      // do_flash_cmd() must execute from RAM.
      (*((void (*)(volatile uint8_t *))(
                                    (uint32_t)do_flash_cmd | 1)))(&FTFL->FSTAT);
      __enable_irq();
      status = FTFL->FSTAT;
      if (status & (FTFL_FSTAT_RDCOLERR|FTFL_FSTAT_ACCERR|FTFL_FSTAT_FPVIOL))
      {
        FTFL->FSTAT = (status &
                     (FTFL_FSTAT_RDCOLERR|FTFL_FSTAT_ACCERR|FTFL_FSTAT_FPVIOL));
        eepp->errors = EEPROM_ERR_CONFIG;
        return; // error
      }
    }
    else // Check if the configured size matches the config value
    {
      FTFL->FCCOB0 = 0x03; // RDRSRC = Read Resource Command
      FTFL->FCCOB1 = 0x80; // [23:16]  [23] == 1 => Data Flash resources;
      FTFL->FCCOB2 = 0x00; // [15:8]: 0x00
      FTFL->FCCOB3 = 0xFC; // [7:0], [1:0] must be 00, so alignb 0xFD to 0xFC
      FTFL->FCCOB8 = 0x00; // Resource Select code: 0x00 => IFR
      __disable_irq();
      // do_flash_cmd() must execute from RAM.
      (*((void (*)(volatile uint8_t *))(
                                    (uint32_t)do_flash_cmd | 1)))(&FTFL->FSTAT);
      __enable_irq();
      status = FTFL->FSTAT;
      if (status & (FTFL_FSTAT_RDCOLERR|FTFL_FSTAT_ACCERR|FTFL_FSTAT_FPVIOL))
      {
        FTFL->FSTAT = (status &
                     (FTFL_FSTAT_RDCOLERR|FTFL_FSTAT_ACCERR|FTFL_FSTAT_FPVIOL));
        eepp->errors = EEPROM_ERR_CONFIG;
        return; // error
      }
      if((FTFL->FCCOB6&0xF) != eesize)
      {
        eepp->errors = EEPROM_ERR_CONFIG;
        return;
      }
    }
  }
  /* Configures the peripheral.*/
  while(!eeprom_lld_ready());
}

/**
 * @brief   Deactivates the EEPROM peripheral.
 *
 * @param[in] eepp      pointer to the @p EEPROMDriver object
 *
 * @notapi
 */
void eeprom_lld_stop(EEPROMDriver *eepp) {

  if (eepp->state == EEPROM_READY) {
    /* Resets the peripheral.*/

    /* Disables the peripheral.*/
    //~ if (&EEPROM == eepp) {

    //~ }
  }
}

uint8_t eeprom_lld_read_byte(EEPROMDriver *eepp, const uint8_t *addr)
{
	uint32_t offset = (uint32_t)addr;
	if (offset < eepp->config->size)
  {
    eeprom_lld_wait_until_ready();
    return FlexRAM[offset];
  }
  // Throw error
  return 0;
}

uint16_t eeprom_lld_read_word(EEPROMDriver *eepp, const uint16_t *addr)
{
	uint32_t offset = (uint32_t)addr;
	if (offset < eepp->config->size-1)
  {
    eeprom_lld_wait_until_ready();
    return *(uint16_t *)(&FlexRAM[offset]);
  }
  // Throw error
  return 0;
}

uint32_t eeprom_lld_read_dword(EEPROMDriver *eepp, const uint32_t *addr)
{
	uint32_t offset = (uint32_t)addr;
	if (offset < eepp->config->size-3)
  {
    eeprom_lld_wait_until_ready();
    return *(uint32_t *)(&FlexRAM[offset]);
  }
  // Throw error
  return 0;
}

float eeprom_lld_read_float(EEPROMDriver *eepp, const float *addr)
{
	union {float f; uint32_t u32;} u;
	u.u32 = eeprom_lld_read_dword(eepp,(const uint32_t *)addr);
	return u.f;
}

void eeprom_lld_read_block(EEPROMDriver *eepp, const void *addr,
                                                        void *buf, uint32_t len)
{
	uint32_t offset = (uint32_t)addr;
	uint8_t *dest = (uint8_t *)buf;
	uint32_t end = offset + len;

	eeprom_lld_wait_until_ready();
	if (end <= eepp->config->size)
  {
    while (offset < end) {
      *dest++ = FlexRAM[offset++];
    }
  }
  else
  {
    // Throw error
  }
}

void eeprom_lld_write_byte(EEPROMDriver *eepp, const uint8_t *addr,
                                                                  uint8_t value)
{
	uint32_t offset = (uint32_t)addr;

	if (offset < eepp->config->size)
  {
    eeprom_lld_wait_until_ready();
    if (FlexRAM[offset] != value)
    {
      FlexRAM[offset] = value;
      eeprom_lld_wait_until_ready();
    }
  }
  else
  {
    // Throw error
  }
}

void eeprom_lld_write_word(EEPROMDriver *eepp, const uint16_t *addr,
                                                                 uint16_t value)
{
	uint32_t offset = (uint32_t)addr;

	if (offset < eepp->config->size-1)
  {
      eeprom_lld_wait_until_ready();
  #ifdef HANDLE_UNALIGNED_WRITES
    if ((offset & 1) == 0) {
  #endif
      if (*(uint16_t *)(&FlexRAM[offset]) != value) {
        *(uint16_t *)(&FlexRAM[offset]) = value;
        eeprom_lld_wait_until_ready();
      }
  #ifdef HANDLE_UNALIGNED_WRITES
    } else {
      if (FlexRAM[offset] != value) {
        FlexRAM[offset] = value;
        eeprom_lld_wait_until_ready();
      }
      if (FlexRAM[offset + 1] != (value >> 8)) {
        FlexRAM[offset + 1] = value >> 8;
        eeprom_lld_wait_until_ready();
      }
    }
  #endif
  }
  else
  {
    // Throw error
  }
}

void eeprom_lld_write_dword(EEPROMDriver *eepp, const uint32_t *addr,
                                                                 uint32_t value)
{
	uint32_t offset = (uint32_t)addr;

	if (offset < eepp->config->size-3)
  {
    eeprom_lld_wait_until_ready();
  #ifdef HANDLE_UNALIGNED_WRITES
    switch (offset & 3) {
    case 0:
  #endif
      if (*(uint32_t *)(&FlexRAM[offset]) != value) {
        *(uint32_t *)(&FlexRAM[offset]) = value;
        eeprom_lld_wait_until_ready();
      }
      return;
  #ifdef HANDLE_UNALIGNED_WRITES
    case 2:
      if (*(uint16_t *)(&FlexRAM[offset]) != value) {
        *(uint16_t *)(&FlexRAM[offset]) = value;
        eeprom_lld_wait_until_ready();
      }
      if (*(uint16_t *)(&FlexRAM[offset + 2]) != (value >> 16)) {
        *(uint16_t *)(&FlexRAM[offset + 2]) = value >> 16;
        eeprom_lld_wait_until_ready();
      }
      return;
    default:
      if (FlexRAM[offset] != value) {
        FlexRAM[offset] = value;
        eeprom_lld_wait_until_ready();
      }
      if (*(uint16_t *)(&FlexRAM[offset + 1]) != (value >> 8)) {
        *(uint16_t *)(&FlexRAM[offset + 1]) = value >> 8;
        eeprom_lld_wait_until_ready();
      }
      if (FlexRAM[offset + 3] != (value >> 24)) {
        FlexRAM[offset + 3] = value >> 24;
        eeprom_lld_wait_until_ready();
      }
    }
  #endif
  }
  else
  {
    // Throw error
  }
}

void eeprom_lld_write_block(EEPROMDriver *eepp, const void *addr,
                                                  const void *buf, uint32_t len)
{
	uint32_t offset = (uint32_t)addr;
	const uint8_t *src = (const uint8_t *)buf;

	if (offset < eepp->config->size)
  {
    eeprom_lld_wait_until_ready();
    if (len < eepp->config->size && offset + len < eepp->config->size)
    {
      while (len > 0) {
        uint32_t lsb = offset & 3;
        if (lsb == 0 && len >= 4) {
          // write aligned 32 bits
          uint32_t val32;
          val32 = *src++;
          val32 |= (*src++ << 8);
          val32 |= (*src++ << 16);
          val32 |= (*src++ << 24);
          if (*(uint32_t *)(&FlexRAM[offset]) != val32) {
            *(uint32_t *)(&FlexRAM[offset]) = val32;
            eeprom_lld_wait_until_ready();
          }
          offset += 4;
          len -= 4;
        } else if ((lsb == 0 || lsb == 2) && len >= 2) {
          // write aligned 16 bits
          uint16_t val16;
          val16 = *src++;
          val16 |= (*src++ << 8);
          if (*(uint16_t *)(&FlexRAM[offset]) != val16) {
            *(uint16_t *)(&FlexRAM[offset]) = val16;
            eeprom_lld_wait_until_ready();
          }
          offset += 2;
          len -= 2;
        } else {
          // write 8 bits
          uint8_t val8 = *src++;
          if (FlexRAM[offset] != val8) {
            FlexRAM[offset] = val8;
            eeprom_lld_wait_until_ready();
          }
          offset++;
          len--;
        }
      }
    }
    else
    {
      // Throw error
    }
  }
  else
  {
    // Throw error
  }
}

void eeprom_lld_write_float(EEPROMDriver *eepp, const float *addr, float value)
{
	union {float f; uint32_t u32;} u;
	u.f = value;
	eeprom_lld_write_dword(eepp,(uint32_t *)addr, u.u32);
}

#endif /* HAL_USE_EEPROM */

/** @} */
