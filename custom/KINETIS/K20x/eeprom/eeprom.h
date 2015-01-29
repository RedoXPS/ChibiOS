/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    eeprom.h
 * @brief   EEPROM Driver macros and structures.
 *
 * @addtogroup I2C
 * @{
 */

#ifndef _EEPROM_H_
#define _EEPROM_H_

#if HAL_USE_EEPROM || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/* TODO: To be reviewed, too KINETIS-centric.*/
/**
 * @name    EEPROM error conditions
 * @{
 */
typedef enum {
  EEPROM_NO_ERROR         = 0x00,          /**< @brief No error               */
} eeprom_errors_t;

/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Enables the mutual exclusion APIs on the I2C bus.
 */
#if !defined(EEPROM_USE_MUTUAL_EXCLUSION) || defined(__DOXYGEN__)
#define EEPROM_USE_MUTUAL_EXCLUSION    TRUE
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if EEPROM_USE_MUTUAL_EXCLUSION && !CH_CFG_USE_MUTEXES && !CH_CFG_USE_SEMAPHORES
#error "EEPROM_USE_MUTUAL_EXCLUSION requires "\
       "CH_CFG_USE_MUTEXES and/or CH_CFG_USE_SEMAPHORES"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  EEPROM_UNINIT = 0,                       /**< Not initialized.           */
  EEPROM_STOP = 1,                         /**< Stopped.                   */
  EEPROM_READY = 2,                        /**< Ready.                     */
  EEPROM_LOCKED = 5                        /**> Bus or driver locked.      */
} eeprom_state_t;

#include "eeprom_lld.h"

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void eepromInit(void);
  void eepromObjectInit(EEPROMDriver *eepp);
  void eepromStart(EEPROMDriver *eepp, const EEPROMConfig *config);
  void eepromStop(EEPROMDriver *eepp);
  eeprom_errors_t eepromGetErrors(EEPROMDriver *eepp);
  uint8_t  eepromReadByte( EEPROMDriver *eepp, const uint8_t *addr);
  uint16_t eepromReadWord( EEPROMDriver *eepp, const uint16_t *addr);
  uint32_t eepromReadDWord(EEPROMDriver *eepp, const uint32_t *addr);
  float    eepromReadFloat(EEPROMDriver *eepp, const float *addr);
  void     eepromReadBlock(EEPROMDriver *eepp, const void *addr,
                                                     void *buf,  uint32_t len);
  void eepromWriteByte( EEPROMDriver *eepp, const uint8_t *addr,
                                                                 uint8_t value);
  void eepromWriteWord( EEPROMDriver *eepp, const uint16_t *addr,
                                                                uint16_t value);
  void eepromWriteDWord(EEPROMDriver *eepp, const uint32_t *addr,
                                                                uint32_t value);
  void eepromWriteFloat(EEPROMDriver *eepp, const float *addr,
                                                                   float value);
  void eepromWriteBlock(EEPROMDriver *eepp, const void *addr,
                                                const void *buf,  uint32_t len);

#if EEPROM_USE_MUTUAL_EXCLUSION
  void eepromAcquire(EEPROMDriver *eepp);
  void eepromRelease(EEPROMDriver *eepp);
#endif /* EEPROM_USE_MUTUAL_EXCLUSION */

#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_EEPROM */

#endif /* _EEPROM_H_ */

/** @} */
