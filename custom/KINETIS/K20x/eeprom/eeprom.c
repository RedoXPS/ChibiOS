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
 * @file    eeprom.c
 * @brief   EEPROM Driver code.
 *
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

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   I2C Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void eepromInit(void) {
  eeprom_lld_init();
}

/**
 * @brief   Initializes the standard part of a @p EEPROMDriver structure.
 *
 * @param[out] eepp     pointer to the @p EEPROMDriver object
 *
 * @init
 */
void eepromObjectInit(EEPROMDriver *eepp) {

  eepp->state  = EEPROM_STOP;
  eepp->config = NULL;

#if EEPROM_USE_MUTUAL_EXCLUSION
  osalMutexObjectInit(&eepp->mutex);
#endif /* EEPROM_USE_MUTUAL_EXCLUSION */

#if defined(EEPROM_DRIVER_EXT_INIT_HOOK)
  EEPROM_DRIVER_EXT_INIT_HOOK(eepp);
#endif
}

/**
 * @brief   Configures and activates the EEPROM peripheral.
 *
 * @param[in] eepp      pointer to the @p EEPROMDriver object
 * @param[in] config    pointer to the @p EEPROMConfig object
 *
 * @api
 */
void eepromStart(EEPROMDriver *eepp, const EEPROMConfig *config) {

  osalDbgCheck((eepp != NULL) && (config != NULL));
  osalDbgAssert((eepp->state == EEPROM_STOP) ||
                (eepp->state == EEPROM_READY) ||
                (eepp->state == EEPROM_LOCKED), "invalid state");

  osalSysLock();
  eepp->config = config;
  eeprom_lld_start(eepp);
  if(eepp->errors == EEPROM_ERR_NO_ERROR)
    eepp->state = EEPROM_READY;
  osalSysUnlock();
}

/**
 * @brief   Deactivates the I2C peripheral.
 *
 * @param[in] eepp      pointer to the @p EEPROMDriver object
 *
 * @api
 */
void eepromStop(EEPROMDriver *eepp) {

  osalDbgCheck(eepp != NULL);
  osalDbgAssert((eepp->state == EEPROM_STOP) ||
                (eepp->state == EEPROM_READY) ||
                (eepp->state == EEPROM_LOCKED), "invalid state");

  osalSysLock();
  eeprom_lld_stop(eepp);
  eepp->state = EEPROM_STOP;
  osalSysUnlock();
}

/**
 * @brief   Returns the errors mask associated to the previous operation.
 *
 * @param[in] eepp      pointer to the @p EEPROMDriver object
 * @return              The errors mask.
 *
 * @api
 */
eeprom_errors_t eepromGetErrors(EEPROMDriver *eepp) {

  osalDbgCheck(eepp != NULL);
  osalDbgAssert(eepp->state == EEPROM_READY, "invalid state");

  return eeprom_lld_get_errors(eepp);
}

uint8_t  eepromReadByte( EEPROMDriver *eepp, const uint8_t *addr) {
  osalDbgCheck(eepp != NULL);
  osalDbgAssert(eepp->state == EEPROM_READY, "invalid state");
  return eeprom_lld_read_byte(eepp,addr);
}
uint16_t eepromReadWord( EEPROMDriver *eepp, const uint16_t *addr) {
  osalDbgCheck(eepp != NULL);
  osalDbgAssert(eepp->state == EEPROM_READY, "invalid state");
  return eeprom_lld_read_word(eepp,addr);
}
uint32_t eepromReadDWord(EEPROMDriver *eepp, const uint32_t *addr) {
  osalDbgCheck(eepp != NULL);
  osalDbgAssert(eepp->state == EEPROM_READY, "invalid state");
  return eeprom_lld_read_dword(eepp,addr);
}
float eepromReadFloat(EEPROMDriver *eepp, const float *addr) {
  osalDbgCheck(eepp != NULL);
  osalDbgAssert(eepp->state == EEPROM_READY, "invalid state");
  return eeprom_lld_read_float(eepp,addr);
}
void eepromReadBlock(EEPROMDriver *eepp, const void *addr, void *buf,
                                                                 uint32_t len) {
  osalDbgCheck(eepp != NULL);
  osalDbgAssert(eepp->state == EEPROM_READY, "invalid state");
  return eeprom_lld_read_block(eepp,addr,buf,len);
}
void eepromWriteByte(EEPROMDriver *eepp, const uint8_t *addr, uint8_t value) {
  osalDbgCheck(eepp != NULL);
  osalDbgAssert(eepp->state == EEPROM_READY, "invalid state");
  eeprom_lld_write_byte(eepp,addr,value);
}
void eepromWriteWord(EEPROMDriver *eepp, const uint16_t *addr, uint16_t value) {
  osalDbgCheck(eepp != NULL);
  osalDbgAssert(eepp->state == EEPROM_READY, "invalid state");
  eeprom_lld_write_word(eepp,addr,value);
}
void eepromWriteDWord(EEPROMDriver *eepp, const uint32_t *addr,
                                                               uint32_t value) {
  osalDbgCheck(eepp != NULL);
  osalDbgAssert(eepp->state == EEPROM_READY, "invalid state");
  eeprom_lld_write_dword(eepp,addr,value);
}
void eepromWriteFloat(EEPROMDriver *eepp, const float *addr, float value) {
  osalDbgCheck(eepp != NULL);
  osalDbgAssert(eepp->state == EEPROM_READY, "invalid state");
  eeprom_lld_write_float(eepp,addr,value);
}
void eepromWriteBlock(EEPROMDriver *eepp, const void *addr, const void *buf,
                                                                 uint32_t len) {
  osalDbgCheck(eepp != NULL);
  osalDbgAssert(eepp->state == EEPROM_READY, "invalid state");
  eeprom_lld_write_block(eepp,addr,buf,len);
}
#if EEPROM_USE_MUTUAL_EXCLUSION || defined(__DOXYGEN__)
/**
 * @brief   Gains exclusive access to the EEPROM
 * @details This function tries to gain ownership to the EEPROM, if it
 *          is already being used then the invoking thread is queued.
 * @pre     In order to use this function the option
 *          @p EEPROM_USE_MUTUAL_EXCLUSION must be enabled.
 *
 * @param[in] eepp      pointer to the @p EEPROMDriver object
 *
 * @api
 */
void eepromAcquire(EEPROMDriver *eepp) {

  osalDbgCheck(eepp != NULL);

#if CH_CFG_USE_MUTEXES
  chMtxLock(&eepp->mutex);
#elif CH_CFG_USE_SEMAPHORES
  chSemWait(&eepp->semaphore);
#endif
}

/**
 * @brief   Releases exclusive access to the EEPROM.
 * @pre     In order to use this function the option
 *          @p EEPROM_USE_MUTUAL_EXCLUSION must be enabled.
 *
 * @param[in] eepp      pointer to the @p EEPROMDriver object
 *
 * @api
 */
void eepromRelease(EEPROMDriver *eepp) {

  osalDbgCheck(eepp != NULL);

#if CH_CFG_USE_MUTEXES
  chMtxUnlock(&eepp->mutex);
#elif CH_CFG_USE_SEMAPHORES
  chSemSignal(&eepp->semaphore);
#endif
}
#endif /* EEPROM_USE_MUTUAL_EXCLUSION */

#endif /* HAL_USE_EEPROM */

/** @} */
