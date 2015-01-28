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
/*
   Concepts and parts of this file have been contributed by Uladzimir Pylinsky
   aka barthess.
 */

/**
 * @file    i2c_slave.c
 * @brief   I2C Slave Driver code.
 *
 * @addtogroup I2C
 * @{
 */
#include "hal.h"

#if HAL_USE_I2C_SLAVE || defined(__DOXYGEN__)

#include "i2c_slave.h"

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
void i2cSlaveInit(void) {

  i2c_slave_lld_init();
}

/**
 * @brief   Initializes the standard part of a @p I2CDriver structure.
 *
 * @param[out] i2cp     pointer to the @p I2CDriver object
 *
 * @init
 */
void i2cSlaveObjectInit(I2CSlaveDriver *i2cp) {

  i2cp->state  = I2C_SLAVE_STOP;
  i2cp->config = NULL;

#if I2C_SLAVE_USE_MUTUAL_EXCLUSION
  osalMutexObjectInit(&i2cp->mutex);
#endif /* I2C_SLAVE_USE_MUTUAL_EXCLUSION */

#if defined(I2C_SLAVE_DRIVER_EXT_INIT_HOOK)
  I2C_SLAVE_DRIVER_EXT_INIT_HOOK(i2cp);
#endif
}

/**
 * @brief   Configures and activates the I2C peripheral.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] config    pointer to the @p I2CConfig object
 *
 * @api
 */
void i2cSlaveStart(I2CSlaveDriver *i2cp, const I2CSlaveConfig *config) {

  osalDbgCheck((i2cp != NULL) && (config != NULL));
  osalDbgAssert((i2cp->state == I2C_SLAVE_STOP) ||
                (i2cp->state == I2C_SLAVE_READY) ||
                (i2cp->state == I2C_SLAVE_LOCKED), "invalid state");

  osalSysLock();
  i2cp->config = config;
  i2c_slave_lld_start(i2cp);
  i2cp->state = I2C_SLAVE_READY;
  osalSysUnlock();
}

/**
 * @brief   Deactivates the I2C peripheral.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @api
 */
void i2cSlaveStop(I2CSlaveDriver *i2cp) {

  osalDbgCheck(i2cp != NULL);
  osalDbgAssert((i2cp->state == I2C_SLAVE_STOP) ||
                (i2cp->state == I2C_SLAVE_READY) ||
                (i2cp->state == I2C_SLAVE_LOCKED), "invalid state");

  osalSysLock();
  i2c_slave_lld_stop(i2cp);
  i2cp->state = I2C_SLAVE_STOP;
  osalSysUnlock();
}

/**
 * @brief   Returns the errors mask associated to the previous operation.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @return              The errors mask.
 *
 * @api
 */
i2c_slave_error_flags_t i2cSlaveGetErrors(I2CSlaveDriver *i2cp) {

  osalDbgCheck(i2cp != NULL);

  return i2c_slave_lld_get_errors(i2cp);
}

#if I2C_USE_MUTUAL_EXCLUSION || defined(__DOXYGEN__)
/**
 * @brief   Gains exclusive access to the I2C bus.
 * @details This function tries to gain ownership to the I2C bus, if the bus
 *          is already being used then the invoking thread is queued.
 * @pre     In order to use this function the option @p I2C_USE_MUTUAL_EXCLUSION
 *          must be enabled.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @api
 */
void i2cSlaveAcquireBus(I2CSlaveDriver *i2cp) {

  osalDbgCheck(i2cp != NULL);

#if CH_CFG_USE_MUTEXES
  chMtxLock(&i2cp->mutex);
#elif CH_CFG_USE_SEMAPHORES
  chSemWait(&i2cp->semaphore);
#endif
}

/**
 * @brief   Releases exclusive access to the I2C bus.
 * @pre     In order to use this function the option @p I2C_USE_MUTUAL_EXCLUSION
 *          must be enabled.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @api
 */
void i2cSlaveReleaseBus(I2CSlaveDriver *i2cp) {

  osalDbgCheck(i2cp != NULL);

#if CH_CFG_USE_MUTEXES
  chMtxUnlock(&i2cp->mutex);
#elif CH_CFG_USE_SEMAPHORES
  chSemSignal(&i2cp->semaphore);
#endif
}
#endif /* I2C_USE_MUTUAL_EXCLUSION */

#endif /* HAL_USE_I2C_SLAVE */

/** @} */
