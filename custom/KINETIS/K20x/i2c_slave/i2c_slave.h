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
 * @file    i2c_slave.h
 * @brief   I2C Slave Driver macros and structures.
 *
 * @addtogroup I2C
 * @{
 */

#ifndef _I2C_SLAVE_H_
#define _I2C_SLAVE_H_

#if HAL_USE_I2C_SLAVE || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/* TODO: To be reviewed, too STM32-centric.*/
/**
 * @name    I2C bus error conditions
 * @{
 */
typedef enum {
  I2C_SLAVE_NO_ERROR         = 0x00,       /**< @brief No error               */
  //~ I2C_SLAVE_BUS_ERROR        = 0x01,       /**< @brief Bus Error              */
  I2C_SLAVE_ARBITRATION_LOST = 0x02,       /**< @brief Arbitration Lost       */
  //~ I2C_SLAVE_ACK_FAILURE      = 0x04,       /**< @brief Acknowledge Failure.   */
  //~ I2C_SLAVE_OVERRUN          = 0x08,       /**< @brief Overrun/Underrun.      */
  //~ I2C_SLAVE_PEC_ERROR        = 0x10,       /**< @brief PEC Error in reception */
  I2C_SLAVE_TIMEOUT          = 0x20,       /**< @brief Hardware timeout.      */
  //~ I2C_SLAVE_SMB_ALERT        = 0x40,       /**< @brief SMBus Alert.           */
} i2c_slave_error_flags_t;

/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Enables the mutual exclusion APIs on the I2C bus.
 */
#if !defined(I2C_USE_MUTUAL_EXCLUSION) || defined(__DOXYGEN__)
#define I2C_USE_MUTUAL_EXCLUSION    TRUE
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if I2C_USE_MUTUAL_EXCLUSION && !CH_CFG_USE_MUTEXES && !CH_CFG_USE_SEMAPHORES
#error "I2C_USE_MUTUAL_EXCLUSION requires "\
       "CH_CFG_USE_MUTEXES and/or CH_CFG_USE_SEMAPHORES"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  I2C_SLAVE_UNINIT = 0,                       /**< Not initialized.           */
  I2C_SLAVE_STOP = 1,                         /**< Stopped.                   */
  I2C_SLAVE_READY = 2,                        /**< Ready.                     */
  I2C_SLAVE_ACTIVE_TX = 3,                    /**< Transmitting.              */
  I2C_SLAVE_ACTIVE_RX = 4,                    /**< Receiving.                 */
  I2C_SLAVE_LOCKED = 5                        /**> Bus or driver locked.      */
} i2c_slave_state_t;

#include "i2c_slave_lld.h"

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Wakes up the waiting thread notifying no errors.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
#define _i2c_slave_wakeup_isr(i2cp) do {                                          \
  osalSysLockFromISR();                                                     \
  osalThreadResumeI(&(i2cp)->thread, MSG_OK);                               \
  osalSysUnlockFromISR();                                                   \
} while(0)

/**
 * @brief   Wakes up the waiting thread notifying errors.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
#define _i2c_slave_wakeup_error_isr(i2cp) do {                                    \
  osalSysLockFromISR();                                                     \
  osalThreadResumeI(&(i2cp)->thread, MSG_RESET);                            \
  osalSysUnlockFromISR();                                                   \
} while(0)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void i2cSlaveInit(void);
  void i2cSlaveObjectInit(I2CSlaveDriver *i2cp);
  void i2cSlaveStart(I2CSlaveDriver *i2cp, const I2CSlaveConfig *config);
  void i2cSlaveStop(I2CSlaveDriver *i2cp);
  i2c_slave_error_flags_t i2cSlaveGetErrors(I2CSlaveDriver *i2cp);

#if I2C_USE_MUTUAL_EXCLUSION
  void i2cSlaveAcquireBus(I2CSlaveDriver *i2cp);
  void i2cSlaveReleaseBus(I2CSlaveDriver *i2cp);
#endif /* I2C_USE_MUTUAL_EXCLUSION */

#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_I2C_SLAVE */

#endif /* _I2C_SLAVE_H_ */

/** @} */
