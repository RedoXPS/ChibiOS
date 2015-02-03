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
 * @file    i2c_slave_lld.h
 * @brief   PLATFORM I2C subsystem low level driver header.
 *
 * @addtogroup I2C
 * @{
 */

#ifndef _I2C_SLAVE_LLD_H_
#define _I2C_SLAVE_LLD_H_

#if HAL_USE_I2C_SLAVE || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/
#define STATE_STOP    0x00
#define STATE_SEND    0x01
#define STATE_RECV    0x02
#define STATE_DUMMY   0x03
/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/
/**
 * @brief   I2C0 driver enable switch.
 * @details If set to @p TRUE the support for I2C0 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(KINETIS_I2C_SLAVE_USE_I2C0) || defined(__DOXYGEN__)
#define KINETIS_I2C_SLAVE_USE_I2C0                  FALSE
#endif
/**
 * @brief   I2C1 driver enable switch.
 * @details If set to @p TRUE the support for I2C is included.
 * @note    The default is @p FALSE.
 */
#if !defined(KINETIS_I2C_SLAVE_USE_I2C1) || defined(__DOXYGEN__)
#define KINETIS_I2C_SLAVE_USE_I2C                  FALSE
#endif
/** @} */

/**
 * @brief   I2C0 interrupt priority level setting.
 */
#if !defined(KINETIS_I2C_SLAVE_I2C0_PRIORITY) || defined(__DOXYGEN__)
#define KINETIS_I2C_SLAVE_I2C0_PRIORITY        12
#endif

/**
 * @brief   I2C1 interrupt priority level setting.
 */
#if !defined(KINETIS_I2C_SLAVE_I2C1_PRIORITY) || defined(__DOXYGEN__)
#define KINETIS_I2C_SLAVE_I2C1_PRIORITY        12
#endif
/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type representing an I2C address.
 */
typedef uint16_t i2c_slave_addr_t;

typedef struct I2CSlaveDriver I2CSlaveDriver;

typedef void (*i2c_buff_cb_t)(I2CSlaveDriver *i2cp, uint8_t *buf, size_t n);
typedef void (*i2c_byte_cb_t)(I2CSlaveDriver *i2cp, uint8_t c);

/**
 * @brief   Type of I2C driver configuration structure.
 * @note    Implementations may extend this structure to contain more,
 *          architecture dependent, fields.
 */
typedef struct {
  /* End of the mandatory fields.*/
  uint32_t             clock;     /* @brief Clock to be used for the I2C bus. */

  i2c_slave_addr_t     address;   /* @brief 7bits Address of the I2C Slave    */
  i2c_byte_cb_t       start_cb;
  //~ i2c_byte_cb_t        rxbyte_cb;
  i2c_buff_cb_t       rxend_cb;
  //~ i2c_byte_cb_t        rxbyte_cb;
  i2c_buff_cb_t       txend_cb;
  //~ i2c_byte_cb_t       error_cb;
} I2CSlaveConfig;

/**
 * @brief   Structure representing an I2C driver.
 */
struct I2CSlaveDriver {
  /**
   * @brief   Driver state.
   */
  i2c_slave_state_t                state;
  /**
   * @brief   Current configuration data.
   */
  const I2CSlaveConfig           *config;
  /**
   * @brief   Error flags.
   */
  i2c_slave_error_flags_t         errors;
#if I2C_USE_MUTUAL_EXCLUSION || defined(__DOXYGEN__)
#if CH_CFG_USE_MUTEXES || defined(__DOXYGEN__)
  /**
   * @brief   Mutex protecting the bus.
   */
  mutex_t                   mutex;
#elif CH_CFG_USE_SEMAPHORES
  semaphore_t               semaphore;
#endif
#endif /* I2C_USE_MUTUAL_EXCLUSION */
#if defined(I2C_SLAVE_DRIVER_EXT_FIELDS)
  I2C_DRIVER_EXT_FIELDS
#endif
  /* @brief Thread waiting for I/O completion. */
  thread_reference_t        thread;

  /* End of the mandatory fields.*/

  /* @brief Pointer to the buffer with data to send. */
  uint8_t                   *txbuf;
  /* @brief Number of bytes of data to send. */
  size_t                    txbytes;
  /* @brief Size of the buffer for sent data  */
  size_t                    txsize;
  /* @brief Current index in buffer when sending data. */
  size_t                    txidx;
  /* @brief Pointer to the buffer to put received data. */
  uint8_t                   *rxbuf;
  /* @brief Size of the buffer for received data */
  size_t                    rxsize;
  /* @brief Number of bytes of data to receive. */
  size_t                    rxbytes;
  /* @brief Current index in buffer when receiving data. */
  size_t                    rxidx;
  /* @brief Low-level register access. */
  I2C_TypeDef               *i2c;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Get errors from I2C driver.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
#define i2c_slave_lld_get_errors(i2cp) ((i2cp)->errors)

// TODO: Fix rxbytes later with an accessor function
#define i2c_slave_lld_set_rxbuf(i2cp,b,n) do { \
  (i2cp)->rxbuf = (b); \
  (i2cp)->rxsize = (n); \
  (i2cp)->rxbytes = (n); \
  } while(0)

// TODO: Fix txbytes later with an accessor function
#define i2c_slave_lld_set_txbuf(i2cp,b,n) do { \
  (i2cp)->txbuf = (b); \
  (i2cp)->txsize = (n); \
  } while(0)


/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(__DOXYGEN__)
#if KINETIS_I2C_SLAVE_USE_I2C0
extern I2CSlaveDriver I2CSD1;
#endif
#if KINETIS_I2C_SLAVE_USE_I2C1
extern I2CSlaveDriver I2CSD2;
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void i2c_slave_lld_init(void);
  void i2c_slave_lld_start(I2CSlaveDriver *i2cp);
  void i2c_slave_lld_stop(I2CSlaveDriver *i2cp);
  //~ void  i2c_lld_slaveReceive(I2CSlaveDriver *i2cp, const I2CSlaveMsg *rxMsg);
  //~ void  i2c_lld_slaveReply(I2CSlaveDriver *i2cp, const I2CSlaveMsg *replyMsg);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_I2C_SLAVE */

#endif /* _I2C_SLAVE_LLD_H_ */

/** @} */
