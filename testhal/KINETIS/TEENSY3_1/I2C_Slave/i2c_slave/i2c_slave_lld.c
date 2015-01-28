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
 * @file    i2c_slave_lld.c
 * @brief   I2C Slave subsystem low level driver source.
 *
 * @addtogroup I2C
 * @{
 */
#include "hal.h"

#if HAL_USE_I2C_SLAVE || defined(__DOXYGEN__)

#include "i2c_slave.h"

static uint8_t i2c_txbuf[4] = {0x00,0x00,0x00,0x00};
static uint8_t i2c_rxbuf[4] = {0x00,0x00,0x00,0x00};
/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   I2C1 driver identifier.
 */
#if KINETIS_I2C_SLAVE_USE_I2C0 || defined(__DOXYGEN__)
I2CSlaveDriver I2CSD1;
#endif
/**
 * @brief   I2C2 driver identifier.
 */
#if KINETIS_I2C_SLAVE_USE_I2C1 || defined(__DOXYGEN__)
I2CSlaveDriver I2CSD2;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

void slave_config_frequency(I2CSlaveDriver *i2cp) {

  /* Each index in the table corresponds to a a frequency
   * divider used to generate the SCL clock from the main
   * system clock.
   */
  uint16_t icr_table[] = {
    /* 0x00 - 0x0F */
    20,22,24,26,28,30,34,40,28,32,36,40,44,48,56,68,
    /* 0x10 - 0x1F */
    48,56,64,72,80,88,104,128,80,96,112,128,144,160,192,240,
    /* 0x20 - 0x2F */
    160,192,224,256,288,320,384,480,320,384,448,512,576,640,768,960,
    /* 0x30 - 0x3F */
    640,768,896,1024,1152,1280,1536,1920,1280,1536,1792,2048,2304,2560,3072,3840,
  };
  uint16_t scl_stop_table[] = {
    /* 0x00 - 0x0F */
    11,12,13,14,15,16,18,21,15,17,19,21,23,25,29,35,
    /* 0x10 - 0x1F */
    25,29,33,37,41,45,53,65,41,49,57,65,73,81,97,121,
    /* 0x20 - 0x2F */
    81,97,113,129,145,161,193,241,161,193,225,257,289,321,384,481,
    /* 0x30 - 0x3F */
    321,385,449,513,577,641,769,961,641,769,897,1024,1153,1284,1537,1921
  };

  int length = sizeof(icr_table) / sizeof(icr_table[0]);
  uint16_t divisor;
  uint8_t i = 0, index = 0;
  uint16_t best, diff;

  if (i2cp->config != NULL)
    divisor = KINETIS_SYSCLK_FREQUENCY / i2cp->config->clock;
  else
    divisor = KINETIS_SYSCLK_FREQUENCY / 100000;

  best = ~0;
  index = 0;
  /* Tries to find the SCL clock which is the closest
   * approximation to the clock passed in config. To
   * stay on the safe side, only values that generate
   * lower frequency are used.
   */
  for (i = 0; i < length; i++) {
    if (icr_table[i] >= divisor) {
      diff = icr_table[i] - divisor;
      if (diff < best) {
        best = diff;
        index = i;
      }
    }
  }

  i2cp->i2c->F = index;
  uint16_t t = 512*1*scl_stop_table[index];
  i2cp->i2c->SLTH = t>>8;
  i2cp->i2c->SLTL = t&0xFF;
}

/**
 * @brief   Common IRQ handler.
 * @note    Tries hard to clear all the pending interrupt sources, we don't
 *          want to go through the whole ISR and have another interrupt soon
 *          after.
 *
 * @param[in] i2cp         pointer to an I2CDriver
 */


#if 0
#define cb(x) do { \
    chSysLockFromISR(); \
    x; \
    chSysUnlockFromISR(); \
  } while(0)
#else
  #define cb(x) x
#endif


static void serve_interrupt(I2CSlaveDriver *i2cp) {

  I2C_TypeDef *i2c = i2cp->i2c;

  uint8_t i2cS = i2c->S;
  uint8_t i2cSMB = i2c->SMB;
  uint8_t i2cC1 = i2c->C1&(~(I2Cx_C1_TXAK|I2Cx_C1_TX));// Default to Ack, RxMode
  uint8_t c=0;
  i2cp->errors = I2C_SLAVE_NO_ERROR;
  if((i2cSMB & (I2Cx_SMB_SHTF2|I2Cx_SMB_SHTF2IE)) ==
                                              (I2Cx_SMB_SHTF2|I2Cx_SMB_SHTF2IE))
  {
    i2cSMB &= ~I2Cx_SMB_SHTF2IE;
    i2c->SMB = i2cSMB | I2Cx_SMB_SHTF2 | I2Cx_SMB_SLTF;
    i2c->S = i2cS | I2Cx_S_IICIF;
    i2cp->state = I2C_SLAVE_READY;
      if (i2cp->rxidx && i2cp->config->rxend_cb != NULL)
    cb(i2cp->config->rxend_cb(i2cp));
  }
  else if (!(i2cSMB & I2Cx_SMB_FACK) && (i2cS & I2Cx_S_TCF))
  {
    i2c->S = i2cS | I2Cx_S_IICIF;
    if (i2cS & I2Cx_S_ARBL)
    {
      i2cp->errors |= I2C_ARBITRATION_LOST;
      i2cp->state = I2C_SLAVE_READY;
      i2c->S = i2cS | I2Cx_S_ARBL;
    }
    if(i2cS & I2Cx_S_IAAS)                                              // Start
    {
      if(i2cp->state == I2C_SLAVE_ACTIVE_RX)                          // ReStart
      {
        i2cSMB &= ~I2Cx_SMB_SHTF2IE;
        // Callback I2C Stop for RX
        if (i2cp->rxidx && i2cp->config->rxend_cb != NULL)
        {
          cb(i2cp->config->rxend_cb(i2cp));
        }
      }
      if(i2cS & I2Cx_S_SRW)
      {
        i2cC1 |= I2Cx_C1_TX;
        i2c->C1 = i2cC1;
      }
      c = i2c->D; // Read Address
      if (i2cp->config->start_cb != NULL)
      {
        cb(i2cp->config->start_cb(i2cp,c));
      }
      if(i2cS & I2Cx_S_SRW) 		                                 // Start TX-ing
      {
        i2cp->state = I2C_SLAVE_ACTIVE_TX;
        c = 0xFF; // Dummy byte
        if(i2cp->txbuf != NULL && i2cp->txbytes)
        {
          c = i2cp->txbuf[0x00];
          i2cp->txidx=0x1;
        }
        i2c->D = c; // Send first byte
      }
      else 	                    										             // Start RX-ing
      {
        i2cp->state = I2C_SLAVE_ACTIVE_RX;
        i2c->SMB = i2cSMB | I2Cx_SMB_SHTF2IE;
        i2cp->rxidx=0;
      }
      i2c->C1 = i2cC1;
    }
    else
    {
      if(i2cp->state == I2C_SLAVE_ACTIVE_TX) {			                   // TX-ing
        if(!(i2cS & I2Cx_S_RXAK)) {		             // Master Ack-ed, Keep TX-ing
          i2cp->state = I2C_SLAVE_ACTIVE_TX;
          c = 0xFF;
          if(i2cp->txbuf != NULL
             && i2cp->txidx<i2cp->txbytes)    // TxBuffer not empty, Keep TX-ing
          {
            c = i2cp->txbuf[i2cp->txidx++];
          }
          i2c->D = c;                       // Send next byte
          i2c->C1 = i2cC1|(I2Cx_C1_TX);     // TxMode
        }
        else
        {											              // Stop TX-ing (Master just NAck-ed)
          i2cp->state = I2C_SLAVE_READY;
          i2c->C1 = i2cC1;                  // RxMode
          (void)i2c->D;	// Dummy read
          // Callback End TX
          if (i2cp->txbytes && i2cp->txidx==i2cp->txbytes
              && i2cp->config->txend_cb != NULL)
          {
            cb(i2cp->config->txend_cb(i2cp));
          }
        }
      }
      else									                                           // RX-ing
      {
        if(i2cp->rxbuf != NULL
           && i2cp->rxidx < i2cp->rxbytes)       // Buffer not full, Keep RX-ing
        {
          i2cp->state = I2C_SLAVE_ACTIVE_RX;
          if(i2cp->rxidx+1 >= i2cp->rxbytes)      // Stop RX-ing after this byte
          {
            i2cC1 |= I2Cx_C1_TXAK;          // RxMode, Nack
          }
          i2c->C1 = i2cC1;
          i2cp->rxbuf[i2cp->rxidx++] = i2c->D;	// Read =)
          // Callback Byte RX-ed
        }
        else                       // Buffer full, Stop RX-ing after this byte
        {
          i2cp->state = I2C_SLAVE_READY;
          i2c->C1 = i2cC1|I2Cx_C1_TXAK;  // RxMode, Nack
          (void)i2c->D;	// Dummy read
        }
      }
    }
  }
  i2c->S = i2cS | I2Cx_S_IICIF;
  if (i2cp->errors != I2C_SLAVE_NO_ERROR)
    _i2c_slave_wakeup_error_isr(i2cp);
  else if (i2cp->state == I2C_SLAVE_READY)
    _i2c_slave_wakeup_isr(i2cp);
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if KINETIS_I2C_SLAVE_USE_I2C0 || defined(__DOXYGEN__)

PORT_IRQ_HANDLER(KINETIS_I2C0_IRQ_VECTOR) {

  PORT_IRQ_PROLOGUE();
  serve_interrupt(&I2CSD1);
  PORT_IRQ_EPILOGUE();
}

#endif

#if KINETIS_I2C_SLAVE_USE_I2C1 || defined(__DOXYGEN__)

PORT_IRQ_HANDLER(KINETIS_I2C1_IRQ_VECTOR) {

  PORT_IRQ_PROLOGUE();
  serve_interrupt(&I2CSD2);
  PORT_IRQ_EPILOGUE();
}

#endif
/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level I2C driver initialization.
 *
 * @notapi
 */
void i2c_slave_lld_init(void) {

#if KINETIS_I2C_SLAVE_USE_I2C0
  i2cSlaveObjectInit(&I2CSD1);
  I2CSD1.thread = NULL;
  I2CSD1.i2c = I2C0;
#endif /* PLATFORM_I2C_SLAVE_USE_I2C0 */
#if KINETIS_I2C_SLAVE_USE_I2C1
  i2cSlaveObjectInit(&I2CSD2);
  I2CSD2.thread = NULL;
  I2CSD2.i2c = I2C1;
#endif /* PLATFORM_I2C_SLAVE_USE_I2C1 */
}

/**
 * @brief   Configures and activates the I2C peripheral.
 *
 * @param[in] i2cp      pointer to t1he @p I2CDriver object
 *
 * @notapi
 */

void i2c_slave_lld_start(I2CSlaveDriver *i2cp) {

  if (i2cp->state == I2C_SLAVE_STOP) {
    /* Enables the peripheral.*/
#if KINETIS_I2C_SLAVE_USE_I2C0
    if (&I2CSD1 == i2cp) {
      SIM->SCGC4 |= SIM_SCGC4_I2C0;
      nvicEnableVector(I2C0_IRQn, KINETIS_I2C_I2C0_PRIORITY);
    }
#endif /* KINETIS_I2C_SLAVE_USE_I2C0 */
#if KINETIS_I2C_SLAVE_USE_I2C1
    if (&I2CSD2 == i2cp) {
      SIM->SCGC4 |= SIM_SCGC4_I2C1;
      nvicEnableVector(I2C0_IRQn, KINETIS_I2C_I2C1_PRIORITY);
    }
#endif /* KINETIS_I2C_SLAVE_USE_I2C1 */
  }

  slave_config_frequency(i2cp);

  i2cp->i2c->A1 = I2Cx_A1_AD(i2cp->config->address);
  i2cp->i2c->C1 = I2Cx_C1_IICEN | I2Cx_C1_IICIE;
  i2cp->i2c->SMB = I2Cx_SMB_TCKSEL;

  i2cp->txbytes=0;
  i2cp->txsize=4;
  i2cp->txbuf = i2c_txbuf;
  i2cp->txidx=0;
  i2cp->rxbytes=2;
  i2cp->rxsize=4;
  i2cp->rxbuf = i2c_rxbuf;
  i2cp->rxidx=0;
}

/**
 * @brief   Deactivates the I2C peripheral.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
void i2c_slave_lld_stop(I2CSlaveDriver *i2cp) {

  if (i2cp->state != I2C_SLAVE_STOP) {
    i2cp->i2c->C1 &= ~(I2Cx_C1_IICEN | I2Cx_C1_IICIE);

    /* Disables the peripheral.*/
#if KINETIS_I2C_SLAVE_USE_I2C0
    if (&I2CSD1 == i2cp) {
      SIM->SCGC4 &= ~SIM_SCGC4_I2C0;
      nvicDisableVector(I2C0_IRQn);
    }
#endif /* KINETIS_I2C_SLAVE_USE_I2C0 */
#if KINETIS_I2C_SLAVE_USE_I2C1
    if (&I2CSD2 == i2cp) {
      SIM->SCGC4 &= ~SIM_SCGC4_I2C1;
      nvicDisableVector(I2C1_IRQn);
    }
#endif /* KINETIS_I2C_SLAVE_USE_I2C1 */
  }
}


#endif /* HAL_USE_I2C_SLAVE */

/** @} */
