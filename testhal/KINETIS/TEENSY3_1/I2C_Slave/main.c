/*
    ChibiOS/RT - Copyright (C) 2006-2014 Giovanni Di Sirio

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
#include "hal.h"

#include "i2c_slave.h"

static uint8_t _i2cs_rx[16],_i2cs_tx[16];

void start_cb(I2CSlaveDriver *i2cp,uint8_t c)
{
  (void)i2cp;
  (void)c;
}

void endrx_cb(I2CSlaveDriver *i2cp,uint8_t *buf, size_t n)
{
  (void)i2cp;
  uint8_t i;
  for(i=0;i<n;i++)
  {
    _i2cs_tx[i] = buf[n-1-i];
  }
  _i2cs_tx[2]++;
  i2cp->txbytes=n;
}

void endtx_cb(I2CSlaveDriver *i2cp,uint8_t *buf, size_t n)
{
  (void)i2cp;
  (void)buf;
  (void)n;
}

static I2CSlaveConfig i2cscfg = {
  400000,
  0x21,
  start_cb,   // Start
  //~ NULL,   // Rx Byte
  endrx_cb,   // Rx End
  //~ NULL,   // Tx Byte
  endtx_cb,   // Tx End
  //~ NULL,   // Error
};

/*
 * Application entry point.
 */
int main(void) {
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  i2cSlaveInit();
  i2cSlaveObjectInit(&I2CSD1);

  palSetPadMode(IOPORT2, 0, PAL_MODE_ALTERNATIVE_2);
  PORTB->PCR[0] |= PORTx_PCRn_ODE;
  palSetPadMode(IOPORT2, 1, PAL_MODE_ALTERNATIVE_2);
  PORTB->PCR[1] |= PORTx_PCRn_ODE;
  i2cSlaveStart(&I2CSD1, &i2cscfg);

  i2cSlaveSetRxBuffer(&I2CSD1,_i2cs_rx,16);
  i2cSlaveSetTxBuffer(&I2CSD1,_i2cs_tx,16);

  while (1) {
    chThdSleepMilliseconds(1000);
    palTogglePad(GPIOC, PORTC_TEENSY_PIN13);
  }
}
