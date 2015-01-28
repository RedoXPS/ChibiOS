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
#include "chprintf.h"

#include "i2c_slave.h"

void start_cb(I2CSlaveDriver *i2cp,uint8_t c)
{
  (void)i2cp;
  (void)c;
  //~ chprintf((BaseSequentialStream *)&SD1,"\r\n[(%X) ",c);
}

void endrx_cb(I2CSlaveDriver *i2cp)
{
  (void)i2cp;
  uint8_t i;
  uint8_t lim = i2cp->rxidx < i2cp->rxbytes ? i2cp->rxidx-1 : i2cp->rxbytes-1;
  for(i=0;i<=lim;i++)
  {
    i2cp->txbuf[i] = i2cp->rxbuf[lim-i];
  }
  i2cp->txbuf[2]++;
  i2cp->txbytes=i2cp->rxidx;
  //~ chprintf((BaseSequentialStream *)&SD1,"r%X",i2cp->txbuf[2]);
}

void endtx_cb(I2CSlaveDriver *i2cp)
{
  (void)i2cp;
  //~ chprintf((BaseSequentialStream *)&SD1,"t");
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

static SerialConfig sercfg = {
  115200
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
  sdStart(&SD1, &sercfg);

  i2cSlaveInit();
  i2cSlaveObjectInit(&I2CSD1);

  palSetPadMode(IOPORT2, 0, PAL_MODE_ALTERNATIVE_2);
  palSetPadMode(IOPORT2, 1, PAL_MODE_ALTERNATIVE_2);
  i2cSlaveStart(&I2CSD1, &i2cscfg);

  while (1) {
    chThdSleepMilliseconds(1000);
    palTogglePad(GPIOC, PORTC_TEENSY_PIN13);
  }
}
