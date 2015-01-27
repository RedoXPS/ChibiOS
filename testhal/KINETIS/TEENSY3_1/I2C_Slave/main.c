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

//~ static bool i2cOk = false;

//~ static THD_WORKING_AREA(waThread1, 64);
//~ static THD_FUNCTION(Thread1, arg) {

  //~ (void)arg;
  //~ chRegSetThreadName("Blinker");
  //~ while (TRUE) {
    //~ if (i2cOk) {
      //~ palTogglePad(IOPORT3, PORTC_TEENSY_PIN13);
      //~ i2cOk=0;
    //~ }
    //~ chThdSleepMilliseconds(500);
  //~ }

  //~ return 0;
//~ }

void start_cb(I2CSlaveDriver *i2cp,uint8_t c)
{
  (void)i2cp;
  chprintf((BaseSequentialStream *)&SD1,"\r\n[(%X) ",c);
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
  chprintf((BaseSequentialStream *)&SD1,"Rx(%d/%d) %x %x %x] ",i2cp->rxidx,i2cp->rxsize,i2cp->rxbuf[0],i2cp->rxbuf[1],i2cp->rxbuf[2]);
}

void endtx_cb(I2CSlaveDriver *i2cp)
{
  (void)i2cp;
  chprintf((BaseSequentialStream *)&SD1,"Tx(%d/%d) %x %x %x] ",i2cp->txidx,i2cp->txsize,i2cp->txbuf[0],i2cp->txbuf[1],i2cp->txbuf[2]);
}

static I2CSlaveConfig i2cscfg = {
  10000,
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

//~ static I2CConfig i2ccfg = {
  //~ 40000
//~ };
/*
 * Application entry point.
 */
int main(void) {

  uint8_t tx[8], rx[8];
  uint8_t i=0;

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

  //~ palSetPadMode(IOPORT2, 0, PAL_MODE_ALTERNATIVE_2);
  //~ palSetPadMode(IOPORT2, 1, PAL_MODE_ALTERNATIVE_2);
  //~ i2cStart(&I2CD2, &i2ccfg);

  palSetPadMode(IOPORT2, 0, PAL_MODE_ALTERNATIVE_2);
  palSetPadMode(IOPORT2, 1, PAL_MODE_ALTERNATIVE_2);
  i2cSlaveStart(&I2CSD1, &i2cscfg);

  //~ chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  while (1) {
    //~ tx[0] = 0x10;
    //~ tx[1] = 0x02;
    //~ i2cMasterTransmitTimeout(&I2CD2, 0x21, tx, 2, rx, 6, TIME_INFINITE);
    //~ i2cOk = (rx[0] == 0x10) ? true : false;
    chThdSleepMilliseconds(1000);
    palTogglePad(GPIOC, PORTC_TEENSY_PIN13);
    //~ chprintf((BaseSequentialStream *)&SD1,"Hello World %d\r\n",i++);
  }
}
