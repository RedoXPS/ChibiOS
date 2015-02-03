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

static uint8_t i2cOk = false;

static THD_WORKING_AREA(waThread1, 64);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("Blinker");
  while (TRUE) {
    if (i2cOk) {
      palTogglePad(IOPORT3, PORTC_TEENSY_PIN13);
      i2cOk=0;
    }
    chThdSleepMilliseconds(10);
  }

  return 0;
}

static I2CConfig i2ccfg = {
  400000
};

static SerialConfig sercfg = {
  115200
};

/*
 * Application entry point.
 */
int main(void) {

  uint8_t tx[8], rx[8];
  uint16_t i=0;

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  sdStart(&SD1,&sercfg);
  palSetPadMode(IOPORT2, 0, PAL_MODE_ALTERNATIVE_2);
  PORTB->PCR[0] |= PORTx_PCRn_ODE;
  palSetPadMode(IOPORT2, 1, PAL_MODE_ALTERNATIVE_2);
  PORTB->PCR[1] |= PORTx_PCRn_ODE;
  i2cStart(&I2CD1, &i2ccfg);

  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
  chThdSleepMilliseconds(1000); // Let the Slaves complete their init
  while (1) {
    tx[0] = 0x10; tx[1] = 0x02;
    rx[0] = 0x00;
    i2cMasterTransmitTimeout(&I2CD1, 0x21, tx, 2, rx, 2, TIME_INFINITE);
    i2cOk = (rx[0] == tx[1]) ?  0x10: 0x00;
    rx[0] = 0x00;
    i2cMasterTransmitTimeout(&I2CD1, 0x22, tx, 2, rx, 2, TIME_INFINITE);
    i2cOk |= (rx[0] == tx[1]) ? 0x01 : 0x00;
    chprintf((BaseSequentialStream *)&SD1,"%4X - %02X\r\n",i++,i2cOk);
    chThdSleepMilliseconds(50);

  }
}
