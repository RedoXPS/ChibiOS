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
#include "hal.h"

#include "eeprom.h"

static EEPROMConfig eep_cfg = {
  512,
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

  eepromInit();

  eepromStart(&EEPROM,&eep_cfg);
  uint16_t milli=1000;
  eepromWriteByte(&EEPROM,(uint8_t*)0x00,0x42);
  if(eepromReadByte(&EEPROM,(uint8_t*)0x00) != 0x42)
    milli = 50;
  while (!chThdShouldTerminateX()) {
    palTogglePad(IOPORT3,5);
    chThdSleepMilliseconds(milli);
  }
  return 0;
}
