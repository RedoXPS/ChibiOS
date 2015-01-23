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

#define K20x_MCUCONF

/*
 * HAL driver system settings.
 */

/*
 * SERIAL driver system settings.
 */
#define KINETIS_SERIAL_USE_UART0              FALSE
/*
 * ADC driver system settings.
 */
#define KINETIS_ADC_USE_ADC0                  FALSE
/*
 * PWM driver system settings.
 */
#define KINETIS_PWM_USE_FTM0                  FALSE
/*
 * I2C driver system settings.
 */
#define KINETIS_I2C_USE_I2C0                  FALSE
/*
 * SPI driver system settings.
 */
#define KINETIS_SPI_USE_SPI0                  TRUE

