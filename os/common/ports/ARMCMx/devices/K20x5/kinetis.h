/*
    ChibiOS/HAL - Copyright (C) 2014 Derek Mulcahy

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
 * @file    K20x5/kinetis_registry.h
 * @brief   K20x5 capabilities registry.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef _KINETIS_REGISTRY_H_
#define _KINETIS_REGISTRY_H_

/*===========================================================================*/
/* Platform capabilities.                                                    */
/*===========================================================================*/

/**
 * @name    K20x5 capabilities
 * @{
 */

/**
 * @brief   Maximum system and core clock (f_SYS) frequency.
 */
#define KINETIS_SYSCLK_MAX      50000000L

/**
 * @brief   Maximum bus clock (f_BUS) frequency.
 */
#define KINETIS_BUSCLK_MAX      50000000L

/**
 * @brief   Maximum flash clock (f_FLASH) frequency.
 */
#define KINETIS_FLASHCLK_MAX	25000000L

/**
 * @name    K20x5 attributes
 * @{
 */

#define KINETIS_PORTA_IRQ_VECTOR    VectorE0
#define KINETIS_PORTB_IRQ_VECTOR    VectorE4
#define KINETIS_PORTC_IRQ_VECTOR    VectorE8
#define KINETIS_PORTD_IRQ_VECTOR    VectorEC
#define KINETIS_PORTE_IRQ_VECTOR    VectorF0

/* ADC attributes.*/
#define KINETIS_HAS_ADC0            TRUE
#define KINETIS_ADC0_IRC_VECTOR     Vector98

/* I2C attributes.*/
#define KINETIS_HAS_I2C0            TRUE
#define KINETIS_I2C0_IRQ_VECTOR     VectorA0

/* Serial attributes */
#define KINETIS_HAS_SERIAL0         TRUE
#define KINETIS_SERIAL0_IRQ_VECTOR  Vector80 // 88, 90

/* FlexTimer attributes */
#define KINETIS_FTM0_CHANNELS 8
#define KINETIS_FTM1_CHANNELS 2
#define KINETIS_FTM2_CHANNELS 2

#define KINETIS_FTM0_IRQ_VECTOR  Vector138
#define KINETIS_FTM1_IRQ_VECTOR  Vector13C
#define KINETIS_FTM2_IRQ_VECTOR  Vector140

/* GPT */
#define KINETIS_PIT0_HANDLER    VectorB8
#define KINETIS_PIT1_HANDLER    VectorBC
#define KINETIS_PIT2_HANDLER    VectorC0
#define KINETIS_PIT3_HANDLER    VectorC4

/* DMA */
#define KINETIS_DMA0_IRQ_VECTOR  Vector40
/** @} */

/* USB attributes */
#define KINETIS_USB_IRQ_VECTOR      VectorCC

#endif /* _KINETIS_REGISTRY_H_ */

/** @} */
