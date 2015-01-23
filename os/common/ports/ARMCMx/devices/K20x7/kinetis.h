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
 * @file    K20x7/kinetis_registry.h
 * @brief   K20x7 capabilities registry.
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
 * @name    K20x7 capabilities
 * @{
 */

/**
 * @brief   Maximum system and core clock (f_SYS) frequency.
 */
#define KINETIS_SYSCLK_MAX      72000000

/**
 * @brief   Maximum bus clock (f_BUS) frequency.
 */
#define KINETIS_BUSCLK_MAX      36000000

/**
 * @name    K20x7 attributes
 * @{
 */

/* EXT attributes.*/

#define KINETIS_PORTA_IRQ_VECTOR    Vector19C
#define KINETIS_PORTB_IRQ_VECTOR    Vector1A0
#define KINETIS_PORTC_IRQ_VECTOR    Vector1A4
#define KINETIS_PORTD_IRQ_VECTOR    Vector1A8
#define KINETIS_PORTE_IRQ_VECTOR    Vector1AC

/* ADC attributes.*/
#define KINETIS_HAS_ADC0            TRUE
#define KINETIS_ADC0_IRQ_VECTOR     Vector124
#define KINETIS_HAS_ADC1            TRUE
#define KINETIS_ADC1_IRQ_VECTOR     Vector128

/* I2C attributes.*/
#define KINETIS_HAS_I2C0            TRUE
#define KINETIS_I2C0_IRQ_VECTOR     VectorA0

/* Serial attributes */
#define KINETIS_HAS_SERIAL0         TRUE
#define KINETIS_SERIAL0_IRQ_VECTOR  VectorF4

/* FlexTimer attributes */
#define KINETIS_FTM0_CHANNELS 8
#define KINETIS_FTM1_CHANNELS 2
#define KINETIS_FTM2_CHANNELS 2

#define KINETIS_FTM0_IRQ_VECTOR  Vector138
#define KINETIS_FTM1_IRQ_VECTOR  Vector13C
#define KINETIS_FTM2_IRQ_VECTOR  Vector140

/* GPT */
#define KINETIS_PIT0_IRQ_VECTOR  VectorB8
#define KINETIS_PIT1_IRQ_VECTOR  VectorBC
#define KINETIS_PIT2_IRQ_VECTOR  VectorC0
#define KINETIS_PIT3_IRQ_VECTOR  VectorC4

/* DMA */
#define KINETIS_DMA0_IRQ_VECTOR  Vector40
/** @} */

#endif /* _KINETIS_REGISTRY_H_ */

/** @} */
