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
 * @file    K20x5/kinetis.h
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
#define KINETIS_FLASHCLK_MAX	  25000000L

/**
 * @name    K20x5 attributes
 * @{
 */

/* DMA */
#define KINETIS_DMA0_IRQ_VECTOR             Vector40
#define KINETIS_DMA1_IRQ_VECTOR             Vector44
#define KINETIS_DMA2_IRQ_VECTOR             Vector48
#define KINETIS_DMA3_IRQ_VECTOR             Vector4C
#define KINETIS_DMAERROR_IRQ_VECTOR         Vector50

/* Flash */
#define KINETIS_FLASHCOMPLETE_IRQ_VECTOR    Vector58
#define KINETIS_FLASHCOLLISION_IRQ_VECTOR   Vector5C

/* Low voltage */
#define KINETIS_LOWVOLTAGE_IRQ_VECTOR       Vector60

/* Low Leakage */
#define KINETIS_LOWLEAKAGE_IRQ_VECTOR       Vector64

/* Watchdog */
#define KINETIS_WDOG_IRQ_VECTOR             Vector68

/* I2C attributes */
#define KINETIS_HAS_I2C0                    TRUE
#define KINETIS_I2C0_IRQ_VECTOR             Vector6C

/* SPI attributes */
#define KINETIS_HAS_SPI0                    TRUE
#define KINETIS_SPI0_IRQ_VECTOR             Vector70

/* I2S */
#define KINETIS_I2STX_IRQ_VECTOR            Vector74
#define KINETIS_I2SRX_IRQ_VECTOR            Vector78

/* Serial attributes */
#define KINETIS_HAS_SERIAL0                 TRUE
#define KINETIS_SERIAL0_IRQ_VECTOR          Vector80
#define KINETIS_SERIAL0ERROR_IRQ_VECTOR     Vector84
#define KINETIS_HAS_SERIAL1                 TRUE
#define KINETIS_SERIAL1_IRQ_VECTOR          Vector88
#define KINETIS_SERIAL1ERROR_IRQ_VECTOR     Vector8C
#define KINETIS_HAS_SERIAL2                 TRUE
#define KINETIS_SERIAL2_IRQ_VECTOR          Vector90
#define KINETIS_SERIAL2ERROR_IRQ_VECTOR     Vector94
/* ADC attributes */
#define KINETIS_HAS_ADC0                    TRUE
#define KINETIS_ADC0_IRQ_VECTOR             Vector98

/* Comparator */
#define KINETIS_CMP0_IRQ_VECTOR             Vector9C
#define KINETIS_CMP1_IRQ_VECTOR             VectorA0

/* FlexTimer attributes */
// FIXME
#define KINETIS_FTM0_CHANNELS               8
#define KINETIS_FTM1_CHANNELS               2

#define KINETIS_FTM0_IRQ_VECTOR             VectorA4
#define KINETIS_FTM1_IRQ_VECTOR             VectorA8

/* Carrier Modulator Transmitter */
#define KINETIS_CMT_IRQ_VECTOR              VectorAC

/* RTC */
#define KINETIS_RTCALARM_IRQ_VECTOR         VectorB0
#define KINETIS_RTCSECONDS_IRQ_VECTOR       VectorB4

/* GPT */
#define KINETIS_HAS_PIT0                    TRUE
#define KINETIS_PIT0_IRQ_VECTOR             VectorB8
#define KINETIS_HAS_PIT1                    TRUE
#define KINETIS_PIT1_IRQ_VECTOR             VectorBC
#define KINETIS_HAS_PIT2                    TRUE
#define KINETIS_PIT2_IRQ_VECTOR             VectorC0
#define KINETIS_HAS_PIT3                    TRUE
#define KINETIS_PIT3_IRQ_VECTOR             VectorC4

/* Programmable Delay Block */
#define KINETIS_PDB_IRQ_VECTOR              VectorC8

/* USB attributes */
#define KINETIS_HAS_USB                     TRUE
#define KINETIS_USB_IRQ_VECTOR              VectorCC
#define KINETIS_USBCD_IRQ_VECTOR            VectorD0

/* Touch Sense Input */
#define KINETIS_TSI_IRQ_VECTOR              VectorD4

/* Multipurpose Clock Generator */
#define KINETIS_MCG_IRQ_VECTOR              VectorD8

/* Low Power Timer */
#define KINETIS_LPTMR_IRQ_VECTOR            VectorDC

/* Pin Detect */
#define KINETIS_PORTA_IRQ_VECTOR            VectorE0
#define KINETIS_PORTB_IRQ_VECTOR            VectorE4
#define KINETIS_PORTC_IRQ_VECTOR            VectorE8
#define KINETIS_PORTD_IRQ_VECTOR            VectorEC
#define KINETIS_PORTE_IRQ_VECTOR            VectorF0

/** @} */
#endif /* _KINETIS_REGISTRY_H_ */
