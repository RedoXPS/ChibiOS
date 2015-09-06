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
 * @file    K20x7/kinetis.h
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
#define KINETIS_SYSCLK_MAX      72000000L

/**
 * @brief   Maximum bus clock (f_BUS) frequency.
 */
#define KINETIS_BUSCLK_MAX      50000000L

/**
 * @brief   Maximum flash clock (f_FLASH) frequency.
 */
#define KINETIS_FLASHCLK_MAX	  25000000L

/**
 * @name    K20x7 attributes
 * @{
 */

/* DMA */
#define KINETIS_DMA0_IRQ_VECTOR             Vector40
#define KINETIS_DMA1_IRQ_VECTOR             Vector44
#define KINETIS_DMA2_IRQ_VECTOR             Vector48
#define KINETIS_DMA3_IRQ_VECTOR             Vector4C
#define KINETIS_DMA4_IRQ_VECTOR             Vector50
#define KINETIS_DMA5_IRQ_VECTOR             Vector54
#define KINETIS_DMA6_IRQ_VECTOR             Vector58
#define KINETIS_DMA7_IRQ_VECTOR             Vector5C
#define KINETIS_DMA8_IRQ_VECTOR             Vector60
#define KINETIS_DMA9_IRQ_VECTOR             Vector64
#define KINETIS_DMA10_IRQ_VECTOR            Vector68
#define KINETIS_DMA11_IRQ_VECTOR            Vector6C
#define KINETIS_DMA12_IRQ_VECTOR            Vector70
#define KINETIS_DMA13_IRQ_VECTOR            Vector74
#define KINETIS_DMA14_IRQ_VECTOR            Vector78
#define KINETIS_DMA15_IRQ_VECTOR            Vector7C
#define KINETIS_DMAERROR_IRQ_VECTOR         Vector80

/* Flash */
#define KINETIS_FLASHCOMPLETE_IRQ_VECTOR    Vector88
#define KINETIS_FLASHCOLLISION_IRQ_VECTOR   Vector8C

/* Low voltage */
#define KINETIS_LOWVOLTAGE_IRQ_VECTOR       Vector90

/* Low Leakage */
#define KINETIS_LOWLEAKAGE_IRQ_VECTOR       Vector94

/* Watchdog */
#define KINETIS_WDOG_IRQ_VECTOR             Vector98

/* I2C attributes */
#define KINETIS_HAS_I2C0                    TRUE
#define KINETIS_I2C0_IRQ_VECTOR             VectorA0
#define KINETIS_HAS_I2C1                    TRUE
#define KINETIS_I2C1_IRQ_VECTOR             VectorA4

/* SPI attributes */
#define KINETIS_HAS_SPI0                    TRUE
#define KINETIS_SPI0_IRQ_VECTOR             VectorA8
#define KINETIS_HAS_SPI1                    TRUE
#define KINETIS_SPI1_IRQ_VECTOR             VectorAC

/* CAN */
#define KINETIS_CAN0MSG_IRQ_VECTOR          VectorB4
#define KINETIS_CAN0BUSOFF_IRQ_VECTOR       VectorB8
#define KINETIS_CAN0ERROR_IRQ_VECTOR        VectorBC
#define KINETIS_CAN0TXWARNING_IRQ_VECTOR    VectorC0
#define KINETIS_CAN0RXWARNING_IRQ_VECTOR    VectorC4
#define KINETIS_CAN0WAKEUP_IRQ_VECTOR       VectorC8
#define KINETIS_CAN0TX_IRQ_VECTOR           VectorCC
#define KINETIS_CAN0RX_IRQ_VECTOR           VectorD0

/* I2S */
#define KINETIS_I2STX_IRQ_VECTOR            VectorCC
#define KINETIS_I2SRX_IRQ_VECTOR            VectorD0

/* Serial attributes */
#define KINETIS_HAS_SERIAL0                 TRUE
#define KINETIS_SERIAL0LON_IRQ_VECTOR       VectorF0
#define KINETIS_SERIAL0_IRQ_VECTOR          VectorF4
#define KINETIS_SERIAL0ERROR_IRQ_VECTOR     VectorF8
#define KINETIS_HAS_SERIAL1                 TRUE
#define KINETIS_SERIAL1_IRQ_VECTOR          VectorFC
#define KINETIS_SERIAL1ERROR_IRQ_VECTOR     Vector100
#define KINETIS_HAS_SERIAL2                 TRUE
#define KINETIS_SERIAL2_IRQ_VECTOR          Vector104
#define KINETIS_SERIAL2ERROR_IRQ_VECTOR     Vector108

/* ADC attributes */
#define KINETIS_HAS_ADC0                    TRUE
#define KINETIS_ADC0_IRQ_VECTOR             Vector124
#define KINETIS_HAS_ADC1                    TRUE
#define KINETIS_ADC1_IRQ_VECTOR             Vector128

/* Comparator */
#define KINETIS_CMP0_IRQ_VECTOR             Vector12C
#define KINETIS_CMP1_IRQ_VECTOR             Vector130
#define KINETIS_CMP2_IRQ_VECTOR             Vector134

/* FlexTimer attributes */
#define KINETIS_FTM0_CHANNELS               8
#define KINETIS_FTM1_CHANNELS               2
#define KINETIS_FTM2_CHANNELS               2

#define KINETIS_FTM0_IRQ_VECTOR             Vector138
#define KINETIS_FTM1_IRQ_VECTOR             Vector13C
#define KINETIS_FTM2_IRQ_VECTOR             Vector140

/* Carrier Modulator Transmitter */
#define KINETIS_CMT_IRQ_VECTOR              Vector144

/* RTC */
#define KINETIS_RTCALARM_IRQ_VECTOR         Vector148
#define KINETIS_RTCSECONDS_IRQ_VECTOR       Vector14C

/* GPT */
#define KINETIS_HAS_PIT0                    TRUE
#define KINETIS_PIT0_IRQ_VECTOR             Vector150
#define KINETIS_HAS_PIT1                    TRUE
#define KINETIS_PIT1_IRQ_VECTOR             Vector154
#define KINETIS_HAS_PIT2                    TRUE
#define KINETIS_PIT2_IRQ_VECTOR             Vector158
#define KINETIS_HAS_PIT3                    TRUE
#define KINETIS_PIT3_IRQ_VECTOR             Vector15C

/* Programmable Delay Block */
#define KINETIS_PDB_IRQ_VECTOR              Vector160

/* USB attributes */
#define KINETIS_USB_IRQ_VECTOR              Vector164
#define KINETIS_USBCD_IRQ_VECTOR            Vector168

/* DAC attributes */
#define KINETIS_HAS_DAC0                    TRUE
#define KINETIS_DAC0_IRQ_VECTOR             Vector184

/* Touch Sense Input */
#define KINETIS_TSI_IRQ_VECTOR              Vector18C

/* Multipurpose Clock Generator */
#define KINETIS_MCG_IRQ_VECTOR              Vector190

/* Low Power Timer */
#define KINETIS_LPTMR_IRQ_VECTOR            Vector194

/* Pin Detect */
#define KINETIS_PORTA_IRQ_VECTOR            Vector19C
#define KINETIS_PORTB_IRQ_VECTOR            Vector1A0
#define KINETIS_PORTC_IRQ_VECTOR            Vector1A4
#define KINETIS_PORTD_IRQ_VECTOR            Vector1A8
#define KINETIS_PORTE_IRQ_VECTOR            Vector1AC

#endif /* _KINETIS_REGISTRY_H_ */

/** @} */
