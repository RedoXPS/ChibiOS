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

/**
 * @file    eeprom_lld.h
 * @brief   KINETIS EEPROM subsystem low level driver header.
 *
 * @{
 */

#ifndef _EEPROM_LLD_H_
#define _EEPROM_LLD_H_

#if HAL_USE_EEPROM || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

#define FlexRAM                 ((uint8_t *)0x14000000)

#define FTFL_BASE               ((uint32_t)0x40020000)
typedef struct {
  __IO uint8_t  FSTAT;          // 0x00
  __IO uint8_t  FCNFG;          // 0x01
  __I  uint8_t  FSEC;           // 0x02
  __I  uint8_t  FOPT;           // 0x03
  __IO uint8_t  FCCOB0;         // 0x04
  __IO uint8_t  FCCOB1;         // 0x05
  __IO uint8_t  FCCOB2;         // 0x06
  __IO uint8_t  FCCOB3;         // 0x07
  __IO uint8_t  FCCOB4;         // 0x08
  __IO uint8_t  FCCOB5;         // 0x09
  __IO uint8_t  FCCOB6;         // 0x0A
  __IO uint8_t  FCCOB7;         // 0x0B
  __IO uint8_t  FCCOB8;         // 0x0C
  __IO uint8_t  FCCOB9;         // 0x0D
  __IO uint8_t  FCCOBA;         // 0x0E
  __IO uint8_t  FCCOBB;         // 0x0F
  __IO uint8_t  FPROT0;         // 0x10
  __IO uint8_t  FPROT1;         // 0x11
  __IO uint8_t  FPROT2;         // 0x12
  __IO uint8_t  FPROT3;         // 0x13
       uint8_t  RESERVED0[2];
  __IO uint8_t  FEPROT;         // 0x16
  __IO uint8_t  FDPROT;         // 0x16
} FTFL_Typedef;

#define FTFL                    ((FTFL_Typedef *)  FTFL_BASE)

/***********  Bits definition for FTFL_FSTAT register  **************/
#define FTFL_FSTAT_CCIF         ((uint8_t)0x80)
#define FTFL_FSTAT_RDCOLERR     ((uint8_t)0x40)
#define FTFL_FSTAT_ACCERR       ((uint8_t)0x20)
#define FTFL_FSTAT_FPVIOL       ((uint8_t)0x10)
#define FTFL_FSTAT_MGSTAT0      ((uint8_t)0x01)

/***********  Bits definition for FTFL_FCNFG register  **************/
#define FTFL_FCNFG_CCIE         ((uint8_t)0x80)
#define FTFL_FCNFG_RDCOLLIE     ((uint8_t)0x40)
#define FTFL_FCNFG_ERSAREQ      ((uint8_t)0x20)
#define FTFL_FCNFG_ERSSUSP      ((uint8_t)0x10)
#define FTFL_FCNFG_PFLSH        ((uint8_t)0x04)
#define FTFL_FCNFG_RAMRDY       ((uint8_t)0x02)
#define FTFL_FCNFG_EEERDY       ((uint8_t)0x01)

/***********  Bits definition for FTFL_FSEC register  **************/
#define FTFL_FSEC_KEYEN_MASK    ((uint8_t)0xC0)
#define FTFL_FSEC_MEEN_MASK     ((uint8_t)0x30)
#define FTFL_FSEC_FSLACC_MASK   ((uint8_t)0x0C)
#define FTFL_FSEC_SEC_MASK      ((uint8_t)0x03)

// Writing unaligned 16 or 32 bit data is handled automatically when
// this is defined, but at a cost of extra code size.  Without this,
// any unaligned write will cause a hard fault exception!  If you're
// absolutely sure all 16 and 32 bit writes will be aligned, you can
// remove the extra unnecessary code.
//
#define HANDLE_UNALIGNED_WRITES

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Possible EEPROM failure causes.
 */
typedef enum {
  EEPROM_ERR_NO_ERROR  = 0,
  EEPROM_ERR_CONFIG    = 1,               /**< Configuration error occured    */
  EEPROM_ERR_COLLISION = 2,               /**< Read collision occured         */
  EEPROM_ERR_ACCESS    = 3,               /**< Illegal access                 */
  EEPROM_ERR_VIOLATION = 4                /**< Write to a protected area      */
} eeprom_error_t;

/**
 * @brief   Type of a structure representing an EEPROM driver.
 */
typedef struct EEPROMDriver EEPROMDriver;

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
  uint32_t                  size;
} EEPROMConfig;


/**
 * @brief   Structure representing an ADC driver.
 */
struct EEPROMDriver {
  /**
   * @brief Driver state.
   */
  eeprom_state_t                state;

  eeprom_errors_t               errors;
  /**
   * @brief Current configuration data.
   */
  const EEPROMConfig           *config;

#if EEPROM_USE_MUTUAL_EXCLUSION || defined(__DOXYGEN__)
  /**
   * @brief Mutex protecting the peripheral.
   */
  mutex_t                   mutex;
#endif /* EEPROM_USE_MUTUAL_EXCLUSION */
#if defined(EEPROM_DRIVER_EXT_FIELDS)
  EEPROM_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
extern EEPROMDriver EEPROM;

#ifdef __cplusplus
extern "C" {
#endif
  void eeprom_lld_init(void);
  void eeprom_lld_start(EEPROMDriver *eepp);
  void eeprom_lld_stop(EEPROMDriver *eepp);
  #define eeprom_lld_ready()  ((FTFL->FCNFG & FTFL_FCNFG_EEERDY) ? 1 : 0)
  #define eeprom_lld_wait_until_ready() while(!eeprom_lld_ready())
  #define eeprom_lld_get_errors(eepp) ((eepp)->errors)

  uint8_t  eeprom_lld_read_byte( EEPROMDriver *eepp, const uint8_t *addr);
  uint16_t eeprom_lld_read_word( EEPROMDriver *eepp, const uint16_t *addr);
  uint32_t eeprom_lld_read_dword(EEPROMDriver *eepp, const uint32_t *addr);
  float    eeprom_lld_read_float(EEPROMDriver *eepp, const float *addr);
  void     eeprom_lld_read_block(EEPROMDriver *eepp, const void *addr,
                                                     void *buf,  uint32_t len);
  void eeprom_lld_write_byte( EEPROMDriver *eepp, const uint8_t *addr,
                                                                 uint8_t value);
  void eeprom_lld_write_word( EEPROMDriver *eepp, const uint16_t *addr,
                                                                uint16_t value);
  void eeprom_lld_write_dword(EEPROMDriver *eepp, const uint32_t *addr,
                                                                uint32_t value);
  void eeprom_lld_write_float(EEPROMDriver *eepp, const float *addr,
                                                                   float value);
  void eeprom_lld_write_block(EEPROMDriver *eepp, const void *addr,
                                                const void *buf,  uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_EEPROM */

#endif /* _EEPROM_LLD_H_ */

/** @} */
