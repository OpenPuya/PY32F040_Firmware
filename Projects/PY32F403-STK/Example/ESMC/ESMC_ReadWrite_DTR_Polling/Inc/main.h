/**
  ******************************************************************************
  * @file    main.h
  * @author  MCU Application Team
  * @brief   Header for main.c file.
  *          This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) Puya Semiconductor Co.
  * All rights reserved.</center></h2>
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "py32f4xx_hal.h"
#include "py32f403xx_Start_Kit.h"

/* Private includes ----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Exported variables prototypes ---------------------------------------------*/


/* Exported functions prototypes ---------------------------------------------*/
void APP_ErrorHandler(void);

/* Private defines -----------------------------------------------------------*/
/***** READ MEMORY Operations ************************************************/
#define P25Q64_READ_CMD                        0x03U   /* Normal Read Address                    */
#define P25Q64_FAST_READ_CMD                   0x0BU   /* Fast Read Address                      */
#define P25Q64_DTR_FAST_READ_CMD               0x0DU   /* DTR Fast Read Address                  */
#define P25Q64_1I2O_FAST_READ_CMD              0x3BU   /* Dual Output Fast Read Address          */
#define P25Q64_2IO_FAST_READ_CMD               0xBBU   /* Dual Input/Output Fast Read Address    */
#define P25Q64_1I4O_FAST_READ_CMD              0x6BU   /* Quad Output Fast Read Address          */
#define P25Q64_4IO_FAST_READ_CMD               0xEBU   /* Quad Input/Output Fast Read Address    */
#define P25Q64_4IO_WORD_READ_CMD               0xE7U   /* Quad Input/Output Word Read Address    */

/***** WRITE Operations ******************************************************/
#define P25Q64_WRITE_ENABLE_CMD                0x06U   /* Write Enable                           */
#define P25Q64_WRITE_DISABLE_CMD               0x04U   /* Write Disable                          */

/***** PROGRAM Operations ****************************************************/
#define P25Q64_PAGE_PROG_CMD                   0x02U   /* Page Program Address                   */
#define P25Q64_DUAL_INPUT_PAGE_PROG_CMD        0xA2U   /* Dual Input Page Program Address        */
#define P25Q64_QUAD_INPUT_PAGE_PROG_CMD        0x32U   /* Quad Input Page Program Address        */

/***** ERASE Operations ******************************************************/
#define P25Q64_PAGE_ERASE_256_CMD              0x81U   /* SubSector Erase 32KB 3/4 Byte Address  */
#define P25Q64_SECTOR_ERASE_4K_CMD             0x20U   /* Sector Erase 4KB Byte Address          */
#define P25Q64_SUBBLOCK_ERASE_32K_CMD          0x52U   /* SUBBLOCK Erase 32KB Byte Address       */
#define P25Q64_BLOCK_ERASE_64K_CMD             0xD8U   /* BLOCK Erase 64KB Byte Address          */
#define P25Q64_CHIP_ERASE_CMD                  0x60U   /* CHIP Erase                             */

/***** READ REGISTER Operations **********************************************/
#define P25Q64_READ_STATUS_REG_CMD             0x05U   /* Read Status Register                   */
#define P25Q64_READ_STATUS_REG_1_CMD           0x35U   /* Read Status Register1                  */

/***** WRITE REGISTER Operations *********************************************/
#define P25Q64_WRITE_STATUS_REG_CMD            0x01U   /* Write Status Register                  */
#define P25Q64_WRITE_STATUS_REG_1_CMD          0x31U   /* Write Status Register1                 */

/* Default dummy clocks cycles */
#define DUMMY_CLOCK_CYCLES_READ                8
#define DUMMY_CLOCK_CYCLES_READ_DTR            6


/* Page size */
#define P25Q64_PAGE_SIZE                       256

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
