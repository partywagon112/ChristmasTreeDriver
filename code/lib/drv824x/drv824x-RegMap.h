/* --COPYRIGHT--,BSD
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

/*******************************************************************************
 *
 *  drv824x-RegMap.h - Declaration file for utility functions and global variables
 *  DRV824x-Q1EVM_FIRMWARE
 *  1/12/2021
 *  updated 1/17/2022 by Murugavel Raju
 *
 ******************************************************************************/
#ifndef DRV824X_DRV824X_REGMAP_H_
#define DRV824X_DRV824X_REGMAP_H_

#include "../dataTypeDefinition.h"

/*********************************************************************************************************
 * MACROs
 *********************************************************************************************************/

/*********************************************************************************************************
 * DRV824x SPI Register ADDRESS
 *********************************************************************************************************/

#define SPI_REG_DEVICE_ID                       (uint8_t)(0x00)         /* Device ID Register */
#define SPI_REG_FAULT_SUMMARY                   (uint8_t)(0x01)         /* Fault Summary Register */
#define SPI_REG_STATUS1                         (uint8_t)(0x02)         /* STATUS1 Register */
#define SPI_REG_STATUS2                         (uint8_t)(0x03)         /* STATUS2 Register */
#define SPI_REG_COMMAND                         (uint8_t)(0x08)         /* COMMAND Register */
#define SPI_REG_SPI_IN                          (uint8_t)(0x09)         /* SPI Input control Register */
#define SPI_REG_CONFIG1                         (uint8_t)(0x0A)         /* CONFIG1 Register   */
#define SPI_REG_CONFIG2                         (uint8_t)(0x0B)         /* CONFIG2 Register   */
#define SPI_REG_CONFIG3                         (uint8_t)(0x0C)         /* CONFIG3 Register   */
#define SPI_REG_CONFIG4                         (uint8_t)(0x0D)         /* CONFIG4 Register   */

/* Mask for Register Bit Definitions */

/* SPI_REG_00 : Device ID Register */
#define REV_ID_MASK         (0x07)
#define DEV_ID_MASK         (0xF8)
#define DEV_ID_SHIFT        (3)
#define REV_ID_SHIFT        (0)


/* SPI_REG_01 : Fault Summary Register */
#define OLA_MASK            (0x01)         /* Open load fault indicator found during active state*/
#define TSD_MASK            (0x02)         /* Over temperature fault indicator*/
#define OCP_MASK            (0x04)         /* Over current fault indicator*/
#define VMUV_MASK           (0x08)         /* VM under voltage fault condition*/
#define VMOV_MASK           (0x10)         /* VM over voltage fault condition*/
#define FAULT_MASK          (0x20)         /* Fault indicator.  Mirrors the nFAULT pin*/
#define POR_MASK            (0x40)         /* Indicates power-on-reset condition*/
#define SPI_ERR_MASK        (0x80)         /* SPI communications fault in previous frame detected*/

/* SPI_REG_02 : STATUS1 Register */
#define OCP_L2_MASK         (0x01)         /* Over current low-side half bridge 2 (OUT2->GND path)*/
#define OCP_H2_MASK         (0x02)         /* Over current high-side half bridge 2 (VM->OUT2 path)*/
#define OCP_L1_MASK         (0x04)         /* Over current low-side half bridge 1 (OUT1->GND path)*/
#define OCP_H1_MASK         (0x08)         /* Over current high-side half bridge 1 (VM->OUT1 path)*/
#define OLP_CMP_MASK_1      (0x10)         /* 1p0 OLP comparator output status. Latched when register read is commanded*/
#define ACTIVE              (0x10)         /* 2p0 Device enabled and active*/
#define ITRIP_CMP_MASK      (0x20)         /* ITRIP comparator output status.  Latched when register read is commanded*/
#define OLA2_MASK           (0x40)         /* Open load fault indicator on OUT2 during active state*/
#define OLA1_MASK           (0x80)         /* Open load fault indicator on OUT1 during active state*/

/* SPI_REG_03 : STATUS2 Register 2P0 */
#define OLP_CMP_MASK_2      (0x01)         /* OLP comparator output status. Latched when register read is commanded*/
#define ACTIVE              (0x10)         /* Device enabled and active*/
#define DRVOFF_STAT         (0x80)         /* Open load fault indicator on OUT1 during active state*/

/* SPI_REG_08 : COMMAND Register */
#define REG_LOCK_MASK       (0x3)
#define REG_LOCK_SHIFT      (0)
#define SPI_IN_LOCK_MASK    (0x18)
#define SPI_IN_LOCK_SHIFT   (1)
#define CLR_FLT_MASK        (0x80)
#define CLR_FLT_SHIFT       (7)

/* SPI_REG_09 : SPI_IN Register */
#define S_PH_IN2_MASK       (0x1)
#define S_PH_IN2_SHIFT      (0)
#define S_EN_IN_1_MASK      (0x2)
#define S_EN_IN_1_SHIFT     (1)
#define S_DRVOFF2_MASK      (0x4)
#define S_DRVOFF2_SHIFT     (2)
#define S_DRVOFF_MASK       (0x8)
#define S_DRVOFF_SHIFT      (3)

/* SPI_REG_0C : CONFIG3 Register */
#define S_MODE_MASK         (0x3)
#define S_MODE_SHIFT        (0)
#define S_SR_MASK           (0x1C)
#define S_SR_SHIFT          (2)
#define TOFF_MASK           (0xC0)
#define TOFF_SHIFT          (6)

/* SPI_REG_0D : CONFIG4 Register */
#define PH_IN2_SEL_MASK     (0x1)
#define PH_IN2_SEL_SHIFT    (0)
#define EN_IN1_SEL_MASK     (0x2)
#define EN_IN1_SEL_SHIFT    (1)
#define DRVOFF_SEL_MASK     (0x4)
#define DRVOFF_SEL_SHIFT    (2)
#define OCP_SEL_MASK        (0x18)
#define OCP_SEL_SHIFT       (3)
#define TOCP_SEL_MASK       (0xC0)
#define TOCP_SEL_SHIFT      (6)


#define DRV8243             (0x6)
#define DRV8244             (0x8)
#define DRV8245             (0xA)
#define DRV8143             (0x17)
#define DRV8144             (0x19)
#define DRV8145             (0x1B)

/*******************************************************************************
 * Structure
 *******************************************************************************/

/**
 *  Structure initializing the DRV824x registers.
 */
typedef struct DRV824x_REG
{
    uint8_t device_id_reg;
    uint8_t fault_summary_reg;
    uint8_t status1_reg;
    uint8_t status2_reg;
    uint8_t command_reg;
    uint8_t spi_in_reg;
    uint8_t config1_reg;
    uint8_t config2_reg;
    uint8_t config3_reg;
    uint8_t config4_reg;
} DRV824x_Q1_REG_t;

#endif // DRV824X_DRV824X_REGMAP_H_
