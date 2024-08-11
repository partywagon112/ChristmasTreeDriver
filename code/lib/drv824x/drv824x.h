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
 *  drv824x.h - Declaration file for utility functions and global variables
 *  DRV824x-Q1EVM_FIRMWARE
 *  1/12/2021
 *  updated 1/17/2022 by Murugavel Raju
 *
 ******************************************************************************/

#ifndef DRV824X_DRV824X_H_
#define DRV824X_DRV824X_H_

/******************************************************************************
 * HEADER FILES
 *****************************************************************************/
#include "drv824x/drv824x-RegMap.h"
#include "msp430.h"


/******************************************************************************
 * MACROS
 *****************************************************************************/

/*
 * GPIO Port 1 Definitions
 */
#define TX          BIT1    /**<P1.1: UART Transmit pin*/
#define RX          BIT2    /**<P1.2: UART Receive pin*/
#define PUSH        BIT3    /**<P1.3*/
#define SCLK        BIT5    /**<Pin-5,P1.5: Serial clock*/
#define SDO         BIT6    /**<Pin-3,P1.6: Serial data output*/
#define SDI         BIT7    /**<Pin-4,P1.7: Serial data input*/

/*
 * GPIO Port 2 Definitions
 */
#define nSCS        BIT0    /**<P2.0: SPI Chip Select*/
#define PHIN2       BIT2    /**<P2.2: PH/IN2 output (PWM -> TA1.1 - TA1CCR1)*/
#define nFAULT      BIT3    /**<P2.3: Fault indication pin*/
#define nSLEEP      BIT4    /**<P2.4: Sleep input*/
#define STATUS      BIT5    /**<P2.5: Status LED */
#define ENIN1       BIT6    /**<P2.6: EN/IN1 output (PWM -> TA0.1 - TA0CCR1) */
#define DRVOFF      BIT7    /**<P2.7: DRVOFF (H-bridge disable) output*/

#define _PWM_TIMER_PERIOD 100
#define PWM_TIMER_PERIOD (_PWM_TIMER_PERIOD-1)
#define PWM_SCALE (_PWM_TIMER_PERIOD/100)

/*
 * GPIO Port 3 Definitions
 */
#define EVMID1      BIT0    /**<P3.0: ID1 input */
#define EVMID2      BIT1    /**<P3.1: ID2 input */
#define EVMID3      BIT2    /**<P3.2: ID3 input */

/*SPI Protocol*/
#define SPI_BUSY_FLAG       0x01         /**<User define flag to check SPI busy status*/

#define SPI_ADDRESS_MASK    0x3F00       /**< Mask for SPI register address bit */
#define SPI_ADDRESS_POS     8           /**< Position for SPI register address bit */
#define SPI_DATA_MASK       0x00FF       /**< Mask for SPI register data bit */
#define SPI_DATA_POS        0x0          /**< Position for SPI register data bit */
#define SPI_RW_BIT_MASK     0x4000       /**< Mask for SPI register read write indication bit */

#define set_nSLEEP_hi       (P2OUT |= nSLEEP)
#define set_nSLEEP_lo       (P2OUT &= ~nSLEEP)

#define set_DRVOFF_hi       (P2OUT |= DRVOFF)
#define set_DRVOFF_lo       (P2OUT &= ~DRVOFF)

#define read_FAULT_pin      (P2IN & nFAULT)
#define read_nSLEEP_pin     (P2IN & nSLEEP)
#define read_DRVOFF_pin     (P2IN & DRVOFF)

#define read_EVMID1_pin     (P3IN & EVMID1)
#define read_EVMID2_pin     (P3IN & EVMID2)
#define read_EVMID3_pin     (P3IN & EVMID3)

#define MODE_PHEN           0
#define MODE_INDEPENDENT    1
// #define MODE_PWM            2 // for 2p0
#define MODE_PWM            3

#define MAX_PWM_DUTY        100
#define MIN_PWM_DUTY        0

#define DRV8243S            8243
#define DRV8244S            8244
#define DRV8245S            8245
#define DRV8143S            8143
#define DRV8144S            8144
#define DRV8145S            8145

#define DRV8243P            823
#define DRV8244P            824
#define DRV8245P            825
#define DRV8143P            813
#define DRV8144P            814
#define DRV8145P            815

// -H version are in hex...
#define DRV8243H            0x8243
#define DRV8244H            0x8244
#define DRV8245H            0x8245
#define DRV8143H            0x8143
#define DRV8144H            0x8144
#define DRV8145H            0x8145

#define HALF_BRIDGE         ('H')
#define FULL_BRIDGE         ('F')

#define REV_PG1_0           0
#define REV_PG2_0           2

#define SPI_INTERFACE       1
#define HW_INTERFACE        2
#define UNDEFINED           0

#define RESET_TIMEOUT       224     // cycle counts based on 16MHz clock and while loop execution

/******************************************************************************
* DATA STRUCTURES
 *****************************************************************************/
/*
 * Ramp duty controller
 */
typedef struct DRV824x_RMPCNTL
{
  volatile uint8_t rampDelayMax;    // Parameter: Maximum delay rate (0-25)
  volatile uint8_t rampDelayCount;  // Variable: Incremental delay (0-25)
  volatile uint8_t setpointValue;   // Output: Target output(0-100)

} RMPCNTL_t;

typedef struct DRV824x_Device_Obj
{
  float firmwareVersion;

  /* Device Parameters*/
  volatile uint16_t deviceID;   // device ID value
  volatile uint8_t bridgeMode;  // GUI bridge mode setting
  volatile uint8_t nSleep;      // 0: Device Sleep , 1: Device Awake
  volatile uint8_t nFault;      // 0: fault , 1: No fault
  volatile uint8_t drvoff;      // tied to DRVOFF pin
  volatile uint8_t startMotor;  // software control to enable motor
  volatile uint8_t clrFlt;      // clear button binding value
  volatile uint8_t ROLP_PU1;    // pull up resistor for OUT1
  volatile uint8_t ROLP_PD1;    // pull down resistor for OUT1
  volatile uint8_t ROLP_PU2;    // pull up resistor for OUT2
  volatile uint8_t ROLP_PD2;    // pull down resistor for OUT2
  volatile uint8_t en_in1_diag; // set, reset IN1 pin for passive diagnostics
  volatile uint8_t ph_in2_diag; // set, reset IN2 pin for passive diagnostics
  volatile uint8_t Loadhighlow; // low side and high side load selection switch information
  volatile uint16_t ipropiAdc;  // moving average result for Iout using IPROPI
  volatile uint16_t ipropiAdcmax; // peak result for Iout using IPROPI

  /*Ramp controller*/
  RMPCNTL_t rampInput1;
  RMPCNTL_t rampInput2;

  /*Control Reference*/
  volatile uint8_t en_in1;      // in1 PWM duty cycle
  volatile uint8_t ph_in2;      // in2 PWM duty cycle
  volatile uint8_t ph;          // direction(phase)
  volatile uint8_t pre_ph;      // previous direction (phase)
  volatile char PWMfreq;        // PWM frequency

  volatile bool startPassiveDiagnostics;
  volatile bool active;

  /* Register Read Write */
   uint16_t   ManWriteAddr;
   uint16_t   ManReadAddr;
   uint16_t   ManWriteData;
   uint16_t   ManReadData;
   bool       WriteCmd;
   bool       ManWriteCmd;
   bool       ManReadCmd;
   bool       ReadCmd;

   uint16_t   revID;

} DRV824x_Q1_Device_Obj_t;

/******************************************************************************
 * EXTERN DECLARATIONS
 *****************************************************************************/
extern volatile DRV824x_Q1_Device_Obj_t gBrushedObj;
extern volatile DRV824x_Q1_REG_t gDeviceSpiReg;

void gui_deviceControlUpdateTask(void);
void gui_regUpdateTask(void);
void gui_regRead(void);
void setBridgeMode(uint8_t bridgeMode);
void initMcu(void);
void firmwareInit(void);
void getEVMID(void);
int drv824xq1_init(void);
void updatePWM(void);
void drv_startPWM(void);
void drv_stopPWM(void);
void drv_setCtrlMode(void);
inline void drv_RESET(void);
inline void drv_PHENRampControl(void);
void drv_rampControl(volatile RMPCNTL_t* rmpCntl, uint8_t targetValue);
inline void drv824xADCTriggerCtrl(void);
uint16_t spi_readRegister(uint8_t address);
uint16_t spi_writeRegister(uint8_t address, uint16_t data);
void cmdRxIsr();
void uartTxByte(unsigned char byte);
void ipropi_calc(void);
void PassiveDiagnostics(void);

#endif /* DRV824X_DRV824X_H_ */
