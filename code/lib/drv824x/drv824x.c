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

/********************************************************************************
 *
 *  drv824x.c - Definition file for utility functions
 *  DRV824x-Q1EVM_FIRMWARE
 *  1/12/2021
 *  updated 1/17/2022 Murugavel Raju
 *
 *******************************************************************************/
#define FIRMWARE_VERSION 0.60                  // updates to support 2p0, new features in v0.5.0 GUI app included

#include "drv824x.h"
#include "Serial_Cmd_Monitor.h"

volatile uint8_t gInterfaceType = UNDEFINED;   // container for SPI or HW device interface type
volatile uint8_t gBridgeType = UNDEFINED;      // full-bridge or half-bridge?

/********************************************************************************
 * Data Structures exported for GUI Composer
 *******************************************************************************/
#pragma DATA_SECTION(gBrushedObj,".drvVarSection");
#pragma DATA_SECTION(gDeviceSpiReg,".drvRegSection");
volatile DRV824x_Q1_REG_t gDeviceSpiReg;
volatile DRV824x_Q1_Device_Obj_t gBrushedObj;


/*******************************************************************************
 * API IMPLEMENTATION
 *******************************************************************************/
volatile char prev_in1_duty = 0;               // for duty cycle backup
volatile char prev_in2_duty = 0;               // for duty cycle backup
volatile char PDflag = 0;                      // for passive diagnostics

/*
 * gui_deviceControlUpdateTask
 *
 * Synchronize values in the global data structure written to by the GUI with
 * the hardware.
 */
void gui_deviceControlUpdateTask()
{
    // read_pin macros return raw byte port value -> convert to plain binary matching data structure flags
    uint8_t drvoffState = read_DRVOFF_pin;
    uint8_t nSleepState = read_nSLEEP_pin;

    if (drvoffState > 0)
        drvoffState = 1;
    if (nSleepState > 0)
        nSleepState = 1;

    // sync DRVOFF in data structure with actual pin state
    if (gBrushedObj.drvoff != drvoffState)
    {
        if (gBrushedObj.drvoff)
        {
            if (gBrushedObj.startMotor && gBrushedObj.active)
            {
                set_DRVOFF_hi;                     // DRVOFF = 1, driver HiZ
                __delay_cycles(160000);            // 10ms delay
                prev_in1_duty = gBrushedObj.en_in1;// back up duty cycle setting
                prev_in2_duty = gBrushedObj.ph_in2;// back up duty cycle setting
                gBrushedObj.en_in1 = 0;            // Reset GUI EN/IN1 PWM duty cycle settings
                gBrushedObj.ph_in2 = 0;            // Reset GUI PH/IN2 PWM duty cycle settings
            }
            else
            {
                if ((gBrushedObj.bridgeMode == MODE_PWM && gBrushedObj.revID == 0) || (gBrushedObj.bridgeMode == MODE_INDEPENDENT && gBrushedObj.Loadhighlow))
                {
                    P2OUT |= (ENIN1+PHIN2);        // ENIN1 and PHIN2 set to high when bridge is HiZ in these bridge modes
                    P2SEL &= ~(ENIN1+PHIN2);
                }
                else                               // ENIN1 and PHIN2 reset to low when bridge is HiZ in rest of the bridge modes
                {
                    P2OUT &= ~(ENIN1+PHIN2);
                    P2SEL &= ~(ENIN1+PHIN2);
                }
                __delay_cycles(160);               // 10us delay for IO settling*/
                set_DRVOFF_hi;                     // DRVOFF = 1, driver HiZ
            }
        }
        else
        {
            if (!gBrushedObj.startMotor)
            {
                if ((gBrushedObj.bridgeMode == MODE_PWM && gBrushedObj.revID == 0) || (gBrushedObj.bridgeMode == MODE_INDEPENDENT && gBrushedObj.Loadhighlow == 1))
                {
                    P2OUT |= (ENIN1+PHIN2);        // ENIN1 and PHIN2 set to high for these defined bridge modes
                    P2SEL &= ~(ENIN1+PHIN2);
                }
                else
                {
                    P2OUT &= ~(ENIN1+PHIN2);       // ENIN1 and PHIN2 reset to low for rest of the bridge modes
                    P2SEL &= ~(ENIN1+PHIN2);
                }
            }
            else
            {
                gBrushedObj.en_in1 = prev_in1_duty;// Restore EN/IN1 PWM duty cycle settings
                gBrushedObj.ph_in2 = prev_in2_duty;// Restore PH/IN2 PWM duty cycle settings
            }

            __delay_cycles(160000);                // 10ms delay
            set_DRVOFF_lo;                         // DRVOFF = 0, driver active
        }
    } // end

    // sync nSLEEP in data structure with actual pin state
    if (gBrushedObj.nSleep != nSleepState)
    {
        if (gBrushedObj.nSleep && !nSleepState)
        {
            /*
             * Bring DRV out of sleep and clear nFAULT
             */
            set_nSLEEP_hi;
            __delay_cycles(20000);             // 1.25ms delay
            if (read_FAULT_pin == 0)
                drv_RESET();                   // clear nFAULT
        }
        else if (!gBrushedObj.nSleep && nSleepState)
        {
            // enter sleep mode
            set_DRVOFF_hi;
            set_nSLEEP_lo;
        }
    }

    // Select PWM frequency 20 kHz, 10 kHz, 5 kHz and 2.5 kHz
    if (!gBrushedObj.active && drvoffState && !gBrushedObj.en_in1 && !gBrushedObj.ph_in2)
    {
        switch (gBrushedObj.PWMfreq)
        {
        case 0:
            TA0CTL    &= ~ID_3;
            TA1CTL    &= ~ID_3;
            TA0CTL    |= ID_0 | TACLR;
            TA1CTL    |= ID_0 | TACLR;
            break;
        case 1:
            TA0CTL    &= ~ID_3;
            TA1CTL    &= ~ID_3;
            TA0CTL    |= ID_1 | TACLR;
            TA1CTL    |= ID_1 | TACLR;
            break;
        case 2:
            TA0CTL    &= ~ID_3;
            TA1CTL    &= ~ID_3;
            TA0CTL    |= ID_2 | TACLR;
            TA1CTL    |= ID_2 | TACLR;
            break;
        case 3:
            TA0CTL    |= ID_3 | TACLR;
            TA1CTL    |= ID_3 | TACLR;
            break;
        default:
            break;
        }
    }

    if (gInterfaceType == SPI_INTERFACE)
    {
        if (gBridgeType == HALF_BRIDGE)
        {
            gBrushedObj.bridgeMode = MODE_INDEPENDENT;
        }
        if (gBridgeType == FULL_BRIDGE)
        {
            if (gBrushedObj.bridgeMode != (gDeviceSpiReg.config3_reg & S_MODE_MASK))
            setBridgeMode(gBrushedObj.bridgeMode);
        }
    }
} // end gui_deviceControlUpdateTask()

void gui_regWrite(void)
{
    spi_writeRegister(SPI_REG_COMMAND, gDeviceSpiReg.command_reg);
    spi_writeRegister(SPI_REG_SPI_IN, gDeviceSpiReg.spi_in_reg);
    spi_writeRegister(SPI_REG_CONFIG1, gDeviceSpiReg.config1_reg);
    spi_writeRegister(SPI_REG_CONFIG2, gDeviceSpiReg.config2_reg);
    spi_writeRegister(SPI_REG_CONFIG3, gDeviceSpiReg.config3_reg);
    if (gBrushedObj.revID > REV_PG1_0)
        spi_writeRegister(SPI_REG_CONFIG4, gDeviceSpiReg.config4_reg);
} // end gui_regWrite

void gui_regRead(void)
{
    gDeviceSpiReg.device_id_reg = (uint8_t) spi_readRegister(SPI_REG_DEVICE_ID);
    gDeviceSpiReg.fault_summary_reg = (uint8_t) spi_readRegister(SPI_REG_FAULT_SUMMARY);
    gDeviceSpiReg.status1_reg = (uint8_t) spi_readRegister(SPI_REG_STATUS1);
    if (gBrushedObj.revID > REV_PG1_0)
        gDeviceSpiReg.status2_reg = (uint8_t) spi_readRegister(SPI_REG_STATUS2);
    gDeviceSpiReg.command_reg = (uint8_t) spi_readRegister(SPI_REG_COMMAND);
    gDeviceSpiReg.spi_in_reg = (uint8_t) spi_readRegister(SPI_REG_SPI_IN);
    gDeviceSpiReg.config1_reg = (uint8_t) spi_readRegister(SPI_REG_CONFIG1);
    gDeviceSpiReg.config2_reg = (uint8_t) spi_readRegister(SPI_REG_CONFIG2);
    gDeviceSpiReg.config3_reg = (uint8_t) spi_readRegister(SPI_REG_CONFIG3);
    if (gBrushedObj.revID > REV_PG1_0)
        gDeviceSpiReg.config4_reg = (uint8_t) spi_readRegister(SPI_REG_CONFIG4);
} // end gui_regRead( )

/**
 * This function handles syncing the device registers with the GUI
 */
void gui_regUpdateTask(void)
{
    uint16_t drvDataNew;
    uint8_t drvRegAddr;

    /*
     * nSleep == 0 --> DRV is sleeping
     * nSleep == 1 --> DRV is awake
     */
    if (gBrushedObj.nSleep == 1)
    {
        /* Handle Clear Faults bit if set by GUI*/
        if ((gDeviceSpiReg.command_reg & CLR_FLT_MASK) ||
                (gBrushedObj.clrFlt == 1))
        {
            drv_RESET();                       // clear nFAULT
            gBrushedObj.clrFlt = 0;
        }

        if (read_FAULT_pin >> 3)
        {
            P2IFG &= ~nFAULT;                  // clear nFAULT IFG
            P2IE |= nFAULT;                    // nFAULT interrupt is enabled to detect a new fault
        }

        // Write to all registers at once
        if (gBrushedObj.WriteCmd)
        {
            gui_regWrite();
            gBrushedObj.WriteCmd = false;
        }

        // Write to single register
        if (gBrushedObj.ManWriteCmd)
        {
            drvRegAddr = gBrushedObj.ManWriteAddr;
            drvDataNew = gBrushedObj.ManWriteData;

            spi_writeRegister((uint8_t) drvRegAddr,
                                    (uint16_t) drvDataNew);

            switch (drvRegAddr)
            {
            case SPI_REG_COMMAND:
                gDeviceSpiReg.command_reg = drvDataNew;
                break;
            case SPI_REG_SPI_IN:
                gDeviceSpiReg.spi_in_reg = drvDataNew;
                break;
            case SPI_REG_CONFIG1:
                gDeviceSpiReg.config1_reg = drvDataNew;
                break;
            case SPI_REG_CONFIG2:
                gDeviceSpiReg.config2_reg = drvDataNew;
                break;
            case SPI_REG_CONFIG3:
                gDeviceSpiReg.config3_reg = drvDataNew;
                gBrushedObj.bridgeMode = (gDeviceSpiReg.config3_reg & S_MODE_MASK) >> S_MODE_SHIFT;
                break;
            case SPI_REG_CONFIG4:
                gDeviceSpiReg.config4_reg = drvDataNew;
                break;
            default:
                break;
            }
            gBrushedObj.ManWriteCmd = false;
        }

        // Read all registers at once
        if (gBrushedObj.ReadCmd)
        {
            gui_regRead();
            gBrushedObj.ReadCmd = false;
        }

        // Read single register
        if (gBrushedObj.ManReadCmd)
        {
            drvRegAddr = gBrushedObj.ManReadAddr;
            drvDataNew = spi_readRegister((uint8_t) drvRegAddr);

            gBrushedObj.ManReadData = drvDataNew;
            switch (drvRegAddr)
            {
            case SPI_REG_DEVICE_ID:
                gDeviceSpiReg.device_id_reg = drvDataNew;
                break;
            case SPI_REG_FAULT_SUMMARY:
                gDeviceSpiReg.fault_summary_reg = drvDataNew;
                break;
            case SPI_REG_STATUS1:
                gDeviceSpiReg.status1_reg = drvDataNew;
                break;
            case SPI_REG_STATUS2:
                gDeviceSpiReg.status2_reg = drvDataNew;
                break;
            case SPI_REG_COMMAND:
                gDeviceSpiReg.command_reg = drvDataNew;
                break;
            case SPI_REG_SPI_IN:
                gDeviceSpiReg.spi_in_reg = drvDataNew;
                break;
            case SPI_REG_CONFIG1:
                gDeviceSpiReg.config1_reg = drvDataNew;
                break;
            case SPI_REG_CONFIG2:
                gDeviceSpiReg.config2_reg = drvDataNew;
                break;
            case SPI_REG_CONFIG3 :
                gDeviceSpiReg.config3_reg = drvDataNew;
                break;
            case SPI_REG_CONFIG4 :
                gDeviceSpiReg.config4_reg = drvDataNew;
                break;
             default:
                break;
            }
            gBrushedObj.ManReadCmd = false;
        }
    }
} // end gui_regUpdateTask( )

/********************************************************************************
 * LOCAL ROUTINES
 *******************************************************************************/
void setBridgeMode(uint8_t bridgeMode)
{
    if (gInterfaceType == SPI_INTERFACE)
    {
        gDeviceSpiReg.config3_reg = ((uint8_t) spi_readRegister(SPI_REG_CONFIG3) & 0xfc);
        gDeviceSpiReg.config3_reg |= (bridgeMode & S_MODE_MASK) << S_MODE_SHIFT;
        spi_writeRegister(SPI_REG_CONFIG3, gDeviceSpiReg.config3_reg);
    }

    gBrushedObj.bridgeMode = bridgeMode;
}

uint8_t getBridgeMode(void)
{
    uint8_t ret = 0;

    if (gInterfaceType == SPI_INTERFACE)
    {
        gDeviceSpiReg.config3_reg = (uint8_t) spi_readRegister(SPI_REG_CONFIG3);
        ret = ((gDeviceSpiReg.config3_reg & S_MODE_MASK) >> S_MODE_SHIFT);
    }

    else if (gInterfaceType != SPI_INTERFACE && gBridgeType == HALF_BRIDGE) // choose independent mode format for half bridge hardware variants
    {
        ret = MODE_INDEPENDENT;
    }

    else
        // firmware does not know the jumper position -> this value would be set
        // by the GUI
        ret = gBrushedObj.bridgeMode;

    return ret;
}

void getEVMID(void)
{
    /* Identify the Device connected
     *
     * SPI devices can be queried by their device_id register
     * and this step is redundant.
     *
     * This method is used for Hardware interface devices
     * that lack a user register interface and can be a redundant
     * check for SPI devices.
     *
     * Initially each ID pin is set to output a logic high,
     * with its pull-up enabled.  If the ID pin is pulled-down
     * externally, the pin state will be low.  If the ID pin is
     * floating or pulled-up externally, the pin will be high.
     *
     * Next we drive a logic low on the ID pin, and read each pin.
     * If the pin is pulled down, or externally floating, the
     * pin value will be 0.  If the pin is pulled-up, the pin
     * will be high.
     *
     * From the above, we can determine if a pin is externally
     * pulled-up, pulled-down or floating.  We ignore the
     * indeterminate case because what monster would populate both
     * pull-up and pull-downs on the ID pins?
     *
     *
     *   IDx=1       IDx=0      Pin state
     *     1            1       Pull-up installed
     *     0            0       Pull-down installed
     *     1            0       Pin floating
     *
     */

    uint8_t evmID[2][3] = {0};
    uint8_t i = 0, j = 0;

    for (i = 0; i < 2; i++)
    {
        if (i == 0)
        {
            // Start with driving the pins high.
            P3OUT = (EVMID1 | EVMID2 | EVMID3);
            P3REN |= (EVMID1 | EVMID2 | EVMID3);
        }
        else
            P3OUT &= ~(EVMID1 | EVMID2 | EVMID3);

        __delay_cycles(8000);

        j = 0;
        evmID[i][j] = read_EVMID1_pin; j++;
        evmID[i][j] = read_EVMID2_pin; j++;
        evmID[i][j] = read_EVMID3_pin;
    }

    /*
     pin state  ID1      ID2     ID3
           H    FullBrg  8x43    rev0
           L    HalfBrg  8x44    rev1
           X    SPI      8x45    rev2

                      ID1  ID2
        DRV8243S/P     X    H
        DRV8243H       H    H
        DRV8244S/P     X    L
        DRV8244H       H    L
        DRV8245S/P     X    X
        DRV8245H       H    X
        DRV8144S/P     X    L
        DRV8144H       L    L
        DRV8145S/P     X    X
        DRV8145H       L    X

        Note: REV can also denote a completely different
        ID pin decode scheme if rev0 is unsufficient for
        some reason.

        Note 2: Notice above that there are duplicates because
        we do rely on the SPI device telling us the device
        type.
     */

    /*
     * First, check if this is a SPI device, then look at strapping
     * resistors to decode populated device.
     *
     * Note Rev E1 boards did not have strapping resistors, and will
     * default to DRV8243H.  Modify this behavior as needed.
     */

    gDeviceSpiReg.device_id_reg = (uint8_t) spi_readRegister(SPI_REG_DEVICE_ID);
    gBrushedObj.deviceID = (gDeviceSpiReg.device_id_reg & DEV_ID_MASK) >> DEV_ID_SHIFT;
    gBrushedObj.revID = (gDeviceSpiReg.device_id_reg & REV_ID_MASK) >> REV_ID_SHIFT;

    switch(gBrushedObj.deviceID)
    {
        case DRV8243:
            gBrushedObj.deviceID = DRV8243S;
            gBridgeType = FULL_BRIDGE;
            break;
        case DRV8244:
            gBrushedObj.deviceID = DRV8244S;
            gBridgeType = FULL_BRIDGE;
            break;
        case DRV8245:
            gBrushedObj.deviceID = DRV8245S;
            gBridgeType = FULL_BRIDGE;
            break;
        case DRV8143:
            gBrushedObj.deviceID = DRV8143S;
            gBridgeType = HALF_BRIDGE;
            break;
        case DRV8144:
            gBrushedObj.deviceID = DRV8144S;
            gBridgeType = HALF_BRIDGE;
            break;
        case DRV8145:
            gBrushedObj.deviceID = DRV8145S;
            gBridgeType = HALF_BRIDGE;
            break;
        default:
            // Not a SPI device - will populate below
            gBrushedObj.deviceID = 0;
    }

    // GUI Device ID for P variants
    if ((gBrushedObj.deviceID == DRV8245S) && (gBrushedObj.revID  == 0x6))
        gBrushedObj.deviceID = DRV8245P;
    if ((gBrushedObj.deviceID == DRV8244S) && (gBrushedObj.revID  == 0x6))
        gBrushedObj.deviceID = DRV8244P;
    if ((gBrushedObj.deviceID == DRV8243S) && (gBrushedObj.revID  == 0x6))
        gBrushedObj.deviceID = DRV8243P;
    if ((gBrushedObj.deviceID == DRV8145S) && (gBrushedObj.revID  == 0x6))
        gBrushedObj.deviceID = DRV8145P;
    if ((gBrushedObj.deviceID == DRV8144S) && (gBrushedObj.revID  == 0x6))
        gBrushedObj.deviceID = DRV8144P;
    if ((gBrushedObj.deviceID == DRV8143S) && (gBrushedObj.revID  == 0x6))
        gBrushedObj.deviceID = DRV8143P;

    gBrushedObj.revID &= 0x3;                  // BIT1 and BIT0 are the silicon revision designators

    if (gBrushedObj.deviceID != 0)
    {
        gInterfaceType = SPI_INTERFACE;
        return;  // SPI device: we're done
    }
    else
        gInterfaceType = HW_INTERFACE;

    // is this an E1 (all ID pins floating)?
    if (evmID[0][0] != evmID[1][0] &&
            evmID[0][1] != evmID[1][1] &&
            evmID[0][2] != evmID[1][2])
    {
        gBrushedObj.deviceID = DRV8243H;
        gBrushedObj.revID = REV_PG1_0;
        return;
    }

    // ID1
    if (evmID[0][0] != evmID[1][0])     // ID1 floating
    {
        ; // already handled this case above
    }
    else if (evmID[0][0] == 0)          // ID1 pull-down installed
        gBridgeType = HALF_BRIDGE;
    else if(evmID[0][0] == BIT0)        // ID2 pull-up installed
        gBridgeType = FULL_BRIDGE;

    // ID2
    if (evmID[0][1] != evmID[1][1])     // ID2 floating
        if (gBridgeType == FULL_BRIDGE)
            gBrushedObj.deviceID = DRV8245H;
        else
            gBrushedObj.deviceID = DRV8145H;

    else if (evmID[0][1] == 0)          // ID2 pull-down
        if (gBridgeType == FULL_BRIDGE)
            gBrushedObj.deviceID = DRV8244H;
        else
            gBrushedObj.deviceID = DRV8144H;

    else if (evmID[0][1] == BIT1)       // ID2 pull-up
        if (gBridgeType == FULL_BRIDGE)
            gBrushedObj.deviceID = DRV8243H;
        else
            gBrushedObj.deviceID = DRV8143H;

    // ID3
    if (evmID[0][2] == 0)               // ID3 pull-down
        gBrushedObj.revID = REV_PG2_0;

    else                                // ID3 pull-up or floating
        gBrushedObj.revID = REV_PG1_0;
}

/*
 * SPI initialization
 */
static inline void spiInit(void)
{
    P2OUT |= nSCS;
    P2DIR |= nSCS;

    P1SEL |= (SCLK | SDO | SDI);
    P1SEL2 |= (SCLK | SDO | SDI);

    UCB0CTL1 = UCSWRST;
    UCB0CTL0 |= UCMSB | UCMST | UCSYNC;
    UCB0CTL1 |= UCSSEL_2; // SMCLK
    UCB0BR0 |= 0x2; // 1MHz
    UCB0BR1 = 0; //
    UCB0CTL1 &= ~UCSWRST; // **Initialize USCI state machine**
}

/*
 * MCU Clock initialization
 */
static inline void clockInit(void)
{
    // 16MHz DCO required for 2MHz SMCLK
    DCOCTL = 0x00;
    BCSCTL1= CALBC1_16MHZ;
    DCOCTL= CALDCO_16MHZ;

    // 2MHz SMCLK required for PWM frequency
    BCSCTL2 |= DIVS_3;
    BCSCTL3 |= LFXT1S_2;
}

/*
 * UART initialization
 * 115200 BAUD
 */
static inline void uartInit(void)
{
    // UART
    /* P1.1 = RXD, P1.2=TXD */
    P1SEL |= TX | RX;
    P1SEL2 |= TX | RX;

    /* SMCLK */
    UCA0CTL1 |= UCSSEL_2;

    /* 2MHz 9602 */
    UCA0BR0 = 0xd0;
    UCA0BR1 = 0;

    /* Modulation UCBRSx = 1 */
    UCA0MCTL = 0x22;// UCBRS0

    /* **Initialize USCI state machine** */
    UCA0CTL1 &= ~UCSWRST;

    /* Enable USCI_A0 RX interrupt */
    IE2 |= UCA0RXIE;
}

/*
 * GPIO initialization. This block configures all the DRV8873x PINs.
 */
static inline void gpioInit()
{
    P2SEL &= ~(nSCS | PHIN2 | nFAULT | nSLEEP | STATUS | ENIN1 | DRVOFF);
    P2SEL2 &= ~(nSCS | PHIN2 | nFAULT | nSLEEP | STATUS | ENIN1 | DRVOFF);

    P2OUT = nSCS | DRVOFF;      // startup with DRVOFF asserted (disable outputs) and nSCS high
    P2OUT &= ~(PHIN2 | nFAULT| nSLEEP | STATUS | ENIN1);
    P2DIR |= (nSCS | PHIN2 | nSLEEP | STATUS | ENIN1 | DRVOFF);
    P3OUT = (EVMID1 | EVMID2 | EVMID3);
    P3REN |= (EVMID1 | EVMID2 | EVMID3);

    /* Setup PUSH button for triggering IRQ */
    // External pull up resistor 47kOhms R18
    P1DIR &= ~PUSH;   // input mode
    P1IES |= PUSH;    // falling edge
    P1IFG &= ~PUSH;   // clear IRQ that may have been generated by setting IES
    P1IE  |= PUSH;    // enable IRQ on PUSH button press

    /* Setup nFAULT for triggering IRQ external 10k pull up resistor on nFAULT*/
    P2DIR &= ~nFAULT; // input mode
    P2IES |= nFAULT;  // falling edge
    P2IFG &= ~nFAULT; // clear irq that may have been generated by setting IES


    // test code to check SMCLK on pin 1.4 --> probe point added to E2+ EVMs
    P1DIR |= BIT4;   // TP3 setup for output pulse
    P1OUT &= ~BIT4;  // TP3 low
    //P1DIR |= 0x13; // P1.0,1 and P1.4 outputs
    //P1SEL |= 0x11; // P1.0,4 ACLK, SMCLK output
}

/*
 * PWM control timer initialization. Provide PWM to control input EN_IN1 & PH_EN2
 */
static inline void pwmTimerInit(void)
{
  /*
   * TA1CTL PWM frequencies
   * These are based on 16MHz CLK and PWM_TIMER_PERIOD=100
   * PWM_TIMER_PERIOD needs to be adjusted in increments of 100, else
   * you will need to modify the scaling factor logic
   *    ID_0 --> PWM = 20KHz
   *    ID_1 --> PWM = 10KHz
   *    ID_2 --> PWM = 5KHz
   *    ID_3 --> PWM = 2.5KHz
   */

  /**< Initialize Timer1 for PH_EN2 */
  TA1CTL    = TASSEL_2 | MC_0 | ID_0 | TACLR;
  TA1CCR0   = PWM_TIMER_PERIOD;       /** < Timer period value*/
  TA1CCR1   = 0;                      /** < Timer compare value for 0 duty cycle*/
  TA1CCTL1  |= OUTMOD_7;              /** < Changed timer mode to PWM reset/set*/

  /* Initialize Timer0 for EN/IN1 */
  TA0CTL    = TASSEL_2 | MC_0 | ID_0 | TACLR;
  TA0CCR0   = PWM_TIMER_PERIOD;       /** < Timer period value*/
  TA0CCR1   = 0;                      /** < Timer compare value for 0 duty cycle*/
  TA0CCTL1  |= OUTMOD_7;              /** < [EN/IN1] Changed timer mode to PWM reset/set*/
}


/*
 * ADC10 initialization. Capture the IPROPI current value in the ADC10 ISR.
 */
static inline void adc10Init(void)
{
  /*
   * INCH_0    : ADC channel, P1.0 for IPROPI
   * SHS_0     : Select the ADC trigger with ADCSC
   * CONSEQ_2  : Single channel continuous conversion
   * ADC10CLK  : ADC10CLK
   */
    ADC10CTL1 = INCH_0 + SHS_0 + CONSEQ_2 + ADC10SSEL_0 + ADC10DIV_0;
  /*
   * Initialized ADC10 to read IPROPI1(P1.0:ADC10 analog input A0) and IPROPI2(P1.4:ADC10 analog input A4)
   * SREF_0     : Select the VCC as the user set voltage (instead of VREF)
   * ADC10SHT_2 : Set the ADC10’s sample and hold time at 16xADC10CLKs
   * MSC        : Multiple sample and coversion mode for repeated coversions
   * ADC10ON    : Turn on the ADC10 converter
   * ADC10IE    : Enables the ADC interrupt
   */
  ADC10CTL0 = SREF_0 + ADC10SHT_0 + MSC + ADC10ON + ADC10IE;

  /*
   * ADC10 Enable conversions
   */
  ADC10CTL0 |= ENC;

  /*
   * P1.0 ADC input select
   */
  ADC10AE0 |= BIT0;  // analog input A0
}

/*
 * Setup Watchdog Timer as an Interval Timer(250ms)
 * Timer Interval to Blink the STATUS LED(P2.7)
 */
static inline void watchdogTimerInit(void)
{
    WDTCTL = WDT_ADLY_1_9; /** < Configure time interval for periodic wake up and processing 1.9 ms using SMCLK*/
    IE1 |= WDTIE; /** < Interrupt enable*/
}

void initMcu(void)
{
    // Stop Watchdog Timer
    WDTCTL = WDTPW | WDTHOLD;

    // Setup CLKs
    gpioInit();
    clockInit();
    uartInit();
    spiInit();
    adc10Init();
    pwmTimerInit();
    watchdogTimerInit();
}

void firmwareInit(void)
{
    /*Device ID and version configurations*/
    //targetInfo_Init();
    gBrushedObj.firmwareVersion = FIRMWARE_VERSION;

    /*Parameters Initialization*/
    gBrushedObj.ipropiAdc        = 0;
    gBrushedObj.ipropiAdcmax     = 0;

    if (gBridgeType == HALF_BRIDGE)
    {
        gBrushedObj.bridgeMode = MODE_INDEPENDENT;
    }
    else
    {
        gBrushedObj.bridgeMode = MODE_PHEN; /*PHEN, Default Mode for FULL_BRIDGE*/
    }
    gBrushedObj.Loadhighlow      = 0;          // default low side load for 824x independent half bridge mode and 814x
    gBrushedObj.en_in1           = 0;
    gBrushedObj.ph_in2           = 0;
    gBrushedObj.ph               = 1;
    gBrushedObj.pre_ph           = 0;
    gBrushedObj.PWMfreq          = 0;

    /*Ramp controller init for en_in1*/
    gBrushedObj.rampInput1.rampDelayMax   = 25;
    gBrushedObj.rampInput1.rampDelayCount = 0;
    gBrushedObj.rampInput1.setpointValue  = 10;

    /*Ramp controller init ph_in2*/
    gBrushedObj.rampInput2.rampDelayMax = 25;
    gBrushedObj.rampInput2.rampDelayCount = 0;
    gBrushedObj.rampInput2.setpointValue = 10;
}

int drv824xq1_init(void)
{
    // disable nFAULT IRQ so it does not get asserted coming out of sleep
    P2IE  &= ~(nFAULT);                        // disable nFAULT IRQ in sleep mode

    // Bring device out of sleep mode
    set_DRVOFF_hi;
    set_nSLEEP_hi;
    __delay_cycles(1000000);

    getEVMID();

    /*
     * nFAULT is active LOW.  The MSP430 GPIO attached to nFAULT has the internal
     * pull-up enabled.
     *
     * Toggling nSLEEP will reset the DRV device, and nFAULT will be asserted.
     * If nFAULT is _NOT_ asserted after toggling nSLEEP, this likely means
     * VM is not powered and we should abort until VM is active
     * nFAULT's GPIO has its pull-up enabled
     */
    if (read_FAULT_pin == 0)
         drv_RESET();                          // clear nFAULT
    else
        return 0;                              // problem reported

    // enable nFAULT IRQ after clearing nFAULT following initial reset
    P2IFG &= ~nFAULT;
    P2IE  |= nFAULT;

    if (gInterfaceType == SPI_INTERFACE)
    {
        gBrushedObj.nSleep = 1;                // active LOW
    }
    else
    {
        gBrushedObj.nSleep = 0;                // active LOW, start H variants in sleep mode
    }
    gBrushedObj.drvoff = 1;                    // active HIGH
    gBrushedObj.ph = 1;
    gBrushedObj.startMotor = 0;
    gBrushedObj.active = false;
    gBrushedObj.bridgeMode = getBridgeMode();

    return 1;
}

void updatePWM(void)
{
    if (gBrushedObj.bridgeMode == MODE_PHEN)
    { /*PH/EN mode*/

        if (gBrushedObj.en_in1 == MAX_PWM_DUTY && TA0CCR1 == PWM_TIMER_PERIOD) // GUI and FW have reached 100% PWM
        {
            if (gBrushedObj.ph && !gBrushedObj.pre_ph) //Direction change, forward
                TA0CCR1 = PWM_TIMER_PERIOD - 1;
            else if (!gBrushedObj.ph && gBrushedObj.pre_ph) //Direction change, reverse
                TA0CCR1 = PWM_TIMER_PERIOD - 1;
            else                               // no direction change at 100% PWM, just make sure output is GPIO 100%
            {
                P2OUT |= ENIN1;                // set the output to 100%
                P2SEL &= ~ENIN1;               // Set IN1 to GPIO output
            }
        }
        else
        {
            P2SEL |= ENIN1;                    // Set IN1 to timer output
            P2SEL &= ~PHIN2;                   // Set IN2 to GPIO output
            drv_PHENRampControl();
            TA0CCR1 = gBrushedObj.rampInput1.setpointValue * PWM_SCALE;
        }
    }
    else if (gBrushedObj.bridgeMode == MODE_INDEPENDENT && gBrushedObj.Loadhighlow == 0)
    {/*Independent half bridge control Low Side Load*/
        // HB1 --> mapped to ENIN1
        if (gBrushedObj.en_in1 == MIN_PWM_DUTY && TA0CCR1 == 0)
        {
            P2OUT &= ~ENIN1;                   // set the output to 0%
            P2SEL &= ~ENIN1;                   // Set IN1 to GPIO output
        }
        else if (gBrushedObj.en_in1 == MAX_PWM_DUTY && TA0CCR1 == PWM_TIMER_PERIOD)
        {
            P2OUT |= ENIN1;                    // set the output to 100%
            P2SEL &= ~ENIN1;                   // Set IN1 to GPIO output
        }
        else
        {
            P2SEL |= ENIN1;                    // Set IN1 to timer output
            drv_rampControl(&gBrushedObj.rampInput1, gBrushedObj.en_in1);
            TA0CCR1 = gBrushedObj.rampInput1.setpointValue * PWM_SCALE;
        }

        // HB2 --> mapped to PHIN2
        if (gBrushedObj.ph_in2 == MIN_PWM_DUTY && TA1CCR1 == 0)
        {
            P2OUT &= ~PHIN2;                   // set the output to 0%
            P2SEL &= ~PHIN2;                   // Set IN1 to GPIO output
        }
        else if (gBrushedObj.ph_in2 == MAX_PWM_DUTY && TA1CCR1 == PWM_TIMER_PERIOD)
        {
            P2OUT |= PHIN2;                    // set the output to 100%
            P2SEL &= ~PHIN2;                   // Set IN2 to GPIO output
        }
        else
        {
            P2SEL |= PHIN2;                    // Set IN2 to timer output
            drv_rampControl(&gBrushedObj.rampInput2, gBrushedObj.ph_in2);
            TA1CCR1 = gBrushedObj.rampInput2.setpointValue * PWM_SCALE;
        }
    }
    else if (gBrushedObj.bridgeMode == MODE_INDEPENDENT && gBrushedObj.Loadhighlow == 1)
        {/*Independent half bridge control High Side Load*/
            // HB1 --> mapped to ENIN1
            if (gBrushedObj.en_in1 == MIN_PWM_DUTY && TA0CCR1 == 0)
            {
                P2OUT |= ENIN1;                // set the output to 100%
                P2SEL &= ~ENIN1;               // Set IN1 to GPIO output
            }
            else if (gBrushedObj.en_in1 == MAX_PWM_DUTY && TA0CCR1 == PWM_TIMER_PERIOD)
            {
                P2OUT &= ~ENIN1;               // set the output to 0%
                P2SEL &= ~ENIN1;               // Set IN1 to GPIO output
            }
            else
            {
                P2SEL |= ENIN1;                // Set IN1 to timer output
                drv_rampControl(&gBrushedObj.rampInput1, gBrushedObj.en_in1);
                TA0CCR1 = gBrushedObj.rampInput1.setpointValue * PWM_SCALE;
            }

            // HB2 --> mapped to PHIN2
            if (gBrushedObj.ph_in2 == MIN_PWM_DUTY && TA1CCR1 == 0)
            {
                P2OUT |= PHIN2;                // set the output to 100%
                P2SEL &= ~PHIN2;               // Set IN2 to GPIO output
            }
            else if (gBrushedObj.ph_in2 == MAX_PWM_DUTY && TA1CCR1 == PWM_TIMER_PERIOD)
            {
                P2OUT &= ~PHIN2;               // set the output to 0%
                P2SEL &= ~PHIN2;               // Set IN1 to GPIO output
            }
            else
            {
                P2SEL |= PHIN2;                // Set IN2 to timer output
                drv_rampControl(&gBrushedObj.rampInput2, gBrushedObj.ph_in2);
                TA1CCR1 = gBrushedObj.rampInput2.setpointValue * PWM_SCALE;
            }
        }
    else if (gBrushedObj.bridgeMode == MODE_PWM && gBrushedObj.revID == 0)
    {
        if (gBrushedObj.en_in1 == MIN_PWM_DUTY && TA0CCR1 == 0)
        {
            P2OUT |= ENIN1;                    // set the output to 100%
            P2SEL &= ~ENIN1;                   // Set IN1 to GPIO output
        }
        else
        {
            if (TA1CCR1 == 0)                  // No direction until motor has stopped completely
            {
                P2SEL |= ENIN1;                // Set IN1 to timer output
                drv_rampControl(&gBrushedObj.rampInput1, gBrushedObj.en_in1);
                TA0CCR1 = gBrushedObj.rampInput1.setpointValue * PWM_SCALE;
            }
        }

        if (gBrushedObj.ph_in2 == MIN_PWM_DUTY && TA1CCR1 == 0)
        {
            P2OUT |= PHIN2;                    // set the output to 100%
            P2SEL &= ~PHIN2;                   // Set IN2 to GPIO output
        }
        else
        {
            if (TA0CCR1 == 0)                  // No direction until motor has stopped completely
            {
                P2SEL |= PHIN2;                // Set IN2 to timer output
                drv_rampControl(&gBrushedObj.rampInput2, gBrushedObj.ph_in2);
                TA1CCR1 = gBrushedObj.rampInput2.setpointValue * PWM_SCALE;
            }
        }
    }
    else if (gBrushedObj.bridgeMode == MODE_PWM && gBrushedObj.revID > 0)
        {
            if (gBrushedObj.en_in1 == MIN_PWM_DUTY && TA0CCR1 == 0)
            {
                P2OUT &= ~ENIN1;               // set the output to 0%
                P2SEL &= ~ENIN1;               // Set IN1 to GPIO output
            }
            else
            {
                if (TA1CCR1 == 0)              // No direction change until motor has stopped completely
                {
                    P2SEL |= ENIN1;            // Set IN1 to timer output
                    drv_rampControl(&gBrushedObj.rampInput1, gBrushedObj.en_in1);
                    TA0CCR1 = gBrushedObj.rampInput1.setpointValue * PWM_SCALE;
                }
            }

            if (gBrushedObj.ph_in2 == MIN_PWM_DUTY && TA1CCR1 == 0)
            {
                P2OUT &= ~PHIN2;               // set the output to 0%
                P2SEL &= ~PHIN2;               // Set IN2 to GPIO output
            }
            else
            {
                if (TA0CCR1 == 0)              // No direction until motor has stopped completely
                {
                    P2SEL |= PHIN2;            // Set IN2 to timer output
                    drv_rampControl(&gBrushedObj.rampInput2, gBrushedObj.ph_in2);
                    TA1CCR1 = gBrushedObj.rampInput2.setpointValue * PWM_SCALE;
                }
            }
        }
}

void drv_startPWM(void)
{
    gBrushedObj.active = true;
    gBrushedObj.ipropiAdc = 0;
    gBrushedObj.rampInput1.setpointValue = MIN_PWM_DUTY;
    gBrushedObj.rampInput2.setpointValue = MIN_PWM_DUTY;

    drv_setCtrlMode();

    ADC10CTL0 |= ADC10SC;                      // start repeat single channel sequence of conversions
    TA0CTL |= TASSEL_2 | MC_1 | TACLR;         // Timer used for EN_IN1 */
    TA1CTL |= TASSEL_2 | MC_1 | TACLR;         // Timer used for PH_IN2 */
}

inline void drv_RESET(void)
{
    if (gInterfaceType == SPI_INTERFACE)
    {
        gDeviceSpiReg.command_reg = (uint8_t) spi_readRegister(SPI_REG_COMMAND);
        spi_writeRegister(SPI_REG_COMMAND, (gDeviceSpiReg.command_reg |= CLR_FLT_MASK));
        __delay_cycles(1000000);
        gui_regRead();
    }
    else
    {
        /*
         * Per datasheet:
         * ACK of nFAULT on POR with nSLEEP pulse shall be of period tRESET
         * tReset = 5us (min) - 20us (max)
         * 160 cycles = 10us based on 16MHz clock
         *
         */
        set_nSLEEP_lo;
        __delay_cycles(RESET_TIMEOUT);         // duration for nSLEEP reset pulse for HW variants
        set_nSLEEP_hi;
//        while (read_FAULT_pin == 0);           // wait until nFAULT = 1, reset completed
        // the device is ready for drive operation
    }
}

/*
 * Control duty cycle ramp for PH_EN mode
 */
inline void drv_PHENRampControl(void)
{

    if ((gBrushedObj.ph == 1) && (gBrushedObj.pre_ph == 0)) // Direction control
    {
        drv_rampControl(&gBrushedObj.rampInput1, MIN_PWM_DUTY); // EN_IN1 ramp during direction reversal
        if(gBrushedObj.rampInput1.setpointValue == MIN_PWM_DUTY)
        {
            gBrushedObj.pre_ph = 1;
            P2OUT |= PHIN2;
        }
    }
    else if ((gBrushedObj.ph == 0) && (gBrushedObj.pre_ph == 1)) // Direction control
    {
        drv_rampControl(&gBrushedObj.rampInput1, MIN_PWM_DUTY);  // EN_IN1 ramp during direction reversal
        if(gBrushedObj.rampInput1.setpointValue == MIN_PWM_DUTY)
        {
            gBrushedObj.pre_ph = 0;
            P2OUT &= ~PHIN2;
        }
    }
    else                                       // EN_IN1 ramp for PH/EN drive
    {
        if(gBrushedObj.startMotor == 1)        // motor is running
        {
            drv_rampControl(&gBrushedObj.rampInput1, gBrushedObj.en_in1);
        }
        else // Motor stop
        {
            drv_rampControl(&gBrushedObj.rampInput1, MIN_PWM_DUTY);
            if(gBrushedObj.rampInput1.setpointValue == MIN_PWM_DUTY)
            {
                TA0CTL = TASSEL_2 | MC_0 | TACLR; // Stop EN_IN1 PWM timer
                TA0CCTL1 = OUTMOD_0;
                P2OUT &= ~ENIN1;
                P2SEL &= ~ENIN1;
            }
        }
    }
}

/*
 * Function to run the RMPCntl Calculations
 */
void drv_rampControl(volatile RMPCNTL_t* rmpCntl, uint8_t targetValue)
{
    if (rmpCntl->rampDelayMax == 0)
    {
        rmpCntl->setpointValue = targetValue;
    }

    else
    {
        if (rmpCntl->rampDelayCount++ >= rmpCntl->rampDelayMax)
        {
            if (targetValue > rmpCntl->setpointValue)
            {
                rmpCntl->setpointValue++;
            }
            else if(targetValue < rmpCntl->setpointValue)
            {
                rmpCntl->setpointValue--;
            }
            rmpCntl->rampDelayCount = 0;
        }
    }
}

void drv_setCtrlMode()
{
    // Update timers based on device mode
    if (gBrushedObj.bridgeMode == MODE_INDEPENDENT && gBridgeType == FULL_BRIDGE && gBrushedObj.Loadhighlow == 0)
    { // independent 1/2 B Mode DRV824x Low side load

        gBrushedObj.ph = 0;

        // ENIN1 --> TimerA0
        P2OUT &= ~ENIN1;                       // set ENIN1 to low
        P2SEL &= ~ENIN1;                       // Set ENIN1 to GPIO outputs
        TA0CCTL1 |= OUTMOD_7;                  // ENIN1 setup as a timer reset/set PWM output

        // PHIN2 --> TimerA1
        P2OUT &= ~PHIN2;                       // set PHIN2 to low
        P2SEL &= ~PHIN2;                       // Set PHIN2 to GPIO outputs
        TA1CCTL1 |= OUTMOD_7;                  // PHIN2 setup as a timer reset/set PWM output
        TA0CCR1 = 0;
        TA1CCR1 = 0;
        P2SEL |= ENIN1 + PHIN2;                // Set ENIN1 and PHIN2 to timer outputs
        TA0CCTL1 |= CCIE;                      // Enable Timer0_A1 Interrupt --> ENIN1
        TA1CCTL1 |= CCIE;                      // Enable Timer1_A1 Interrupt --> PHIN2
    }
    else if (gBrushedObj.bridgeMode == MODE_INDEPENDENT && gBridgeType == HALF_BRIDGE && gBrushedObj.Loadhighlow == 0)
        { // half bridge DRV814x Low side load

            gBrushedObj.ph = 0;

            // ENIN1 --> TimerA0
            P2OUT &= ~ENIN1;                   // set ENIN1 to low
            P2SEL &= ~ENIN1;                   // Set ENIN1 to GPIO outputs
            // PHIN2 --> GPIO
            P2OUT &= ~PHIN2;                   // set PHIN2 to low
            P2SEL &= ~PHIN2;                   // Set PHIN2 to GPIO outputs
            TA0CCTL1 |= OUTMOD_7;              // ENIN1 setup as a timer reset/set PWM output
            TA0CCR1 = 0;
            P2SEL |= ENIN1 + PHIN2;            // Set ENIN1 and PHIN2 to timer outputs
            TA0CCTL1 |= CCIE;                  // Enable Timer0_A1 Interrupt --> ENIN1
        }
    else if (gBrushedObj.bridgeMode == MODE_INDEPENDENT && gBridgeType == FULL_BRIDGE && gBrushedObj.Loadhighlow == 1)
    { // independent 1/2 B Mode DRV824x High Side load

        gBrushedObj.ph = 0;

        // ENIN1 --> TimerA0
        P2OUT &= ~ENIN1;                       // set ENIN1 to low
        P2SEL &= ~ENIN1;                       // Set ENIN1 to GPIO outputs
        TA0CCTL1 &= ~OUTMOD_7;                 // Clear Timer output mode
        TA0CCTL1 |= OUTMOD_3;                  // Timer output set/reset mode

        // PHIN2 --> TimerA1
        P2OUT &= ~PHIN2;                       // set PHIN2 to low
        P2SEL &= ~PHIN2;                       // Set PHIN2 to GPIO outputs
        TA1CCTL1 &= ~OUTMOD_7;                 // Clear Timer output mode
        TA1CCTL1 |= OUTMOD_3;                  // Timer output set/reset mode
        TA0CCR1 = 0;
        TA1CCR1 = 0;
        P2SEL |= ENIN1 + PHIN2;                // Set ENIN1 and PHIN2 to timer outputs
        TA0CCTL1 |= CCIE;                      // Enable Timer0_A1 Interrupt --> ENIN1
        TA1CCTL1 |= CCIE;                      // Enable Timer1_A1 Interrupt --> PHIN2
    }
    else if (gBrushedObj.bridgeMode == MODE_INDEPENDENT && gBridgeType == HALF_BRIDGE && gBrushedObj.Loadhighlow == 1)
        { // half bridge DRV814x High side load

            gBrushedObj.ph = 0;

            // ENIN1 --> TimerA0
            P2OUT &= ~ENIN1;                   // set ENIN1 to low
            P2SEL &= ~ENIN1;                   // Set ENIN1 to GPIO outputs
            // PHIN2 --> GPIO
            P2OUT &= ~PHIN2;                   // set PHIN2 to low
            P2SEL &= ~PHIN2;                   // Set PHIN2 to GPIO outputs
            TA0CCTL1 &= ~OUTMOD_7;             // Clear Timer output mode
            TA0CCTL1 |= OUTMOD_3;              // Timer output set/reset mode
            TA0CCR1 = 0;
            P2SEL |= ENIN1 + PHIN2;            // Set ENIN1 and PHIN2 to timer outputs
            TA0CCTL1 |= CCIE;                  // Enable Timer0_A1 Interrupt --> ENIN1
        }
    else if (gBrushedObj.bridgeMode == MODE_PHEN)
    { // phase enable
        gBrushedObj.pre_ph = gBrushedObj.ph;   // if ph is 0, pre_PH becomes 0
        P2SEL &= ~PHIN2;                       // Set IN2 to GPIO output

        if (gBrushedObj.ph == 1)               // forward
            P2OUT |= PHIN2;
        else if (gBrushedObj.ph == 0)          // reverse
            P2OUT &= ~PHIN2;

        // turn off IN2 timer PWM output mode
        TA1CCTL1 &= ~OUTMOD_7;                 // PHIN2 timer output in mode 0 fixed output with OUT = 0

        // turn on IN1 timer PWM output mode
        TA0CCR1 = 0;
        TA0CCTL1 |= OUTMOD_7;                  // ENIN1 setup as a timer reset/set PWM output
        P2SEL |= ENIN1;                        // Set ENIN1 to timer output
        TA0CCTL1 |= CCIE;                      // Enable Timer0_A1 Interrupt
    }
    else if (gBrushedObj.bridgeMode == MODE_PWM && gBrushedObj.revID == 0)
    { // PWM Mode
        // ENIN1 --> TimerA0
        P2OUT |= ENIN1;                        // set ENIN1 high
        P2SEL &= ~ENIN1;                       // Set ENIN1 to GPIO outputs
        TA0CCTL1 &= ~OUTMOD_7;                 // Clear Timer output mode
        TA0CCTL1 |= OUTMOD_3;                  // Timer output set/reset mode

        // PHIN2 --> TimerA1
        P2OUT |= PHIN2;                        // set PHIN2 high
        P2SEL &= ~PHIN2;                       // Set PHIN2 to GPIO outputs
        TA1CCTL1 &= ~OUTMOD_7;                 // Clear Timer output mode
        TA1CCTL1 |= OUTMOD_3;                  // Timer output set/reset mode
        __delay_cycles(10);                    // Timer clock is SCLK 2MHz, MCLK is 16MHz delay required for timers to clear
        TA0CCR1 = 0;
        TA1CCR1 = 0;
        __delay_cycles(10);                    // Timer clock is SCLK 2MHz, MCLK is 16MHz delay required for timers to clear
        TA0CCTL1 |= CCIE;                      // Enable Timer0_A1 Interrupt --> ENIN1
        TA1CCTL1 |= CCIE;                      // Enable Timer1_A1 Interrupt --> PHIN2
        __delay_cycles(160000);                // delay to combat glitch in outputs when start/stop is pressed in PWM H-bridge mode
        P2SEL |= ENIN1 + PHIN2;                // Set ENIN1 and PHIN2 to timer outputs
    }
    else if (gBrushedObj.bridgeMode == MODE_PWM && gBrushedObj.revID > 1)
    { // PWM Mode
        // ENIN1 --> TimerA0
        P2OUT &= ~ENIN1;                       // set ENIN1 low
        P2SEL &= ~ENIN1;                       // Set ENIN1 to GPIO outputs
        TA0CCTL1 |= OUTMOD_7;                  // Timer output reset/set mode

        // PHIN2 --> TimerA1
        P2OUT &= ~PHIN2;                       // set PHIN2 low
        P2SEL &= ~PHIN2;                       // Set PHIN2 to GPIO outputs
        TA1CCTL1 |= OUTMOD_7;                  // Timer output reset/set mode
        __delay_cycles(10);                    // Timer clock is SCLK 2MHz, MCLK is 16MHz delay required for timers to clear
        TA0CCR1 = 0;
        TA1CCR1 = 0;
        __delay_cycles(10);                    // Timer clock is SCLK 2MHz, MCLK is 16MHz delay required for timers to clear
        TA0CCTL1 |= CCIE;                      // Enable Timer0_A1 Interrupt --> ENIN1
        TA1CCTL1 |= CCIE;                      // Enable Timer1_A1 Interrupt --> PHIN2
        __delay_cycles(160000);                // delay to combat glitch in outputs when start/stop is pressed in PWM H-bridge mode
        P2SEL |= ENIN1 + PHIN2;                // Set ENIN1 and PHIN2 to timer outputs
    }
}

/*
 * This routine calculates moving average and peak values of 64 ADC10 results of VIPROPI voltage
 */
void ipropi_calc(void)
{
    ADC10CTL0 &= ~ENC;                         // disable ADC
    volatile uint8_t loop;
    volatile uint16_t Adcsum = 0;
    extern uint16_t ipropiRaw[];
    volatile uint16_t Adcmax = 0;

    if (!gBrushedObj.startMotor || ((gBrushedObj.rampInput1.setpointValue == MIN_PWM_DUTY) && (gBrushedObj.rampInput2.setpointValue == MIN_PWM_DUTY)))
    {
        gBrushedObj.ipropiAdc = 0;             // display 0
        gBrushedObj.ipropiAdcmax = 0;          // display 0
    }
    else
    {
        loop = 0;

        while (loop != 64)
        {
            Adcsum = Adcsum + ipropiRaw[loop];
            if (ipropiRaw[loop] > Adcmax)
                Adcmax = ipropiRaw[loop];
            loop++;
        }

        gBrushedObj.ipropiAdc = Adcsum/64;     // divide by 64 for moving average calculation
        gBrushedObj.ipropiAdcmax = Adcmax;
    }
    ADC10CTL0 |= ENC + ADC10SC;                // enable ADC and start repeat single channel sequence of conversions
//    __no_operation();                        // for debug with breakpoint
}

/*
 * This routine is for running passive diagnostics
 */
void PassiveDiagnostics()
{
    if (PDflag == 0)
    {
        if (gBrushedObj.startPassiveDiagnostics)
            {
                PDflag = 1;                    // set firmware passive diagnostics flag to track passive diagnostics start

//                spi_writeRegister(SPI_REG_COMMAND, (spi_readRegister(SPI_REG_COMMAND) |= CLR_FLT_MASK));
//                __delay_cycles(8);
//                gDeviceSpiReg.config1_reg = (uint8_t) spi_readRegister(SPI_REG_COMMAND);
            }
    }
    else                                       // we're in passive diagnostics mode
    {
        if (gBrushedObj.startPassiveDiagnostics)
        {
            P2SEL &= ~(ENIN1 + PHIN2);         // ENIN1 and PHIN2 to GPIO output
            if (gBrushedObj.en_in1_diag == 1)
                P2OUT |= ENIN1;                // output high
            else
                P2OUT &= ~ENIN1;               // output low

            if (gBrushedObj.ph_in2_diag == 1)
                P2OUT |= PHIN2;                // output high
            else
                P2OUT &= ~PHIN2;               // output low

            if (gBrushedObj.en_in1_diag == 0 && gBrushedObj.ph_in2_diag == 0)
            {
                gBrushedObj.ROLP_PU1 = 0;      // ROLP_PU1 indicate red
                gBrushedObj.ROLP_PD1 = 0;      // ROLP_PD1 indicate red
                gBrushedObj.ROLP_PU2 = 0;      // ROLP_PU2 indicate red
                gBrushedObj.ROLP_PD2 = 0;      // ROLP_PD2 indicate red
            }

            if (gBrushedObj.bridgeMode == MODE_PHEN || gBrushedObj.bridgeMode == MODE_PWM)
            {
                if (gBrushedObj.en_in1_diag == 1 && gBrushedObj.ph_in2_diag == 0)
                {
                    P2OUT |= ENIN1;            // output high
                    P2OUT &= ~PHIN2;           // output low
                    gBrushedObj.ROLP_PU1 = 1;  // ROLP_PU1 indicate green
                    gBrushedObj.ROLP_PD1 = 0;  // ROLP_PD1 indicate red
                    gBrushedObj.ROLP_PU2 = 0;  // ROLP_PU2 indicate red
                    gBrushedObj.ROLP_PD2 = 1;  // ROLP_PD2 indicate green
                }
                else if (gBrushedObj.en_in1_diag == 0 && gBrushedObj.ph_in2_diag == 1)
                {
                    P2OUT &= ~ENIN1;           // output low
                    P2OUT |= PHIN2;            // output high
                    P2SEL &= ~(ENIN1 + PHIN2); // ENIN1 and PHIN2 to GPIO output
                    gBrushedObj.ROLP_PU1 = 1;  // ROLP_PU1 indicate green
                    gBrushedObj.ROLP_PD1 = 0;  // ROLP_PD1 indicate red
                    gBrushedObj.ROLP_PU2 = 0;  // ROLP_PU2 indicate red
                    gBrushedObj.ROLP_PD2 = 1;  // ROLP_PD2 indicate green
                 }
                else if (gBrushedObj.en_in1_diag == 1 && gBrushedObj.ph_in2_diag == 1)
                {
                    P2OUT |= ENIN1;            // output low
                    P2OUT |= PHIN2;            // output high
                    P2SEL &= ~(ENIN1 + PHIN2); // ENIN1 and PHIN2 to GPIO output
                    gBrushedObj.ROLP_PU1 = 0;  // ROLP_PU1 indicate red
                    gBrushedObj.ROLP_PD1 = 1;  // ROLP_PD1 indicate green
                    gBrushedObj.ROLP_PU2 = 1;  // ROLP_PU2 indicate green
                    gBrushedObj.ROLP_PD2 = 0;  // ROLP_PD2 indicate red
                }
            }

            if (gBrushedObj.bridgeMode == MODE_INDEPENDENT)
            {
                if (gInterfaceType == SPI_INTERFACE)
                {
                    if (gBrushedObj.en_in1_diag == 1 && (((uint8_t) spi_readRegister(SPI_REG_CONFIG2) & 0x60) == 0x20))
                    {
                        P2OUT |= ENIN1;            // output high
                        P2OUT &= ~PHIN2;           // output low
                        P2SEL &= ~(ENIN1 + PHIN2); // ENIN1 and PHIN2 to GPIO output
                        gBrushedObj.ROLP_PU1 = 1;  // ROLP_PU1 indicate green
                        gBrushedObj.ROLP_PD1 = 0;  // ROLP_PD1 indicate red
                        gBrushedObj.ROLP_PU2 = 0;  // ROLP_PU2 indicate red
                        gBrushedObj.ROLP_PD2 = 0;  // ROLP_PD2 indicate red
                    }
                    else if (gBrushedObj.en_in1_diag == 0 && gBrushedObj.ph_in2_diag == 1 && (((uint8_t) spi_readRegister(SPI_REG_CONFIG2) & 0x60) == 0x20))
                    {
                        P2OUT &= ~ENIN1;           // output low
                        P2OUT |= PHIN2;            // output high
                        P2SEL &= ~(ENIN1 + PHIN2); // ENIN1 and PHIN2 to GPIO output
                        gBrushedObj.ROLP_PU1 = 0;  // ROLP_PU1 indicate red
                        gBrushedObj.ROLP_PD1 = 0;  // ROLP_PD1 indicate red
                        gBrushedObj.ROLP_PU2 = 1;  // ROLP_PU2 indicate green
                        gBrushedObj.ROLP_PD2 = 0;  // ROLP_PD2 indicate red
                     }
                    else if (gBrushedObj.en_in1_diag == 1 && (((uint8_t) spi_readRegister(SPI_REG_CONFIG2) & 0x60) == 0x60))
                    {
                        P2OUT |= ENIN1;            // output high
                        P2OUT &= ~PHIN2;           // output low
                        P2SEL &= ~(ENIN1 + PHIN2); // ENIN1 and PHIN2 to GPIO output
                        gBrushedObj.ROLP_PU1 = 0;  // ROLP_PU1 indicate red
                        gBrushedObj.ROLP_PD1 = 1;  // ROLP_PU1 indicate green
                        gBrushedObj.ROLP_PU2 = 0;  // ROLP_PU1 indicate red
                        gBrushedObj.ROLP_PD2 = 0;  // ROLP_PU1 indicate red
                    }
                    else if (gBrushedObj.en_in1_diag == 0 && gBrushedObj.ph_in2_diag == 1 && (((uint8_t) spi_readRegister(SPI_REG_CONFIG2) & 0x60) == 0x60))
                    {
                        P2OUT &= ~ENIN1;           // output low
                        P2OUT |= PHIN2;            // output high
                        P2SEL &= ~(ENIN1 + PHIN2); // ENIN1 and PHIN2 to GPIO output
                        gBrushedObj.ROLP_PU1 = 0;  // ROLP_PU1 indicate red
                        gBrushedObj.ROLP_PD1 = 0;  // ROLP_PD1 indicate red
                        gBrushedObj.ROLP_PU2 = 0;  // ROLP_PU2 indicate red
                        gBrushedObj.ROLP_PD2 = 1;  // ROLP_PD2 indicate green
                     }
                }

                else
                {
                    if (gBrushedObj.en_in1_diag == 1 && gBrushedObj.Loadhighlow == 0)
                    {
                        P2OUT |= ENIN1;            // output high
                        P2OUT &= ~PHIN2;           // output low
                        P2SEL &= ~(ENIN1 + PHIN2); // ENIN1 and PHIN2 to GPIO output
                        gBrushedObj.ROLP_PU1 = 1;  // ROLP_PU1 indicate green
                        gBrushedObj.ROLP_PD1 = 0;  // ROLP_PD1 indicate red
                        gBrushedObj.ROLP_PU2 = 0;  // ROLP_PU2 indicate red
                        gBrushedObj.ROLP_PD2 = 0;  // ROLP_PD2 indicate red
                    }
                    else if (gBrushedObj.en_in1_diag == 0 && gBrushedObj.ph_in2_diag == 1 && gBrushedObj.Loadhighlow == 0)
                    {
                        P2OUT &= ~ENIN1;           // output low
                        P2OUT |= PHIN2;            // output high
                        P2SEL &= ~(ENIN1 + PHIN2); // ENIN1 and PHIN2 to GPIO output
                        gBrushedObj.ROLP_PU1 = 0;  // ROLP_PU1 indicate red
                        gBrushedObj.ROLP_PD1 = 0;  // ROLP_PD1 indicate red
                        gBrushedObj.ROLP_PU2 = 1;  // ROLP_PU2 indicate green
                        gBrushedObj.ROLP_PD2 = 0;  // ROLP_PD2 indicate red
                     }
                    else if (gBrushedObj.en_in1_diag == 1 && gBrushedObj.Loadhighlow == 1)
                    {
                        P2OUT |= ENIN1;            // output high
                        P2OUT &= ~PHIN2;           // output low
                        P2SEL &= ~(ENIN1 + PHIN2); // ENIN1 and PHIN2 to GPIO output
                        gBrushedObj.ROLP_PU1 = 0;  // ROLP_PU1 indicate red
                        gBrushedObj.ROLP_PD1 = 1;  // ROLP_PU1 indicate green
                        gBrushedObj.ROLP_PU2 = 0;  // ROLP_PU1 indicate red
                        gBrushedObj.ROLP_PD2 = 0;  // ROLP_PU1 indicate red
                    }
                    else if (gBrushedObj.en_in1_diag == 0 && gBrushedObj.ph_in2_diag == 1 && gBrushedObj.Loadhighlow == 1)
                    {
                        P2OUT &= ~ENIN1;           // output low
                        P2OUT |= PHIN2;            // output high
                        P2SEL &= ~(ENIN1 + PHIN2); // ENIN1 and PHIN2 to GPIO output
                        gBrushedObj.ROLP_PU1 = 0;  // ROLP_PU1 indicate red
                        gBrushedObj.ROLP_PD1 = 0;  // ROLP_PD1 indicate red
                        gBrushedObj.ROLP_PU2 = 0;  // ROLP_PU2 indicate red
                        gBrushedObj.ROLP_PD2 = 1;  // ROLP_PD2 indicate green
                     }
                }
            }
        }

        else
        {
            PDflag == 0;                       // clear firmware passive diagnostics flag
            P2OUT &= ~ENIN1;                   // output low
            P2OUT &= ~PHIN2;                   // output low
            P2SEL |= (ENIN1 + PHIN2);          // ENIN1 and PHIN2 to timer output
            gBrushedObj.en_in1_diag = 0;       // EN/IN1 off
            gBrushedObj.ph_in2_diag = 0;       // PH/IN2 off
            gBrushedObj.ROLP_PU1 = 0;          // ROLP_PU1 indicate red
            gBrushedObj.ROLP_PD1 = 0;          // ROLP_PD1 indicate red
            gBrushedObj.ROLP_PU2 = 0;          // ROLP_PU2 indicate red
            gBrushedObj.ROLP_PD2 = 0;          // ROLP_PD2 indicate red
//            drv_RESET();                       // clear nFAULT
        }                                      // we're exiting passive diagnostics mode
    }
}


/*
 * This is SPI interface, used to read the device configurations,parameters and
 * status information for S version of device.
 * Register format |0|R/W|A5|A4|A3|A2|A1|A0|D7|D6|D5|D4|D3|D2|D1|D0|
 * Ax is address bit, Dx is data bits and R/W is read write bit.
 * For read R/W bit should be 1.
 */
uint16_t spi_readRegister(uint8_t address)
{
    volatile uint16_t reg_value = 0;
    volatile uint8_t dataMSB = 0;
    volatile uint8_t dataLSB = 0;

    reg_value |= SPI_RW_BIT_MASK; /*Set R/W bit*/
    reg_value |= ((address << SPI_ADDRESS_POS) & SPI_ADDRESS_MASK); /* Configure register address value*/
    P2OUT &= ~nSCS; //SPI set

    while (!(IFG2 & UCB0TXIFG))
        ; /* USCI_B0 TX buffer ready*/
    UCB0TXBUF = (uint8_t) ((reg_value >> 8) & 0xFF); /* Transmit the Address(MSB Byte)*/

    while (UCB0STAT & SPI_BUSY_FLAG)
    {
        ; /* Wait till Transmission is complete*/
    }
    dataMSB = UCB0RXBUF; /* Recieve the First byte of data, MSB byte*/

    UCB0TXBUF = 0x00; /* Transmit Second byte*/
    while (UCB0STAT & SPI_BUSY_FLAG)
    {
        ; /* Wait till Transmission is complete*/
    }
    dataLSB = UCB0RXBUF & 0xFF; /* Recieve the Second byte,LSB byte*/

    P2OUT |= nSCS; //SPI Reset
    _delay_cycles(10);

    reg_value = ((((dataMSB << 8) | dataLSB) & SPI_DATA_MASK) >> SPI_DATA_POS); /* complete data */

    return (reg_value);
}

/*
 * This is SPI interface, used to write the set device configurations and operating
 * parameters of S version of device.
 * Register format |R/W|A4|A3|A2|A1|A0|*|*|D7|D6|D5|D4|D3|D2|D1|D0|
 * Ax is address bit, Dx is data bits and R/W is read write bit.
 * For write R/W bit should 0.
 */
uint16_t spi_writeRegister(uint8_t address, uint16_t data)
{
    volatile uint16_t reg_value = 0;

//  reg_value &=~ SPI_RW_BIT_MASK;                                        /*Clear R/W bit*/
    reg_value |= ((address << SPI_ADDRESS_POS) & SPI_ADDRESS_MASK); /* Adding register address value*/
    reg_value |= ((data << SPI_DATA_POS) & SPI_DATA_MASK); /*Adding data value */

    P2OUT &= ~nSCS; /*SPI set*/

    while (!(IFG2 & UCB0TXIFG))
        ; /* USCI_B0 TX buffer ready?*/

    UCB0TXBUF = (uint8_t) ((reg_value >> 8) & 0xFF); /* Transmit first Byte,MSB-Byte*/

    while (!(IFG2 & UCB0TXIFG))
        ; /* USCI_B0 TX buffer ready*/

    UCB0TXBUF = (uint8_t) (reg_value & 0xFF); /* Transmit Second Byte, LSB-Byte*/

    while (UCB0STAT & SPI_BUSY_FLAG)
    {
        ; /* Wait till Transmission is complete*/
    }

    P2OUT |= nSCS; /*SPI Reset*/
    _delay_cycles(10);

    return 0;
}

void cmdRxIsr()
{
// disable UART interrupt
    IE2 &= ~UCA0RXIE;
    __no_operation();
    __no_operation();
    __bis_SR_register(GIE); // 15Apr2014 -- RLD re-enabled interrupt to allow timer interrupts
//
    /* USCI_A0 TX buffer ready? */
    while (!(IFG2 & UCA0TXIFG))
        ;

    /* Parse received command immediately */
    receivedDataCommand(UCA0RXBUF);

// re-enable UART interrupt
    __bic_SR_register(GIE); // 15Apr2014 -- RLD disabled interrupt
    __no_operation();
    __no_operation();
    IE2 |= UCA0RXIE;
}

void uartTxByte(unsigned char byte)
{
    /* USCI_A0 TX buffer ready? */
    while (!(IFG2 & UCA0TXIFG))
        ;
    /* transmite data */
    UCA0TXBUF = byte;
}

