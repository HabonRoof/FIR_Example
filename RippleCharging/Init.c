/*
 * init.c
 *
 *  Created on: 2021¦~6¤ë11¤é
 *      Author: Johnson
 */

#include "F28x_Project.h"

#include <stdio.h>
#include "dac.h"

#define LED4    23      // Build in LED 4
#define LED5    34      // Build in LED 5

#define Relay1  5       // Relay for CC charging
#define Relay2  58      // Relay for CV charging
#define Relay3  25      // Relay for EIS measuring

// GPIO initialization

void InitGPIO(void)
{
    //
    // Initialize GPIO and configure the GPIO pin as a push-pull output
    //
    InitGpio();
    // Setup board LED as indicator
    GPIO_SetupPinMux(LED4, GPIO_MUX_CPU1, 0);       // Red LED
    GPIO_SetupPinOptions(LED4, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(LED5, GPIO_MUX_CPU1, 0);       // Green LED
    GPIO_SetupPinOptions(LED5, GPIO_OUTPUT, GPIO_PUSHPULL);

    // Setup Relay1 pin
    // Relay1 = 0, Stop CC charging
    // Relay1 = 1, Start SRC charging
    GPIO_SetupPinMux(Relay1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(Relay1, GPIO_OUTPUT, GPIO_PUSHPULL);

    // Setup Relay2 pin
    // Relay2 = 0, Stop CV charging
    // Relay2 = 1, Start CV charging
    GPIO_SetupPinMux(Relay2, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(Relay2, GPIO_OUTPUT, GPIO_PUSHPULL);

    // Setup Relay3 pin
    // Relay3 = 0, Stop EIS
    // Relay3 = 1, Start EIS
    GPIO_SetupPinMux(Relay3, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(Relay3, GPIO_OUTPUT, GPIO_PUSHPULL);


    //Initialize SCIA
//    GPIO_SetupPinMux(28, GPIO_MUX_CPU1, 1);
//    GPIO_SetupPinOptions(28, GPIO_INPUT, GPIO_PULLUP);
//    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 1);
//    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);

    //Initialize SCIB
    GPIO_SetupPinMux(13, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(13, GPIO_INPUT, GPIO_PULLUP);
    GPIO_SetupPinMux(40, GPIO_MUX_CPU1, 9);
    GPIO_SetupPinOptions(40, GPIO_OUTPUT, GPIO_PUSHPULL);
}

// PIE initialization

void InitPIE(void)
{
    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    InitPieVectTable();
    EINT;
}

// Initialize SPIA setting

void InitSPI(void)
{
    //
    // Initialize SPIA port, the function InitSpi and InitSpiGpio only initialize
    // SPIA, if want to use SPIB, need to modify the code into SPIB register
    //
    InitSpi();
    InitSpiGpio();
}

// initADC - Function to configure and power up ADCA.

void InitADC(void)
{
    // Setup VREF as internal
    // In order to set ADCC REF=3.3V, we need to setup both ADCA,ADCB,ADCC to 3v3.
    SetVREF(ADC_ADCA, ADC_INTERNAL, ADC_VREF3P3);
    SetVREF(ADC_ADCB, ADC_INTERNAL, ADC_VREF3P3);
    SetVREF(ADC_ADCC, ADC_INTERNAL, ADC_VREF3P3);

    EALLOW;

    // ADCA group setup-----------

    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;    // Set ADCCLK divider to /4

    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;    // Set pulse positions to late

    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;    // Power up the ADC

    // ADCB group setup------------

    AdcbRegs.ADCCTL2.bit.PRESCALE = 6;    // Set ADCCLK divider to /4

    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;    // Set pulse positions to late

    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;    // Power up the ADC

    // ADCC group setup------------

    AdccRegs.ADCCTL2.bit.PRESCALE = 6;    // Set ADCCLK divider to /4

    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;    // Set pulse positions to late

    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1; // Power up the ADC and then delay for 1 ms

    EDIS;

    DELAY_US(1000);
}

void InitADCSOC(void)
{
    // Select the channels to convert and the end of conversion flag
    EALLOW;

    // ADCASOC0
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 1;     // SOC0 will convert pin A1 (Vbat+)
                                           // 0:A0  1:A1  2:A2  3:A3
                                           // 4:A4   5:A5   6:A6   7:A7
                                           // 8:A8   9:A9   A:A10  B:A11
                                           // C:A12  D:A13  E:A14  F:A15
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 19;     // Sample window is 20 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 1;   // Trigger on CPU Timer0 TINT0n
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; // End of SOC0 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   // Enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // Make sure INT1 flag is cleared

    // ADCBSOC0
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 1;     // SOC0 will convert pin B1 (Ibat)
                                           // 0:A0  1:A1  2:A2  3:A3
                                           // 4:A4   5:A5   6:A6   7:A7
                                           // 8:A8   9:A9   A:A10  B:A11
                                           // C:A12  D:A13  E:A14  F:A15
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 19;     // Sample window is 20 SYSCLK cycles
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 1;   // Trigger on CPU Timer0 TINT0n
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0; // End of SOC0 will set INT1 flag
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   // Enable INT1 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // Make sure INT1 flag is cleared

    // ADCCSOC0
    AdccRegs.ADCSOC0CTL.bit.CHSEL = 1;     // SOC0 will convert pin C1 (Vbat-)
                                           // 0:A0  1:A1  2:A2  3:A3
                                           // 4:A4   5:A5   6:A6   7:A7
                                           // 8:A8   9:A9   A:A10  B:A11
                                           // C:A12  D:A13  E:A14  F:A15
    AdccRegs.ADCSOC0CTL.bit.ACQPS = 19;     // Sample window is 20 SYSCLK cycles
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 1;   // Trigger on CPU Timer0 TINT0n
    AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 0; // End of SOC0 will set INT1 flag
    AdccRegs.ADCINTSEL1N2.bit.INT1E = 1;   // Enable INT1 flag
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // Make sure INT1 flag is cleared

    EDIS;

}

void InitCPUTimer(void)
{

    //
    // Initialize the Device Peripheral. For this example, only initialize the
    // Cpu Timers.
    //
    InitCpuTimers();
    //
    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 100MHz CPU Freq, 1 second Period (in uSeconds)
    //
    ConfigCpuTimer(&CpuTimer0, 100, 100);

    //
    // To ensure precise timing, use write-only instructions to write to the
    // entire register. Therefore, if any of the configuration bits are changed
    // in ConfigCpuTimer and InitCpuTimers, the below settings must also be
    // be updated.
    //
    CpuTimer0Regs.TCR.all = 0x4000;

    //
    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2
    //
    IER |= M_INT1;

    //
    // Enable TINT0 in the PIE: Group 1 interrupt 7
    //
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

}

//
//  initSCIAEchoback - Initialize SCI-A for echoback
//
void InitSCIB(void)
{
    //
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function
    //
    ScibRegs.SCICCR.all = 0x0007;           // 1 stop bit,  No loopback
                                            // No parity, 8 char bits,
                                            // async mode, idle-line protocol
    ScibRegs.SCICTL1.all = 0x0003;          // enable TX, RX, internal SCICLK,
                                            // Disable RX ERR, SLEEP, TXWAKE
    ScibRegs.SCICTL2.all = 0x0000;          // Disable TX RX interrupt
    //
    // SCIB at 115200 baud
    // @LSPCLK = 25 MHz (100 MHz SYSCLK) HBAUD = 0x00  and LBAUD = 0x1A.
    //
    ScibRegs.SCIHBAUD.all = 0x0000;
    ScibRegs.SCILBAUD.all = 0x001A;

    ScibRegs.SCIFFTX.all = 0xC001;  // Reset transmit FIFO, enable SCI FIFO, enable TX FIFO interrupt TX FIFO set 1 layer fill
    ScibRegs.SCIFFRX.all = 0x0021;  // Reset receive FIFO, clear FIFO, enable RX FIFO interrupt, RX FIFO set 1 layer fill
    ScibRegs.SCIFFCT.all = 0x0;

    ScibRegs.SCICTL1.all = 0x0023;          // Software reset SCIB
    ScibRegs.SCIFFTX.bit.TXFIFORESET = 1;
    ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;
}

void InitDAC(void){
    //
        // Set VDAC as the DAC reference voltage.
        // Edit here to use ADC VREF as the reference voltage.
        //
        DAC_setReferenceVoltage(DACA_BASE, DAC_REF_ADC_VREFHI);

        //
        // Enable the DAC output
        //
        DAC_enableOutput(DACA_BASE);

        //
        // Set the DAC shadow output to 0
        //
        DAC_setShadowValue(DACA_BASE, 0);

        DAC_setGainMode(DACA_BASE,DAC_GAIN_TWO);


        //
        // Delay for buffered DAC to power up
        //
        DELAY_US(10);
}
