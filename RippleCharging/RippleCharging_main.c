/**
 * @file RippleCharging_main.c
 * @brief Main firmware of SRC board
 *
 * It charge, discharge, and measure EIS of battery.
 * Also shows how to control AD9833 DDS signal generator
 * using SPIA interface.
 * The AD9833 DDS chip SPI timing diagram determine the SPI mode, which is MODE2
 * Set the SPI mode in "f28004x_spi.c", InitSpi section.
 * Communicate to the EIS measurement board through ADICuP 3029 dev board.
 * The CN0510 hat can measure the Lithium battery impedance.
 * Also, the hardware connection is as below:
 * F280049      LaunchPad pin        AD9833          ADICuP 3029
 * SPIA_STE     Pin19               FNC             x
 * SPIA_MOSI    Pin15               SPI_MISO        x
 * SPIA_CLK     Pin7                CLK             x
 * SCIA_RX      Pin10               x               TX
 * SCIA_TX      Pin9                x               RX
 *
 * Notice that the original LaunchPad card in box has a lot of editing error
 * please make sure the pin out is as like the F280049 datasheet shows, include the GPIO mux.
 *
 * @author: Chun-Lin Chen johnson35762@gmail.com
 * @bugs: Sometime the SCIB Rx will reveived something, let ccFlag, cvFlag, and disFlag into wrong data.
 *
 * Date: 2021-10-26
 * License: GPL-3.0
 *
 * Please reference to Lab/實驗室研究資料區/陳俊霖/Lab105(NAS)/弦波充電 for more PCB detail
 * PCB editor convert to KiCad for fully opensource access.
 */

#include "F28x_Project.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "Init.h"       // Include initial setup
#include "AD9833.h"     // Include AD9833 lib
#include "CN0510.h"     // Include CN0510 lib
#include "spi.h"        // Include SPI lib
#include "sci.h"        // Include SCI lib
#include "dac.h"        // Include DAC lib
#include "pid.h"        // Include PID lib
#include "FIRFilter.h"

#define LED4    23      // Build in LED 4
#define LED5    34      // Build in LED 5

#define Relay1  5       // Relay for CC charging
#define Relay2  58      // Relay for CV charging
#define Relay3  25      // Relay for EIS measuring

typedef enum            // Ensure no variable name conflict
{
    CHARGE = 0x0000, DISCHARGE = 0x0001, MEASURE = 0x0011
} BAT_STATUS;

// -----------------------------------------------------------------------------------------------
// __Flash_RAM = 1 , the code will storage into flash memory
// __Flash_RAM = 0 , the code will storage into RAM
// Set project property to corresponding cmd file.

#define __FLASH_RAM 1
#if __Flash_RAM

extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;

#endif

// Variables -------------------------------------------------------------------------------------
unsigned int curBuff = 0;
unsigned int batPosBuff = 0;
unsigned int batNegBuff = 0;
unsigned int maCtr = 0;

PID_STRUCT *psPID;

uint16_t currentBuffer[FILTER_ORDER_NUM];
uint16_t batPosBuffer[FILTER_ORDER_NUM];
uint16_t batNegBuffer[FILTER_ORDER_NUM];

bool batRst = false;        // Battery reset flag
bool ccFlag = false;        // CC charging flag
bool cvFlag = false;        // CV charging flag
unsigned int rstTmr = 0; // Battery reset timer after charging or discharging

double current = 0.0f;
double batPos = 0.0f;
double batNeg = 0.0f;
double batVolt = 0.0f;

circular_handle_t curr_cbuf;
circular_handle_t batPos_cbuf;
circular_handle_t batNeg_cbuf;

unsigned int batMode = 0;                // Battery charging status
// status 0: Symmetric sine current charging
// status 1: Asymmetric sine current charging
// status 2: Constant voltage charging
// status 3: Discharging
// status 4: EIS measuring

unsigned int uartReceived;      // flag for data received
unsigned char *uart_cmd = "Start!\r\n";     // use for SCIB command string

unsigned char uart_msg[100];    // UART MUX transmit char array

uint16_t addrRecvTemp;          // received data array address

unsigned char recvTemp[64];     // temp data from RX
unsigned char recvData[64];     // Data from SCIA RX

// Function Prototypes ----------------------------------------------------------------------------

__interrupt void CPUTimer0ISR(void);
__interrupt void SCIBRxISR(void);

void transmitSCIBChar(uint16_t a);
void transmitSCIBMessage(unsigned char *msg);

void batChgMod(int mode);
float MAfilter(uint16_t adcBuff, uint16_t *samples, uint16_t maCtr,
               float lastAvg);

//void adi_InitEIS();
float adi_RcalMeasure(float ImRcal, float ReRcal);
float adi_EISMeasure(float frequency);

// Main --------------------------------------------------------------------------------------------
void main(void)
 {
    // Decide where the code will store into, RAM or Flash memory
#if __FLASH_RAM

    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t) &RamfuncsLoadSize);

#endif

    // Initialize the system, the parameter or more module need to start refer to "Init.c"
    // to add more initialization process
    InitSysCtrl();
    InitGPIO();
    InitSPI();
    InitPIE();
    InitADC();
    InitDAC();
    InitADCSOC();
    InitCPUTimer();
    psPID = Init_pid();
    batRst = 0;
    curr_cbuf = circular_buf_init(currentBuffer,FILTER_ORDER_NUM);
    batPos_cbuf = circular_buf_init(batPosBuffer,FILTER_ORDER_NUM);
    batNeg_cbuf = circular_buf_init(batNegBuffer,FILTER_ORDER_NUM);
    // Map ISR functions

    EALLOW;

    PieVectTable.TIMER0_INT = &CPUTimer0ISR;    // Enable cpuTimer0
    PieVectTable.SCIB_RX_INT = &SCIBRxISR;

    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;      // Enable cpuTimer0
    PieCtrlRegs.PIEIER9.bit.INTx3 = 1;      // Enable SCIB RX
    PieCtrlRegs.PIEIER9.bit.INTx4 = 1;      // Enable SCIB TX

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // Clear INT GROUP1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9; // Clear INT GROUP9 flag

    IER |= M_INT1;  // Enable group 1 interrupts, for Timer0
    IER |= M_INT9;  // Enable group 9 interrupts, for SCIA

    EDIS;

    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    EINT;
    ERTM;

    // Initialize SCI interface
    InitSCIB();  // Initialize the SCIB

    memset(recvTemp, 0, 64);                    // Reset recvData string
    ScibRegs.SCIHBAUD.all = 0x0000;             // Set SCI baud rate = 115200
    ScibRegs.SCILBAUD.all = 0x001A;
    // Set waveform frequency to 923 Hz
    AD9833_SetFrequency(923);

    // Set waveform type
    AD9833_SetWaveform(SINE_WAVE);

    // Enable AD9833 output
    AD9833_OutputEn(true);
    // Reset relays
    GPIO_WritePin(Relay1, 0);
    GPIO_WritePin(Relay2, 0);
    GPIO_WritePin(Relay3, 0);

    GPIO_WritePin(LED4, 0);
    GPIO_WritePin(LED5, 0);

    // Set DAV output value
    DAC_setShadowValue(DACA_BASE, 1700);

    // System startup
    // adi_initEIS();
    // adi_RcalMeasure();
    // adi_EISMeasure();

    batChgMod(2);                // Default mode is reset
    ccFlag = 1;

    memset(recvTemp, 0, 64);
    memset(recvData, 0, 64);
    // Main loop
    while (1)
    {
        if (maCtr > 254)
        {
            if (batRst == 0)        // If battery not need rest 30min
            {
                if (batVolt > 3540 || cvFlag == 1)
                {     // If battery reached 4.2V
                    cvFlag = 1;     // Latch CV mode for ADC jitter
                    ccFlag = 0;
                    batChgMod(1);    // Set relay to CV mode
                    if (current <2140) //@@@@ SRC current = 2140 CCV current = 2150
                    {      // If charging current < 130mA(0.05C)
                        cvFlag = 0;
                        ccFlag = 1;
                        batRst = 1;
                        batChgMod(2);   // Reset battery
                    }
                }
                else if (ccFlag == 1)
                {   // battery voltage 2.5V ~ 4.2V
                    cvFlag = 0;     // Latch CC mode for ADC jitter
                    ccFlag = 1;
                    batChgMod(0);   // CC mode
                }
            }
            else
            {
                    batChgMod(3);      // Change mode to reset
                    //DELAY_US(1000000);      // Delay 1s
                    rstTmr++;
            }
      }
//Testing Code

    }
}

// SCIA connect to the UART MUX
__interrupt void SCIBRxISR(void)
{
    unsigned char c;
    int i;
    c = SCI_readCharBlockingFIFO(SCIB_BASE);     // read FIFO char
    recvTemp[addrRecvTemp] = c;     // put char into string array
    addrRecvTemp++;                 // put char to next address

    if (recvTemp[addrRecvTemp - 2] == '\r'
            && recvTemp[addrRecvTemp - 1] == '\n') // If received "\r\n" string, clear the string array and reset array address
    {
        printf("\r\n recv: %s", recvTemp);        // Print the data received
        for (i = 0; i < addrRecvTemp; i++) // Move the recvTemp data into recvData array
            recvData[i] = recvTemp[i];
        addrRecvTemp = 0;                           // Reset array address
        memset(recvTemp, 0, 64);                    // Clear array
    }
    ScibRegs.SCIFFRX.bit.RXFFOVRCLR = 1;
    ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1;        // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}

// transmitSCIAChar - Transmit a character from the SCIA
void transmitSCIBChar(uint16_t a)
{
    while (ScibRegs.SCIFFTX.bit.TXFFST != 0)
    {

    }
    ScibRegs.SCITXBUF.all = a;
}

// transmitSCIAMessage - Transmit message via SCIA

void transmitSCIBMessage(unsigned char *msg)
{
    int i;
    i = 0;
    while (msg[i] != '\0')
    {
        transmitSCIBChar(msg[i]);
        i++;
    }
}

// Timer0 for ADC trigger
__interrupt void CPUTimer0ISR(void)
{
    curBuff = AdcbResultRegs.ADCRESULT0;
    batPosBuff = AdcaResultRegs.ADCRESULT0;
    batNegBuff = AdccResultRegs.ADCRESULT0;


    //psPID->Feedback_AdcPoint = AdcbResultRegs.ADCRESULT0;
    //pid_process(psPID);
    //bias = psPID->Output;
    //DAC_setShadowValue(DACA_BASE, bias);

    circular_buf_put(curr_cbuf, curBuff);
    circular_buf_put(batPos_cbuf, batPosBuff);
    circular_buf_put(batNeg_cbuf, batNegBuff);

    current = filter_process(curr_cbuf, FIRfilter);
    batPos = filter_process(batPos_cbuf, FIRfilter);
    batNeg = filter_process(batNeg_cbuf, FIRfilter);
    batVolt = batPos - batNeg;
    DAC_setShadowValue(DACA_BASE, batNeg);
    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

void batChgMod(int mode)
{
    switch (mode)
    {
    case 0:             // status 0: SRC charging
        GPIO_WritePin(Relay1, 1);             // 1 for CC
        GPIO_WritePin(Relay2, 0);             // 1 for CV
        GPIO_WritePin(Relay3, 0);             // 1 for RST
        GPIO_WritePin(LED4, 0);               // 1 for RED LED
        GPIO_WritePin(LED5, 1);               // 1 for GREEN LED
        break;

    case 1:             // status 1: CV charging
        GPIO_WritePin(Relay1, 0);             // 1 for CC
        GPIO_WritePin(Relay2, 1);             // 1 for CV
        GPIO_WritePin(Relay3, 0);             // 1 for RST
        GPIO_WritePin(LED4, 1);               // 1 for RED LED
        GPIO_WritePin(LED5, 0);               // 1 for GREEN
        break;

    case 2:             // status 2: RST
        GPIO_WritePin(Relay1, 0);             // 1 for CC
        GPIO_WritePin(Relay2, 0);             // 1 for CV
        GPIO_WritePin(Relay3, 0);             // 1 for RST
        GPIO_WritePin(LED4, 1);               // 1 for RED LED
        GPIO_WritePin(LED5, 1);               // 1 for GREEN
        break;

    default:             // default battery not connect
        GPIO_WritePin(Relay1, 0);
        GPIO_WritePin(Relay2, 0);
        GPIO_WritePin(Relay3, 0);
        GPIO_WritePin(LED4, 1);               // 1 for RED LED
        GPIO_WritePin(LED5, 1);               // 1 for GREEN
        break;
    }
}
