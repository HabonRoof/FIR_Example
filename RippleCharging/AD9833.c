/*
 * AD9833.c
 * Reference code: https://www.arduino.cc/reference/en/libraries/md_ad9833/
 *  Date: 2021-06-10
 *  Author: Chun-Lin Chen
 */
//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C" {
#endif

#include "F28x_Project.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "spi.h"

#include "AD9833.h"

// Reset AD9833 module
void AD9833_Reset(void)
{
    SPI_writeDataNonBlocking(SPIA_BASE, RESET_CMD);
    DELAY_US(100);
}

// Set frequency
void AD9833_SetFrequency(float frequency)
{
    if (frequency > 12.5e6)
        frequency = 12.5e6;
    if (frequency < 0.0)
        frequency = 0.0;
    int32_t freqWord = (frequency * pow2_28) / (float) refFrequency;
    int16_t upper14 = (int16_t) ((freqWord & 0xFFFC000) >> 14),
            lower14 = (int16_t) (freqWord & 0x0003FFF);

    SPI_writeDataNonBlocking(SPIA_BASE, 0X2100);
    DELAY_US(100);
    SPI_writeDataNonBlocking(SPIA_BASE, (0x4000 | lower14));
    DELAY_US(100);
    SPI_writeDataNonBlocking(SPIA_BASE, (0x4000 | upper14));
    DELAY_US(100);
}

/* Set waveform type
 *  SINE_WAVE =         0x2000,
 *  TRIANGLE_WAVE =     0x2002,
 *  SQUARE_WAVE =       0x2028,
 *  HALF_SQUARE_WAVE =  0x2020
 */
void AD9833_SetWaveform(WaveformType waveformtype)
{
    SPI_writeDataNonBlocking(SPIA_BASE, waveformtype);
    DELAY_US(100);
}

// Output enable
void AD9833_OutputEn(bool enable)
{
    if (enable == true)
        SPI_writeDataNonBlocking(SPIA_BASE, ~RESET_CMD);
    else
        SPI_writeDataNonBlocking(SPIA_BASE, RESET_CMD);
    DELAY_US(100);
}

// Swap frequency
void AD9833_SwapFreq(float freq){

    if (freq < 100000)
            {
                freq += 500;
                AD9833_SetFrequency(freq);
                AD9833_SetWaveform(SINE_WAVE);
                AD9833_OutputEn(true);
                DELAY_US(100);
            }
            else
                freq = 10.0;
}
