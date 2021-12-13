/*
 * AD9833.h
 * Reference code: https://www.arduino.cc/reference/en/libraries/md_ad9833/
 *  Date: 2021-06-10
 *  Author: Chun-Lin Chen
 */

#ifndef AD9833_H_
#define AD9833_H_

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C" {
#endif


#define pow2_28             268435456L  // 2^28 used in frequency word calculation
#define refFrequency        25000000

#define RESET_CMD           0x0100      // Reset enabled (also CMD RESET)
/*      Sleep mode
 * D7   1 = internal clock is disabled
 * D6   1 = put DAC to sleepT
 */
#define SLEEP_MODE          0x00C0      // Both DAC and Internal Clock
#define DISABLE_DAC         0x0040
#define DISABLE_INT_CLK     0x0080

#define PHASE_WRITE_CMD     0xC000      // Setup for Phase write
#define PHASE1_WRITE_REG    0x2000      // Which phase register
#define FREQ0_WRITE_REG     0x4000      //
#define FREQ1_WRITE_REG     0x8000
#define PHASE1_OUTPUT_REG   0x0400      // Output is based off REG0/REG1
#define FREQ1_OUTPUT_REG    0x0800      // ditto

typedef enum
{
    SINE_WAVE = 0x2000,
    TRIANGLE_WAVE = 0x2002,
    SQUARE_WAVE = 0x2028,
    HALF_SQUARE_WAVE = 0x2020
} WaveformType;

//Reset AD9833 module
void AD9833_Reset(void);

//Set frequency
void AD9833_SetFrequency(float frequency);

//Set phase
void AD9833_SetPhase(float phase);

//Set waveform
void AD9833_SetWaveform(WaveformType waveformtype);

//Enable output
void AD9833_OutputEn(bool enable);

//Swap frequency
void AD9833_SwapFreq(float freq);

#endif /* AD9833_H_ */
