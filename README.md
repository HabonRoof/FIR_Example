# FIR filter Example for F280049C
This example shows how to implement FIR filter in F280049C Launch Pad.

## File structure
* FIR_Example_define.h
    Define circulat buffer structure, FIR filter coefficient, and API function.
* FIR_Example_API.c
    Implement API function, calculate FIR output and some useful tool.

## Function
|Function | Doing |
| ------ | ------ |
| circular_buf_empty | Check buffer is empty or not.|
| circular_buf_full | Check buffer is full or not.|
| circular_buf_init | Initialize circular buffer. This function need to be execute before system interrupt enable.|
| increase_pointer | Increase circular buffer pointer.|
| circular_buf_put| Put new element into circular buffer.  This function need to be in the interrupt section of main code.|
| filter_process | Calculate FIR output, include coefficient and ADC data.|
## Usage
### Definition
This variables need to be define before main loop.

Define buffers
```
unsigned int curBuff = 0;
unsigned int batPosBuff = 0;
unsigned int batNegBuff = 0;
```
Define buffer array (Real circular buffer array)
```
uint16_t currentBuffer[FILTER_ORDER_NUM];
uint16_t batPosBuffer[FILTER_ORDER_NUM];
uint16_t batNegBuffer[FILTER_ORDER_NUM];
```
Define output variables
```
double current = 0.0f;
double batPos = 0.0f;
double batNeg = 0.0f;
double batVolt = 0.0f;
```
Define circular_handle_t type structures
```
circular_handle_t curr_cbuf;
circular_handle_t batPos_cbuf;
circular_handle_t batNeg_cbuf;
```
### Main function _main_
Before interrupt enabled, you need initialize circulat buffer, distribute real memorylocation for them.
```
curr_cbuf = circular_buf_init(currentBuffer,FILTER_ORDER_NUM);
batPos_cbuf = circular_buf_init(batPosBuffer,FILTER_ORDER_NUM);
batNeg_cbuf = circular_buf_init(batNegBuffer,FILTER_ORDER_NUM);
```

### ISR function _CPUTimer0ISR_
In interrupt function, we using ADC sample voltage. The sample rate can be modified _InitCPUTimer()_ section of _init.c_ file in _RippleCharging_ folder.
Default sample rate is 10kHz.
First, put ADC result into temp variable.
```
curBuff = AdcbResultRegs.ADCRESULT0;
batPosBuff = AdcaResultRegs.ADCRESULT0;
batNegBuff = AdccResultRegs.ADCRESULT0;
```
Then put temp value into buffer
```
circular_buf_put(curr_cbuf, curBuff);
circular_buf_put(batPos_cbuf, batPosBuff);
circular_buf_put(batNeg_cbuf, batNegBuff);
```
And process the FIR formula, calculate output value.
```
current = filter_process(curr_cbuf, FIRfilter);
batPos = filter_process(batPos_cbuf, FIRfilter);
batNeg = filter_process(batNeg_cbuf, FIRfilter);
```
Ypu can choose output the filter result using DAC output.
```
DAC_setShadowValue(DACA_BASE, batNeg);
```

## Reference
1. circular buffer: https://embeddedartistry.com/blog/2017/05/17/creating-a-circular-buffer-in-c-and-c/
2. FIR implemention using STM32: https://www.youtube.com/watch?v=uNNNj9AZisM
3. FIR implemention: https://github.com/pms67/HadesFCS/tree/master/Filtering
4. Free FIR calculator: http://t-filter.engineerjs.com/
5. 