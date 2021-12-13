/*
 * init.h
 *
 *  Created on: 2021¦~6¤ë11¤é
 *      Author: Johnson
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

#ifndef INIT_H_
#define INIT_H_

extern void InitGPIO(void);
extern void InitPIE(void);
extern void InitSPI(void);
extern void InitADC(void);
extern void InitADCSOC(void);
extern void InitCPUTimer(void);
extern void InitSCIB(void);
extern void InitDAC(void);

#endif /* INIT_H_ */
