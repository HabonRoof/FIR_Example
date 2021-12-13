/*
 * pid.h
 *
 *  Created on: 2021¦~11¤ë15¤é
 *      Author: johnson
 */

#ifndef PID_H_
#define PID_H_


#include "F28x_Project.h"



typedef struct PID_STRUCT{
    int  RefSetPoint;         // Input: Reference input
    int  Feedback_AdcPoint;         // Input: Feedback input

    float  Error;         // Variable: Error
    float  Error_1;
    float  Error_2;

    float  Kp;          // Parameter: Proportional gain
    float  Ki;          // Parameter: Integral gain
    float  Kd;          // Parameter: Derivative gain

    float  LastOutput;
    float  incOutput;   // Variable: Pre-saturated output
    float  OutputMax;      // Parameter: Maximum output
    float  OutputMin;      // Parameter: Minimum output
    float  Output;         // Output: PID output


}PID_STRUCT;


extern PID_STRUCT* Init_pid(void);
extern void pid_process(PID_STRUCT* psPID);



#endif /* PID_H_ */
