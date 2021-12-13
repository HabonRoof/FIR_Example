/*
 * pid.c
 *
 *  Created on: 2021¦~11¤ë15¤é
 *      Author: johnson
 */

#include    "pid.h"

static PID_STRUCT sPID;

PID_STRUCT* Init_pid(void)
{
    // Set PID Coefficients
    //sPID.Kp = 4.4674;
    sPID.Kp = 0.0002;

    //sPID.Ki = 0.073693;
    sPID.Ki = 0.00005;

    //sPID.Kd =0.0006771;
    sPID.Kd = 0;

    // Set PID Setpoint
    //sPID.RefSetPoint = 1237;        // Default output voltage 10.0V
    sPID.RefSetPoint = 2800;        // Default output voltage 24.0V

    sPID.OutputMax = 1550;
    sPID.OutputMin = 1450;
    sPID.Error_2 = 0;
    sPID.Error_1 = 0;
    sPID.LastOutput = 0.0;
    return &sPID;
}

void pid_process(PID_STRUCT *psPID)
{
    // Compute the error
    psPID->Error = psPID->RefSetPoint - psPID->Feedback_AdcPoint;

    // Compute the proportional output
    //psPID->Up = psPID->Kp * psPID->Error;
    psPID->incOutput = psPID->Kp * (psPID->Error - psPID->Error_1);

    // Compute the integral output
    //psPID->Ui = psPID->Ui + (psPID->Ki * psPID->Up) + (psPID->Kc * psPID->SatErr);
    psPID->incOutput += psPID->Ki * (psPID->Error + psPID->Error_1);

    // Compute the derivative output
    //psPID->Ud = psPID->Kd * (psPID->Up - psPID->Up1);
    psPID->incOutput += psPID->Kd * (psPID->Error - 2 * psPID->Error_1 + psPID->Error_2);

    psPID->Output = psPID->LastOutput + psPID->incOutput;

    // Saturate the output
    if (psPID->Output > psPID->OutputMax)
        psPID->Output = psPID->OutputMax;

    else if (psPID->Output < psPID->OutputMin)
        psPID->Output = psPID->OutputMin;

    // Update the previous output
    psPID->LastOutput = psPID->Output;

    // Update last error
    psPID->Error_2 = psPID->Error_1;
    psPID->Error_1 = psPID->Error;

}


