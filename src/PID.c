#include "PID.h"
// USB
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"

void PID_Calc(PID_DATA_t* pid)
{
    float P_Term, I_Term, D_Term, Error, Output;
    Error = pid->SetPoint - pid->Degrees;

    // Calculate the P contribution
    P_Term = Kp * Error;

    // Calculate the I contribution
    pid->I_Term += (Ki * 3333 * Error);
    if(pid->I_Term > 1.0f){
        pid->I_Term = 1.0f;
    }
    else if(pid->I_Term < -1.0f) {
        I_Term = -1.0f;
    }
    I_Term = pid->I_Term;

    // Calculate the D contribution
    D_Term = Kd * pid->Velocity;

    //Calculate output
    Output = P_Term + I_Term - D_Term;

    // Check boundaries
    if(Output > 1.0f){
        Output = 1.0f;
    }
    else if(Output < -1.0f) {
        Output = -1.0f;
    }

    pid->Output = Output;
    //printf("PID: %.5f\r\n", Output);
}
