#ifndef __ARITHMETIC_H
#define __ARITHMETIC_H

#include "matrix.h"
#include "HVPM_Sensorless-Settings.h"

#define opt

#define udc 128

#if (BUILDLEVEL == LEVEL7)
void PMSM_MPC_CCS(float id,float iq,float we,float idref,float weref,float *vd,float *vq,float Ts);
#endif

#if (BUILDLEVEL == LEVEL8)
void PMSM_MPC_CCS_Current(float id,float iq,float we,float idref,float iqref,float *vd,float *vq,float Ts, float ParaMis);
extern float iqmax;
extern float iqmin;
#endif

//void PMSM_MPC_CCS_OPT(float id,float iq,float we,float idref,float weref,float *vd,float *vq,float Ts);
#if (BUILDLEVEL == LEVEL7 || BUILDLEVEL == LEVEL8)
void MPC_CCS_Init(void);
#endif

//void MPC_CCS_Release(void);
typedef struct {
       float id;       // Input: Electrical angle (pu)
       float iq;        // Variable: Direction of rotation (Q0) - independently with global Q
       float we;        // History: Electrical angle at previous step (pu)
       float idref;           // Output: Speed in per-unit  (pu)
       float weref;      // Parameter: Base speed in rpm (Q0) - independently with global Q
       float iqref;
       float vd;            // Parameter: Constant for differentiator (Q21) - independently with global Q
       float vq;              // Parameter: Constant for low-pass filter (pu)
       float Ts;              // Parameter: Constant for low-pass filter (pu)
       } CCS_MPC_CONTROL;    // Data type created

/*-----------------------------------------------------------------------------
Default initalizer for the SPEED_MEAS_QEP object.
-----------------------------------------------------------------------------*/
#define CCS_MPC_CONTROL_DEFAULTS  { 0.0, \
                                    0.0, \
                                    0.0, \
                                    0.0, \
                                    0.0, \
                                    0.0, \
                                    0.0, \
                                    0.0, \
                                    0.0, \
                                    }
#endif

