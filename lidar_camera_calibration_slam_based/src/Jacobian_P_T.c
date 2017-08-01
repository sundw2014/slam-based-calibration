/******************************************************************************
 *                      Code generated with sympy 1.1.1                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                       This file is part of 'project'                       *
 ******************************************************************************/
#include "Jacobian_P_T.h"
#include <math.h>

void Jacobian_P_T(const double *P_L, const double *T, double *out_8569497029995662505) {
   sinT3 = sin(T[3]); cosT3 = cos(T[3]);
   sinT4 = sin(T[4]); cosT4 = cos(T[4]);
   sinT5 = sin(T[5]); cosT5 = cos(T[5]);

   out_8569497029995662505[0] = 500*P_L[3]/(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2]);
   out_8569497029995662505[1] = 0;
   out_8569497029995662505[2] = -((-100*sinT4 + 500*cosT4*cosT5)*P_L[0] + (500*T[0] + 100*T[2])*P_L[3] + (500*sinT3*sinT5 + 500*sinT4*cosT3*cosT5 + 100*cosT3*cosT4)*P_L[2] + (500*sinT3*sinT4*cosT5 + 100*sinT3*cosT4 - 500*sinT5*cosT3)*P_L[1])*P_L[3]/pow(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2], 2) + 100*P_L[3]/(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2]);
   out_8569497029995662505[3] = ((500*sinT3*sinT5 + 500*sinT4*cosT3*cosT5 + 100*cosT3*cosT4)*P_L[1] + (-500*sinT3*sinT4*cosT5 - 100*sinT3*cosT4 + 500*sinT5*cosT3)*P_L[2])/(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2]) + (sinT3*cosT4*P_L[2] - cosT3*cosT4*P_L[1])*((-100*sinT4 + 500*cosT4*cosT5)*P_L[0] + (500*T[0] + 100*T[2])*P_L[3] + (500*sinT3*sinT5 + 500*sinT4*cosT3*cosT5 + 100*cosT3*cosT4)*P_L[2] + (500*sinT3*sinT4*cosT5 + 100*sinT3*cosT4 - 500*sinT5*cosT3)*P_L[1])/pow(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2], 2);
   out_8569497029995662505[4] = ((-100*sinT3*sinT4 + 500*sinT3*cosT4*cosT5)*P_L[1] + (-100*sinT4*cosT3 + 500*cosT3*cosT4*cosT5)*P_L[2] + (-500*sinT4*cosT5 - 100*cosT4)*P_L[0])/(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2]) + (sinT3*sinT4*P_L[1] + sinT4*cosT3*P_L[2] + cosT4*P_L[0])*((-100*sinT4 + 500*cosT4*cosT5)*P_L[0] + (500*T[0] + 100*T[2])*P_L[3] + (500*sinT3*sinT5 + 500*sinT4*cosT3*cosT5 + 100*cosT3*cosT4)*P_L[2] + (500*sinT3*sinT4*cosT5 + 100*sinT3*cosT4 - 500*sinT5*cosT3)*P_L[1])/pow(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2], 2);
   out_8569497029995662505[5] = ((500*sinT3*cosT5 - 500*sinT4*sinT5*cosT3)*P_L[2] + (-500*sinT3*sinT4*sinT5 - 500*cosT3*cosT5)*P_L[1] - 500*sinT5*cosT4*P_L[0])/(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2]);
   out_8569497029995662505[6] = 0;
   out_8569497029995662505[7] = 500*P_L[3]/(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2]);
   out_8569497029995662505[8] = -((100*sinT4 + 500*sinT5*cosT4)*P_L[0] + (500*T[1] - 100*T[2])*P_L[3] + (-500*sinT3*cosT5 + 500*sinT4*sinT5*cosT3 - 100*cosT3*cosT4)*P_L[2] + (500*sinT3*sinT4*sinT5 - 100*sinT3*cosT4 + 500*cosT3*cosT5)*P_L[1])*P_L[3]/pow(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2], 2) - 100*P_L[3]/(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2]);
   out_8569497029995662505[9] = ((-500*sinT3*cosT5 + 500*sinT4*sinT5*cosT3 - 100*cosT3*cosT4)*P_L[1] + (-500*sinT3*sinT4*sinT5 + 100*sinT3*cosT4 - 500*cosT3*cosT5)*P_L[2])/(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2]) + (sinT3*cosT4*P_L[2] - cosT3*cosT4*P_L[1])*((100*sinT4 + 500*sinT5*cosT4)*P_L[0] + (500*T[1] - 100*T[2])*P_L[3] + (-500*sinT3*cosT5 + 500*sinT4*sinT5*cosT3 - 100*cosT3*cosT4)*P_L[2] + (500*sinT3*sinT4*sinT5 - 100*sinT3*cosT4 + 500*cosT3*cosT5)*P_L[1])/pow(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2], 2);
   out_8569497029995662505[10] = ((100*sinT3*sinT4 + 500*sinT3*sinT5*cosT4)*P_L[1] + (-500*sinT4*sinT5 + 100*cosT4)*P_L[0] + (100*sinT4*cosT3 + 500*sinT5*cosT3*cosT4)*P_L[2])/(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2]) + (sinT3*sinT4*P_L[1] + sinT4*cosT3*P_L[2] + cosT4*P_L[0])*((100*sinT4 + 500*sinT5*cosT4)*P_L[0] + (500*T[1] - 100*T[2])*P_L[3] + (-500*sinT3*cosT5 + 500*sinT4*sinT5*cosT3 - 100*cosT3*cosT4)*P_L[2] + (500*sinT3*sinT4*sinT5 - 100*sinT3*cosT4 + 500*cosT3*cosT5)*P_L[1])/pow(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2], 2);
   out_8569497029995662505[11] = ((500*sinT3*sinT5 + 500*sinT4*cosT3*cosT5)*P_L[2] + (500*sinT3*sinT4*cosT5 - 500*sinT5*cosT3)*P_L[1] + 500*cosT4*cosT5*P_L[0])/(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2]);
   out_8569497029995662505[12] = 0;
   out_8569497029995662505[13] = 0;
   out_8569497029995662505[14] = P_L[3];
   out_8569497029995662505[15] = -sinT3*cosT4*P_L[2] + cosT3*cosT4*P_L[1];
   out_8569497029995662505[16] = -sinT3*sinT4*P_L[1] - sinT4*cosT3*P_L[2] - cosT4*P_L[0];
   out_8569497029995662505[17] = 0;

}
