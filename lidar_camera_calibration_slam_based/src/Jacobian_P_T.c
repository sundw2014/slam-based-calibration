/******************************************************************************
 *                      Code generated with sympy 1.1.1                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                       This file is part of 'project'                       *
 ******************************************************************************/
#include "Jacobian_P_T.h"
#include <math.h>

void Jacobian_P_T(double *P_L, double *T, double *out_8569497029995662505) {

   out_8569497029995662505[0] = 500*P_L[3]/(sin(T[3])*cos(T[4])*P_L[1] - sin(T[4])*P_L[0] + cos(T[3])*cos(T[4])*P_L[2] + P_L[3]*T[2]);
   out_8569497029995662505[1] = 0;
   out_8569497029995662505[2] = -((-100*sin(T[4]) + 500*cos(T[4])*cos(T[5]))*P_L[0] + (500*T[0] + 100*T[2])*P_L[3] + (500*sin(T[3])*sin(T[5]) + 500*sin(T[4])*cos(T[3])*cos(T[5]) + 100*cos(T[3])*cos(T[4]))*P_L[2] + (500*sin(T[3])*sin(T[4])*cos(T[5]) + 100*sin(T[3])*cos(T[4]) - 500*sin(T[5])*cos(T[3]))*P_L[1])*P_L[3]/pow(sin(T[3])*cos(T[4])*P_L[1] - sin(T[4])*P_L[0] + cos(T[3])*cos(T[4])*P_L[2] + P_L[3]*T[2], 2) + 100*P_L[3]/(sin(T[3])*cos(T[4])*P_L[1] - sin(T[4])*P_L[0] + cos(T[3])*cos(T[4])*P_L[2] + P_L[3]*T[2]);
   out_8569497029995662505[3] = ((500*sin(T[3])*sin(T[5]) + 500*sin(T[4])*cos(T[3])*cos(T[5]) + 100*cos(T[3])*cos(T[4]))*P_L[1] + (-500*sin(T[3])*sin(T[4])*cos(T[5]) - 100*sin(T[3])*cos(T[4]) + 500*sin(T[5])*cos(T[3]))*P_L[2])/(sin(T[3])*cos(T[4])*P_L[1] - sin(T[4])*P_L[0] + cos(T[3])*cos(T[4])*P_L[2] + P_L[3]*T[2]) + (sin(T[3])*cos(T[4])*P_L[2] - cos(T[3])*cos(T[4])*P_L[1])*((-100*sin(T[4]) + 500*cos(T[4])*cos(T[5]))*P_L[0] + (500*T[0] + 100*T[2])*P_L[3] + (500*sin(T[3])*sin(T[5]) + 500*sin(T[4])*cos(T[3])*cos(T[5]) + 100*cos(T[3])*cos(T[4]))*P_L[2] + (500*sin(T[3])*sin(T[4])*cos(T[5]) + 100*sin(T[3])*cos(T[4]) - 500*sin(T[5])*cos(T[3]))*P_L[1])/pow(sin(T[3])*cos(T[4])*P_L[1] - sin(T[4])*P_L[0] + cos(T[3])*cos(T[4])*P_L[2] + P_L[3]*T[2], 2);
   out_8569497029995662505[4] = ((-100*sin(T[3])*sin(T[4]) + 500*sin(T[3])*cos(T[4])*cos(T[5]))*P_L[1] + (-100*sin(T[4])*cos(T[3]) + 500*cos(T[3])*cos(T[4])*cos(T[5]))*P_L[2] + (-500*sin(T[4])*cos(T[5]) - 100*cos(T[4]))*P_L[0])/(sin(T[3])*cos(T[4])*P_L[1] - sin(T[4])*P_L[0] + cos(T[3])*cos(T[4])*P_L[2] + P_L[3]*T[2]) + (sin(T[3])*sin(T[4])*P_L[1] + sin(T[4])*cos(T[3])*P_L[2] + cos(T[4])*P_L[0])*((-100*sin(T[4]) + 500*cos(T[4])*cos(T[5]))*P_L[0] + (500*T[0] + 100*T[2])*P_L[3] + (500*sin(T[3])*sin(T[5]) + 500*sin(T[4])*cos(T[3])*cos(T[5]) + 100*cos(T[3])*cos(T[4]))*P_L[2] + (500*sin(T[3])*sin(T[4])*cos(T[5]) + 100*sin(T[3])*cos(T[4]) - 500*sin(T[5])*cos(T[3]))*P_L[1])/pow(sin(T[3])*cos(T[4])*P_L[1] - sin(T[4])*P_L[0] + cos(T[3])*cos(T[4])*P_L[2] + P_L[3]*T[2], 2);
   out_8569497029995662505[5] = ((500*sin(T[3])*cos(T[5]) - 500*sin(T[4])*sin(T[5])*cos(T[3]))*P_L[2] + (-500*sin(T[3])*sin(T[4])*sin(T[5]) - 500*cos(T[3])*cos(T[5]))*P_L[1] - 500*sin(T[5])*cos(T[4])*P_L[0])/(sin(T[3])*cos(T[4])*P_L[1] - sin(T[4])*P_L[0] + cos(T[3])*cos(T[4])*P_L[2] + P_L[3]*T[2]);
   out_8569497029995662505[6] = 0;
   out_8569497029995662505[7] = 500*P_L[3]/(sin(T[3])*cos(T[4])*P_L[1] - sin(T[4])*P_L[0] + cos(T[3])*cos(T[4])*P_L[2] + P_L[3]*T[2]);
   out_8569497029995662505[8] = -((100*sin(T[4]) + 500*sin(T[5])*cos(T[4]))*P_L[0] + (500*T[1] - 100*T[2])*P_L[3] + (-500*sin(T[3])*cos(T[5]) + 500*sin(T[4])*sin(T[5])*cos(T[3]) - 100*cos(T[3])*cos(T[4]))*P_L[2] + (500*sin(T[3])*sin(T[4])*sin(T[5]) - 100*sin(T[3])*cos(T[4]) + 500*cos(T[3])*cos(T[5]))*P_L[1])*P_L[3]/pow(sin(T[3])*cos(T[4])*P_L[1] - sin(T[4])*P_L[0] + cos(T[3])*cos(T[4])*P_L[2] + P_L[3]*T[2], 2) - 100*P_L[3]/(sin(T[3])*cos(T[4])*P_L[1] - sin(T[4])*P_L[0] + cos(T[3])*cos(T[4])*P_L[2] + P_L[3]*T[2]);
   out_8569497029995662505[9] = ((-500*sin(T[3])*cos(T[5]) + 500*sin(T[4])*sin(T[5])*cos(T[3]) - 100*cos(T[3])*cos(T[4]))*P_L[1] + (-500*sin(T[3])*sin(T[4])*sin(T[5]) + 100*sin(T[3])*cos(T[4]) - 500*cos(T[3])*cos(T[5]))*P_L[2])/(sin(T[3])*cos(T[4])*P_L[1] - sin(T[4])*P_L[0] + cos(T[3])*cos(T[4])*P_L[2] + P_L[3]*T[2]) + (sin(T[3])*cos(T[4])*P_L[2] - cos(T[3])*cos(T[4])*P_L[1])*((100*sin(T[4]) + 500*sin(T[5])*cos(T[4]))*P_L[0] + (500*T[1] - 100*T[2])*P_L[3] + (-500*sin(T[3])*cos(T[5]) + 500*sin(T[4])*sin(T[5])*cos(T[3]) - 100*cos(T[3])*cos(T[4]))*P_L[2] + (500*sin(T[3])*sin(T[4])*sin(T[5]) - 100*sin(T[3])*cos(T[4]) + 500*cos(T[3])*cos(T[5]))*P_L[1])/pow(sin(T[3])*cos(T[4])*P_L[1] - sin(T[4])*P_L[0] + cos(T[3])*cos(T[4])*P_L[2] + P_L[3]*T[2], 2);
   out_8569497029995662505[10] = ((100*sin(T[3])*sin(T[4]) + 500*sin(T[3])*sin(T[5])*cos(T[4]))*P_L[1] + (-500*sin(T[4])*sin(T[5]) + 100*cos(T[4]))*P_L[0] + (100*sin(T[4])*cos(T[3]) + 500*sin(T[5])*cos(T[3])*cos(T[4]))*P_L[2])/(sin(T[3])*cos(T[4])*P_L[1] - sin(T[4])*P_L[0] + cos(T[3])*cos(T[4])*P_L[2] + P_L[3]*T[2]) + (sin(T[3])*sin(T[4])*P_L[1] + sin(T[4])*cos(T[3])*P_L[2] + cos(T[4])*P_L[0])*((100*sin(T[4]) + 500*sin(T[5])*cos(T[4]))*P_L[0] + (500*T[1] - 100*T[2])*P_L[3] + (-500*sin(T[3])*cos(T[5]) + 500*sin(T[4])*sin(T[5])*cos(T[3]) - 100*cos(T[3])*cos(T[4]))*P_L[2] + (500*sin(T[3])*sin(T[4])*sin(T[5]) - 100*sin(T[3])*cos(T[4]) + 500*cos(T[3])*cos(T[5]))*P_L[1])/pow(sin(T[3])*cos(T[4])*P_L[1] - sin(T[4])*P_L[0] + cos(T[3])*cos(T[4])*P_L[2] + P_L[3]*T[2], 2);
   out_8569497029995662505[11] = ((500*sin(T[3])*sin(T[5]) + 500*sin(T[4])*cos(T[3])*cos(T[5]))*P_L[2] + (500*sin(T[3])*sin(T[4])*cos(T[5]) - 500*sin(T[5])*cos(T[3]))*P_L[1] + 500*cos(T[4])*cos(T[5])*P_L[0])/(sin(T[3])*cos(T[4])*P_L[1] - sin(T[4])*P_L[0] + cos(T[3])*cos(T[4])*P_L[2] + P_L[3]*T[2]);
   out_8569497029995662505[12] = 0;
   out_8569497029995662505[13] = 0;
   out_8569497029995662505[14] = P_L[3];
   out_8569497029995662505[15] = -sin(T[3])*cos(T[4])*P_L[2] + cos(T[3])*cos(T[4])*P_L[1];
   out_8569497029995662505[16] = -sin(T[3])*sin(T[4])*P_L[1] - sin(T[4])*cos(T[3])*P_L[2] - cos(T[4])*P_L[0];
   out_8569497029995662505[17] = 0;

}
