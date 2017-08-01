/******************************************************************************
 *                      Code generated with sympy 1.1.1                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                       This file is part of 'project'                       *
 ******************************************************************************/
#include "Jacobian_P_T.h"
#include <math.h>

void Jacobian_P_T(const double *P_L, const double *T, double *out_4693495860126994303){

  double sinT3 = sin(T[3]); double cosT3 = cos(T[3]);
  double sinT4 = sin(T[4]); double cosT4 = cos(T[4]);
  double sinT5 = sin(T[5]); double cosT5 = cos(T[5]);
  out_4693495860126994303[0] = 374.672943115*P_L[3]/(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2]);
  out_4693495860126994303[1] = 0;
  out_4693495860126994303[2] = -((-316.473266602*sinT4 + 374.672943115*cosT4*cosT5)*P_L[0] + (374.672943115*T[0] + 316.473266602*T[2])*P_L[3] + (374.672943115*sinT3*sinT5 + 374.672943115*sinT4*cosT3*cosT5 + 316.473266602*cosT3*cosT4)*P_L[2] + (374.672943115*sinT3*sinT4*cosT5 + 316.473266602*sinT3*cosT4 - 374.672943115*sinT5*cosT3)*P_L[1])*P_L[3]/pow(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2], 2) + 316.473266602*P_L[3]/(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2]);
  out_4693495860126994303[3] = ((374.672943115*sinT3*sinT5 + 374.672943115*sinT4*cosT3*cosT5 + 316.473266602*cosT3*cosT4)*P_L[1] + (-374.672943115*sinT3*sinT4*cosT5 - 316.473266602*sinT3*cosT4 + 374.672943115*sinT5*cosT3)*P_L[2])/(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2]) + (sinT3*cosT4*P_L[2] - cosT3*cosT4*P_L[1])*((-316.473266602*sinT4 + 374.672943115*cosT4*cosT5)*P_L[0] + (374.672943115*T[0] + 316.473266602*T[2])*P_L[3] + (374.672943115*sinT3*sinT5 + 374.672943115*sinT4*cosT3*cosT5 + 316.473266602*cosT3*cosT4)*P_L[2] + (374.672943115*sinT3*sinT4*cosT5 + 316.473266602*sinT3*cosT4 - 374.672943115*sinT5*cosT3)*P_L[1])/pow(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2], 2);
  out_4693495860126994303[4] = ((-316.473266602*sinT3*sinT4 + 374.672943115*sinT3*cosT4*cosT5)*P_L[1] + (-316.473266602*sinT4*cosT3 + 374.672943115*cosT3*cosT4*cosT5)*P_L[2] + (-374.672943115*sinT4*cosT5 - 316.473266602*cosT4)*P_L[0])/(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2]) + (sinT3*sinT4*P_L[1] + sinT4*cosT3*P_L[2] + cosT4*P_L[0])*((-316.473266602*sinT4 + 374.672943115*cosT4*cosT5)*P_L[0] + (374.672943115*T[0] + 316.473266602*T[2])*P_L[3] + (374.672943115*sinT3*sinT5 + 374.672943115*sinT4*cosT3*cosT5 + 316.473266602*cosT3*cosT4)*P_L[2] + (374.672943115*sinT3*sinT4*cosT5 + 316.473266602*sinT3*cosT4 - 374.672943115*sinT5*cosT3)*P_L[1])/pow(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2], 2);
  out_4693495860126994303[5] = ((374.672943115*sinT3*cosT5 - 374.672943115*sinT4*sinT5*cosT3)*P_L[2] + (-374.672943115*sinT3*sinT4*sinT5 - 374.672943115*cosT3*cosT5)*P_L[1] - 374.672943115*sinT5*cosT4*P_L[0])/(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2]);
  out_4693495860126994303[6] = 0;
  out_4693495860126994303[7] = 930.62701416*P_L[3]/(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2]);
  out_4693495860126994303[8] = -((-239.77923584*sinT4 + 930.62701416*sinT5*cosT4)*P_L[0] + (930.62701416*T[1] + 239.77923584*T[2])*P_L[3] + (-930.62701416*sinT3*cosT5 + 930.62701416*sinT4*sinT5*cosT3 + 239.77923584*cosT3*cosT4)*P_L[2] + (930.62701416*sinT3*sinT4*sinT5 + 239.77923584*sinT3*cosT4 + 930.62701416*cosT3*cosT5)*P_L[1])*P_L[3]/pow(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2], 2) + 239.77923584*P_L[3]/(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2]);
  out_4693495860126994303[9] = ((-930.62701416*sinT3*cosT5 + 930.62701416*sinT4*sinT5*cosT3 + 239.77923584*cosT3*cosT4)*P_L[1] + (-930.62701416*sinT3*sinT4*sinT5 - 239.77923584*sinT3*cosT4 - 930.62701416*cosT3*cosT5)*P_L[2])/(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2]) + (sinT3*cosT4*P_L[2] - cosT3*cosT4*P_L[1])*((-239.77923584*sinT4 + 930.62701416*sinT5*cosT4)*P_L[0] + (930.62701416*T[1] + 239.77923584*T[2])*P_L[3] + (-930.62701416*sinT3*cosT5 + 930.62701416*sinT4*sinT5*cosT3 + 239.77923584*cosT3*cosT4)*P_L[2] + (930.62701416*sinT3*sinT4*sinT5 + 239.77923584*sinT3*cosT4 + 930.62701416*cosT3*cosT5)*P_L[1])/pow(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2], 2);
  out_4693495860126994303[10] = ((-239.77923584*sinT3*sinT4 + 930.62701416*sinT3*sinT5*cosT4)*P_L[1] + (-930.62701416*sinT4*sinT5 - 239.77923584*cosT4)*P_L[0] + (-239.77923584*sinT4*cosT3 + 930.62701416*sinT5*cosT3*cosT4)*P_L[2])/(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2]) + (sinT3*sinT4*P_L[1] + sinT4*cosT3*P_L[2] + cosT4*P_L[0])*((-239.77923584*sinT4 + 930.62701416*sinT5*cosT4)*P_L[0] + (930.62701416*T[1] + 239.77923584*T[2])*P_L[3] + (-930.62701416*sinT3*cosT5 + 930.62701416*sinT4*sinT5*cosT3 + 239.77923584*cosT3*cosT4)*P_L[2] + (930.62701416*sinT3*sinT4*sinT5 + 239.77923584*sinT3*cosT4 + 930.62701416*cosT3*cosT5)*P_L[1])/pow(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2], 2);
  out_4693495860126994303[11] = ((930.62701416*sinT3*sinT5 + 930.62701416*sinT4*cosT3*cosT5)*P_L[2] + (930.62701416*sinT3*sinT4*cosT5 - 930.62701416*sinT5*cosT3)*P_L[1] + 930.62701416*cosT4*cosT5*P_L[0])/(sinT3*cosT4*P_L[1] - sinT4*P_L[0] + cosT3*cosT4*P_L[2] + P_L[3]*T[2]);
  out_4693495860126994303[12] = 0;
  out_4693495860126994303[13] = 0;
  out_4693495860126994303[14] = P_L[3];
  out_4693495860126994303[15] = -sinT3*cosT4*P_L[2] + cosT3*cosT4*P_L[1];
  out_4693495860126994303[16] = -sinT3*sinT4*P_L[1] - sinT4*cosT3*P_L[2] - cosT4*P_L[0];
  out_4693495860126994303[17] = 0;

}
