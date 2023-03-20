/*
 * File: pass_primitive.h
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 03-Dec-2022 14:11:43
 */

#ifndef PASS_PRIMITIVE_H
#define PASS_PRIMITIVE_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void pass_primitive(double v0, double a0, double sf, double vfmin,
                           double vfmax, double Tmin, double Tmax,
                           double coeffsT2[6], double *v2, double *T2,
                           double coeffsT1[6], double *v1, double *T1);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for pass_primitive.h
 *
 * [EOF]
 */
