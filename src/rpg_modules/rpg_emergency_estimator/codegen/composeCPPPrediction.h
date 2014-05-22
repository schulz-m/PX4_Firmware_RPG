/*
 * File: composeCPPPrediction.h
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 16-May-2014 12:41:07
 */

#ifndef __COMPOSECPPPREDICTION_H__
#define __COMPOSECPPPREDICTION_H__

/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "composeCPPPrediction_types.h"

/* Function Declarations */
extern void composeCPPPrediction(const double state[9], const double inputs[6],
  const double parameters[8], double f_vec[9], double A_matrix[81], double
  U_matrix[54]);

#endif

/*
 * File trailer for composeCPPPrediction.h
 *
 * [EOF]
 */
