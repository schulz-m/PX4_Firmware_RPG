/* 
 * File: _coder_composeCPPPrediction_api.h 
 *  
 * MATLAB Coder version            : 2.6 
 * C/C++ source code generated on  : 16-May-2014 12:41:07 
 */

#ifndef ___CODER_COMPOSECPPPREDICTION_API_H__
#define ___CODER_COMPOSECPPPREDICTION_API_H__
/* Include files */
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"

/* Function Declarations */
extern void composeCPPPrediction_initialize(emlrtContext *aContext);
extern void composeCPPPrediction_terminate(void);
extern void composeCPPPrediction_atexit(void);
extern void composeCPPPrediction_api(const mxArray *prhs[3], const mxArray *plhs[3]);
extern void composeCPPPrediction(double state[9], double inputs[6], double parameters[8], double f_vec[9], double A_matrix[81], double U_matrix[54]);
extern void composeCPPPrediction_xil_terminate(void);

#endif
/* 
 * File trailer for _coder_composeCPPPrediction_api.h 
 *  
 * [EOF] 
 */
