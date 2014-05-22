/*
 * _coder_composeCPPPrediction_mex.c
 *
 * Code generation for function 'composeCPPPrediction'
 *
 */

/* Include files */
#include "mex.h"
#include "_coder_composeCPPPrediction_api.h"

/* Function Declarations */
static void composeCPPPrediction_mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);

/* Variable Definitions */
emlrtContext emlrtContextGlobal = { true, false, EMLRT_VERSION_INFO, NULL, "composeCPPPrediction", NULL, false, {2045744189U,2170104910U,2743257031U,4284093946U}, NULL };
void *emlrtRootTLSGlobal = NULL;

/* Function Definitions */
static void composeCPPPrediction_mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  const mxArray *outputs[3];
  const mxArray *inputs[3];
  int n = 0;
  int nOutputs = (nlhs < 1 ? 1 : nlhs);
  int nInputs = nrhs;
  emlrtStack st = { NULL, NULL, NULL };
  /* Module initialization. */
  composeCPPPrediction_initialize(&emlrtContextGlobal);
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, mxINT32_CLASS, 3, mxCHAR_CLASS, 20, "composeCPPPrediction");
  } else if (nlhs > 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, mxCHAR_CLASS, 20, "composeCPPPrediction");
  }
  /* Temporary copy for mex inputs. */
  for (n = 0; n < nInputs; ++n) {
    inputs[n] = prhs[n];
  }
  /* Call the function. */
  composeCPPPrediction_api(inputs, outputs);
  /* Copy over outputs to the caller. */
  for (n = 0; n < nOutputs; ++n) {
    plhs[n] = emlrtReturnArrayR2009a(outputs[n]);
  }
  /* Module finalization. */
  composeCPPPrediction_terminate();
}

void composeCPPPrediction_atexit_wrapper(void)
{
   composeCPPPrediction_atexit();
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  /* Initialize the memory manager. */
  mexAtExit(composeCPPPrediction_atexit_wrapper);
  /* Dispatch the entry-point. */
  composeCPPPrediction_mexFunction(nlhs, plhs, nrhs, prhs);
}
/* End of code generation (_coder_composeCPPPrediction_mex.c) */
