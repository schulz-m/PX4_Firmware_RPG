/*
 * File: _coder_composeCPPPrediction_api.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 16-May-2014 12:41:07
 */

/* Include files */
#include "_coder_composeCPPPrediction_api.h"

/* Function Declarations */
static double (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *state,
  const char *identifier))[9];
static double (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[9];
static double (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *inputs,
  const char *identifier))[6];
static double (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[6];
static double (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *parameters, const char *identifier))[8];
static double (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[8];
static const mxArray *emlrt_marshallOut(const double u[9]);
static const mxArray *b_emlrt_marshallOut(const double u[81]);
static const mxArray *c_emlrt_marshallOut(const double u[54]);
static double (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[9];
static double (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[6];
static double (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[8];

/* Function Definitions */

/*
 * Arguments    : emlrtContext *aContext
 * Return Type  : void
 */
void composeCPPPrediction_initialize(emlrtContext *aContext)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, aContext, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void composeCPPPrediction_terminate(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void composeCPPPrediction_atexit(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  composeCPPPrediction_xil_terminate();
}

/*
 * Arguments    : const mxArray *prhs[3]
 *                const mxArray *plhs[3]
 * Return Type  : void
 */
void composeCPPPrediction_api(const mxArray *prhs[3], const mxArray *plhs[3])
{
  double (*f_vec)[9];
  double (*A_matrix)[81];
  double (*U_matrix)[54];
  double (*state)[9];
  double (*inputs)[6];
  double (*parameters)[8];
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  f_vec = (double (*)[9])mxMalloc(sizeof(double [9]));
  A_matrix = (double (*)[81])mxMalloc(sizeof(double [81]));
  U_matrix = (double (*)[54])mxMalloc(sizeof(double [54]));
  prhs[0] = emlrtProtectR2012b(prhs[0], 0, false, -1);
  prhs[1] = emlrtProtectR2012b(prhs[1], 1, false, -1);
  prhs[2] = emlrtProtectR2012b(prhs[2], 2, false, -1);

  /* Marshall function inputs */
  state = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "state");
  inputs = c_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "inputs");
  parameters = e_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "parameters");

  /* Invoke the target function */
  composeCPPPrediction(*state, *inputs, *parameters, *f_vec, *A_matrix,
                       *U_matrix);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*f_vec);
  plhs[1] = b_emlrt_marshallOut(*A_matrix);
  plhs[2] = c_emlrt_marshallOut(*U_matrix);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *state
 *                const char *identifier
 * Return Type  : double (*)[9]
 */
static double (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *state,
  const char *identifier))[9]
{
  double (*y)[9];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = b_emlrt_marshallIn(sp, emlrtAlias(state), &thisId);
  emlrtDestroyArray(&state);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : double (*)[9]
 */
  static double (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[9]
{
  double (*y)[9];
  y = g_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *inputs
 *                const char *identifier
 * Return Type  : double (*)[6]
 */
static double (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *inputs,
  const char *identifier))[6]
{
  double (*y)[6];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = d_emlrt_marshallIn(sp, emlrtAlias(inputs), &thisId);
  emlrtDestroyArray(&inputs);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : double (*)[6]
 */
  static double (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[6]
{
  double (*y)[6];
  y = h_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *parameters
 *                const char *identifier
 * Return Type  : double (*)[8]
 */
static double (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *parameters, const char *identifier))[8]
{
  double (*y)[8];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = f_emlrt_marshallIn(sp, emlrtAlias(parameters), &thisId);
  emlrtDestroyArray(&parameters);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : double (*)[8]
 */
  static double (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[8]
{
  double (*y)[8];
  y = i_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const double u[9]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const double u[9])
{
  const mxArray *y;
  static const int iv0[1] = { 0 };

  const mxArray *m0;
  static const int iv1[1] = { 9 };

  y = NULL;
  m0 = emlrtCreateNumericArray(1, iv0, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m0, (void *)u);
  emlrtSetDimensions((mxArray *)m0, iv1, 1);
  emlrtAssign(&y, m0);
  return y;
}

/*
 * Arguments    : const double u[81]
 * Return Type  : const mxArray *
 */
static const mxArray *b_emlrt_marshallOut(const double u[81])
{
  const mxArray *y;
  static const int iv2[2] = { 0, 0 };

  const mxArray *m1;
  static const int iv3[2] = { 9, 9 };

  y = NULL;
  m1 = emlrtCreateNumericArray(2, iv2, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m1, (void *)u);
  emlrtSetDimensions((mxArray *)m1, iv3, 2);
  emlrtAssign(&y, m1);
  return y;
}

/*
 * Arguments    : const double u[54]
 * Return Type  : const mxArray *
 */
static const mxArray *c_emlrt_marshallOut(const double u[54])
{
  const mxArray *y;
  static const int iv4[2] = { 0, 0 };

  const mxArray *m2;
  static const int iv5[2] = { 9, 6 };

  y = NULL;
  m2 = emlrtCreateNumericArray(2, iv4, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m2, (void *)u);
  emlrtSetDimensions((mxArray *)m2, iv5, 2);
  emlrtAssign(&y, m2);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : double (*)[9]
 */
static double (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[9]
{
  double (*ret)[9];
  int iv6[1];
  iv6[0] = 9;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, iv6);
  ret = (double (*)[9])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : double (*)[6]
 */
  static double (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[6]
{
  double (*ret)[6];
  int iv7[1];
  iv7[0] = 6;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, iv7);
  ret = (double (*)[6])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : double (*)[8]
 */
static double (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[8]
{
  double (*ret)[8];
  int iv8[1];
  iv8[0] = 8;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, iv8);
  ret = (double (*)[8])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * File trailer for _coder_composeCPPPrediction_api.c
 *
 * [EOF]
 */
