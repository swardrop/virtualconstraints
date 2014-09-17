/* Include files */

#include <stddef.h>
#include "blas.h"
#include "RobotSim_sfun.h"
#include "c2_RobotSim.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "RobotSim_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c2_debug_family_names[10] = { "M", "C", "G", "B", "nargin",
  "nargout", "u", "q", "qd", "qdd" };

static const char * c2_b_debug_family_names[7] = { "g", "m", "l", "I", "r",
  "nargin", "nargout" };

static const char * c2_c_debug_family_names[13] = { "B", "I", "l", "m", "g",
  "B_perp", "nargin", "nargout", "q", "qd", "M", "C", "G" };

/* Function Declarations */
static void initialize_c2_RobotSim(SFc2_RobotSimInstanceStruct *chartInstance);
static void initialize_params_c2_RobotSim(SFc2_RobotSimInstanceStruct
  *chartInstance);
static void enable_c2_RobotSim(SFc2_RobotSimInstanceStruct *chartInstance);
static void disable_c2_RobotSim(SFc2_RobotSimInstanceStruct *chartInstance);
static void c2_update_debugger_state_c2_RobotSim(SFc2_RobotSimInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c2_RobotSim(SFc2_RobotSimInstanceStruct
  *chartInstance);
static void set_sim_state_c2_RobotSim(SFc2_RobotSimInstanceStruct *chartInstance,
  const mxArray *c2_st);
static void finalize_c2_RobotSim(SFc2_RobotSimInstanceStruct *chartInstance);
static void sf_c2_RobotSim(SFc2_RobotSimInstanceStruct *chartInstance);
static void c2_chartstep_c2_RobotSim(SFc2_RobotSimInstanceStruct *chartInstance);
static void initSimStructsc2_RobotSim(SFc2_RobotSimInstanceStruct *chartInstance);
static void registerMessagesc2_RobotSim(SFc2_RobotSimInstanceStruct
  *chartInstance);
static void c2_dynMatrices(SFc2_RobotSimInstanceStruct *chartInstance, real_T
  c2_q[2], real_T c2_qd[2], real_T c2_M[4], real_T c2_C[4], real_T c2_G[2]);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static void c2_emlrt_marshallIn(SFc2_RobotSimInstanceStruct *chartInstance,
  const mxArray *c2_qdd, const char_T *c2_identifier, real_T c2_y[2]);
static void c2_b_emlrt_marshallIn(SFc2_RobotSimInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[2]);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_c_emlrt_marshallIn(SFc2_RobotSimInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_d_emlrt_marshallIn(SFc2_RobotSimInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[4]);
static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_e_emlrt_marshallIn(SFc2_RobotSimInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[2]);
static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[42]);
static void c2_eml_scalar_eg(SFc2_RobotSimInstanceStruct *chartInstance);
static void c2_eml_warning(SFc2_RobotSimInstanceStruct *chartInstance);
static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_f_emlrt_marshallIn(SFc2_RobotSimInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static uint8_T c2_g_emlrt_marshallIn(SFc2_RobotSimInstanceStruct *chartInstance,
  const mxArray *c2_b_is_active_c2_RobotSim, const char_T *c2_identifier);
static uint8_T c2_h_emlrt_marshallIn(SFc2_RobotSimInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void init_dsm_address_info(SFc2_RobotSimInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c2_RobotSim(SFc2_RobotSimInstanceStruct *chartInstance)
{
  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c2_is_active_c2_RobotSim = 0U;
}

static void initialize_params_c2_RobotSim(SFc2_RobotSimInstanceStruct
  *chartInstance)
{
}

static void enable_c2_RobotSim(SFc2_RobotSimInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c2_RobotSim(SFc2_RobotSimInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c2_update_debugger_state_c2_RobotSim(SFc2_RobotSimInstanceStruct
  *chartInstance)
{
}

static const mxArray *get_sim_state_c2_RobotSim(SFc2_RobotSimInstanceStruct
  *chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  int32_T c2_i0;
  real_T c2_u[2];
  const mxArray *c2_b_y = NULL;
  uint8_T c2_hoistedGlobal;
  uint8_T c2_b_u;
  const mxArray *c2_c_y = NULL;
  real_T (*c2_qdd)[2];
  c2_qdd = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellarray(2), FALSE);
  for (c2_i0 = 0; c2_i0 < 2; c2_i0++) {
    c2_u[c2_i0] = (*c2_qdd)[c2_i0];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 2), FALSE);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_hoistedGlobal = chartInstance->c2_is_active_c2_RobotSim;
  c2_b_u = c2_hoistedGlobal;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  sf_mex_assign(&c2_st, c2_y, FALSE);
  return c2_st;
}

static void set_sim_state_c2_RobotSim(SFc2_RobotSimInstanceStruct *chartInstance,
  const mxArray *c2_st)
{
  const mxArray *c2_u;
  real_T c2_dv0[2];
  int32_T c2_i1;
  real_T (*c2_qdd)[2];
  c2_qdd = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c2_doneDoubleBufferReInit = TRUE;
  c2_u = sf_mex_dup(c2_st);
  c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 0)), "qdd",
                      c2_dv0);
  for (c2_i1 = 0; c2_i1 < 2; c2_i1++) {
    (*c2_qdd)[c2_i1] = c2_dv0[c2_i1];
  }

  chartInstance->c2_is_active_c2_RobotSim = c2_g_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c2_u, 1)), "is_active_c2_RobotSim");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_RobotSim(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_RobotSim(SFc2_RobotSimInstanceStruct *chartInstance)
{
}

static void sf_c2_RobotSim(SFc2_RobotSimInstanceStruct *chartInstance)
{
  int32_T c2_i2;
  int32_T c2_i3;
  int32_T c2_i4;
  real_T *c2_u;
  real_T (*c2_qd)[2];
  real_T (*c2_q)[2];
  real_T (*c2_qdd)[2];
  c2_qd = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 2);
  c2_q = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 1);
  c2_qdd = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_u = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c2_u, 0U);
  for (c2_i2 = 0; c2_i2 < 2; c2_i2++) {
    _SFD_DATA_RANGE_CHECK((*c2_qdd)[c2_i2], 1U);
  }

  for (c2_i3 = 0; c2_i3 < 2; c2_i3++) {
    _SFD_DATA_RANGE_CHECK((*c2_q)[c2_i3], 2U);
  }

  for (c2_i4 = 0; c2_i4 < 2; c2_i4++) {
    _SFD_DATA_RANGE_CHECK((*c2_qd)[c2_i4], 3U);
  }

  chartInstance->c2_sfEvent = CALL_EVENT;
  c2_chartstep_c2_RobotSim(chartInstance);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_RobotSimMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c2_chartstep_c2_RobotSim(SFc2_RobotSimInstanceStruct *chartInstance)
{
  real_T c2_hoistedGlobal;
  real_T c2_u;
  int32_T c2_i5;
  real_T c2_q[2];
  int32_T c2_i6;
  real_T c2_qd[2];
  uint32_T c2_debug_family_var_map[10];
  real_T c2_M[4];
  real_T c2_C[4];
  real_T c2_G[2];
  real_T c2_B[2];
  real_T c2_nargin = 3.0;
  real_T c2_nargout = 1.0;
  real_T c2_qdd[2];
  int32_T c2_i7;
  real_T c2_b_q[2];
  int32_T c2_i8;
  real_T c2_b_qd[2];
  real_T c2_b_G[2];
  real_T c2_b_C[4];
  real_T c2_b_M[4];
  int32_T c2_i9;
  int32_T c2_i10;
  int32_T c2_i11;
  int32_T c2_i12;
  real_T c2_b;
  int32_T c2_i13;
  int32_T c2_i14;
  int32_T c2_i15;
  real_T c2_b_b[2];
  int32_T c2_i16;
  real_T c2_y[2];
  int32_T c2_i17;
  int32_T c2_i18;
  int32_T c2_i19;
  int32_T c2_i20;
  real_T c2_x;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_b_y;
  real_T c2_d_x;
  real_T c2_e_x;
  real_T c2_c_y;
  real_T c2_d;
  real_T c2_f_x;
  real_T c2_g_x;
  real_T c2_h_x;
  real_T c2_d_y;
  real_T c2_i_x;
  real_T c2_j_x;
  real_T c2_e_y;
  real_T c2_b_d;
  int32_T c2_r1;
  int32_T c2_r2;
  real_T c2_k_x;
  real_T c2_f_y;
  real_T c2_a21;
  real_T c2_a;
  real_T c2_c_b;
  real_T c2_g_y;
  real_T c2_a22;
  real_T c2_b_a;
  real_T c2_d_b;
  real_T c2_h_y;
  real_T c2_l_x;
  real_T c2_i_y;
  real_T c2_z;
  real_T c2_c_a;
  real_T c2_e_b;
  real_T c2_j_y;
  real_T c2_m_x;
  real_T c2_k_y;
  real_T c2_b_z;
  int32_T c2_i21;
  real_T (*c2_b_qdd)[2];
  real_T *c2_b_u;
  real_T (*c2_c_qd)[2];
  real_T (*c2_c_q)[2];
  boolean_T guard1 = FALSE;
  c2_c_qd = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 2);
  c2_c_q = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 1);
  c2_b_qdd = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_b_u = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  c2_hoistedGlobal = *c2_b_u;
  c2_u = c2_hoistedGlobal;
  for (c2_i5 = 0; c2_i5 < 2; c2_i5++) {
    c2_q[c2_i5] = (*c2_c_q)[c2_i5];
  }

  for (c2_i6 = 0; c2_i6 < 2; c2_i6++) {
    c2_qd[c2_i6] = (*c2_c_qd)[c2_i6];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 10U, 10U, c2_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_M, 0U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_C, 1U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_G, 2U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_B, 3U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 4U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 5U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_u, 6U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_q, 7U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_qd, 8U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_qdd, 9U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 4);
  for (c2_i7 = 0; c2_i7 < 2; c2_i7++) {
    c2_b_q[c2_i7] = c2_q[c2_i7];
  }

  for (c2_i8 = 0; c2_i8 < 2; c2_i8++) {
    c2_b_qd[c2_i8] = c2_qd[c2_i8];
  }

  c2_dynMatrices(chartInstance, c2_b_q, c2_b_qd, c2_b_M, c2_b_C, c2_b_G);
  for (c2_i9 = 0; c2_i9 < 4; c2_i9++) {
    c2_M[c2_i9] = c2_b_M[c2_i9];
  }

  for (c2_i10 = 0; c2_i10 < 4; c2_i10++) {
    c2_C[c2_i10] = c2_b_C[c2_i10];
  }

  for (c2_i11 = 0; c2_i11 < 2; c2_i11++) {
    c2_G[c2_i11] = c2_b_G[c2_i11];
  }

  for (c2_i12 = 0; c2_i12 < 2; c2_i12++) {
    c2_B[c2_i12] = 1.0 - (real_T)c2_i12;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 6);
  c2_b = c2_u;
  for (c2_i13 = 0; c2_i13 < 2; c2_i13++) {
    c2_b_G[c2_i13] = (1.0 - (real_T)c2_i13) * c2_b;
  }

  for (c2_i14 = 0; c2_i14 < 4; c2_i14++) {
    c2_b_M[c2_i14] = c2_C[c2_i14];
  }

  for (c2_i15 = 0; c2_i15 < 2; c2_i15++) {
    c2_b_b[c2_i15] = c2_qd[c2_i15];
  }

  c2_eml_scalar_eg(chartInstance);
  c2_eml_scalar_eg(chartInstance);
  for (c2_i16 = 0; c2_i16 < 2; c2_i16++) {
    c2_y[c2_i16] = 0.0;
    c2_i17 = 0;
    for (c2_i18 = 0; c2_i18 < 2; c2_i18++) {
      c2_y[c2_i16] += c2_b_M[c2_i17 + c2_i16] * c2_b_b[c2_i18];
      c2_i17 += 2;
    }
  }

  for (c2_i19 = 0; c2_i19 < 4; c2_i19++) {
    c2_b_M[c2_i19] = c2_M[c2_i19];
  }

  for (c2_i20 = 0; c2_i20 < 2; c2_i20++) {
    c2_b_G[c2_i20] = (c2_b_G[c2_i20] - c2_y[c2_i20]) - c2_G[c2_i20];
  }

  c2_x = c2_b_M[1];
  c2_b_x = c2_x;
  c2_c_x = c2_b_x;
  c2_b_y = muDoubleScalarAbs(c2_c_x);
  c2_d_x = 0.0;
  c2_e_x = c2_d_x;
  c2_c_y = muDoubleScalarAbs(c2_e_x);
  c2_d = c2_b_y + c2_c_y;
  c2_f_x = c2_b_M[0];
  c2_g_x = c2_f_x;
  c2_h_x = c2_g_x;
  c2_d_y = muDoubleScalarAbs(c2_h_x);
  c2_i_x = 0.0;
  c2_j_x = c2_i_x;
  c2_e_y = muDoubleScalarAbs(c2_j_x);
  c2_b_d = c2_d_y + c2_e_y;
  if (c2_d > c2_b_d) {
    c2_r1 = 2;
    c2_r2 = 1;
  } else {
    c2_r1 = 1;
    c2_r2 = 2;
  }

  c2_k_x = c2_b_M[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r2), 1, 2, 1, 0) - 1];
  c2_f_y = c2_b_M[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r1), 1, 2, 1, 0) - 1];
  c2_a21 = c2_k_x / c2_f_y;
  c2_a = c2_a21;
  c2_c_b = c2_b_M[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r1), 1, 2, 1, 0) + 1];
  c2_g_y = c2_a * c2_c_b;
  c2_a22 = c2_b_M[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r2), 1, 2, 1, 0) + 1] - c2_g_y;
  guard1 = FALSE;
  if (c2_a22 == 0.0) {
    guard1 = TRUE;
  } else {
    if (c2_b_M[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_r1), 1, 2, 1, 0) - 1] == 0.0) {
      guard1 = TRUE;
    }
  }

  if (guard1 == TRUE) {
    c2_eml_warning(chartInstance);
  }

  c2_eml_scalar_eg(chartInstance);
  c2_b_a = c2_b_G[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r1), 1, 2, 1, 0) - 1];
  c2_d_b = c2_a21;
  c2_h_y = c2_b_a * c2_d_b;
  c2_l_x = c2_b_G[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r2), 1, 2, 1, 0) - 1] - c2_h_y;
  c2_i_y = c2_a22;
  c2_z = c2_l_x / c2_i_y;
  c2_qdd[1] = c2_z;
  c2_c_a = c2_qdd[1];
  c2_e_b = c2_b_M[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r1), 1, 2, 1, 0) + 1];
  c2_j_y = c2_c_a * c2_e_b;
  c2_m_x = c2_b_G[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r1), 1, 2, 1, 0) - 1] - c2_j_y;
  c2_k_y = c2_b_M[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r1), 1, 2, 1, 0) - 1];
  c2_b_z = c2_m_x / c2_k_y;
  c2_qdd[0] = c2_b_z;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -6);
  _SFD_SYMBOL_SCOPE_POP();
  for (c2_i21 = 0; c2_i21 < 2; c2_i21++) {
    (*c2_b_qdd)[c2_i21] = c2_qdd[c2_i21];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
}

static void initSimStructsc2_RobotSim(SFc2_RobotSimInstanceStruct *chartInstance)
{
}

static void registerMessagesc2_RobotSim(SFc2_RobotSimInstanceStruct
  *chartInstance)
{
}

static void c2_dynMatrices(SFc2_RobotSimInstanceStruct *chartInstance, real_T
  c2_q[2], real_T c2_qd[2], real_T c2_M[4], real_T c2_C[4], real_T c2_G[2])
{
  uint32_T c2_debug_family_var_map[13];
  real_T c2_B[2];
  real_T c2_I;
  real_T c2_l;
  real_T c2_m;
  real_T c2_g;
  real_T c2_B_perp[2];
  real_T c2_nargin = 2.0;
  real_T c2_nargout = 4.0;
  uint32_T c2_b_debug_family_var_map[7];
  real_T c2_b_g;
  real_T c2_b_m;
  real_T c2_b_l;
  real_T c2_b_I;
  real_T c2_r;
  real_T c2_b_nargin = 0.0;
  real_T c2_b_nargout = 4.0;
  real_T c2_x;
  real_T c2_b_x;
  real_T c2_b;
  real_T c2_y;
  real_T c2_c_x;
  real_T c2_d_x;
  real_T c2_b_b;
  real_T c2_b_y;
  real_T c2_e_x;
  real_T c2_f_x;
  real_T c2_c_b;
  real_T c2_c_y;
  real_T c2_g_x;
  real_T c2_h_x;
  real_T c2_d_b;
  real_T c2_d_y;
  real_T c2_a;
  real_T c2_e_b[4];
  int32_T c2_i22;
  real_T c2_i_x;
  real_T c2_j_x;
  real_T c2_k_x;
  real_T c2_l_x;
  real_T c2_m_x;
  real_T c2_n_x;
  real_T c2_f_b;
  real_T c2_e_y;
  real_T c2_g_b[2];
  int32_T c2_i23;
  int32_T c2_i24;
  int32_T c2_i25;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 13U, 13U, c2_c_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_B, 0U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_I, 1U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_l, 2U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_m, 3U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_g, 4U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_B_perp, 5U, c2_d_sf_marshallOut,
    c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 6U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 7U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_q, 8U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_qd, 9U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_M, 10U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_C, 11U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_G, 12U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 5);
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 7U, 7U, c2_b_debug_family_names,
    c2_b_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_g, 0U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_b_m, 1U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_b_l, 2U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_I, 3U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_r, 4U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_nargin, 5U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_nargout, 6U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  CV_SCRIPT_FCN(1, 0);
  _SFD_SCRIPT_CALL(1U, chartInstance->c2_sfEvent, 4);
  c2_b_m = 0.5;
  _SFD_SCRIPT_CALL(1U, chartInstance->c2_sfEvent, 5);
  c2_b_l = 1.0;
  _SFD_SCRIPT_CALL(1U, chartInstance->c2_sfEvent, 6);
  c2_r = 0.05;
  _SFD_SCRIPT_CALL(1U, chartInstance->c2_sfEvent, 7);
  c2_b_I = 0.041979166666666665;
  _SFD_SCRIPT_CALL(1U, chartInstance->c2_sfEvent, 8);
  c2_b_g = 9.81;
  _SFD_SCRIPT_CALL(1U, chartInstance->c2_sfEvent, -8);
  _SFD_SYMBOL_SCOPE_POP();
  c2_I = 0.041979166666666665;
  c2_l = 1.0;
  c2_m = 0.5;
  c2_g = 9.81;
  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 7);
  c2_x = c2_q[0];
  c2_b_x = c2_x;
  c2_b_x = muDoubleScalarCos(c2_b_x);
  c2_b = c2_b_x - 0.5;
  c2_y = 0.25 * c2_b;
  c2_c_x = c2_q[0];
  c2_d_x = c2_c_x;
  c2_d_x = muDoubleScalarCos(c2_d_x);
  c2_b_b = c2_d_x - 0.5;
  c2_b_y = 0.25 * c2_b_b;
  c2_e_x = c2_q[0];
  c2_f_x = c2_e_x;
  c2_f_x = muDoubleScalarCos(c2_f_x);
  c2_c_b = 1.5 - c2_f_x;
  c2_c_y = 0.5 * c2_c_b;
  c2_M[0] = 0.125 + c2_I;
  c2_M[2] = c2_y - c2_I;
  c2_M[1] = c2_b_y - c2_I;
  c2_M[3] = c2_c_y + 0.083958333333333329;
  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 12);
  c2_g_x = c2_q[0];
  c2_h_x = c2_g_x;
  c2_h_x = muDoubleScalarSin(c2_h_x);
  c2_d_b = c2_h_x;
  c2_d_y = 0.25 * c2_d_b;
  c2_a = c2_d_y;
  c2_e_b[0] = 0.0;
  c2_e_b[2] = -c2_qd[1];
  c2_e_b[1] = c2_qd[1] - c2_qd[0];
  c2_e_b[3] = c2_qd[0];
  for (c2_i22 = 0; c2_i22 < 4; c2_i22++) {
    c2_C[c2_i22] = c2_a * c2_e_b[c2_i22];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 14);
  c2_i_x = c2_q[0] - c2_q[1];
  c2_j_x = c2_i_x;
  c2_j_x = muDoubleScalarSin(c2_j_x);
  c2_k_x = c2_q[0] - c2_q[1];
  c2_l_x = c2_k_x;
  c2_l_x = muDoubleScalarSin(c2_l_x);
  c2_m_x = c2_q[1];
  c2_n_x = c2_m_x;
  c2_n_x = muDoubleScalarSin(c2_n_x);
  c2_f_b = c2_n_x;
  c2_e_y = 3.0 * c2_f_b;
  c2_g_b[0] = c2_j_x;
  c2_g_b[1] = -c2_l_x - c2_e_y;
  for (c2_i23 = 0; c2_i23 < 2; c2_i23++) {
    c2_G[c2_i23] = 2.4525 * c2_g_b[c2_i23];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 16);
  for (c2_i24 = 0; c2_i24 < 2; c2_i24++) {
    c2_B[c2_i24] = 1.0 - (real_T)c2_i24;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 18);
  for (c2_i25 = 0; c2_i25 < 2; c2_i25++) {
    c2_B_perp[c2_i25] = (real_T)c2_i25;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, -18);
  _SFD_SYMBOL_SCOPE_POP();
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber)
{
  _SFD_SCRIPT_TRANSLATION(c2_chartNumber, 0U, sf_debug_get_script_id(
    "C:/Users/yak/My Documents/GitHub/thesis/MATLAB/dynMatrices.m"));
  _SFD_SCRIPT_TRANSLATION(c2_chartNumber, 1U, sf_debug_get_script_id(
    "C:/Users/yak/My Documents/GitHub/thesis/MATLAB/dynParams.m"));
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i26;
  real_T c2_b_inData[2];
  int32_T c2_i27;
  real_T c2_u[2];
  const mxArray *c2_y = NULL;
  SFc2_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc2_RobotSimInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i26 = 0; c2_i26 < 2; c2_i26++) {
    c2_b_inData[c2_i26] = (*(real_T (*)[2])c2_inData)[c2_i26];
  }

  for (c2_i27 = 0; c2_i27 < 2; c2_i27++) {
    c2_u[c2_i27] = c2_b_inData[c2_i27];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 2), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_emlrt_marshallIn(SFc2_RobotSimInstanceStruct *chartInstance,
  const mxArray *c2_qdd, const char_T *c2_identifier, real_T c2_y[2])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_qdd), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_qdd);
}

static void c2_b_emlrt_marshallIn(SFc2_RobotSimInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[2])
{
  real_T c2_dv1[2];
  int32_T c2_i28;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv1, 1, 0, 0U, 1, 0U, 1, 2);
  for (c2_i28 = 0; c2_i28 < 2; c2_i28++) {
    c2_y[c2_i28] = c2_dv1[c2_i28];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_qdd;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[2];
  int32_T c2_i29;
  SFc2_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc2_RobotSimInstanceStruct *)chartInstanceVoid;
  c2_qdd = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_qdd), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_qdd);
  for (c2_i29 = 0; c2_i29 < 2; c2_i29++) {
    (*(real_T (*)[2])c2_outData)[c2_i29] = c2_y[c2_i29];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc2_RobotSimInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_c_emlrt_marshallIn(SFc2_RobotSimInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d0, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_nargout;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc2_RobotSimInstanceStruct *)chartInstanceVoid;
  c2_nargout = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_nargout), &c2_thisId);
  sf_mex_destroy(&c2_nargout);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i30;
  int32_T c2_i31;
  int32_T c2_i32;
  real_T c2_b_inData[4];
  int32_T c2_i33;
  int32_T c2_i34;
  int32_T c2_i35;
  real_T c2_u[4];
  const mxArray *c2_y = NULL;
  SFc2_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc2_RobotSimInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i30 = 0;
  for (c2_i31 = 0; c2_i31 < 2; c2_i31++) {
    for (c2_i32 = 0; c2_i32 < 2; c2_i32++) {
      c2_b_inData[c2_i32 + c2_i30] = (*(real_T (*)[4])c2_inData)[c2_i32 + c2_i30];
    }

    c2_i30 += 2;
  }

  c2_i33 = 0;
  for (c2_i34 = 0; c2_i34 < 2; c2_i34++) {
    for (c2_i35 = 0; c2_i35 < 2; c2_i35++) {
      c2_u[c2_i35 + c2_i33] = c2_b_inData[c2_i35 + c2_i33];
    }

    c2_i33 += 2;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 2, 2), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_d_emlrt_marshallIn(SFc2_RobotSimInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[4])
{
  real_T c2_dv2[4];
  int32_T c2_i36;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv2, 1, 0, 0U, 1, 0U, 2, 2, 2);
  for (c2_i36 = 0; c2_i36 < 4; c2_i36++) {
    c2_y[c2_i36] = c2_dv2[c2_i36];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_C;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[4];
  int32_T c2_i37;
  int32_T c2_i38;
  int32_T c2_i39;
  SFc2_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc2_RobotSimInstanceStruct *)chartInstanceVoid;
  c2_C = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_C), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_C);
  c2_i37 = 0;
  for (c2_i38 = 0; c2_i38 < 2; c2_i38++) {
    for (c2_i39 = 0; c2_i39 < 2; c2_i39++) {
      (*(real_T (*)[4])c2_outData)[c2_i39 + c2_i37] = c2_y[c2_i39 + c2_i37];
    }

    c2_i37 += 2;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i40;
  real_T c2_b_inData[2];
  int32_T c2_i41;
  real_T c2_u[2];
  const mxArray *c2_y = NULL;
  SFc2_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc2_RobotSimInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i40 = 0; c2_i40 < 2; c2_i40++) {
    c2_b_inData[c2_i40] = (*(real_T (*)[2])c2_inData)[c2_i40];
  }

  for (c2_i41 = 0; c2_i41 < 2; c2_i41++) {
    c2_u[c2_i41] = c2_b_inData[c2_i41];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 1, 2), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_e_emlrt_marshallIn(SFc2_RobotSimInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[2])
{
  real_T c2_dv3[2];
  int32_T c2_i42;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv3, 1, 0, 0U, 1, 0U, 2, 1, 2);
  for (c2_i42 = 0; c2_i42 < 2; c2_i42++) {
    c2_y[c2_i42] = c2_dv3[c2_i42];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_B_perp;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[2];
  int32_T c2_i43;
  SFc2_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc2_RobotSimInstanceStruct *)chartInstanceVoid;
  c2_B_perp = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_B_perp), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_B_perp);
  for (c2_i43 = 0; c2_i43 < 2; c2_i43++) {
    (*(real_T (*)[2])c2_outData)[c2_i43] = c2_y[c2_i43];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

const mxArray *sf_c2_RobotSim_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo;
  c2_ResolvedFunctionInfo c2_info[42];
  const mxArray *c2_m0 = NULL;
  int32_T c2_i44;
  c2_ResolvedFunctionInfo *c2_r0;
  c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  c2_info_helper(c2_info);
  sf_mex_assign(&c2_m0, sf_mex_createstruct("nameCaptureInfo", 1, 42), FALSE);
  for (c2_i44 = 0; c2_i44 < 42; c2_i44++) {
    c2_r0 = &c2_info[c2_i44];
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->context)), "context", "nameCaptureInfo",
                    c2_i44);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c2_r0->name)), "name", "nameCaptureInfo", c2_i44);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c2_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c2_i44);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->resolved)), "resolved", "nameCaptureInfo",
                    c2_i44);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c2_i44);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c2_i44);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c2_i44);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c2_i44);
  }

  sf_mex_assign(&c2_nameCaptureInfo, c2_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c2_nameCaptureInfo);
  return c2_nameCaptureInfo;
}

static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[42])
{
  c2_info[0].context = "";
  c2_info[0].name = "dynMatrices";
  c2_info[0].dominantType = "double";
  c2_info[0].resolved =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/dynMatrices.m";
  c2_info[0].fileTimeLo = 1410930659U;
  c2_info[0].fileTimeHi = 0U;
  c2_info[0].mFileTimeLo = 0U;
  c2_info[0].mFileTimeHi = 0U;
  c2_info[1].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/dynMatrices.m";
  c2_info[1].name = "dynParams";
  c2_info[1].dominantType = "";
  c2_info[1].resolved =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/dynParams.m";
  c2_info[1].fileTimeLo = 1410766076U;
  c2_info[1].fileTimeHi = 0U;
  c2_info[1].mFileTimeLo = 0U;
  c2_info[1].mFileTimeHi = 0U;
  c2_info[2].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/dynParams.m";
  c2_info[2].name = "mrdivide";
  c2_info[2].dominantType = "double";
  c2_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c2_info[2].fileTimeLo = 1357915548U;
  c2_info[2].fileTimeHi = 0U;
  c2_info[2].mFileTimeLo = 1319697566U;
  c2_info[2].mFileTimeHi = 0U;
  c2_info[3].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c2_info[3].name = "rdivide";
  c2_info[3].dominantType = "double";
  c2_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[3].fileTimeLo = 1346481588U;
  c2_info[3].fileTimeHi = 0U;
  c2_info[3].mFileTimeLo = 0U;
  c2_info[3].mFileTimeHi = 0U;
  c2_info[4].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[4].name = "eml_scalexp_compatible";
  c2_info[4].dominantType = "double";
  c2_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m";
  c2_info[4].fileTimeLo = 1286786396U;
  c2_info[4].fileTimeHi = 0U;
  c2_info[4].mFileTimeLo = 0U;
  c2_info[4].mFileTimeHi = 0U;
  c2_info[5].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[5].name = "eml_div";
  c2_info[5].dominantType = "double";
  c2_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[5].fileTimeLo = 1313319010U;
  c2_info[5].fileTimeHi = 0U;
  c2_info[5].mFileTimeLo = 0U;
  c2_info[5].mFileTimeHi = 0U;
  c2_info[6].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/dynParams.m";
  c2_info[6].name = "mtimes";
  c2_info[6].dominantType = "double";
  c2_info[6].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[6].fileTimeLo = 1289483692U;
  c2_info[6].fileTimeHi = 0U;
  c2_info[6].mFileTimeLo = 0U;
  c2_info[6].mFileTimeHi = 0U;
  c2_info[7].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/dynParams.m";
  c2_info[7].name = "mpower";
  c2_info[7].dominantType = "double";
  c2_info[7].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c2_info[7].fileTimeLo = 1286786442U;
  c2_info[7].fileTimeHi = 0U;
  c2_info[7].mFileTimeLo = 0U;
  c2_info[7].mFileTimeHi = 0U;
  c2_info[8].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c2_info[8].name = "power";
  c2_info[8].dominantType = "double";
  c2_info[8].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[8].fileTimeLo = 1348163130U;
  c2_info[8].fileTimeHi = 0U;
  c2_info[8].mFileTimeLo = 0U;
  c2_info[8].mFileTimeHi = 0U;
  c2_info[9].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c2_info[9].name = "eml_scalar_eg";
  c2_info[9].dominantType = "double";
  c2_info[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[9].fileTimeLo = 1286786396U;
  c2_info[9].fileTimeHi = 0U;
  c2_info[9].mFileTimeLo = 0U;
  c2_info[9].mFileTimeHi = 0U;
  c2_info[10].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c2_info[10].name = "eml_scalexp_alloc";
  c2_info[10].dominantType = "double";
  c2_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[10].fileTimeLo = 1352388860U;
  c2_info[10].fileTimeHi = 0U;
  c2_info[10].mFileTimeLo = 0U;
  c2_info[10].mFileTimeHi = 0U;
  c2_info[11].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c2_info[11].name = "floor";
  c2_info[11].dominantType = "double";
  c2_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c2_info[11].fileTimeLo = 1343801580U;
  c2_info[11].fileTimeHi = 0U;
  c2_info[11].mFileTimeLo = 0U;
  c2_info[11].mFileTimeHi = 0U;
  c2_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c2_info[12].name = "eml_scalar_floor";
  c2_info[12].dominantType = "double";
  c2_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c2_info[12].fileTimeLo = 1286786326U;
  c2_info[12].fileTimeHi = 0U;
  c2_info[12].mFileTimeLo = 0U;
  c2_info[12].mFileTimeHi = 0U;
  c2_info[13].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power";
  c2_info[13].name = "eml_scalar_eg";
  c2_info[13].dominantType = "double";
  c2_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[13].fileTimeLo = 1286786396U;
  c2_info[13].fileTimeHi = 0U;
  c2_info[13].mFileTimeLo = 0U;
  c2_info[13].mFileTimeHi = 0U;
  c2_info[14].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power";
  c2_info[14].name = "mtimes";
  c2_info[14].dominantType = "double";
  c2_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[14].fileTimeLo = 1289483692U;
  c2_info[14].fileTimeHi = 0U;
  c2_info[14].mFileTimeLo = 0U;
  c2_info[14].mFileTimeHi = 0U;
  c2_info[15].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/dynMatrices.m";
  c2_info[15].name = "mtimes";
  c2_info[15].dominantType = "double";
  c2_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[15].fileTimeLo = 1289483692U;
  c2_info[15].fileTimeHi = 0U;
  c2_info[15].mFileTimeLo = 0U;
  c2_info[15].mFileTimeHi = 0U;
  c2_info[16].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/dynMatrices.m";
  c2_info[16].name = "mpower";
  c2_info[16].dominantType = "double";
  c2_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c2_info[16].fileTimeLo = 1286786442U;
  c2_info[16].fileTimeHi = 0U;
  c2_info[16].mFileTimeLo = 0U;
  c2_info[16].mFileTimeHi = 0U;
  c2_info[17].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/dynMatrices.m";
  c2_info[17].name = "cos";
  c2_info[17].dominantType = "double";
  c2_info[17].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c2_info[17].fileTimeLo = 1343801572U;
  c2_info[17].fileTimeHi = 0U;
  c2_info[17].mFileTimeLo = 0U;
  c2_info[17].mFileTimeHi = 0U;
  c2_info[18].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c2_info[18].name = "eml_scalar_cos";
  c2_info[18].dominantType = "double";
  c2_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m";
  c2_info[18].fileTimeLo = 1286786322U;
  c2_info[18].fileTimeHi = 0U;
  c2_info[18].mFileTimeLo = 0U;
  c2_info[18].mFileTimeHi = 0U;
  c2_info[19].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/dynMatrices.m";
  c2_info[19].name = "mrdivide";
  c2_info[19].dominantType = "double";
  c2_info[19].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c2_info[19].fileTimeLo = 1357915548U;
  c2_info[19].fileTimeHi = 0U;
  c2_info[19].mFileTimeLo = 1319697566U;
  c2_info[19].mFileTimeHi = 0U;
  c2_info[20].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/dynMatrices.m";
  c2_info[20].name = "sin";
  c2_info[20].dominantType = "double";
  c2_info[20].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c2_info[20].fileTimeLo = 1343801586U;
  c2_info[20].fileTimeHi = 0U;
  c2_info[20].mFileTimeLo = 0U;
  c2_info[20].mFileTimeHi = 0U;
  c2_info[21].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c2_info[21].name = "eml_scalar_sin";
  c2_info[21].dominantType = "double";
  c2_info[21].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m";
  c2_info[21].fileTimeLo = 1286786336U;
  c2_info[21].fileTimeHi = 0U;
  c2_info[21].mFileTimeLo = 0U;
  c2_info[21].mFileTimeHi = 0U;
  c2_info[22].context = "";
  c2_info[22].name = "mtimes";
  c2_info[22].dominantType = "double";
  c2_info[22].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[22].fileTimeLo = 1289483692U;
  c2_info[22].fileTimeHi = 0U;
  c2_info[22].mFileTimeLo = 0U;
  c2_info[22].mFileTimeHi = 0U;
  c2_info[23].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[23].name = "eml_index_class";
  c2_info[23].dominantType = "";
  c2_info[23].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[23].fileTimeLo = 1323134578U;
  c2_info[23].fileTimeHi = 0U;
  c2_info[23].mFileTimeLo = 0U;
  c2_info[23].mFileTimeHi = 0U;
  c2_info[24].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[24].name = "eml_scalar_eg";
  c2_info[24].dominantType = "double";
  c2_info[24].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[24].fileTimeLo = 1286786396U;
  c2_info[24].fileTimeHi = 0U;
  c2_info[24].mFileTimeLo = 0U;
  c2_info[24].mFileTimeHi = 0U;
  c2_info[25].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[25].name = "eml_xgemm";
  c2_info[25].dominantType = "char";
  c2_info[25].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c2_info[25].fileTimeLo = 1299040772U;
  c2_info[25].fileTimeHi = 0U;
  c2_info[25].mFileTimeLo = 0U;
  c2_info[25].mFileTimeHi = 0U;
  c2_info[26].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c2_info[26].name = "eml_blas_inline";
  c2_info[26].dominantType = "";
  c2_info[26].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[26].fileTimeLo = 1299040768U;
  c2_info[26].fileTimeHi = 0U;
  c2_info[26].mFileTimeLo = 0U;
  c2_info[26].mFileTimeHi = 0U;
  c2_info[27].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c2_info[27].name = "mtimes";
  c2_info[27].dominantType = "double";
  c2_info[27].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[27].fileTimeLo = 1289483692U;
  c2_info[27].fileTimeHi = 0U;
  c2_info[27].mFileTimeLo = 0U;
  c2_info[27].mFileTimeHi = 0U;
  c2_info[28].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c2_info[28].name = "eml_index_class";
  c2_info[28].dominantType = "";
  c2_info[28].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[28].fileTimeLo = 1323134578U;
  c2_info[28].fileTimeHi = 0U;
  c2_info[28].mFileTimeLo = 0U;
  c2_info[28].mFileTimeHi = 0U;
  c2_info[29].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c2_info[29].name = "eml_scalar_eg";
  c2_info[29].dominantType = "double";
  c2_info[29].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[29].fileTimeLo = 1286786396U;
  c2_info[29].fileTimeHi = 0U;
  c2_info[29].mFileTimeLo = 0U;
  c2_info[29].mFileTimeHi = 0U;
  c2_info[30].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c2_info[30].name = "eml_refblas_xgemm";
  c2_info[30].dominantType = "char";
  c2_info[30].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c2_info[30].fileTimeLo = 1299040774U;
  c2_info[30].fileTimeHi = 0U;
  c2_info[30].mFileTimeLo = 0U;
  c2_info[30].mFileTimeHi = 0U;
  c2_info[31].context = "";
  c2_info[31].name = "mldivide";
  c2_info[31].dominantType = "double";
  c2_info[31].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c2_info[31].fileTimeLo = 1357915548U;
  c2_info[31].fileTimeHi = 0U;
  c2_info[31].mFileTimeLo = 1319697566U;
  c2_info[31].mFileTimeHi = 0U;
  c2_info[32].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c2_info[32].name = "eml_lusolve";
  c2_info[32].dominantType = "double";
  c2_info[32].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m";
  c2_info[32].fileTimeLo = 1309422396U;
  c2_info[32].fileTimeHi = 0U;
  c2_info[32].mFileTimeLo = 0U;
  c2_info[32].mFileTimeHi = 0U;
  c2_info[33].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m";
  c2_info[33].name = "eml_index_class";
  c2_info[33].dominantType = "";
  c2_info[33].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[33].fileTimeLo = 1323134578U;
  c2_info[33].fileTimeHi = 0U;
  c2_info[33].mFileTimeLo = 0U;
  c2_info[33].mFileTimeHi = 0U;
  c2_info[34].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolve2x2";
  c2_info[34].name = "eml_index_class";
  c2_info[34].dominantType = "";
  c2_info[34].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[34].fileTimeLo = 1323134578U;
  c2_info[34].fileTimeHi = 0U;
  c2_info[34].mFileTimeLo = 0U;
  c2_info[34].mFileTimeHi = 0U;
  c2_info[35].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolve2x2";
  c2_info[35].name = "eml_xcabs1";
  c2_info[35].dominantType = "double";
  c2_info[35].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c2_info[35].fileTimeLo = 1286786306U;
  c2_info[35].fileTimeHi = 0U;
  c2_info[35].mFileTimeLo = 0U;
  c2_info[35].mFileTimeHi = 0U;
  c2_info[36].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c2_info[36].name = "abs";
  c2_info[36].dominantType = "double";
  c2_info[36].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[36].fileTimeLo = 1343801566U;
  c2_info[36].fileTimeHi = 0U;
  c2_info[36].mFileTimeLo = 0U;
  c2_info[36].mFileTimeHi = 0U;
  c2_info[37].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[37].name = "eml_scalar_abs";
  c2_info[37].dominantType = "double";
  c2_info[37].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c2_info[37].fileTimeLo = 1286786312U;
  c2_info[37].fileTimeHi = 0U;
  c2_info[37].mFileTimeLo = 0U;
  c2_info[37].mFileTimeHi = 0U;
  c2_info[38].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolve2x2";
  c2_info[38].name = "eml_div";
  c2_info[38].dominantType = "double";
  c2_info[38].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[38].fileTimeLo = 1313319010U;
  c2_info[38].fileTimeHi = 0U;
  c2_info[38].mFileTimeLo = 0U;
  c2_info[38].mFileTimeHi = 0U;
  c2_info[39].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolve2x2";
  c2_info[39].name = "mtimes";
  c2_info[39].dominantType = "double";
  c2_info[39].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[39].fileTimeLo = 1289483692U;
  c2_info[39].fileTimeHi = 0U;
  c2_info[39].mFileTimeLo = 0U;
  c2_info[39].mFileTimeHi = 0U;
  c2_info[40].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!warn_singular";
  c2_info[40].name = "eml_warning";
  c2_info[40].dominantType = "char";
  c2_info[40].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c2_info[40].fileTimeLo = 1286786402U;
  c2_info[40].fileTimeHi = 0U;
  c2_info[40].mFileTimeLo = 0U;
  c2_info[40].mFileTimeHi = 0U;
  c2_info[41].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolve2x2";
  c2_info[41].name = "eml_scalar_eg";
  c2_info[41].dominantType = "double";
  c2_info[41].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[41].fileTimeLo = 1286786396U;
  c2_info[41].fileTimeHi = 0U;
  c2_info[41].mFileTimeLo = 0U;
  c2_info[41].mFileTimeHi = 0U;
}

static void c2_eml_scalar_eg(SFc2_RobotSimInstanceStruct *chartInstance)
{
}

static void c2_eml_warning(SFc2_RobotSimInstanceStruct *chartInstance)
{
  int32_T c2_i45;
  static char_T c2_varargin_1[27] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a', 'r', 'M', 'a',
    't', 'r', 'i', 'x' };

  char_T c2_u[27];
  const mxArray *c2_y = NULL;
  for (c2_i45 = 0; c2_i45 < 27; c2_i45++) {
    c2_u[c2_i45] = c2_varargin_1[c2_i45];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 27), FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U,
    14, c2_y));
}

static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc2_RobotSimInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(int32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static int32_T c2_f_emlrt_marshallIn(SFc2_RobotSimInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_y;
  int32_T c2_i46;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i46, 1, 6, 0U, 0, 0U, 0);
  c2_y = c2_i46;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y;
  SFc2_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc2_RobotSimInstanceStruct *)chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static uint8_T c2_g_emlrt_marshallIn(SFc2_RobotSimInstanceStruct *chartInstance,
  const mxArray *c2_b_is_active_c2_RobotSim, const char_T *c2_identifier)
{
  uint8_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_RobotSim), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_RobotSim);
  return c2_y;
}

static uint8_T c2_h_emlrt_marshallIn(SFc2_RobotSimInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_y;
  uint8_T c2_u0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u0, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void init_dsm_address_info(SFc2_RobotSimInstanceStruct *chartInstance)
{
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c2_RobotSim_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1811903703U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1474998898U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3231185479U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1024512989U);
}

mxArray *sf_c2_RobotSim_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("wZRtCcaNavzdaP3qAm6SKD");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,3,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(2);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(2);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(2);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c2_RobotSim_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c2_RobotSim(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"qdd\",},{M[8],M[0],T\"is_active_c2_RobotSim\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_RobotSim_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_RobotSimInstanceStruct *chartInstance;
    chartInstance = (SFc2_RobotSimInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _RobotSimMachineNumber_,
           2,
           1,
           1,
           4,
           0,
           0,
           0,
           0,
           2,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           ssGetPath(S),
           (void *)S);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          init_script_number_translation(_RobotSimMachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_RobotSimMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _RobotSimMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"u");
          _SFD_SET_DATA_PROPS(1,2,0,1,"qdd");
          _SFD_SET_DATA_PROPS(2,1,1,0,"q");
          _SFD_SET_DATA_PROPS(3,1,1,0,"qd");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,110);
        _SFD_CV_INIT_SCRIPT(0,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(0,0,"dynMatrices",0,-1,521);
        _SFD_CV_INIT_SCRIPT(1,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(1,0,"dynParams",0,-1,172);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)
            c2_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          real_T *c2_u;
          real_T (*c2_qdd)[2];
          real_T (*c2_q)[2];
          real_T (*c2_qd)[2];
          c2_qd = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 2);
          c2_q = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 1);
          c2_qdd = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
          c2_u = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c2_u);
          _SFD_SET_DATA_VALUE_PTR(1U, *c2_qdd);
          _SFD_SET_DATA_VALUE_PTR(2U, *c2_q);
          _SFD_SET_DATA_VALUE_PTR(3U, *c2_qd);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _RobotSimMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "8QGgNycR9lvTJQC85TqplB";
}

static void sf_opaque_initialize_c2_RobotSim(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_RobotSimInstanceStruct*) chartInstanceVar)
    ->S,0);
  initialize_params_c2_RobotSim((SFc2_RobotSimInstanceStruct*) chartInstanceVar);
  initialize_c2_RobotSim((SFc2_RobotSimInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c2_RobotSim(void *chartInstanceVar)
{
  enable_c2_RobotSim((SFc2_RobotSimInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c2_RobotSim(void *chartInstanceVar)
{
  disable_c2_RobotSim((SFc2_RobotSimInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c2_RobotSim(void *chartInstanceVar)
{
  sf_c2_RobotSim((SFc2_RobotSimInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c2_RobotSim(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c2_RobotSim((SFc2_RobotSimInstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_RobotSim();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_raw2high'.\n");
  }

  return plhs[0];
}

extern void sf_internal_set_sim_state_c2_RobotSim(SimStruct* S, const mxArray
  *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_RobotSim();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c2_RobotSim((SFc2_RobotSimInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c2_RobotSim(SimStruct* S)
{
  return sf_internal_get_sim_state_c2_RobotSim(S);
}

static void sf_opaque_set_sim_state_c2_RobotSim(SimStruct* S, const mxArray *st)
{
  sf_internal_set_sim_state_c2_RobotSim(S, st);
}

static void sf_opaque_terminate_c2_RobotSim(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_RobotSimInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_RobotSim_optimization_info();
    }

    finalize_c2_RobotSim((SFc2_RobotSimInstanceStruct*) chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_RobotSim((SFc2_RobotSimInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_RobotSim(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_RobotSim((SFc2_RobotSimInstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c2_RobotSim(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_RobotSim_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,2,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,2,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,2);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,3);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 3; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1976121302U));
  ssSetChecksum1(S,(3076216644U));
  ssSetChecksum2(S,(3396334525U));
  ssSetChecksum3(S,(3965234180U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c2_RobotSim(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_RobotSim(SimStruct *S)
{
  SFc2_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc2_RobotSimInstanceStruct *)utMalloc(sizeof
    (SFc2_RobotSimInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc2_RobotSimInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c2_RobotSim;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c2_RobotSim;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c2_RobotSim;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c2_RobotSim;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c2_RobotSim;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c2_RobotSim;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c2_RobotSim;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c2_RobotSim;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_RobotSim;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_RobotSim;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c2_RobotSim;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->S = S;
  ssSetUserData(S,(void *)(&(chartInstance->chartInfo)));/* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c2_RobotSim_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_RobotSim(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_RobotSim(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_RobotSim(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_RobotSim_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
