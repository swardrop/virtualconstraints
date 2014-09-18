/* Include files */

#include <stddef.h>
#include "blas.h"
#include "RobotSim_sfun.h"
#include "c5_RobotSim.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "RobotSim_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c5_debug_family_names[4] = { "nargin", "nargout", "q",
  "theta" };

static const char * c5_b_debug_family_names[5] = { "c", "H0", "H", "nargin",
  "nargout" };

static const char * c5_c_debug_family_names[6] = { "c", "jj", "nargin",
  "nargout", "q", "theta" };

/* Function Declarations */
static void initialize_c5_RobotSim(SFc5_RobotSimInstanceStruct *chartInstance);
static void initialize_params_c5_RobotSim(SFc5_RobotSimInstanceStruct
  *chartInstance);
static void enable_c5_RobotSim(SFc5_RobotSimInstanceStruct *chartInstance);
static void disable_c5_RobotSim(SFc5_RobotSimInstanceStruct *chartInstance);
static void c5_update_debugger_state_c5_RobotSim(SFc5_RobotSimInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c5_RobotSim(SFc5_RobotSimInstanceStruct
  *chartInstance);
static void set_sim_state_c5_RobotSim(SFc5_RobotSimInstanceStruct *chartInstance,
  const mxArray *c5_st);
static void finalize_c5_RobotSim(SFc5_RobotSimInstanceStruct *chartInstance);
static void sf_c5_RobotSim(SFc5_RobotSimInstanceStruct *chartInstance);
static void initSimStructsc5_RobotSim(SFc5_RobotSimInstanceStruct *chartInstance);
static void registerMessagesc5_RobotSim(SFc5_RobotSimInstanceStruct
  *chartInstance);
static real_T c5_phasevar(SFc5_RobotSimInstanceStruct *chartInstance, real_T
  c5_q[2]);
static void init_script_number_translation(uint32_T c5_machineNumber, uint32_T
  c5_chartNumber);
static const mxArray *c5_sf_marshallOut(void *chartInstanceVoid, void *c5_inData);
static real_T c5_emlrt_marshallIn(SFc5_RobotSimInstanceStruct *chartInstance,
  const mxArray *c5_theta, const char_T *c5_identifier);
static real_T c5_b_emlrt_marshallIn(SFc5_RobotSimInstanceStruct *chartInstance,
  const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void c5_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static const mxArray *c5_b_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static const mxArray *c5_c_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static void c5_c_emlrt_marshallIn(SFc5_RobotSimInstanceStruct *chartInstance,
  const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId, real_T c5_y[4]);
static void c5_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static const mxArray *c5_d_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static void c5_d_emlrt_marshallIn(SFc5_RobotSimInstanceStruct *chartInstance,
  const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId, real_T c5_y[2]);
static void c5_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static void c5_info_helper(c5_ResolvedFunctionInfo c5_info[23]);
static void c5_eml_scalar_eg(SFc5_RobotSimInstanceStruct *chartInstance);
static const mxArray *c5_e_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static int32_T c5_e_emlrt_marshallIn(SFc5_RobotSimInstanceStruct *chartInstance,
  const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void c5_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static uint8_T c5_f_emlrt_marshallIn(SFc5_RobotSimInstanceStruct *chartInstance,
  const mxArray *c5_b_is_active_c5_RobotSim, const char_T *c5_identifier);
static uint8_T c5_g_emlrt_marshallIn(SFc5_RobotSimInstanceStruct *chartInstance,
  const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void init_dsm_address_info(SFc5_RobotSimInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c5_RobotSim(SFc5_RobotSimInstanceStruct *chartInstance)
{
  chartInstance->c5_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c5_is_active_c5_RobotSim = 0U;
}

static void initialize_params_c5_RobotSim(SFc5_RobotSimInstanceStruct
  *chartInstance)
{
}

static void enable_c5_RobotSim(SFc5_RobotSimInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c5_RobotSim(SFc5_RobotSimInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c5_update_debugger_state_c5_RobotSim(SFc5_RobotSimInstanceStruct
  *chartInstance)
{
}

static const mxArray *get_sim_state_c5_RobotSim(SFc5_RobotSimInstanceStruct
  *chartInstance)
{
  const mxArray *c5_st;
  const mxArray *c5_y = NULL;
  real_T c5_hoistedGlobal;
  real_T c5_u;
  const mxArray *c5_b_y = NULL;
  uint8_T c5_b_hoistedGlobal;
  uint8_T c5_b_u;
  const mxArray *c5_c_y = NULL;
  real_T *c5_theta;
  c5_theta = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c5_st = NULL;
  c5_st = NULL;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_createcellarray(2), FALSE);
  c5_hoistedGlobal = *c5_theta;
  c5_u = c5_hoistedGlobal;
  c5_b_y = NULL;
  sf_mex_assign(&c5_b_y, sf_mex_create("y", &c5_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c5_y, 0, c5_b_y);
  c5_b_hoistedGlobal = chartInstance->c5_is_active_c5_RobotSim;
  c5_b_u = c5_b_hoistedGlobal;
  c5_c_y = NULL;
  sf_mex_assign(&c5_c_y, sf_mex_create("y", &c5_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c5_y, 1, c5_c_y);
  sf_mex_assign(&c5_st, c5_y, FALSE);
  return c5_st;
}

static void set_sim_state_c5_RobotSim(SFc5_RobotSimInstanceStruct *chartInstance,
  const mxArray *c5_st)
{
  const mxArray *c5_u;
  real_T *c5_theta;
  c5_theta = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c5_doneDoubleBufferReInit = TRUE;
  c5_u = sf_mex_dup(c5_st);
  *c5_theta = c5_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c5_u,
    0)), "theta");
  chartInstance->c5_is_active_c5_RobotSim = c5_f_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c5_u, 1)), "is_active_c5_RobotSim");
  sf_mex_destroy(&c5_u);
  c5_update_debugger_state_c5_RobotSim(chartInstance);
  sf_mex_destroy(&c5_st);
}

static void finalize_c5_RobotSim(SFc5_RobotSimInstanceStruct *chartInstance)
{
}

static void sf_c5_RobotSim(SFc5_RobotSimInstanceStruct *chartInstance)
{
  int32_T c5_i0;
  int32_T c5_i1;
  real_T c5_q[2];
  uint32_T c5_debug_family_var_map[4];
  real_T c5_nargin = 1.0;
  real_T c5_nargout = 1.0;
  real_T c5_theta;
  int32_T c5_i2;
  real_T c5_b_q[2];
  real_T *c5_b_theta;
  real_T (*c5_c_q)[2];
  c5_c_q = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 0);
  c5_b_theta = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 2U, chartInstance->c5_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c5_b_theta, 0U);
  for (c5_i0 = 0; c5_i0 < 2; c5_i0++) {
    _SFD_DATA_RANGE_CHECK((*c5_c_q)[c5_i0], 1U);
  }

  chartInstance->c5_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 2U, chartInstance->c5_sfEvent);
  for (c5_i1 = 0; c5_i1 < 2; c5_i1++) {
    c5_q[c5_i1] = (*c5_c_q)[c5_i1];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c5_debug_family_names,
    c5_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargin, 0U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargout, 1U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c5_q, 2U, c5_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_theta, 3U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 4);
  for (c5_i2 = 0; c5_i2 < 2; c5_i2++) {
    c5_b_q[c5_i2] = c5_q[c5_i2];
  }

  c5_theta = c5_phasevar(chartInstance, c5_b_q);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, -4);
  _SFD_SYMBOL_SCOPE_POP();
  *c5_b_theta = c5_theta;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 2U, chartInstance->c5_sfEvent);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_RobotSimMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc5_RobotSim(SFc5_RobotSimInstanceStruct *chartInstance)
{
}

static void registerMessagesc5_RobotSim(SFc5_RobotSimInstanceStruct
  *chartInstance)
{
}

static real_T c5_phasevar(SFc5_RobotSimInstanceStruct *chartInstance, real_T
  c5_q[2])
{
  real_T c5_theta;
  uint32_T c5_debug_family_var_map[6];
  real_T c5_c[2];
  real_T c5_jj;
  real_T c5_nargin = 1.0;
  real_T c5_nargout = 1.0;
  uint32_T c5_b_debug_family_var_map[5];
  real_T c5_b_c[2];
  real_T c5_H0[2];
  real_T c5_H[4];
  real_T c5_b_nargin = 0.0;
  real_T c5_b_nargout = 3.0;
  int32_T c5_i3;
  int32_T c5_i4;
  int32_T c5_i5;
  static real_T c5_unusedU0[4] = { 1.0, 0.0, 0.0, 1.0 };

  int32_T c5_i6;
  int32_T c5_i7;
  real_T c5_b[2];
  int32_T c5_k;
  int32_T c5_b_k;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 6U, 6U, c5_c_debug_family_names,
    c5_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(c5_c, 0U, c5_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c5_jj, 1U, c5_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargin, 2U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargout, 3U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_q, 4U, c5_b_sf_marshallOut,
    c5_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_theta, 5U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c5_sfEvent, 2);
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 5U, 5U, c5_b_debug_family_names,
    c5_b_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(c5_b_c, 0U, c5_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c5_H0, 1U, c5_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_H, 2U, c5_c_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_b_nargin, 3U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_b_nargout, 4U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  CV_SCRIPT_FCN(1, 0);
  _SFD_SCRIPT_CALL(1U, chartInstance->c5_sfEvent, 4);
  for (c5_i3 = 0; c5_i3 < 2; c5_i3++) {
    c5_H0[c5_i3] = 1.0 - (real_T)c5_i3;
  }

  _SFD_SCRIPT_CALL(1U, chartInstance->c5_sfEvent, 5);
  for (c5_i4 = 0; c5_i4 < 2; c5_i4++) {
    c5_b_c[c5_i4] = (real_T)c5_i4;
  }

  _SFD_SCRIPT_CALL(1U, chartInstance->c5_sfEvent, 6);
  for (c5_i5 = 0; c5_i5 < 4; c5_i5++) {
    c5_H[c5_i5] = c5_unusedU0[c5_i5];
  }

  _SFD_SCRIPT_CALL(1U, chartInstance->c5_sfEvent, -6);
  _SFD_SYMBOL_SCOPE_POP();
  for (c5_i6 = 0; c5_i6 < 2; c5_i6++) {
    c5_c[c5_i6] = (real_T)c5_i6;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c5_sfEvent, 2);
  _SFD_SCRIPT_CALL(0U, chartInstance->c5_sfEvent, 2);
  _SFD_SCRIPT_CALL(0U, chartInstance->c5_sfEvent, 3);
  c5_theta = 0.0;
  _SFD_SCRIPT_CALL(0U, chartInstance->c5_sfEvent, 4);
  c5_jj = 1.0;
  CV_SCRIPT_FOR(0, 0, 1);
  _SFD_SCRIPT_CALL(0U, chartInstance->c5_sfEvent, 5);
  for (c5_i7 = 0; c5_i7 < 2; c5_i7++) {
    c5_b[c5_i7] = c5_q[c5_i7];
  }

  c5_eml_scalar_eg(chartInstance);
  c5_eml_scalar_eg(chartInstance);
  c5_theta = 0.0;
  for (c5_k = 1; c5_k < 3; c5_k++) {
    c5_b_k = c5_k;
    c5_theta += (real_T)(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c5_b_k), 1, 2, 1, 0) - 1) *
      c5_b[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c5_b_k), 1, 2, 1, 0) - 1];
  }

  CV_SCRIPT_FOR(0, 0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c5_sfEvent, -5);
  _SFD_SYMBOL_SCOPE_POP();
  return c5_theta;
}

static void init_script_number_translation(uint32_T c5_machineNumber, uint32_T
  c5_chartNumber)
{
  _SFD_SCRIPT_TRANSLATION(c5_chartNumber, 0U, sf_debug_get_script_id(
    "C:/Users/yak/My Documents/GitHub/thesis/MATLAB/phasevar.m"));
  _SFD_SCRIPT_TRANSLATION(c5_chartNumber, 1U, sf_debug_get_script_id(
    "C:/Users/yak/My Documents/GitHub/thesis/MATLAB/constrMatrices.m"));
}

static const mxArray *c5_sf_marshallOut(void *chartInstanceVoid, void *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  real_T c5_u;
  const mxArray *c5_y = NULL;
  SFc5_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc5_RobotSimInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_u = *(real_T *)c5_inData;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, FALSE);
  return c5_mxArrayOutData;
}

static real_T c5_emlrt_marshallIn(SFc5_RobotSimInstanceStruct *chartInstance,
  const mxArray *c5_theta, const char_T *c5_identifier)
{
  real_T c5_y;
  emlrtMsgIdentifier c5_thisId;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_theta), &c5_thisId);
  sf_mex_destroy(&c5_theta);
  return c5_y;
}

static real_T c5_b_emlrt_marshallIn(SFc5_RobotSimInstanceStruct *chartInstance,
  const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  real_T c5_y;
  real_T c5_d0;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_d0, 1, 0, 0U, 0, 0U, 0);
  c5_y = c5_d0;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void c5_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_theta;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real_T c5_y;
  SFc5_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc5_RobotSimInstanceStruct *)chartInstanceVoid;
  c5_theta = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_theta), &c5_thisId);
  sf_mex_destroy(&c5_theta);
  *(real_T *)c5_outData = c5_y;
  sf_mex_destroy(&c5_mxArrayInData);
}

static const mxArray *c5_b_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i8;
  real_T c5_b_inData[2];
  int32_T c5_i9;
  real_T c5_u[2];
  const mxArray *c5_y = NULL;
  SFc5_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc5_RobotSimInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  for (c5_i8 = 0; c5_i8 < 2; c5_i8++) {
    c5_b_inData[c5_i8] = (*(real_T (*)[2])c5_inData)[c5_i8];
  }

  for (c5_i9 = 0; c5_i9 < 2; c5_i9++) {
    c5_u[c5_i9] = c5_b_inData[c5_i9];
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 1, 2), FALSE);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, FALSE);
  return c5_mxArrayOutData;
}

static const mxArray *c5_c_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i10;
  int32_T c5_i11;
  int32_T c5_i12;
  real_T c5_b_inData[4];
  int32_T c5_i13;
  int32_T c5_i14;
  int32_T c5_i15;
  real_T c5_u[4];
  const mxArray *c5_y = NULL;
  SFc5_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc5_RobotSimInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_i10 = 0;
  for (c5_i11 = 0; c5_i11 < 2; c5_i11++) {
    for (c5_i12 = 0; c5_i12 < 2; c5_i12++) {
      c5_b_inData[c5_i12 + c5_i10] = (*(real_T (*)[4])c5_inData)[c5_i12 + c5_i10];
    }

    c5_i10 += 2;
  }

  c5_i13 = 0;
  for (c5_i14 = 0; c5_i14 < 2; c5_i14++) {
    for (c5_i15 = 0; c5_i15 < 2; c5_i15++) {
      c5_u[c5_i15 + c5_i13] = c5_b_inData[c5_i15 + c5_i13];
    }

    c5_i13 += 2;
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 2, 2, 2), FALSE);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, FALSE);
  return c5_mxArrayOutData;
}

static void c5_c_emlrt_marshallIn(SFc5_RobotSimInstanceStruct *chartInstance,
  const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId, real_T c5_y[4])
{
  real_T c5_dv0[4];
  int32_T c5_i16;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), c5_dv0, 1, 0, 0U, 1, 0U, 2, 2, 2);
  for (c5_i16 = 0; c5_i16 < 4; c5_i16++) {
    c5_y[c5_i16] = c5_dv0[c5_i16];
  }

  sf_mex_destroy(&c5_u);
}

static void c5_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_H;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real_T c5_y[4];
  int32_T c5_i17;
  int32_T c5_i18;
  int32_T c5_i19;
  SFc5_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc5_RobotSimInstanceStruct *)chartInstanceVoid;
  c5_H = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_H), &c5_thisId, c5_y);
  sf_mex_destroy(&c5_H);
  c5_i17 = 0;
  for (c5_i18 = 0; c5_i18 < 2; c5_i18++) {
    for (c5_i19 = 0; c5_i19 < 2; c5_i19++) {
      (*(real_T (*)[4])c5_outData)[c5_i19 + c5_i17] = c5_y[c5_i19 + c5_i17];
    }

    c5_i17 += 2;
  }

  sf_mex_destroy(&c5_mxArrayInData);
}

static const mxArray *c5_d_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i20;
  real_T c5_b_inData[2];
  int32_T c5_i21;
  real_T c5_u[2];
  const mxArray *c5_y = NULL;
  SFc5_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc5_RobotSimInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  for (c5_i20 = 0; c5_i20 < 2; c5_i20++) {
    c5_b_inData[c5_i20] = (*(real_T (*)[2])c5_inData)[c5_i20];
  }

  for (c5_i21 = 0; c5_i21 < 2; c5_i21++) {
    c5_u[c5_i21] = c5_b_inData[c5_i21];
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 2, 1, 2), FALSE);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, FALSE);
  return c5_mxArrayOutData;
}

static void c5_d_emlrt_marshallIn(SFc5_RobotSimInstanceStruct *chartInstance,
  const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId, real_T c5_y[2])
{
  real_T c5_dv1[2];
  int32_T c5_i22;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), c5_dv1, 1, 0, 0U, 1, 0U, 1, 2);
  for (c5_i22 = 0; c5_i22 < 2; c5_i22++) {
    c5_y[c5_i22] = c5_dv1[c5_i22];
  }

  sf_mex_destroy(&c5_u);
}

static void c5_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_q;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real_T c5_y[2];
  int32_T c5_i23;
  SFc5_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc5_RobotSimInstanceStruct *)chartInstanceVoid;
  c5_q = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_q), &c5_thisId, c5_y);
  sf_mex_destroy(&c5_q);
  for (c5_i23 = 0; c5_i23 < 2; c5_i23++) {
    (*(real_T (*)[2])c5_outData)[c5_i23] = c5_y[c5_i23];
  }

  sf_mex_destroy(&c5_mxArrayInData);
}

const mxArray *sf_c5_RobotSim_get_eml_resolved_functions_info(void)
{
  const mxArray *c5_nameCaptureInfo;
  c5_ResolvedFunctionInfo c5_info[23];
  const mxArray *c5_m0 = NULL;
  int32_T c5_i24;
  c5_ResolvedFunctionInfo *c5_r0;
  c5_nameCaptureInfo = NULL;
  c5_nameCaptureInfo = NULL;
  c5_info_helper(c5_info);
  sf_mex_assign(&c5_m0, sf_mex_createstruct("nameCaptureInfo", 1, 23), FALSE);
  for (c5_i24 = 0; c5_i24 < 23; c5_i24++) {
    c5_r0 = &c5_info[c5_i24];
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", c5_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c5_r0->context)), "context", "nameCaptureInfo",
                    c5_i24);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", c5_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c5_r0->name)), "name", "nameCaptureInfo", c5_i24);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", c5_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c5_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c5_i24);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", c5_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c5_r0->resolved)), "resolved", "nameCaptureInfo",
                    c5_i24);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", &c5_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c5_i24);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", &c5_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c5_i24);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", &c5_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c5_i24);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", &c5_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c5_i24);
  }

  sf_mex_assign(&c5_nameCaptureInfo, c5_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c5_nameCaptureInfo);
  return c5_nameCaptureInfo;
}

static void c5_info_helper(c5_ResolvedFunctionInfo c5_info[23])
{
  c5_info[0].context = "";
  c5_info[0].name = "phasevar";
  c5_info[0].dominantType = "double";
  c5_info[0].resolved =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/phasevar.m";
  c5_info[0].fileTimeLo = 1411048616U;
  c5_info[0].fileTimeHi = 0U;
  c5_info[0].mFileTimeLo = 0U;
  c5_info[0].mFileTimeHi = 0U;
  c5_info[1].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/phasevar.m";
  c5_info[1].name = "constrMatrices";
  c5_info[1].dominantType = "";
  c5_info[1].resolved =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/constrMatrices.m";
  c5_info[1].fileTimeLo = 1411047847U;
  c5_info[1].fileTimeHi = 0U;
  c5_info[1].mFileTimeLo = 0U;
  c5_info[1].mFileTimeHi = 0U;
  c5_info[2].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/phasevar.m";
  c5_info[2].name = "length";
  c5_info[2].dominantType = "double";
  c5_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c5_info[2].fileTimeLo = 1303117406U;
  c5_info[2].fileTimeHi = 0U;
  c5_info[2].mFileTimeLo = 0U;
  c5_info[2].mFileTimeHi = 0U;
  c5_info[3].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/phasevar.m";
  c5_info[3].name = "mtimes";
  c5_info[3].dominantType = "double";
  c5_info[3].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c5_info[3].fileTimeLo = 1289483692U;
  c5_info[3].fileTimeHi = 0U;
  c5_info[3].mFileTimeLo = 0U;
  c5_info[3].mFileTimeHi = 0U;
  c5_info[4].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c5_info[4].name = "eml_index_class";
  c5_info[4].dominantType = "";
  c5_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c5_info[4].fileTimeLo = 1323134578U;
  c5_info[4].fileTimeHi = 0U;
  c5_info[4].mFileTimeLo = 0U;
  c5_info[4].mFileTimeHi = 0U;
  c5_info[5].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c5_info[5].name = "eml_scalar_eg";
  c5_info[5].dominantType = "double";
  c5_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c5_info[5].fileTimeLo = 1286786396U;
  c5_info[5].fileTimeHi = 0U;
  c5_info[5].mFileTimeLo = 0U;
  c5_info[5].mFileTimeHi = 0U;
  c5_info[6].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c5_info[6].name = "eml_xdotu";
  c5_info[6].dominantType = "double";
  c5_info[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotu.m";
  c5_info[6].fileTimeLo = 1299040772U;
  c5_info[6].fileTimeHi = 0U;
  c5_info[6].mFileTimeLo = 0U;
  c5_info[6].mFileTimeHi = 0U;
  c5_info[7].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotu.m";
  c5_info[7].name = "eml_blas_inline";
  c5_info[7].dominantType = "";
  c5_info[7].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c5_info[7].fileTimeLo = 1299040768U;
  c5_info[7].fileTimeHi = 0U;
  c5_info[7].mFileTimeLo = 0U;
  c5_info[7].mFileTimeHi = 0U;
  c5_info[8].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotu.m";
  c5_info[8].name = "eml_xdot";
  c5_info[8].dominantType = "double";
  c5_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdot.m";
  c5_info[8].fileTimeLo = 1299040772U;
  c5_info[8].fileTimeHi = 0U;
  c5_info[8].mFileTimeLo = 0U;
  c5_info[8].mFileTimeHi = 0U;
  c5_info[9].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdot.m";
  c5_info[9].name = "eml_blas_inline";
  c5_info[9].dominantType = "";
  c5_info[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c5_info[9].fileTimeLo = 1299040768U;
  c5_info[9].fileTimeHi = 0U;
  c5_info[9].mFileTimeLo = 0U;
  c5_info[9].mFileTimeHi = 0U;
  c5_info[10].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xdot.m";
  c5_info[10].name = "eml_index_class";
  c5_info[10].dominantType = "";
  c5_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c5_info[10].fileTimeLo = 1323134578U;
  c5_info[10].fileTimeHi = 0U;
  c5_info[10].mFileTimeLo = 0U;
  c5_info[10].mFileTimeHi = 0U;
  c5_info[11].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xdot.m";
  c5_info[11].name = "eml_refblas_xdot";
  c5_info[11].dominantType = "double";
  c5_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdot.m";
  c5_info[11].fileTimeLo = 1299040772U;
  c5_info[11].fileTimeHi = 0U;
  c5_info[11].mFileTimeLo = 0U;
  c5_info[11].mFileTimeHi = 0U;
  c5_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdot.m";
  c5_info[12].name = "eml_refblas_xdotx";
  c5_info[12].dominantType = "char";
  c5_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c5_info[12].fileTimeLo = 1299040774U;
  c5_info[12].fileTimeHi = 0U;
  c5_info[12].mFileTimeLo = 0U;
  c5_info[12].mFileTimeHi = 0U;
  c5_info[13].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c5_info[13].name = "eml_scalar_eg";
  c5_info[13].dominantType = "double";
  c5_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c5_info[13].fileTimeLo = 1286786396U;
  c5_info[13].fileTimeHi = 0U;
  c5_info[13].mFileTimeLo = 0U;
  c5_info[13].mFileTimeHi = 0U;
  c5_info[14].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c5_info[14].name = "eml_index_class";
  c5_info[14].dominantType = "";
  c5_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c5_info[14].fileTimeLo = 1323134578U;
  c5_info[14].fileTimeHi = 0U;
  c5_info[14].mFileTimeLo = 0U;
  c5_info[14].mFileTimeHi = 0U;
  c5_info[15].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c5_info[15].name = "eml_index_minus";
  c5_info[15].dominantType = "double";
  c5_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c5_info[15].fileTimeLo = 1286786378U;
  c5_info[15].fileTimeHi = 0U;
  c5_info[15].mFileTimeLo = 0U;
  c5_info[15].mFileTimeHi = 0U;
  c5_info[16].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c5_info[16].name = "eml_index_class";
  c5_info[16].dominantType = "";
  c5_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c5_info[16].fileTimeLo = 1323134578U;
  c5_info[16].fileTimeHi = 0U;
  c5_info[16].mFileTimeLo = 0U;
  c5_info[16].mFileTimeHi = 0U;
  c5_info[17].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c5_info[17].name = "eml_index_times";
  c5_info[17].dominantType = "coder.internal.indexInt";
  c5_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c5_info[17].fileTimeLo = 1286786380U;
  c5_info[17].fileTimeHi = 0U;
  c5_info[17].mFileTimeLo = 0U;
  c5_info[17].mFileTimeHi = 0U;
  c5_info[18].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c5_info[18].name = "eml_index_class";
  c5_info[18].dominantType = "";
  c5_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c5_info[18].fileTimeLo = 1323134578U;
  c5_info[18].fileTimeHi = 0U;
  c5_info[18].mFileTimeLo = 0U;
  c5_info[18].mFileTimeHi = 0U;
  c5_info[19].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c5_info[19].name = "eml_index_plus";
  c5_info[19].dominantType = "coder.internal.indexInt";
  c5_info[19].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c5_info[19].fileTimeLo = 1286786378U;
  c5_info[19].fileTimeHi = 0U;
  c5_info[19].mFileTimeLo = 0U;
  c5_info[19].mFileTimeHi = 0U;
  c5_info[20].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c5_info[20].name = "eml_index_class";
  c5_info[20].dominantType = "";
  c5_info[20].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c5_info[20].fileTimeLo = 1323134578U;
  c5_info[20].fileTimeHi = 0U;
  c5_info[20].mFileTimeLo = 0U;
  c5_info[20].mFileTimeHi = 0U;
  c5_info[21].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c5_info[21].name = "eml_int_forloop_overflow_check";
  c5_info[21].dominantType = "";
  c5_info[21].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c5_info[21].fileTimeLo = 1346481540U;
  c5_info[21].fileTimeHi = 0U;
  c5_info[21].mFileTimeLo = 0U;
  c5_info[21].mFileTimeHi = 0U;
  c5_info[22].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c5_info[22].name = "intmax";
  c5_info[22].dominantType = "char";
  c5_info[22].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c5_info[22].fileTimeLo = 1311226516U;
  c5_info[22].fileTimeHi = 0U;
  c5_info[22].mFileTimeLo = 0U;
  c5_info[22].mFileTimeHi = 0U;
}

static void c5_eml_scalar_eg(SFc5_RobotSimInstanceStruct *chartInstance)
{
}

static const mxArray *c5_e_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_u;
  const mxArray *c5_y = NULL;
  SFc5_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc5_RobotSimInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_u = *(int32_T *)c5_inData;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, FALSE);
  return c5_mxArrayOutData;
}

static int32_T c5_e_emlrt_marshallIn(SFc5_RobotSimInstanceStruct *chartInstance,
  const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  int32_T c5_y;
  int32_T c5_i25;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_i25, 1, 6, 0U, 0, 0U, 0);
  c5_y = c5_i25;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void c5_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_b_sfEvent;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  int32_T c5_y;
  SFc5_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc5_RobotSimInstanceStruct *)chartInstanceVoid;
  c5_b_sfEvent = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_b_sfEvent),
    &c5_thisId);
  sf_mex_destroy(&c5_b_sfEvent);
  *(int32_T *)c5_outData = c5_y;
  sf_mex_destroy(&c5_mxArrayInData);
}

static uint8_T c5_f_emlrt_marshallIn(SFc5_RobotSimInstanceStruct *chartInstance,
  const mxArray *c5_b_is_active_c5_RobotSim, const char_T *c5_identifier)
{
  uint8_T c5_y;
  emlrtMsgIdentifier c5_thisId;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_g_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c5_b_is_active_c5_RobotSim), &c5_thisId);
  sf_mex_destroy(&c5_b_is_active_c5_RobotSim);
  return c5_y;
}

static uint8_T c5_g_emlrt_marshallIn(SFc5_RobotSimInstanceStruct *chartInstance,
  const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  uint8_T c5_y;
  uint8_T c5_u0;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_u0, 1, 3, 0U, 0, 0U, 0);
  c5_y = c5_u0;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void init_dsm_address_info(SFc5_RobotSimInstanceStruct *chartInstance)
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

void sf_c5_RobotSim_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(591367586U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3129723476U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3924571142U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1553308470U);
}

mxArray *sf_c5_RobotSim_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("2wPULTJ8RFWc4RyvNDjVx");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c5_RobotSim_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c5_RobotSim(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[24],T\"theta\",},{M[8],M[0],T\"is_active_c5_RobotSim\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c5_RobotSim_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc5_RobotSimInstanceStruct *chartInstance;
    chartInstance = (SFc5_RobotSimInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _RobotSimMachineNumber_,
           5,
           1,
           1,
           2,
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
          _SFD_SET_DATA_PROPS(0,2,0,1,"theta");
          _SFD_SET_DATA_PROPS(1,1,1,0,"q");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,69);
        _SFD_CV_INIT_SCRIPT(0,1,0,0,0,0,1,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(0,0,"phasevar",0,-1,143);
        _SFD_CV_INIT_SCRIPT_FOR(0,0,81,109,139);
        _SFD_CV_INIT_SCRIPT(1,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(1,0,"constrMatrices",0,-1,92);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)c5_sf_marshallIn);

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c5_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          real_T *c5_theta;
          real_T (*c5_q)[2];
          c5_q = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 0);
          c5_theta = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          _SFD_SET_DATA_VALUE_PTR(0U, c5_theta);
          _SFD_SET_DATA_VALUE_PTR(1U, *c5_q);
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
  return "fdbuUYnHIdbVJzafpRfLDG";
}

static void sf_opaque_initialize_c5_RobotSim(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc5_RobotSimInstanceStruct*) chartInstanceVar)
    ->S,0);
  initialize_params_c5_RobotSim((SFc5_RobotSimInstanceStruct*) chartInstanceVar);
  initialize_c5_RobotSim((SFc5_RobotSimInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c5_RobotSim(void *chartInstanceVar)
{
  enable_c5_RobotSim((SFc5_RobotSimInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c5_RobotSim(void *chartInstanceVar)
{
  disable_c5_RobotSim((SFc5_RobotSimInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c5_RobotSim(void *chartInstanceVar)
{
  sf_c5_RobotSim((SFc5_RobotSimInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c5_RobotSim(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c5_RobotSim((SFc5_RobotSimInstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c5_RobotSim();/* state var info */
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

extern void sf_internal_set_sim_state_c5_RobotSim(SimStruct* S, const mxArray
  *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c5_RobotSim();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c5_RobotSim((SFc5_RobotSimInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c5_RobotSim(SimStruct* S)
{
  return sf_internal_get_sim_state_c5_RobotSim(S);
}

static void sf_opaque_set_sim_state_c5_RobotSim(SimStruct* S, const mxArray *st)
{
  sf_internal_set_sim_state_c5_RobotSim(S, st);
}

static void sf_opaque_terminate_c5_RobotSim(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc5_RobotSimInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_RobotSim_optimization_info();
    }

    finalize_c5_RobotSim((SFc5_RobotSimInstanceStruct*) chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc5_RobotSim((SFc5_RobotSimInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c5_RobotSim(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c5_RobotSim((SFc5_RobotSimInstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c5_RobotSim(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_RobotSim_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      5);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,5,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,5,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,5);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,5,1);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,5,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 1; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,5);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(4001973405U));
  ssSetChecksum1(S,(1980369135U));
  ssSetChecksum2(S,(134433160U));
  ssSetChecksum3(S,(2049607881U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c5_RobotSim(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c5_RobotSim(SimStruct *S)
{
  SFc5_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc5_RobotSimInstanceStruct *)utMalloc(sizeof
    (SFc5_RobotSimInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc5_RobotSimInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c5_RobotSim;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c5_RobotSim;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c5_RobotSim;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c5_RobotSim;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c5_RobotSim;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c5_RobotSim;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c5_RobotSim;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c5_RobotSim;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c5_RobotSim;
  chartInstance->chartInfo.mdlStart = mdlStart_c5_RobotSim;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c5_RobotSim;
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

void c5_RobotSim_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c5_RobotSim(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c5_RobotSim(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c5_RobotSim(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c5_RobotSim_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
