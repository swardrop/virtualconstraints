/* Include files */

#include <stddef.h>
#include "blas.h"
#include "RobotSim_sfun.h"
#include "c4_RobotSim.h"
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
static const char * c4_debug_family_names[11] = { "height", "p2", "ind",
  "nargin", "nargout", "org", "q", "ground", "y_prev", "impact", "y" };

static const char * c4_b_debug_family_names[7] = { "l", "I", "r", "m", "g",
  "nargin", "nargout" };

static const char * c4_c_debug_family_names[6] = { "l", "nargin", "nargout", "q",
  "org", "p2" };

static const char * c4_d_debug_family_names[4] = { "nargin", "nargout", "q", "s"
};

/* Function Declarations */
static void initialize_c4_RobotSim(SFc4_RobotSimInstanceStruct *chartInstance);
static void initialize_params_c4_RobotSim(SFc4_RobotSimInstanceStruct
  *chartInstance);
static void enable_c4_RobotSim(SFc4_RobotSimInstanceStruct *chartInstance);
static void disable_c4_RobotSim(SFc4_RobotSimInstanceStruct *chartInstance);
static void c4_update_debugger_state_c4_RobotSim(SFc4_RobotSimInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c4_RobotSim(SFc4_RobotSimInstanceStruct
  *chartInstance);
static void set_sim_state_c4_RobotSim(SFc4_RobotSimInstanceStruct *chartInstance,
  const mxArray *c4_st);
static void finalize_c4_RobotSim(SFc4_RobotSimInstanceStruct *chartInstance);
static void sf_c4_RobotSim(SFc4_RobotSimInstanceStruct *chartInstance);
static void c4_chartstep_c4_RobotSim(SFc4_RobotSimInstanceStruct *chartInstance);
static void initSimStructsc4_RobotSim(SFc4_RobotSimInstanceStruct *chartInstance);
static void registerMessagesc4_RobotSim(SFc4_RobotSimInstanceStruct
  *chartInstance);
static void init_script_number_translation(uint32_T c4_machineNumber, uint32_T
  c4_chartNumber);
static const mxArray *c4_sf_marshallOut(void *chartInstanceVoid, void *c4_inData);
static real_T c4_emlrt_marshallIn(SFc4_RobotSimInstanceStruct *chartInstance,
  const mxArray *c4_y, const char_T *c4_identifier);
static real_T c4_b_emlrt_marshallIn(SFc4_RobotSimInstanceStruct *chartInstance,
  const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void c4_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static const mxArray *c4_b_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static boolean_T c4_c_emlrt_marshallIn(SFc4_RobotSimInstanceStruct
  *chartInstance, const mxArray *c4_impact, const char_T *c4_identifier);
static boolean_T c4_d_emlrt_marshallIn(SFc4_RobotSimInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void c4_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static const mxArray *c4_c_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static const mxArray *c4_d_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static const mxArray *c4_e_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static const mxArray *c4_f_sf_marshallOut(void *chartInstanceVoid, real_T
  c4_inData_data[1], int32_T c4_inData_sizes[1]);
static void c4_e_emlrt_marshallIn(SFc4_RobotSimInstanceStruct *chartInstance,
  const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId, real_T c4_y_data[1],
  int32_T c4_y_sizes[1]);
static void c4_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, real_T c4_outData_data[1],
  int32_T c4_outData_sizes[1]);
static void c4_f_emlrt_marshallIn(SFc4_RobotSimInstanceStruct *chartInstance,
  const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId, real_T c4_y[2]);
static void c4_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static void c4_g_emlrt_marshallIn(SFc4_RobotSimInstanceStruct *chartInstance,
  const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId, real_T c4_y[2]);
static void c4_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static void c4_info_helper(c4_ResolvedFunctionInfo c4_info[35]);
static const mxArray *c4_g_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static int32_T c4_h_emlrt_marshallIn(SFc4_RobotSimInstanceStruct *chartInstance,
  const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void c4_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static uint8_T c4_i_emlrt_marshallIn(SFc4_RobotSimInstanceStruct *chartInstance,
  const mxArray *c4_b_is_active_c4_RobotSim, const char_T *c4_identifier);
static uint8_T c4_j_emlrt_marshallIn(SFc4_RobotSimInstanceStruct *chartInstance,
  const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void init_dsm_address_info(SFc4_RobotSimInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c4_RobotSim(SFc4_RobotSimInstanceStruct *chartInstance)
{
  chartInstance->c4_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c4_is_active_c4_RobotSim = 0U;
}

static void initialize_params_c4_RobotSim(SFc4_RobotSimInstanceStruct
  *chartInstance)
{
}

static void enable_c4_RobotSim(SFc4_RobotSimInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c4_RobotSim(SFc4_RobotSimInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c4_update_debugger_state_c4_RobotSim(SFc4_RobotSimInstanceStruct
  *chartInstance)
{
}

static const mxArray *get_sim_state_c4_RobotSim(SFc4_RobotSimInstanceStruct
  *chartInstance)
{
  const mxArray *c4_st;
  const mxArray *c4_y = NULL;
  boolean_T c4_hoistedGlobal;
  boolean_T c4_u;
  const mxArray *c4_b_y = NULL;
  real_T c4_b_hoistedGlobal;
  real_T c4_b_u;
  const mxArray *c4_c_y = NULL;
  uint8_T c4_c_hoistedGlobal;
  uint8_T c4_c_u;
  const mxArray *c4_d_y = NULL;
  boolean_T *c4_impact;
  real_T *c4_e_y;
  c4_e_y = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c4_impact = (boolean_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c4_st = NULL;
  c4_st = NULL;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_createcellarray(3), FALSE);
  c4_hoistedGlobal = *c4_impact;
  c4_u = c4_hoistedGlobal;
  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", &c4_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c4_y, 0, c4_b_y);
  c4_b_hoistedGlobal = *c4_e_y;
  c4_b_u = c4_b_hoistedGlobal;
  c4_c_y = NULL;
  sf_mex_assign(&c4_c_y, sf_mex_create("y", &c4_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c4_y, 1, c4_c_y);
  c4_c_hoistedGlobal = chartInstance->c4_is_active_c4_RobotSim;
  c4_c_u = c4_c_hoistedGlobal;
  c4_d_y = NULL;
  sf_mex_assign(&c4_d_y, sf_mex_create("y", &c4_c_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c4_y, 2, c4_d_y);
  sf_mex_assign(&c4_st, c4_y, FALSE);
  return c4_st;
}

static void set_sim_state_c4_RobotSim(SFc4_RobotSimInstanceStruct *chartInstance,
  const mxArray *c4_st)
{
  const mxArray *c4_u;
  boolean_T *c4_impact;
  real_T *c4_y;
  c4_y = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c4_impact = (boolean_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c4_doneDoubleBufferReInit = TRUE;
  c4_u = sf_mex_dup(c4_st);
  *c4_impact = c4_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c4_u, 0)), "impact");
  *c4_y = c4_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c4_u, 1)),
    "y");
  chartInstance->c4_is_active_c4_RobotSim = c4_i_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c4_u, 2)), "is_active_c4_RobotSim");
  sf_mex_destroy(&c4_u);
  c4_update_debugger_state_c4_RobotSim(chartInstance);
  sf_mex_destroy(&c4_st);
}

static void finalize_c4_RobotSim(SFc4_RobotSimInstanceStruct *chartInstance)
{
}

static void sf_c4_RobotSim(SFc4_RobotSimInstanceStruct *chartInstance)
{
  int32_T c4_i0;
  int32_T c4_i1;
  int32_T c4_i2;
  boolean_T *c4_impact;
  real_T *c4_y_prev;
  real_T *c4_y;
  real_T (*c4_ground)[10];
  real_T (*c4_q)[2];
  real_T (*c4_org)[2];
  c4_y = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c4_y_prev = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c4_impact = (boolean_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c4_ground = (real_T (*)[10])ssGetInputPortSignal(chartInstance->S, 2);
  c4_q = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 1);
  c4_org = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 1U, chartInstance->c4_sfEvent);
  for (c4_i0 = 0; c4_i0 < 2; c4_i0++) {
    _SFD_DATA_RANGE_CHECK((*c4_org)[c4_i0], 0U);
  }

  for (c4_i1 = 0; c4_i1 < 2; c4_i1++) {
    _SFD_DATA_RANGE_CHECK((*c4_q)[c4_i1], 1U);
  }

  for (c4_i2 = 0; c4_i2 < 10; c4_i2++) {
    _SFD_DATA_RANGE_CHECK((*c4_ground)[c4_i2], 2U);
  }

  _SFD_DATA_RANGE_CHECK((real_T)*c4_impact, 3U);
  _SFD_DATA_RANGE_CHECK(*c4_y_prev, 4U);
  _SFD_DATA_RANGE_CHECK(*c4_y, 5U);
  chartInstance->c4_sfEvent = CALL_EVENT;
  c4_chartstep_c4_RobotSim(chartInstance);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_RobotSimMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c4_chartstep_c4_RobotSim(SFc4_RobotSimInstanceStruct *chartInstance)
{
  real_T c4_hoistedGlobal;
  int32_T c4_i3;
  real_T c4_org[2];
  int32_T c4_i4;
  real_T c4_q[2];
  int32_T c4_i5;
  real_T c4_ground[10];
  real_T c4_y_prev;
  uint32_T c4_debug_family_var_map[11];
  real_T c4_height;
  real_T c4_p2[2];
  int32_T c4_ind_sizes;
  real_T c4_ind_data[1];
  int32_T c4_height_sizes;
  real_T c4_height_data[1];
  real_T c4_nargin = 4.0;
  real_T c4_nargout = 2.0;
  boolean_T c4_impact;
  real_T c4_y;
  int32_T c4_i6;
  real_T c4_b_q[2];
  int32_T c4_i7;
  real_T c4_b_org[2];
  uint32_T c4_b_debug_family_var_map[6];
  real_T c4_l;
  real_T c4_b_nargin = 2.0;
  real_T c4_b_nargout = 1.0;
  uint32_T c4_c_debug_family_var_map[7];
  real_T c4_b_l;
  real_T c4_I;
  real_T c4_r;
  real_T c4_m;
  real_T c4_g;
  real_T c4_c_nargin = 0.0;
  real_T c4_c_nargout = 2.0;
  int32_T c4_i8;
  real_T c4_x;
  real_T c4_b_x;
  real_T c4_c_x;
  real_T c4_d_x;
  real_T c4_b;
  real_T c4_b_y;
  real_T c4_e_x;
  real_T c4_f_x;
  real_T c4_g_x;
  real_T c4_h_x;
  real_T c4_b_b;
  real_T c4_c_y;
  real_T c4_b_p2;
  int32_T c4_i9;
  boolean_T c4_i_x[5];
  int32_T c4_idx;
  static int32_T c4_iv0[1] = { 1 };

  int32_T c4_ii_sizes;
  int32_T c4_ii;
  int32_T c4_b_ii;
  int32_T c4_ii_data[1];
  int32_T c4_loop_ub;
  int32_T c4_i10;
  int32_T c4_b_loop_ub;
  int32_T c4_i11;
  int32_T c4_i12;
  real_T c4_c_q[2];
  uint32_T c4_d_debug_family_var_map[4];
  real_T c4_d_nargin = 1.0;
  real_T c4_d_nargout = 1.0;
  boolean_T c4_s;
  boolean_T c4_b0;
  real_T *c4_b_y_prev;
  boolean_T *c4_b_impact;
  real_T *c4_d_y;
  real_T (*c4_b_ground)[10];
  real_T (*c4_d_q)[2];
  real_T (*c4_c_org)[2];
  boolean_T guard1 = FALSE;
  boolean_T guard2 = FALSE;
  boolean_T guard3 = FALSE;
  boolean_T exitg1;
  c4_d_y = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c4_b_y_prev = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c4_b_impact = (boolean_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c4_b_ground = (real_T (*)[10])ssGetInputPortSignal(chartInstance->S, 2);
  c4_d_q = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 1);
  c4_c_org = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 1U, chartInstance->c4_sfEvent);
  c4_hoistedGlobal = *c4_b_y_prev;
  for (c4_i3 = 0; c4_i3 < 2; c4_i3++) {
    c4_org[c4_i3] = (*c4_c_org)[c4_i3];
  }

  for (c4_i4 = 0; c4_i4 < 2; c4_i4++) {
    c4_q[c4_i4] = (*c4_d_q)[c4_i4];
  }

  for (c4_i5 = 0; c4_i5 < 10; c4_i5++) {
    c4_ground[c4_i5] = (*c4_b_ground)[c4_i5];
  }

  c4_y_prev = c4_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 11U, 12U, c4_debug_family_names,
    c4_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_height, MAX_uint32_T,
    c4_sf_marshallOut, c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_p2, 1U, c4_d_sf_marshallOut,
    c4_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c4_ind_data, (const int32_T *)
    &c4_ind_sizes, NULL, 0, 2, (void *)c4_f_sf_marshallOut, (void *)
    c4_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c4_height_data, (const int32_T *)
    &c4_height_sizes, NULL, 0, -1, (void *)c4_f_sf_marshallOut, (void *)
    c4_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_nargin, 3U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_nargout, 4U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c4_org, 5U, c4_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c4_q, 6U, c4_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c4_ground, 7U, c4_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c4_y_prev, 8U, c4_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_impact, 9U, c4_b_sf_marshallOut,
    c4_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_y, 10U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 5);
  c4_impact = FALSE;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 6);
  c4_y = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 7);
  c4_height = 0.0;
  _SFD_SYMBOL_SWITCH(0U, 0U);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 13);
  for (c4_i6 = 0; c4_i6 < 2; c4_i6++) {
    c4_b_q[c4_i6] = c4_q[c4_i6];
  }

  for (c4_i7 = 0; c4_i7 < 2; c4_i7++) {
    c4_b_org[c4_i7] = c4_org[c4_i7];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 6U, 6U, c4_c_debug_family_names,
    c4_b_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c4_l, 0U, c4_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_b_nargin, 1U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_b_nargout, 2U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_b_q, 3U, c4_d_sf_marshallOut,
    c4_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_b_org, 4U, c4_e_sf_marshallOut,
    c4_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_p2, 5U, c4_d_sf_marshallOut,
    c4_d_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c4_sfEvent, 3);
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 7U, 7U, c4_b_debug_family_names,
    c4_c_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c4_b_l, 0U, c4_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_I, 1U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c4_r, 2U, c4_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c4_m, 3U, c4_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_g, 4U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_c_nargin, 5U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_c_nargout, 6U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  CV_SCRIPT_FCN(1, 0);
  _SFD_SCRIPT_CALL(1U, chartInstance->c4_sfEvent, 4);
  c4_m = 0.5;
  _SFD_SCRIPT_CALL(1U, chartInstance->c4_sfEvent, 5);
  c4_b_l = 1.0;
  _SFD_SCRIPT_CALL(1U, chartInstance->c4_sfEvent, 6);
  c4_r = 0.05;
  _SFD_SCRIPT_CALL(1U, chartInstance->c4_sfEvent, 7);
  c4_I = 0.041979166666666665;
  _SFD_SCRIPT_CALL(1U, chartInstance->c4_sfEvent, 8);
  c4_g = 9.81;
  _SFD_SCRIPT_CALL(1U, chartInstance->c4_sfEvent, -8);
  _SFD_SYMBOL_SCOPE_POP();
  c4_l = 1.0;
  _SFD_SCRIPT_CALL(0U, chartInstance->c4_sfEvent, 3);
  _SFD_SCRIPT_CALL(0U, chartInstance->c4_sfEvent, 5);
  for (c4_i8 = 0; c4_i8 < 2; c4_i8++) {
    c4_p2[c4_i8] = 0.0;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c4_sfEvent, 6);
  c4_x = c4_b_q[1];
  c4_b_x = c4_x;
  c4_b_x = muDoubleScalarSin(c4_b_x);
  c4_c_x = c4_b_q[0] - c4_b_q[1];
  c4_d_x = c4_c_x;
  c4_d_x = muDoubleScalarSin(c4_d_x);
  c4_b = c4_b_x + c4_d_x;
  c4_b_y = c4_b;
  c4_p2[0] = c4_b_org[0] + c4_b_y;
  _SFD_SCRIPT_CALL(0U, chartInstance->c4_sfEvent, 7);
  c4_e_x = c4_b_q[1];
  c4_f_x = c4_e_x;
  c4_f_x = muDoubleScalarCos(c4_f_x);
  c4_g_x = c4_b_q[0] - c4_b_q[1];
  c4_h_x = c4_g_x;
  c4_h_x = muDoubleScalarCos(c4_h_x);
  c4_b_b = c4_f_x - c4_h_x;
  c4_c_y = c4_b_b;
  c4_p2[1] = c4_b_org[1] + c4_c_y;
  _SFD_SCRIPT_CALL(0U, chartInstance->c4_sfEvent, -7);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 16);
  c4_b_p2 = c4_p2[0];
  for (c4_i9 = 0; c4_i9 < 5; c4_i9++) {
    c4_i_x[c4_i9] = (c4_b_p2 > c4_ground[c4_i9]);
  }

  c4_idx = 0;
  c4_ii_sizes = c4_iv0[0];
  c4_ii = 5;
  exitg1 = FALSE;
  while ((exitg1 == FALSE) && (c4_ii > 0)) {
    c4_b_ii = c4_ii;
    if (c4_i_x[c4_b_ii - 1]) {
      c4_idx = 1;
      _SFD_EML_ARRAY_BOUNDS_CHECK("", 1, 1, c4_ii_sizes, 1, 0);
      c4_ii_data[0] = c4_b_ii;
      exitg1 = TRUE;
    } else {
      c4_ii--;
    }
  }

  if (c4_idx == 0) {
    c4_ii_sizes = 0;
  }

  c4_ind_sizes = c4_ii_sizes;
  c4_loop_ub = c4_ii_sizes - 1;
  for (c4_i10 = 0; c4_i10 <= c4_loop_ub; c4_i10++) {
    c4_ind_data[c4_i10] = (real_T)c4_ii_data[c4_i10];
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 17);
  c4_height_sizes = c4_ind_sizes;
  c4_b_loop_ub = c4_ind_sizes - 1;
  for (c4_i11 = 0; c4_i11 <= c4_b_loop_ub; c4_i11++) {
    c4_height_data[c4_i11] = c4_ground[(int32_T)c4_ind_data[c4_i11] + 4];
  }

  _SFD_SYMBOL_SWITCH(0U, 3U);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 18);
  (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("height", 1, 1, c4_height_sizes, 1, 0);
  c4_height = c4_height_data[0];
  _SFD_SYMBOL_SWITCH(0U, 0U);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 19);
  c4_y = c4_p2[1];
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 22);
  guard1 = FALSE;
  guard2 = FALSE;
  if (CV_EML_COND(0, 1, 0, c4_y <= c4_height)) {
    if (CV_EML_COND(0, 1, 1, c4_y_prev > c4_height)) {
      for (c4_i12 = 0; c4_i12 < 2; c4_i12++) {
        c4_c_q[c4_i12] = c4_q[c4_i12];
      }

      _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c4_d_debug_family_names,
        c4_d_debug_family_var_map);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_d_nargin, 0U, c4_sf_marshallOut,
        c4_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_d_nargout, 1U, c4_sf_marshallOut,
        c4_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_c_q, 2U, c4_d_sf_marshallOut,
        c4_d_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_s, 3U, c4_b_sf_marshallOut,
        c4_b_sf_marshallIn);
      CV_SCRIPT_FCN(2, 0);
      _SFD_SCRIPT_CALL(2U, chartInstance->c4_sfEvent, 5);
      guard3 = FALSE;
      if (c4_c_q[0] <= 0.0) {
        guard3 = TRUE;
      } else if (c4_c_q[1] <= 0.0) {
        guard3 = TRUE;
      } else {
        c4_b0 = FALSE;
      }

      if (guard3 == TRUE) {
        c4_b0 = TRUE;
      }

      c4_s = c4_b0;
      _SFD_SCRIPT_CALL(2U, chartInstance->c4_sfEvent, -5);
      _SFD_SYMBOL_SCOPE_POP();
      if (!CV_EML_COND(0, 1, 2, c4_s)) {
        CV_EML_MCDC(0, 1, 0, TRUE);
        CV_EML_IF(0, 1, 0, TRUE);
        _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 23);
        c4_impact = TRUE;
      } else {
        guard1 = TRUE;
      }
    } else {
      guard2 = TRUE;
    }
  } else {
    guard2 = TRUE;
  }

  if (guard2 == TRUE) {
    guard1 = TRUE;
  }

  if (guard1 == TRUE) {
    CV_EML_MCDC(0, 1, 0, FALSE);
    CV_EML_IF(0, 1, 0, FALSE);
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, -23);
  _SFD_SYMBOL_SCOPE_POP();
  *c4_b_impact = c4_impact;
  *c4_d_y = c4_y;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, chartInstance->c4_sfEvent);
}

static void initSimStructsc4_RobotSim(SFc4_RobotSimInstanceStruct *chartInstance)
{
}

static void registerMessagesc4_RobotSim(SFc4_RobotSimInstanceStruct
  *chartInstance)
{
}

static void init_script_number_translation(uint32_T c4_machineNumber, uint32_T
  c4_chartNumber)
{
  _SFD_SCRIPT_TRANSLATION(c4_chartNumber, 0U, sf_debug_get_script_id(
    "C:/Users/yak/My Documents/GitHub/thesis/MATLAB/endSwingFoot.m"));
  _SFD_SCRIPT_TRANSLATION(c4_chartNumber, 1U, sf_debug_get_script_id(
    "C:/Users/yak/My Documents/GitHub/thesis/MATLAB/dynParams.m"));
  _SFD_SCRIPT_TRANSLATION(c4_chartNumber, 2U, sf_debug_get_script_id(
    "C:/Users/yak/My Documents/GitHub/thesis/MATLAB/scuff.m"));
}

static const mxArray *c4_sf_marshallOut(void *chartInstanceVoid, void *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  real_T c4_u;
  const mxArray *c4_y = NULL;
  SFc4_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc4_RobotSimInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_u = *(real_T *)c4_inData;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", &c4_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, FALSE);
  return c4_mxArrayOutData;
}

static real_T c4_emlrt_marshallIn(SFc4_RobotSimInstanceStruct *chartInstance,
  const mxArray *c4_y, const char_T *c4_identifier)
{
  real_T c4_b_y;
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_b_y = c4_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_y), &c4_thisId);
  sf_mex_destroy(&c4_y);
  return c4_b_y;
}

static real_T c4_b_emlrt_marshallIn(SFc4_RobotSimInstanceStruct *chartInstance,
  const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  real_T c4_y;
  real_T c4_d0;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_d0, 1, 0, 0U, 0, 0U, 0);
  c4_y = c4_d0;
  sf_mex_destroy(&c4_u);
  return c4_y;
}

static void c4_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_y;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_b_y;
  SFc4_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc4_RobotSimInstanceStruct *)chartInstanceVoid;
  c4_y = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_b_y = c4_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_y), &c4_thisId);
  sf_mex_destroy(&c4_y);
  *(real_T *)c4_outData = c4_b_y;
  sf_mex_destroy(&c4_mxArrayInData);
}

static const mxArray *c4_b_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  boolean_T c4_u;
  const mxArray *c4_y = NULL;
  SFc4_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc4_RobotSimInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_u = *(boolean_T *)c4_inData;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", &c4_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, FALSE);
  return c4_mxArrayOutData;
}

static boolean_T c4_c_emlrt_marshallIn(SFc4_RobotSimInstanceStruct
  *chartInstance, const mxArray *c4_impact, const char_T *c4_identifier)
{
  boolean_T c4_y;
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_impact), &c4_thisId);
  sf_mex_destroy(&c4_impact);
  return c4_y;
}

static boolean_T c4_d_emlrt_marshallIn(SFc4_RobotSimInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  boolean_T c4_y;
  boolean_T c4_b1;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_b1, 1, 11, 0U, 0, 0U, 0);
  c4_y = c4_b1;
  sf_mex_destroy(&c4_u);
  return c4_y;
}

static void c4_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_impact;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  boolean_T c4_y;
  SFc4_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc4_RobotSimInstanceStruct *)chartInstanceVoid;
  c4_impact = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_impact), &c4_thisId);
  sf_mex_destroy(&c4_impact);
  *(boolean_T *)c4_outData = c4_y;
  sf_mex_destroy(&c4_mxArrayInData);
}

static const mxArray *c4_c_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i13;
  int32_T c4_i14;
  int32_T c4_i15;
  real_T c4_b_inData[10];
  int32_T c4_i16;
  int32_T c4_i17;
  int32_T c4_i18;
  real_T c4_u[10];
  const mxArray *c4_y = NULL;
  SFc4_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc4_RobotSimInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_i13 = 0;
  for (c4_i14 = 0; c4_i14 < 2; c4_i14++) {
    for (c4_i15 = 0; c4_i15 < 5; c4_i15++) {
      c4_b_inData[c4_i15 + c4_i13] = (*(real_T (*)[10])c4_inData)[c4_i15 +
        c4_i13];
    }

    c4_i13 += 5;
  }

  c4_i16 = 0;
  for (c4_i17 = 0; c4_i17 < 2; c4_i17++) {
    for (c4_i18 = 0; c4_i18 < 5; c4_i18++) {
      c4_u[c4_i18 + c4_i16] = c4_b_inData[c4_i18 + c4_i16];
    }

    c4_i16 += 5;
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 2, 5, 2), FALSE);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, FALSE);
  return c4_mxArrayOutData;
}

static const mxArray *c4_d_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i19;
  real_T c4_b_inData[2];
  int32_T c4_i20;
  real_T c4_u[2];
  const mxArray *c4_y = NULL;
  SFc4_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc4_RobotSimInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  for (c4_i19 = 0; c4_i19 < 2; c4_i19++) {
    c4_b_inData[c4_i19] = (*(real_T (*)[2])c4_inData)[c4_i19];
  }

  for (c4_i20 = 0; c4_i20 < 2; c4_i20++) {
    c4_u[c4_i20] = c4_b_inData[c4_i20];
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 1, 2), FALSE);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, FALSE);
  return c4_mxArrayOutData;
}

static const mxArray *c4_e_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i21;
  real_T c4_b_inData[2];
  int32_T c4_i22;
  real_T c4_u[2];
  const mxArray *c4_y = NULL;
  SFc4_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc4_RobotSimInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  for (c4_i21 = 0; c4_i21 < 2; c4_i21++) {
    c4_b_inData[c4_i21] = (*(real_T (*)[2])c4_inData)[c4_i21];
  }

  for (c4_i22 = 0; c4_i22 < 2; c4_i22++) {
    c4_u[c4_i22] = c4_b_inData[c4_i22];
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 2, 1, 2), FALSE);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, FALSE);
  return c4_mxArrayOutData;
}

static const mxArray *c4_f_sf_marshallOut(void *chartInstanceVoid, real_T
  c4_inData_data[1], int32_T c4_inData_sizes[1])
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_b_inData_sizes;
  int32_T c4_loop_ub;
  int32_T c4_i23;
  real_T c4_b_inData_data[1];
  int32_T c4_u_sizes;
  int32_T c4_b_loop_ub;
  int32_T c4_i24;
  real_T c4_u_data[1];
  const mxArray *c4_y = NULL;
  SFc4_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc4_RobotSimInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_b_inData_sizes = c4_inData_sizes[0];
  c4_loop_ub = c4_inData_sizes[0] - 1;
  for (c4_i23 = 0; c4_i23 <= c4_loop_ub; c4_i23++) {
    c4_b_inData_data[c4_i23] = c4_inData_data[c4_i23];
  }

  c4_u_sizes = c4_b_inData_sizes;
  c4_b_loop_ub = c4_b_inData_sizes - 1;
  for (c4_i24 = 0; c4_i24 <= c4_b_loop_ub; c4_i24++) {
    c4_u_data[c4_i24] = c4_b_inData_data[c4_i24];
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u_data, 0, 0U, 1U, 0U, 1,
    c4_u_sizes), FALSE);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, FALSE);
  return c4_mxArrayOutData;
}

static void c4_e_emlrt_marshallIn(SFc4_RobotSimInstanceStruct *chartInstance,
  const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId, real_T c4_y_data[1],
  int32_T c4_y_sizes[1])
{
  static uint32_T c4_uv0[1] = { 1U };

  uint32_T c4_uv1[1];
  static boolean_T c4_bv0[1] = { TRUE };

  boolean_T c4_bv1[1];
  int32_T c4_tmp_sizes;
  real_T c4_tmp_data[1];
  int32_T c4_loop_ub;
  int32_T c4_i25;
  c4_uv1[0] = c4_uv0[0];
  c4_bv1[0] = c4_bv0[0];
  sf_mex_import_vs(c4_parentId, sf_mex_dup(c4_u), c4_tmp_data, 1, 0, 0U, 1, 0U,
                   1, c4_bv1, c4_uv1, &c4_tmp_sizes);
  c4_y_sizes[0] = c4_tmp_sizes;
  c4_loop_ub = c4_tmp_sizes - 1;
  for (c4_i25 = 0; c4_i25 <= c4_loop_ub; c4_i25++) {
    c4_y_data[c4_i25] = c4_tmp_data[c4_i25];
  }

  sf_mex_destroy(&c4_u);
}

static void c4_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, real_T c4_outData_data[1],
  int32_T c4_outData_sizes[1])
{
  const mxArray *c4_height;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  int32_T c4_y_sizes;
  real_T c4_y_data[1];
  int32_T c4_loop_ub;
  int32_T c4_i26;
  SFc4_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc4_RobotSimInstanceStruct *)chartInstanceVoid;
  c4_height = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_height), &c4_thisId,
                        c4_y_data, *(int32_T (*)[1])&c4_y_sizes);
  sf_mex_destroy(&c4_height);
  c4_outData_sizes[0] = c4_y_sizes;
  c4_loop_ub = c4_y_sizes - 1;
  for (c4_i26 = 0; c4_i26 <= c4_loop_ub; c4_i26++) {
    c4_outData_data[c4_i26] = c4_y_data[c4_i26];
  }

  sf_mex_destroy(&c4_mxArrayInData);
}

static void c4_f_emlrt_marshallIn(SFc4_RobotSimInstanceStruct *chartInstance,
  const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId, real_T c4_y[2])
{
  real_T c4_dv0[2];
  int32_T c4_i27;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), c4_dv0, 1, 0, 0U, 1, 0U, 1, 2);
  for (c4_i27 = 0; c4_i27 < 2; c4_i27++) {
    c4_y[c4_i27] = c4_dv0[c4_i27];
  }

  sf_mex_destroy(&c4_u);
}

static void c4_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_p2;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_y[2];
  int32_T c4_i28;
  SFc4_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc4_RobotSimInstanceStruct *)chartInstanceVoid;
  c4_p2 = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_p2), &c4_thisId, c4_y);
  sf_mex_destroy(&c4_p2);
  for (c4_i28 = 0; c4_i28 < 2; c4_i28++) {
    (*(real_T (*)[2])c4_outData)[c4_i28] = c4_y[c4_i28];
  }

  sf_mex_destroy(&c4_mxArrayInData);
}

static void c4_g_emlrt_marshallIn(SFc4_RobotSimInstanceStruct *chartInstance,
  const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId, real_T c4_y[2])
{
  real_T c4_dv1[2];
  int32_T c4_i29;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), c4_dv1, 1, 0, 0U, 1, 0U, 2, 1, 2);
  for (c4_i29 = 0; c4_i29 < 2; c4_i29++) {
    c4_y[c4_i29] = c4_dv1[c4_i29];
  }

  sf_mex_destroy(&c4_u);
}

static void c4_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_org;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_y[2];
  int32_T c4_i30;
  SFc4_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc4_RobotSimInstanceStruct *)chartInstanceVoid;
  c4_org = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_org), &c4_thisId, c4_y);
  sf_mex_destroy(&c4_org);
  for (c4_i30 = 0; c4_i30 < 2; c4_i30++) {
    (*(real_T (*)[2])c4_outData)[c4_i30] = c4_y[c4_i30];
  }

  sf_mex_destroy(&c4_mxArrayInData);
}

const mxArray *sf_c4_RobotSim_get_eml_resolved_functions_info(void)
{
  const mxArray *c4_nameCaptureInfo;
  c4_ResolvedFunctionInfo c4_info[35];
  const mxArray *c4_m0 = NULL;
  int32_T c4_i31;
  c4_ResolvedFunctionInfo *c4_r0;
  c4_nameCaptureInfo = NULL;
  c4_nameCaptureInfo = NULL;
  c4_info_helper(c4_info);
  sf_mex_assign(&c4_m0, sf_mex_createstruct("nameCaptureInfo", 1, 35), FALSE);
  for (c4_i31 = 0; c4_i31 < 35; c4_i31++) {
    c4_r0 = &c4_info[c4_i31];
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", c4_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c4_r0->context)), "context", "nameCaptureInfo",
                    c4_i31);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", c4_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c4_r0->name)), "name", "nameCaptureInfo", c4_i31);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", c4_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c4_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c4_i31);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", c4_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c4_r0->resolved)), "resolved", "nameCaptureInfo",
                    c4_i31);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", &c4_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c4_i31);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", &c4_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c4_i31);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", &c4_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c4_i31);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", &c4_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c4_i31);
  }

  sf_mex_assign(&c4_nameCaptureInfo, c4_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c4_nameCaptureInfo);
  return c4_nameCaptureInfo;
}

static void c4_info_helper(c4_ResolvedFunctionInfo c4_info[35])
{
  c4_info[0].context = "";
  c4_info[0].name = "endSwingFoot";
  c4_info[0].dominantType = "double";
  c4_info[0].resolved =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/endSwingFoot.m";
  c4_info[0].fileTimeLo = 1410925495U;
  c4_info[0].fileTimeHi = 0U;
  c4_info[0].mFileTimeLo = 0U;
  c4_info[0].mFileTimeHi = 0U;
  c4_info[1].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/endSwingFoot.m";
  c4_info[1].name = "dynParams";
  c4_info[1].dominantType = "";
  c4_info[1].resolved =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/dynParams.m";
  c4_info[1].fileTimeLo = 1410766076U;
  c4_info[1].fileTimeHi = 0U;
  c4_info[1].mFileTimeLo = 0U;
  c4_info[1].mFileTimeHi = 0U;
  c4_info[2].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/dynParams.m";
  c4_info[2].name = "mrdivide";
  c4_info[2].dominantType = "double";
  c4_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c4_info[2].fileTimeLo = 1357915548U;
  c4_info[2].fileTimeHi = 0U;
  c4_info[2].mFileTimeLo = 1319697566U;
  c4_info[2].mFileTimeHi = 0U;
  c4_info[3].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c4_info[3].name = "rdivide";
  c4_info[3].dominantType = "double";
  c4_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c4_info[3].fileTimeLo = 1346481588U;
  c4_info[3].fileTimeHi = 0U;
  c4_info[3].mFileTimeLo = 0U;
  c4_info[3].mFileTimeHi = 0U;
  c4_info[4].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c4_info[4].name = "eml_scalexp_compatible";
  c4_info[4].dominantType = "double";
  c4_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m";
  c4_info[4].fileTimeLo = 1286786396U;
  c4_info[4].fileTimeHi = 0U;
  c4_info[4].mFileTimeLo = 0U;
  c4_info[4].mFileTimeHi = 0U;
  c4_info[5].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c4_info[5].name = "eml_div";
  c4_info[5].dominantType = "double";
  c4_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c4_info[5].fileTimeLo = 1313319010U;
  c4_info[5].fileTimeHi = 0U;
  c4_info[5].mFileTimeLo = 0U;
  c4_info[5].mFileTimeHi = 0U;
  c4_info[6].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/dynParams.m";
  c4_info[6].name = "mtimes";
  c4_info[6].dominantType = "double";
  c4_info[6].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c4_info[6].fileTimeLo = 1289483692U;
  c4_info[6].fileTimeHi = 0U;
  c4_info[6].mFileTimeLo = 0U;
  c4_info[6].mFileTimeHi = 0U;
  c4_info[7].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/dynParams.m";
  c4_info[7].name = "mpower";
  c4_info[7].dominantType = "double";
  c4_info[7].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c4_info[7].fileTimeLo = 1286786442U;
  c4_info[7].fileTimeHi = 0U;
  c4_info[7].mFileTimeLo = 0U;
  c4_info[7].mFileTimeHi = 0U;
  c4_info[8].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c4_info[8].name = "power";
  c4_info[8].dominantType = "double";
  c4_info[8].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c4_info[8].fileTimeLo = 1348163130U;
  c4_info[8].fileTimeHi = 0U;
  c4_info[8].mFileTimeLo = 0U;
  c4_info[8].mFileTimeHi = 0U;
  c4_info[9].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c4_info[9].name = "eml_scalar_eg";
  c4_info[9].dominantType = "double";
  c4_info[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[9].fileTimeLo = 1286786396U;
  c4_info[9].fileTimeHi = 0U;
  c4_info[9].mFileTimeLo = 0U;
  c4_info[9].mFileTimeHi = 0U;
  c4_info[10].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c4_info[10].name = "eml_scalexp_alloc";
  c4_info[10].dominantType = "double";
  c4_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c4_info[10].fileTimeLo = 1352388860U;
  c4_info[10].fileTimeHi = 0U;
  c4_info[10].mFileTimeLo = 0U;
  c4_info[10].mFileTimeHi = 0U;
  c4_info[11].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c4_info[11].name = "floor";
  c4_info[11].dominantType = "double";
  c4_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c4_info[11].fileTimeLo = 1343801580U;
  c4_info[11].fileTimeHi = 0U;
  c4_info[11].mFileTimeLo = 0U;
  c4_info[11].mFileTimeHi = 0U;
  c4_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c4_info[12].name = "eml_scalar_floor";
  c4_info[12].dominantType = "double";
  c4_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c4_info[12].fileTimeLo = 1286786326U;
  c4_info[12].fileTimeHi = 0U;
  c4_info[12].mFileTimeLo = 0U;
  c4_info[12].mFileTimeHi = 0U;
  c4_info[13].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power";
  c4_info[13].name = "eml_scalar_eg";
  c4_info[13].dominantType = "double";
  c4_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[13].fileTimeLo = 1286786396U;
  c4_info[13].fileTimeHi = 0U;
  c4_info[13].mFileTimeLo = 0U;
  c4_info[13].mFileTimeHi = 0U;
  c4_info[14].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power";
  c4_info[14].name = "mtimes";
  c4_info[14].dominantType = "double";
  c4_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c4_info[14].fileTimeLo = 1289483692U;
  c4_info[14].fileTimeHi = 0U;
  c4_info[14].mFileTimeLo = 0U;
  c4_info[14].mFileTimeHi = 0U;
  c4_info[15].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/endSwingFoot.m";
  c4_info[15].name = "sin";
  c4_info[15].dominantType = "double";
  c4_info[15].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c4_info[15].fileTimeLo = 1343801586U;
  c4_info[15].fileTimeHi = 0U;
  c4_info[15].mFileTimeLo = 0U;
  c4_info[15].mFileTimeHi = 0U;
  c4_info[16].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c4_info[16].name = "eml_scalar_sin";
  c4_info[16].dominantType = "double";
  c4_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m";
  c4_info[16].fileTimeLo = 1286786336U;
  c4_info[16].fileTimeHi = 0U;
  c4_info[16].mFileTimeLo = 0U;
  c4_info[16].mFileTimeHi = 0U;
  c4_info[17].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/endSwingFoot.m";
  c4_info[17].name = "mtimes";
  c4_info[17].dominantType = "double";
  c4_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c4_info[17].fileTimeLo = 1289483692U;
  c4_info[17].fileTimeHi = 0U;
  c4_info[17].mFileTimeLo = 0U;
  c4_info[17].mFileTimeHi = 0U;
  c4_info[18].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/endSwingFoot.m";
  c4_info[18].name = "cos";
  c4_info[18].dominantType = "double";
  c4_info[18].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c4_info[18].fileTimeLo = 1343801572U;
  c4_info[18].fileTimeHi = 0U;
  c4_info[18].mFileTimeLo = 0U;
  c4_info[18].mFileTimeHi = 0U;
  c4_info[19].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c4_info[19].name = "eml_scalar_cos";
  c4_info[19].dominantType = "double";
  c4_info[19].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m";
  c4_info[19].fileTimeLo = 1286786322U;
  c4_info[19].fileTimeHi = 0U;
  c4_info[19].mFileTimeLo = 0U;
  c4_info[19].mFileTimeHi = 0U;
  c4_info[20].context = "";
  c4_info[20].name = "find";
  c4_info[20].dominantType = "double";
  c4_info[20].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m";
  c4_info[20].fileTimeLo = 1303117406U;
  c4_info[20].fileTimeHi = 0U;
  c4_info[20].mFileTimeLo = 0U;
  c4_info[20].mFileTimeHi = 0U;
  c4_info[21].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m!eml_find";
  c4_info[21].name = "eml_index_class";
  c4_info[21].dominantType = "";
  c4_info[21].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[21].fileTimeLo = 1323134578U;
  c4_info[21].fileTimeHi = 0U;
  c4_info[21].mFileTimeLo = 0U;
  c4_info[21].mFileTimeHi = 0U;
  c4_info[22].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m!eml_find";
  c4_info[22].name = "eml_scalar_eg";
  c4_info[22].dominantType = "logical";
  c4_info[22].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[22].fileTimeLo = 1286786396U;
  c4_info[22].fileTimeHi = 0U;
  c4_info[22].mFileTimeLo = 0U;
  c4_info[22].mFileTimeHi = 0U;
  c4_info[23].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m!eml_find";
  c4_info[23].name = "floor";
  c4_info[23].dominantType = "double";
  c4_info[23].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c4_info[23].fileTimeLo = 1343801580U;
  c4_info[23].fileTimeHi = 0U;
  c4_info[23].mFileTimeLo = 0U;
  c4_info[23].mFileTimeHi = 0U;
  c4_info[24].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m!eml_find";
  c4_info[24].name = "min";
  c4_info[24].dominantType = "coder.internal.indexInt";
  c4_info[24].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c4_info[24].fileTimeLo = 1311226518U;
  c4_info[24].fileTimeHi = 0U;
  c4_info[24].mFileTimeLo = 0U;
  c4_info[24].mFileTimeHi = 0U;
  c4_info[25].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c4_info[25].name = "eml_min_or_max";
  c4_info[25].dominantType = "char";
  c4_info[25].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c4_info[25].fileTimeLo = 1334042690U;
  c4_info[25].fileTimeHi = 0U;
  c4_info[25].mFileTimeLo = 0U;
  c4_info[25].mFileTimeHi = 0U;
  c4_info[26].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c4_info[26].name = "eml_scalar_eg";
  c4_info[26].dominantType = "coder.internal.indexInt";
  c4_info[26].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[26].fileTimeLo = 1286786396U;
  c4_info[26].fileTimeHi = 0U;
  c4_info[26].mFileTimeLo = 0U;
  c4_info[26].mFileTimeHi = 0U;
  c4_info[27].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c4_info[27].name = "eml_scalexp_alloc";
  c4_info[27].dominantType = "coder.internal.indexInt";
  c4_info[27].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c4_info[27].fileTimeLo = 1352388860U;
  c4_info[27].fileTimeHi = 0U;
  c4_info[27].mFileTimeLo = 0U;
  c4_info[27].mFileTimeHi = 0U;
  c4_info[28].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c4_info[28].name = "eml_index_class";
  c4_info[28].dominantType = "";
  c4_info[28].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[28].fileTimeLo = 1323134578U;
  c4_info[28].fileTimeHi = 0U;
  c4_info[28].mFileTimeLo = 0U;
  c4_info[28].mFileTimeHi = 0U;
  c4_info[29].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c4_info[29].name = "eml_scalar_eg";
  c4_info[29].dominantType = "coder.internal.indexInt";
  c4_info[29].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[29].fileTimeLo = 1286786396U;
  c4_info[29].fileTimeHi = 0U;
  c4_info[29].mFileTimeLo = 0U;
  c4_info[29].mFileTimeHi = 0U;
  c4_info[30].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m!eml_find";
  c4_info[30].name = "eml_int_forloop_overflow_check";
  c4_info[30].dominantType = "";
  c4_info[30].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c4_info[30].fileTimeLo = 1346481540U;
  c4_info[30].fileTimeHi = 0U;
  c4_info[30].mFileTimeLo = 0U;
  c4_info[30].mFileTimeHi = 0U;
  c4_info[31].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c4_info[31].name = "intmin";
  c4_info[31].dominantType = "char";
  c4_info[31].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c4_info[31].fileTimeLo = 1311226518U;
  c4_info[31].fileTimeHi = 0U;
  c4_info[31].mFileTimeLo = 0U;
  c4_info[31].mFileTimeHi = 0U;
  c4_info[32].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m!eml_find";
  c4_info[32].name = "eml_index_plus";
  c4_info[32].dominantType = "double";
  c4_info[32].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c4_info[32].fileTimeLo = 1286786378U;
  c4_info[32].fileTimeHi = 0U;
  c4_info[32].mFileTimeLo = 0U;
  c4_info[32].mFileTimeHi = 0U;
  c4_info[33].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c4_info[33].name = "eml_index_class";
  c4_info[33].dominantType = "";
  c4_info[33].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[33].fileTimeLo = 1323134578U;
  c4_info[33].fileTimeHi = 0U;
  c4_info[33].mFileTimeLo = 0U;
  c4_info[33].mFileTimeHi = 0U;
  c4_info[34].context = "";
  c4_info[34].name = "scuff";
  c4_info[34].dominantType = "double";
  c4_info[34].resolved =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/scuff.m";
  c4_info[34].fileTimeLo = 1410923831U;
  c4_info[34].fileTimeHi = 0U;
  c4_info[34].mFileTimeLo = 0U;
  c4_info[34].mFileTimeHi = 0U;
}

static const mxArray *c4_g_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_u;
  const mxArray *c4_y = NULL;
  SFc4_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc4_RobotSimInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_u = *(int32_T *)c4_inData;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", &c4_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, FALSE);
  return c4_mxArrayOutData;
}

static int32_T c4_h_emlrt_marshallIn(SFc4_RobotSimInstanceStruct *chartInstance,
  const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  int32_T c4_y;
  int32_T c4_i32;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_i32, 1, 6, 0U, 0, 0U, 0);
  c4_y = c4_i32;
  sf_mex_destroy(&c4_u);
  return c4_y;
}

static void c4_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_b_sfEvent;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  int32_T c4_y;
  SFc4_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc4_RobotSimInstanceStruct *)chartInstanceVoid;
  c4_b_sfEvent = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_sfEvent),
    &c4_thisId);
  sf_mex_destroy(&c4_b_sfEvent);
  *(int32_T *)c4_outData = c4_y;
  sf_mex_destroy(&c4_mxArrayInData);
}

static uint8_T c4_i_emlrt_marshallIn(SFc4_RobotSimInstanceStruct *chartInstance,
  const mxArray *c4_b_is_active_c4_RobotSim, const char_T *c4_identifier)
{
  uint8_T c4_y;
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_j_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c4_b_is_active_c4_RobotSim), &c4_thisId);
  sf_mex_destroy(&c4_b_is_active_c4_RobotSim);
  return c4_y;
}

static uint8_T c4_j_emlrt_marshallIn(SFc4_RobotSimInstanceStruct *chartInstance,
  const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  uint8_T c4_y;
  uint8_T c4_u0;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_u0, 1, 3, 0U, 0, 0U, 0);
  c4_y = c4_u0;
  sf_mex_destroy(&c4_u);
  return c4_y;
}

static void init_dsm_address_info(SFc4_RobotSimInstanceStruct *chartInstance)
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

void sf_c4_RobotSim_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(186257537U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2760215682U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1156314796U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1683105684U);
}

mxArray *sf_c4_RobotSim_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("E0EA44JEdzVqZX2y7zpMJH");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(2);
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
      pr[0] = (double)(5);
      pr[1] = (double)(2);
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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(1));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c4_RobotSim_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c4_RobotSim(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x3'type','srcId','name','auxInfo'{{M[1],M[5],T\"impact\",},{M[1],M[10],T\"y\",},{M[8],M[0],T\"is_active_c4_RobotSim\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 3, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c4_RobotSim_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc4_RobotSimInstanceStruct *chartInstance;
    chartInstance = (SFc4_RobotSimInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _RobotSimMachineNumber_,
           4,
           1,
           1,
           6,
           0,
           0,
           0,
           0,
           3,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"org");
          _SFD_SET_DATA_PROPS(1,1,1,0,"q");
          _SFD_SET_DATA_PROPS(2,1,1,0,"ground");
          _SFD_SET_DATA_PROPS(3,2,0,1,"impact");
          _SFD_SET_DATA_PROPS(4,1,1,0,"y_prev");
          _SFD_SET_DATA_PROPS(5,2,0,1,"y");
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
        _SFD_CV_INIT_EML(0,1,1,1,0,0,0,0,0,3,1);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,562);
        _SFD_CV_INIT_EML_IF(0,1,0,488,534,-1,557);

        {
          static int condStart[] = { 491, 506, 526 };

          static int condEnd[] = { 502, 521, 534 };

          static int pfixExpr[] = { 0, 1, -3, 2, -1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,0,491,534,3,0,&(condStart[0]),&(condEnd[0]),
                                6,&(pfixExpr[0]));
        }

        _SFD_CV_INIT_SCRIPT(0,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(0,0,"endSwingFoot",0,-1,252);
        _SFD_CV_INIT_SCRIPT(1,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(1,0,"dynParams",0,-1,172);
        _SFD_CV_INIT_SCRIPT(2,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(2,0,"scuff",0,-1,184);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c4_e_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c4_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 5;
          dimVector[1]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c4_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(3,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_b_sf_marshallOut,(MexInFcnForType)c4_b_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_sf_marshallOut,(MexInFcnForType)c4_sf_marshallIn);

        {
          boolean_T *c4_impact;
          real_T *c4_y_prev;
          real_T *c4_y;
          real_T (*c4_org)[2];
          real_T (*c4_q)[2];
          real_T (*c4_ground)[10];
          c4_y = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
          c4_y_prev = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c4_impact = (boolean_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          c4_ground = (real_T (*)[10])ssGetInputPortSignal(chartInstance->S, 2);
          c4_q = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 1);
          c4_org = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c4_org);
          _SFD_SET_DATA_VALUE_PTR(1U, *c4_q);
          _SFD_SET_DATA_VALUE_PTR(2U, *c4_ground);
          _SFD_SET_DATA_VALUE_PTR(3U, c4_impact);
          _SFD_SET_DATA_VALUE_PTR(4U, c4_y_prev);
          _SFD_SET_DATA_VALUE_PTR(5U, c4_y);
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
  return "rwMBkkk72m7uCtEvIYHKG";
}

static void sf_opaque_initialize_c4_RobotSim(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc4_RobotSimInstanceStruct*) chartInstanceVar)
    ->S,0);
  initialize_params_c4_RobotSim((SFc4_RobotSimInstanceStruct*) chartInstanceVar);
  initialize_c4_RobotSim((SFc4_RobotSimInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c4_RobotSim(void *chartInstanceVar)
{
  enable_c4_RobotSim((SFc4_RobotSimInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c4_RobotSim(void *chartInstanceVar)
{
  disable_c4_RobotSim((SFc4_RobotSimInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c4_RobotSim(void *chartInstanceVar)
{
  sf_c4_RobotSim((SFc4_RobotSimInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c4_RobotSim(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c4_RobotSim((SFc4_RobotSimInstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c4_RobotSim();/* state var info */
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

extern void sf_internal_set_sim_state_c4_RobotSim(SimStruct* S, const mxArray
  *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c4_RobotSim();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c4_RobotSim((SFc4_RobotSimInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c4_RobotSim(SimStruct* S)
{
  return sf_internal_get_sim_state_c4_RobotSim(S);
}

static void sf_opaque_set_sim_state_c4_RobotSim(SimStruct* S, const mxArray *st)
{
  sf_internal_set_sim_state_c4_RobotSim(S, st);
}

static void sf_opaque_terminate_c4_RobotSim(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc4_RobotSimInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_RobotSim_optimization_info();
    }

    finalize_c4_RobotSim((SFc4_RobotSimInstanceStruct*) chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc4_RobotSim((SFc4_RobotSimInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c4_RobotSim(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c4_RobotSim((SFc4_RobotSimInstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c4_RobotSim(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_RobotSim_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      4);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,4,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,4,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,4);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,4,4);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,4,2);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=2; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 4; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,4);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(5806941U));
  ssSetChecksum1(S,(106817137U));
  ssSetChecksum2(S,(3526482639U));
  ssSetChecksum3(S,(3986956755U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c4_RobotSim(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c4_RobotSim(SimStruct *S)
{
  SFc4_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc4_RobotSimInstanceStruct *)utMalloc(sizeof
    (SFc4_RobotSimInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc4_RobotSimInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c4_RobotSim;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c4_RobotSim;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c4_RobotSim;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c4_RobotSim;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c4_RobotSim;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c4_RobotSim;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c4_RobotSim;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c4_RobotSim;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c4_RobotSim;
  chartInstance->chartInfo.mdlStart = mdlStart_c4_RobotSim;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c4_RobotSim;
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

void c4_RobotSim_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c4_RobotSim(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c4_RobotSim(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c4_RobotSim(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c4_RobotSim_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
