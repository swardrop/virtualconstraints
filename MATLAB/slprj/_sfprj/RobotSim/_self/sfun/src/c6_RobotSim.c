/* Include files */

#include <stddef.h>
#include "blas.h"
#include "RobotSim_sfun.h"
#include "c6_RobotSim.h"
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
static const char * c6_debug_family_names[8] = { "q_des", "nargin", "nargout",
  "theta_p", "alpha_p", "q", "theta", "err" };

static const char * c6_b_debug_family_names[10] = { "s", "q_dep", "n", "i",
  "nargin", "nargout", "theta_p", "alpha_p", "theta", "q" };

static const char * c6_c_debug_family_names[4] = { "nargin", "nargout", "q",
  "q_act" };

/* Function Declarations */
static void initialize_c6_RobotSim(SFc6_RobotSimInstanceStruct *chartInstance);
static void initialize_params_c6_RobotSim(SFc6_RobotSimInstanceStruct
  *chartInstance);
static void enable_c6_RobotSim(SFc6_RobotSimInstanceStruct *chartInstance);
static void disable_c6_RobotSim(SFc6_RobotSimInstanceStruct *chartInstance);
static void c6_update_debugger_state_c6_RobotSim(SFc6_RobotSimInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c6_RobotSim(SFc6_RobotSimInstanceStruct
  *chartInstance);
static void set_sim_state_c6_RobotSim(SFc6_RobotSimInstanceStruct *chartInstance,
  const mxArray *c6_st);
static void finalize_c6_RobotSim(SFc6_RobotSimInstanceStruct *chartInstance);
static void sf_c6_RobotSim(SFc6_RobotSimInstanceStruct *chartInstance);
static void initSimStructsc6_RobotSim(SFc6_RobotSimInstanceStruct *chartInstance);
static void registerMessagesc6_RobotSim(SFc6_RobotSimInstanceStruct
  *chartInstance);
static void c6_bezConstraint(SFc6_RobotSimInstanceStruct *chartInstance, real_T
  c6_theta_p[5], real_T c6_alpha_p[5], real_T c6_theta, real_T c6_q[2]);
static void init_script_number_translation(uint32_T c6_machineNumber, uint32_T
  c6_chartNumber);
static const mxArray *c6_sf_marshallOut(void *chartInstanceVoid, void *c6_inData);
static real_T c6_emlrt_marshallIn(SFc6_RobotSimInstanceStruct *chartInstance,
  const mxArray *c6_err, const char_T *c6_identifier);
static real_T c6_b_emlrt_marshallIn(SFc6_RobotSimInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_b_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static const mxArray *c6_c_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_c_emlrt_marshallIn(SFc6_RobotSimInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId, real_T c6_y[2]);
static void c6_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static void c6_d_emlrt_marshallIn(SFc6_RobotSimInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId, real_T c6_y[5]);
static void c6_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static void c6_info_helper(c6_ResolvedFunctionInfo c6_info[60]);
static void c6_eml_li_find(SFc6_RobotSimInstanceStruct *chartInstance, boolean_T
  c6_x, int32_T c6_y_data[1], int32_T c6_y_sizes[2]);
static void c6_eml_scalar_eg(SFc6_RobotSimInstanceStruct *chartInstance);
static void c6_eml_warning(SFc6_RobotSimInstanceStruct *chartInstance);
static real_T c6_power(SFc6_RobotSimInstanceStruct *chartInstance, real_T c6_a,
  real_T c6_b);
static void c6_eml_error(SFc6_RobotSimInstanceStruct *chartInstance);
static const mxArray *c6_d_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static int32_T c6_e_emlrt_marshallIn(SFc6_RobotSimInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static uint8_T c6_f_emlrt_marshallIn(SFc6_RobotSimInstanceStruct *chartInstance,
  const mxArray *c6_b_is_active_c6_RobotSim, const char_T *c6_identifier);
static uint8_T c6_g_emlrt_marshallIn(SFc6_RobotSimInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void init_dsm_address_info(SFc6_RobotSimInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c6_RobotSim(SFc6_RobotSimInstanceStruct *chartInstance)
{
  chartInstance->c6_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c6_is_active_c6_RobotSim = 0U;
}

static void initialize_params_c6_RobotSim(SFc6_RobotSimInstanceStruct
  *chartInstance)
{
}

static void enable_c6_RobotSim(SFc6_RobotSimInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c6_RobotSim(SFc6_RobotSimInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c6_update_debugger_state_c6_RobotSim(SFc6_RobotSimInstanceStruct
  *chartInstance)
{
}

static const mxArray *get_sim_state_c6_RobotSim(SFc6_RobotSimInstanceStruct
  *chartInstance)
{
  const mxArray *c6_st;
  const mxArray *c6_y = NULL;
  real_T c6_hoistedGlobal;
  real_T c6_u;
  const mxArray *c6_b_y = NULL;
  uint8_T c6_b_hoistedGlobal;
  uint8_T c6_b_u;
  const mxArray *c6_c_y = NULL;
  real_T *c6_err;
  c6_err = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c6_st = NULL;
  c6_st = NULL;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_createcellarray(2), FALSE);
  c6_hoistedGlobal = *c6_err;
  c6_u = c6_hoistedGlobal;
  c6_b_y = NULL;
  sf_mex_assign(&c6_b_y, sf_mex_create("y", &c6_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c6_y, 0, c6_b_y);
  c6_b_hoistedGlobal = chartInstance->c6_is_active_c6_RobotSim;
  c6_b_u = c6_b_hoistedGlobal;
  c6_c_y = NULL;
  sf_mex_assign(&c6_c_y, sf_mex_create("y", &c6_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c6_y, 1, c6_c_y);
  sf_mex_assign(&c6_st, c6_y, FALSE);
  return c6_st;
}

static void set_sim_state_c6_RobotSim(SFc6_RobotSimInstanceStruct *chartInstance,
  const mxArray *c6_st)
{
  const mxArray *c6_u;
  real_T *c6_err;
  c6_err = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c6_doneDoubleBufferReInit = TRUE;
  c6_u = sf_mex_dup(c6_st);
  *c6_err = c6_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 0)),
    "err");
  chartInstance->c6_is_active_c6_RobotSim = c6_f_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c6_u, 1)), "is_active_c6_RobotSim");
  sf_mex_destroy(&c6_u);
  c6_update_debugger_state_c6_RobotSim(chartInstance);
  sf_mex_destroy(&c6_st);
}

static void finalize_c6_RobotSim(SFc6_RobotSimInstanceStruct *chartInstance)
{
}

static void sf_c6_RobotSim(SFc6_RobotSimInstanceStruct *chartInstance)
{
  int32_T c6_i0;
  int32_T c6_i1;
  int32_T c6_i2;
  real_T c6_hoistedGlobal;
  int32_T c6_i3;
  real_T c6_theta_p[5];
  int32_T c6_i4;
  real_T c6_alpha_p[5];
  int32_T c6_i5;
  real_T c6_q[2];
  real_T c6_theta;
  uint32_T c6_debug_family_var_map[8];
  real_T c6_q_des[2];
  real_T c6_nargin = 4.0;
  real_T c6_nargout = 1.0;
  real_T c6_err;
  int32_T c6_i6;
  real_T c6_b_theta_p[5];
  int32_T c6_i7;
  real_T c6_b_alpha_p[5];
  real_T c6_dv0[2];
  int32_T c6_i8;
  int32_T c6_i9;
  real_T c6_b_q[2];
  uint32_T c6_b_debug_family_var_map[4];
  real_T c6_b_nargin = 1.0;
  real_T c6_b_nargout = 1.0;
  real_T c6_x;
  real_T c6_xk;
  real_T c6_b_x;
  real_T c6_c_x;
  real_T c6_d_x;
  real_T c6_e_x;
  real_T c6_y;
  real_T c6_f_x;
  real_T c6_b_y;
  real_T c6_b;
  real_T c6_c_y;
  real_T c6_g_x;
  real_T c6_h_x;
  int32_T c6_tmp_sizes[2];
  int32_T c6_tmp_data[1];
  int32_T c6_i10;
  int32_T c6_i11;
  int32_T c6_i12;
  int32_T c6_i13;
  int32_T c6_loop_ub;
  int32_T c6_i14;
  int32_T c6_b_tmp_sizes[2];
  int32_T c6_b_tmp_data[1];
  int32_T c6_c_tmp_sizes[2];
  int32_T c6_i15;
  int32_T c6_i16;
  int32_T c6_b_loop_ub;
  int32_T c6_i17;
  real_T c6_c_tmp_data[1];
  real_T c6_dv1[1];
  int32_T c6_c_loop_ub;
  int32_T c6_i18;
  int32_T c6_i19;
  int32_T c6_i20;
  int32_T c6_i21;
  int32_T c6_i22;
  int32_T c6_d_loop_ub;
  int32_T c6_i23;
  int32_T c6_i24;
  int32_T c6_i25;
  int32_T c6_e_loop_ub;
  int32_T c6_i26;
  real_T c6_dv2[1];
  int32_T c6_f_loop_ub;
  int32_T c6_i27;
  real_T *c6_b_theta;
  real_T *c6_b_err;
  real_T (*c6_c_q)[2];
  real_T (*c6_c_alpha_p)[5];
  real_T (*c6_c_theta_p)[5];
  c6_b_err = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c6_b_theta = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c6_c_q = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 2);
  c6_c_alpha_p = (real_T (*)[5])ssGetInputPortSignal(chartInstance->S, 1);
  c6_c_theta_p = (real_T (*)[5])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 3U, chartInstance->c6_sfEvent);
  for (c6_i0 = 0; c6_i0 < 5; c6_i0++) {
    _SFD_DATA_RANGE_CHECK((*c6_c_theta_p)[c6_i0], 0U);
  }

  for (c6_i1 = 0; c6_i1 < 5; c6_i1++) {
    _SFD_DATA_RANGE_CHECK((*c6_c_alpha_p)[c6_i1], 1U);
  }

  for (c6_i2 = 0; c6_i2 < 2; c6_i2++) {
    _SFD_DATA_RANGE_CHECK((*c6_c_q)[c6_i2], 2U);
  }

  _SFD_DATA_RANGE_CHECK(*c6_b_theta, 3U);
  _SFD_DATA_RANGE_CHECK(*c6_b_err, 4U);
  chartInstance->c6_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 3U, chartInstance->c6_sfEvent);
  c6_hoistedGlobal = *c6_b_theta;
  for (c6_i3 = 0; c6_i3 < 5; c6_i3++) {
    c6_theta_p[c6_i3] = (*c6_c_theta_p)[c6_i3];
  }

  for (c6_i4 = 0; c6_i4 < 5; c6_i4++) {
    c6_alpha_p[c6_i4] = (*c6_c_alpha_p)[c6_i4];
  }

  for (c6_i5 = 0; c6_i5 < 2; c6_i5++) {
    c6_q[c6_i5] = (*c6_c_q)[c6_i5];
  }

  c6_theta = c6_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 8U, 8U, c6_debug_family_names,
    c6_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_q_des, 0U, c6_b_sf_marshallOut,
    c6_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_nargin, 1U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_nargout, 2U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c6_theta_p, 3U, c6_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c6_alpha_p, 4U, c6_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c6_q, 5U, c6_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c6_theta, 6U, c6_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_err, 7U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 5);
  for (c6_i6 = 0; c6_i6 < 5; c6_i6++) {
    c6_b_theta_p[c6_i6] = c6_theta_p[c6_i6];
  }

  for (c6_i7 = 0; c6_i7 < 5; c6_i7++) {
    c6_b_alpha_p[c6_i7] = c6_alpha_p[c6_i7];
  }

  c6_bezConstraint(chartInstance, c6_b_theta_p, c6_b_alpha_p, c6_theta, c6_dv0);
  for (c6_i8 = 0; c6_i8 < 2; c6_i8++) {
    c6_q_des[c6_i8] = c6_dv0[c6_i8];
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 8);
  for (c6_i9 = 0; c6_i9 < 2; c6_i9++) {
    c6_b_q[c6_i9] = c6_q_des[c6_i9] - c6_q[c6_i9];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c6_c_debug_family_names,
    c6_b_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_b_nargin, 0U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_b_nargout, 1U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_b_q, 2U, c6_b_sf_marshallOut,
    c6_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_err, 3U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  CV_SCRIPT_FCN(1, 0);
  _SFD_SCRIPT_CALL(1U, chartInstance->c6_sfEvent, 4);
  c6_err = c6_b_q[0];
  _SFD_SCRIPT_CALL(1U, chartInstance->c6_sfEvent, -4);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 10);
  c6_x = c6_err;
  c6_eml_scalar_eg(chartInstance);
  c6_xk = c6_x;
  c6_b_x = c6_xk;
  c6_eml_scalar_eg(chartInstance);
  c6_err = c6_b_x / 6.2831853071795862;
  c6_c_x = c6_err;
  c6_d_x = c6_c_x;
  c6_d_x = muDoubleScalarRound(c6_d_x);
  c6_e_x = c6_err - c6_d_x;
  c6_y = muDoubleScalarAbs(c6_e_x);
  c6_f_x = c6_err;
  c6_b_y = muDoubleScalarAbs(c6_f_x);
  c6_b = c6_b_y;
  c6_c_y = 2.2204460492503131E-16 * c6_b;
  if (c6_y <= c6_c_y) {
    c6_err = 0.0;
  } else {
    c6_g_x = c6_err;
    c6_h_x = c6_g_x;
    c6_h_x = muDoubleScalarFloor(c6_h_x);
    c6_err = (c6_err - c6_h_x) * 6.2831853071795862;
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 12);
  c6_eml_li_find(chartInstance, c6_err > 3.1415926535897931, c6_tmp_data,
                 c6_tmp_sizes);
  c6_tmp_sizes[0] = 1;
  c6_tmp_sizes[1];
  c6_i10 = c6_tmp_sizes[0];
  c6_i11 = c6_tmp_sizes[1];
  c6_i12 = c6_tmp_sizes[0];
  c6_i13 = c6_tmp_sizes[1];
  c6_loop_ub = c6_i12 * c6_i13 - 1;
  for (c6_i14 = 0; c6_i14 <= c6_loop_ub; c6_i14++) {
    c6_tmp_data[c6_i14]--;
  }

  c6_eml_li_find(chartInstance, c6_err > 3.1415926535897931, c6_b_tmp_data,
                 c6_b_tmp_sizes);
  c6_c_tmp_sizes[0] = 1;
  c6_c_tmp_sizes[1] = c6_tmp_sizes[1];
  c6_i15 = c6_c_tmp_sizes[0];
  c6_i16 = c6_c_tmp_sizes[1];
  c6_b_loop_ub = c6_tmp_sizes[0] * c6_tmp_sizes[1] - 1;
  for (c6_i17 = 0; c6_i17 <= c6_b_loop_ub; c6_i17++) {
    c6_c_tmp_data[c6_i17] = c6_err;
  }

  _SFD_SIZE_EQ_CHECK_1D(c6_b_tmp_sizes[1], c6_c_tmp_sizes[1]);
  c6_dv1[0] = c6_err;
  c6_c_loop_ub = c6_c_tmp_sizes[0] * c6_c_tmp_sizes[1] - 1;
  for (c6_i18 = 0; c6_i18 <= c6_c_loop_ub; c6_i18++) {
    c6_dv1[c6_b_tmp_data[c6_i18] - 1] = c6_c_tmp_data[c6_i18] -
      6.2831853071795862;
  }

  c6_err = c6_dv1[0];
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 13);
  c6_eml_li_find(chartInstance, c6_err < -3.1415926535897931, c6_tmp_data,
                 c6_tmp_sizes);
  c6_tmp_sizes[0] = 1;
  c6_tmp_sizes[1];
  c6_i19 = c6_tmp_sizes[0];
  c6_i20 = c6_tmp_sizes[1];
  c6_i21 = c6_tmp_sizes[0];
  c6_i22 = c6_tmp_sizes[1];
  c6_d_loop_ub = c6_i21 * c6_i22 - 1;
  for (c6_i23 = 0; c6_i23 <= c6_d_loop_ub; c6_i23++) {
    c6_tmp_data[c6_i23]--;
  }

  c6_eml_li_find(chartInstance, c6_err < -3.1415926535897931, c6_b_tmp_data,
                 c6_b_tmp_sizes);
  c6_c_tmp_sizes[0] = 1;
  c6_c_tmp_sizes[1] = c6_tmp_sizes[1];
  c6_i24 = c6_c_tmp_sizes[0];
  c6_i25 = c6_c_tmp_sizes[1];
  c6_e_loop_ub = c6_tmp_sizes[0] * c6_tmp_sizes[1] - 1;
  for (c6_i26 = 0; c6_i26 <= c6_e_loop_ub; c6_i26++) {
    c6_c_tmp_data[c6_i26] = c6_err;
  }

  _SFD_SIZE_EQ_CHECK_1D(c6_b_tmp_sizes[1], c6_c_tmp_sizes[1]);
  c6_dv2[0] = c6_err;
  c6_f_loop_ub = c6_c_tmp_sizes[0] * c6_c_tmp_sizes[1] - 1;
  for (c6_i27 = 0; c6_i27 <= c6_f_loop_ub; c6_i27++) {
    c6_dv2[c6_b_tmp_data[c6_i27] - 1] = c6_c_tmp_data[c6_i27] +
      6.2831853071795862;
  }

  c6_err = c6_dv2[0];
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, -13);
  _SFD_SYMBOL_SCOPE_POP();
  *c6_b_err = c6_err;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 3U, chartInstance->c6_sfEvent);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_RobotSimMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc6_RobotSim(SFc6_RobotSimInstanceStruct *chartInstance)
{
}

static void registerMessagesc6_RobotSim(SFc6_RobotSimInstanceStruct
  *chartInstance)
{
}

static void c6_bezConstraint(SFc6_RobotSimInstanceStruct *chartInstance, real_T
  c6_theta_p[5], real_T c6_alpha_p[5], real_T c6_theta, real_T c6_q[2])
{
  uint32_T c6_debug_family_var_map[10];
  real_T c6_s;
  real_T c6_q_dep;
  real_T c6_n;
  real_T c6_i;
  real_T c6_nargin = 3.0;
  real_T c6_nargout = 1.0;
  real_T c6_A;
  real_T c6_B;
  real_T c6_x;
  real_T c6_y;
  real_T c6_b_x;
  real_T c6_b_y;
  real_T c6_dv3[1];
  int32_T c6_tmp_sizes[2];
  int32_T c6_tmp_data[1];
  int32_T c6_loop_ub;
  int32_T c6_i28;
  real_T c6_dv4[1];
  int32_T c6_b_loop_ub;
  int32_T c6_i29;
  real_T c6_a;
  int32_T c6_b_i;
  real_T c6_k;
  real_T c6_c_x;
  real_T c6_d_x;
  boolean_T c6_b0;
  int32_T c6_i30;
  static char_T c6_cv0[27] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'n', 'c', 'h',
    'o', 'o', 's', 'e', 'k', ':', 'I', 'n', 'v', 'a', 'l', 'i', 'd', 'A', 'r',
    'g', '2' };

  char_T c6_u[27];
  const mxArray *c6_c_y = NULL;
  real_T c6_b_k;
  real_T c6_d_y;
  real_T c6_nmk;
  real_T c6_c_k;
  int32_T c6_i31;
  int32_T c6_j;
  real_T c6_b_j;
  real_T c6_b_a;
  real_T c6_b;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 10U, 10U, c6_b_debug_family_names,
    c6_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_s, 0U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_q_dep, 1U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c6_n, 2U, c6_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_i, 3U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_nargin, 4U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_nargout, 5U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_theta_p, 6U, c6_c_sf_marshallOut,
    c6_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_alpha_p, 7U, c6_c_sf_marshallOut,
    c6_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_theta, 8U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_q, 9U, c6_b_sf_marshallOut,
    c6_b_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 5);
  c6_A = c6_theta - c6_theta_p[0];
  c6_B = c6_theta_p[4] - c6_theta_p[0];
  c6_x = c6_A;
  c6_y = c6_B;
  c6_b_x = c6_x;
  c6_b_y = c6_y;
  c6_s = c6_b_x / c6_b_y;
  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 6);
  c6_dv3[0] = c6_s;
  c6_eml_li_find(chartInstance, c6_s > 1.0, c6_tmp_data, c6_tmp_sizes);
  c6_loop_ub = c6_tmp_sizes[0] * c6_tmp_sizes[1] - 1;
  for (c6_i28 = 0; c6_i28 <= c6_loop_ub; c6_i28++) {
    c6_dv3[c6_tmp_data[c6_i28] - 1] = 1.0;
  }

  c6_s = c6_dv3[0];
  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 7);
  c6_dv4[0] = c6_s;
  c6_eml_li_find(chartInstance, c6_s < 0.0, c6_tmp_data, c6_tmp_sizes);
  c6_b_loop_ub = c6_tmp_sizes[0] * c6_tmp_sizes[1] - 1;
  for (c6_i29 = 0; c6_i29 <= c6_b_loop_ub; c6_i29++) {
    c6_dv4[c6_tmp_data[c6_i29] - 1] = 0.0;
  }

  c6_s = c6_dv4[0];
  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 8);
  c6_a = c6_s;
  c6_s = c6_a;
  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 9);
  c6_q_dep = 0.0;
  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 10);
  c6_n = 4.0;
  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 11);
  c6_i = 0.0;
  c6_b_i = 0;
  while (c6_b_i < 5) {
    c6_i = (real_T)c6_b_i;
    CV_SCRIPT_FOR(0, 0, 1);
    _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 12);
    c6_k = c6_i;
    c6_c_x = c6_k;
    c6_d_x = c6_c_x;
    c6_d_x = muDoubleScalarFloor(c6_d_x);
    c6_b0 = (c6_k == c6_d_x);
    if (c6_b0) {
    } else {
      for (c6_i30 = 0; c6_i30 < 27; c6_i30++) {
        c6_u[c6_i30] = c6_cv0[c6_i30];
      }

      c6_c_y = NULL;
      sf_mex_assign(&c6_c_y, sf_mex_create("y", c6_u, 10, 0U, 1U, 0U, 2, 1, 27),
                    FALSE);
      sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U,
        14, c6_c_y));
    }

    c6_b_k = c6_k;
    c6_eml_scalar_eg(chartInstance);
    if (c6_b_k > 2.0) {
      c6_b_k = 4.0 - c6_b_k;
    }

    c6_d_y = 1.0;
    c6_nmk = 4.0 - c6_b_k;
    c6_c_k = c6_b_k;
    c6_i31 = (int32_T)c6_c_k - 1;
    for (c6_j = 0; c6_j <= c6_i31; c6_j++) {
      c6_b_j = 1.0 + (real_T)c6_j;
      c6_b_a = c6_d_y;
      c6_b = (c6_b_j + c6_nmk) / c6_b_j;
      c6_d_y = c6_b_a * c6_b;
    }

    c6_d_y = muDoubleScalarRound(c6_d_y);
    if (!(c6_d_y <= 9.007199254740992E+15)) {
      c6_eml_warning(chartInstance);
    }

    c6_q_dep += c6_d_y * c6_power(chartInstance, 1.0 - c6_s, 4.0 - c6_i) *
      c6_power(chartInstance, c6_s, c6_i) * c6_alpha_p[(int32_T)(c6_i + 1.0) - 1];
    c6_b_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  CV_SCRIPT_FOR(0, 0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 17);
  c6_q[0] = c6_q_dep;
  c6_q[1] = c6_theta;
  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, -17);
  _SFD_SYMBOL_SCOPE_POP();
}

static void init_script_number_translation(uint32_T c6_machineNumber, uint32_T
  c6_chartNumber)
{
  _SFD_SCRIPT_TRANSLATION(c6_chartNumber, 0U, sf_debug_get_script_id(
    "C:/Users/yak/My Documents/GitHub/thesis/MATLAB/bezConstraint.m"));
  _SFD_SCRIPT_TRANSLATION(c6_chartNumber, 1U, sf_debug_get_script_id(
    "C:/Users/yak/My Documents/GitHub/thesis/MATLAB/actuated.m"));
}

static const mxArray *c6_sf_marshallOut(void *chartInstanceVoid, void *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  real_T c6_u;
  const mxArray *c6_y = NULL;
  SFc6_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc6_RobotSimInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_u = *(real_T *)c6_inData;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static real_T c6_emlrt_marshallIn(SFc6_RobotSimInstanceStruct *chartInstance,
  const mxArray *c6_err, const char_T *c6_identifier)
{
  real_T c6_y;
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_err), &c6_thisId);
  sf_mex_destroy(&c6_err);
  return c6_y;
}

static real_T c6_b_emlrt_marshallIn(SFc6_RobotSimInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  real_T c6_y;
  real_T c6_d0;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_d0, 1, 0, 0U, 0, 0U, 0);
  c6_y = c6_d0;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_err;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y;
  SFc6_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc6_RobotSimInstanceStruct *)chartInstanceVoid;
  c6_err = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_err), &c6_thisId);
  sf_mex_destroy(&c6_err);
  *(real_T *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_b_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i32;
  real_T c6_b_inData[2];
  int32_T c6_i33;
  real_T c6_u[2];
  const mxArray *c6_y = NULL;
  SFc6_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc6_RobotSimInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i32 = 0; c6_i32 < 2; c6_i32++) {
    c6_b_inData[c6_i32] = (*(real_T (*)[2])c6_inData)[c6_i32];
  }

  for (c6_i33 = 0; c6_i33 < 2; c6_i33++) {
    c6_u[c6_i33] = c6_b_inData[c6_i33];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 1, 2), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static const mxArray *c6_c_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i34;
  real_T c6_b_inData[5];
  int32_T c6_i35;
  real_T c6_u[5];
  const mxArray *c6_y = NULL;
  SFc6_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc6_RobotSimInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i34 = 0; c6_i34 < 5; c6_i34++) {
    c6_b_inData[c6_i34] = (*(real_T (*)[5])c6_inData)[c6_i34];
  }

  for (c6_i35 = 0; c6_i35 < 5; c6_i35++) {
    c6_u[c6_i35] = c6_b_inData[c6_i35];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 1, 5), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static void c6_c_emlrt_marshallIn(SFc6_RobotSimInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId, real_T c6_y[2])
{
  real_T c6_dv5[2];
  int32_T c6_i36;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv5, 1, 0, 0U, 1, 0U, 1, 2);
  for (c6_i36 = 0; c6_i36 < 2; c6_i36++) {
    c6_y[c6_i36] = c6_dv5[c6_i36];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_q_des;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[2];
  int32_T c6_i37;
  SFc6_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc6_RobotSimInstanceStruct *)chartInstanceVoid;
  c6_q_des = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_q_des), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_q_des);
  for (c6_i37 = 0; c6_i37 < 2; c6_i37++) {
    (*(real_T (*)[2])c6_outData)[c6_i37] = c6_y[c6_i37];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static void c6_d_emlrt_marshallIn(SFc6_RobotSimInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId, real_T c6_y[5])
{
  real_T c6_dv6[5];
  int32_T c6_i38;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv6, 1, 0, 0U, 1, 0U, 2, 1, 5);
  for (c6_i38 = 0; c6_i38 < 5; c6_i38++) {
    c6_y[c6_i38] = c6_dv6[c6_i38];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_alpha_p;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[5];
  int32_T c6_i39;
  SFc6_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc6_RobotSimInstanceStruct *)chartInstanceVoid;
  c6_alpha_p = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_alpha_p), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_alpha_p);
  for (c6_i39 = 0; c6_i39 < 5; c6_i39++) {
    (*(real_T (*)[5])c6_outData)[c6_i39] = c6_y[c6_i39];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

const mxArray *sf_c6_RobotSim_get_eml_resolved_functions_info(void)
{
  const mxArray *c6_nameCaptureInfo;
  c6_ResolvedFunctionInfo c6_info[60];
  const mxArray *c6_m0 = NULL;
  int32_T c6_i40;
  c6_ResolvedFunctionInfo *c6_r0;
  c6_nameCaptureInfo = NULL;
  c6_nameCaptureInfo = NULL;
  c6_info_helper(c6_info);
  sf_mex_assign(&c6_m0, sf_mex_createstruct("nameCaptureInfo", 1, 60), FALSE);
  for (c6_i40 = 0; c6_i40 < 60; c6_i40++) {
    c6_r0 = &c6_info[c6_i40];
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", c6_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c6_r0->context)), "context", "nameCaptureInfo",
                    c6_i40);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", c6_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c6_r0->name)), "name", "nameCaptureInfo", c6_i40);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", c6_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c6_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c6_i40);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", c6_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c6_r0->resolved)), "resolved", "nameCaptureInfo",
                    c6_i40);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", &c6_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c6_i40);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", &c6_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c6_i40);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", &c6_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c6_i40);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", &c6_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c6_i40);
  }

  sf_mex_assign(&c6_nameCaptureInfo, c6_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c6_nameCaptureInfo);
  return c6_nameCaptureInfo;
}

static void c6_info_helper(c6_ResolvedFunctionInfo c6_info[60])
{
  c6_info[0].context = "";
  c6_info[0].name = "bezConstraint";
  c6_info[0].dominantType = "double";
  c6_info[0].resolved =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/bezConstraint.m";
  c6_info[0].fileTimeLo = 1410865620U;
  c6_info[0].fileTimeHi = 0U;
  c6_info[0].mFileTimeLo = 0U;
  c6_info[0].mFileTimeHi = 0U;
  c6_info[1].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/bezConstraint.m";
  c6_info[1].name = "mrdivide";
  c6_info[1].dominantType = "double";
  c6_info[1].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c6_info[1].fileTimeLo = 1357915548U;
  c6_info[1].fileTimeHi = 0U;
  c6_info[1].mFileTimeLo = 1319697566U;
  c6_info[1].mFileTimeHi = 0U;
  c6_info[2].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c6_info[2].name = "rdivide";
  c6_info[2].dominantType = "double";
  c6_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c6_info[2].fileTimeLo = 1346481588U;
  c6_info[2].fileTimeHi = 0U;
  c6_info[2].mFileTimeLo = 0U;
  c6_info[2].mFileTimeHi = 0U;
  c6_info[3].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c6_info[3].name = "eml_scalexp_compatible";
  c6_info[3].dominantType = "double";
  c6_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m";
  c6_info[3].fileTimeLo = 1286786396U;
  c6_info[3].fileTimeHi = 0U;
  c6_info[3].mFileTimeLo = 0U;
  c6_info[3].mFileTimeHi = 0U;
  c6_info[4].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c6_info[4].name = "eml_div";
  c6_info[4].dominantType = "double";
  c6_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c6_info[4].fileTimeLo = 1313319010U;
  c6_info[4].fileTimeHi = 0U;
  c6_info[4].mFileTimeLo = 0U;
  c6_info[4].mFileTimeHi = 0U;
  c6_info[5].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/bezConstraint.m";
  c6_info[5].name = "eml_li_find";
  c6_info[5].dominantType = "";
  c6_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m";
  c6_info[5].fileTimeLo = 1286786386U;
  c6_info[5].fileTimeHi = 0U;
  c6_info[5].mFileTimeLo = 0U;
  c6_info[5].mFileTimeHi = 0U;
  c6_info[6].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m";
  c6_info[6].name = "eml_index_class";
  c6_info[6].dominantType = "";
  c6_info[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[6].fileTimeLo = 1323134578U;
  c6_info[6].fileTimeHi = 0U;
  c6_info[6].mFileTimeLo = 0U;
  c6_info[6].mFileTimeHi = 0U;
  c6_info[7].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m!compute_nones";
  c6_info[7].name = "eml_index_class";
  c6_info[7].dominantType = "";
  c6_info[7].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[7].fileTimeLo = 1323134578U;
  c6_info[7].fileTimeHi = 0U;
  c6_info[7].mFileTimeLo = 0U;
  c6_info[7].mFileTimeHi = 0U;
  c6_info[8].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m!compute_nones";
  c6_info[8].name = "eml_index_plus";
  c6_info[8].dominantType = "double";
  c6_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c6_info[8].fileTimeLo = 1286786378U;
  c6_info[8].fileTimeHi = 0U;
  c6_info[8].mFileTimeLo = 0U;
  c6_info[8].mFileTimeHi = 0U;
  c6_info[9].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c6_info[9].name = "eml_index_class";
  c6_info[9].dominantType = "";
  c6_info[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[9].fileTimeLo = 1323134578U;
  c6_info[9].fileTimeHi = 0U;
  c6_info[9].mFileTimeLo = 0U;
  c6_info[9].mFileTimeHi = 0U;
  c6_info[10].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m";
  c6_info[10].name = "eml_index_plus";
  c6_info[10].dominantType = "double";
  c6_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c6_info[10].fileTimeLo = 1286786378U;
  c6_info[10].fileTimeHi = 0U;
  c6_info[10].mFileTimeLo = 0U;
  c6_info[10].mFileTimeHi = 0U;
  c6_info[11].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/bezConstraint.m";
  c6_info[11].name = "repmat";
  c6_info[11].dominantType = "double";
  c6_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/repmat.m";
  c6_info[11].fileTimeLo = 1352388860U;
  c6_info[11].fileTimeHi = 0U;
  c6_info[11].mFileTimeLo = 0U;
  c6_info[11].mFileTimeHi = 0U;
  c6_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/repmat.m";
  c6_info[12].name = "eml_assert_valid_size_arg";
  c6_info[12].dominantType = "double";
  c6_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c6_info[12].fileTimeLo = 1286786294U;
  c6_info[12].fileTimeHi = 0U;
  c6_info[12].mFileTimeLo = 0U;
  c6_info[12].mFileTimeHi = 0U;
  c6_info[13].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isintegral";
  c6_info[13].name = "isinf";
  c6_info[13].dominantType = "double";
  c6_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m";
  c6_info[13].fileTimeLo = 1286786360U;
  c6_info[13].fileTimeHi = 0U;
  c6_info[13].mFileTimeLo = 0U;
  c6_info[13].mFileTimeHi = 0U;
  c6_info[14].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!numel_for_size";
  c6_info[14].name = "mtimes";
  c6_info[14].dominantType = "double";
  c6_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[14].fileTimeLo = 1289483692U;
  c6_info[14].fileTimeHi = 0U;
  c6_info[14].mFileTimeLo = 0U;
  c6_info[14].mFileTimeHi = 0U;
  c6_info[15].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c6_info[15].name = "eml_index_class";
  c6_info[15].dominantType = "";
  c6_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[15].fileTimeLo = 1323134578U;
  c6_info[15].fileTimeHi = 0U;
  c6_info[15].mFileTimeLo = 0U;
  c6_info[15].mFileTimeHi = 0U;
  c6_info[16].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c6_info[16].name = "intmax";
  c6_info[16].dominantType = "char";
  c6_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c6_info[16].fileTimeLo = 1311226516U;
  c6_info[16].fileTimeHi = 0U;
  c6_info[16].mFileTimeLo = 0U;
  c6_info[16].mFileTimeHi = 0U;
  c6_info[17].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/repmat.m";
  c6_info[17].name = "eml_index_class";
  c6_info[17].dominantType = "";
  c6_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[17].fileTimeLo = 1323134578U;
  c6_info[17].fileTimeHi = 0U;
  c6_info[17].mFileTimeLo = 0U;
  c6_info[17].mFileTimeHi = 0U;
  c6_info[18].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/repmat.m";
  c6_info[18].name = "eml_index_minus";
  c6_info[18].dominantType = "coder.internal.indexInt";
  c6_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c6_info[18].fileTimeLo = 1286786378U;
  c6_info[18].fileTimeHi = 0U;
  c6_info[18].mFileTimeLo = 0U;
  c6_info[18].mFileTimeHi = 0U;
  c6_info[19].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c6_info[19].name = "eml_index_class";
  c6_info[19].dominantType = "";
  c6_info[19].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[19].fileTimeLo = 1323134578U;
  c6_info[19].fileTimeHi = 0U;
  c6_info[19].mFileTimeLo = 0U;
  c6_info[19].mFileTimeHi = 0U;
  c6_info[20].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/bezConstraint.m";
  c6_info[20].name = "length";
  c6_info[20].dominantType = "double";
  c6_info[20].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c6_info[20].fileTimeLo = 1303117406U;
  c6_info[20].fileTimeHi = 0U;
  c6_info[20].mFileTimeLo = 0U;
  c6_info[20].mFileTimeHi = 0U;
  c6_info[21].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/bezConstraint.m";
  c6_info[21].name = "nchoosek";
  c6_info[21].dominantType = "double";
  c6_info[21].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m";
  c6_info[21].fileTimeLo = 1352388862U;
  c6_info[21].fileTimeHi = 0U;
  c6_info[21].mFileTimeLo = 0U;
  c6_info[21].mFileTimeHi = 0U;
  c6_info[22].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m";
  c6_info[22].name = "floor";
  c6_info[22].dominantType = "double";
  c6_info[22].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c6_info[22].fileTimeLo = 1343801580U;
  c6_info[22].fileTimeHi = 0U;
  c6_info[22].mFileTimeLo = 0U;
  c6_info[22].mFileTimeHi = 0U;
  c6_info[23].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c6_info[23].name = "eml_scalar_floor";
  c6_info[23].dominantType = "double";
  c6_info[23].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c6_info[23].fileTimeLo = 1286786326U;
  c6_info[23].fileTimeHi = 0U;
  c6_info[23].mFileTimeLo = 0U;
  c6_info[23].mFileTimeHi = 0U;
  c6_info[24].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m";
  c6_info[24].name = "eml_error";
  c6_info[24].dominantType = "char";
  c6_info[24].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c6_info[24].fileTimeLo = 1343801558U;
  c6_info[24].fileTimeHi = 0U;
  c6_info[24].mFileTimeLo = 0U;
  c6_info[24].mFileTimeHi = 0U;
  c6_info[25].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m";
  c6_info[25].name = "eml_scalar_eg";
  c6_info[25].dominantType = "double";
  c6_info[25].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[25].fileTimeLo = 1286786396U;
  c6_info[25].fileTimeHi = 0U;
  c6_info[25].mFileTimeLo = 0U;
  c6_info[25].mFileTimeHi = 0U;
  c6_info[26].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m";
  c6_info[26].name = "eml_guarded_nan";
  c6_info[26].dominantType = "";
  c6_info[26].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m";
  c6_info[26].fileTimeLo = 1286786376U;
  c6_info[26].fileTimeHi = 0U;
  c6_info[26].mFileTimeLo = 0U;
  c6_info[26].mFileTimeHi = 0U;
  c6_info[27].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
  c6_info[27].name = "eml_scalar_eg";
  c6_info[27].dominantType = "double";
  c6_info[27].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[27].fileTimeLo = 1286786396U;
  c6_info[27].fileTimeHi = 0U;
  c6_info[27].mFileTimeLo = 0U;
  c6_info[27].mFileTimeHi = 0U;
  c6_info[28].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
  c6_info[28].name = "isfinite";
  c6_info[28].dominantType = "double";
  c6_info[28].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c6_info[28].fileTimeLo = 1286786358U;
  c6_info[28].fileTimeHi = 0U;
  c6_info[28].mFileTimeLo = 0U;
  c6_info[28].mFileTimeHi = 0U;
  c6_info[29].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c6_info[29].name = "isinf";
  c6_info[29].dominantType = "double";
  c6_info[29].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m";
  c6_info[29].fileTimeLo = 1286786360U;
  c6_info[29].fileTimeHi = 0U;
  c6_info[29].mFileTimeLo = 0U;
  c6_info[29].mFileTimeHi = 0U;
  c6_info[30].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c6_info[30].name = "isnan";
  c6_info[30].dominantType = "double";
  c6_info[30].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m";
  c6_info[30].fileTimeLo = 1286786360U;
  c6_info[30].fileTimeHi = 0U;
  c6_info[30].mFileTimeLo = 0U;
  c6_info[30].mFileTimeHi = 0U;
  c6_info[31].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
  c6_info[31].name = "eml_guarded_inf";
  c6_info[31].dominantType = "";
  c6_info[31].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_inf.m";
  c6_info[31].fileTimeLo = 1286786376U;
  c6_info[31].fileTimeHi = 0U;
  c6_info[31].mFileTimeLo = 0U;
  c6_info[31].mFileTimeHi = 0U;
  c6_info[32].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
  c6_info[32].name = "mtimes";
  c6_info[32].dominantType = "double";
  c6_info[32].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[32].fileTimeLo = 1289483692U;
  c6_info[32].fileTimeHi = 0U;
  c6_info[32].mFileTimeLo = 0U;
  c6_info[32].mFileTimeHi = 0U;
  c6_info[33].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
  c6_info[33].name = "round";
  c6_info[33].dominantType = "double";
  c6_info[33].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/round.m";
  c6_info[33].fileTimeLo = 1343801584U;
  c6_info[33].fileTimeHi = 0U;
  c6_info[33].mFileTimeLo = 0U;
  c6_info[33].mFileTimeHi = 0U;
  c6_info[34].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/round.m";
  c6_info[34].name = "eml_scalar_round";
  c6_info[34].dominantType = "double";
  c6_info[34].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_round.m";
  c6_info[34].fileTimeLo = 1307622438U;
  c6_info[34].fileTimeHi = 0U;
  c6_info[34].mFileTimeLo = 0U;
  c6_info[34].mFileTimeHi = 0U;
  c6_info[35].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
  c6_info[35].name = "eml_guarded_nan";
  c6_info[35].dominantType = "";
  c6_info[35].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m";
  c6_info[35].fileTimeLo = 1286786376U;
  c6_info[35].fileTimeHi = 0U;
  c6_info[35].mFileTimeLo = 0U;
  c6_info[35].mFileTimeHi = 0U;
  c6_info[36].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
  c6_info[36].name = "flintmax";
  c6_info[36].dominantType = "char";
  c6_info[36].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/flintmax.m";
  c6_info[36].fileTimeLo = 1348163116U;
  c6_info[36].fileTimeHi = 0U;
  c6_info[36].mFileTimeLo = 0U;
  c6_info[36].mFileTimeHi = 0U;
  c6_info[37].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/flintmax.m";
  c6_info[37].name = "eml_float_model";
  c6_info[37].dominantType = "char";
  c6_info[37].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c6_info[37].fileTimeLo = 1326691996U;
  c6_info[37].fileTimeHi = 0U;
  c6_info[37].mFileTimeLo = 0U;
  c6_info[37].mFileTimeHi = 0U;
  c6_info[38].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
  c6_info[38].name = "eml_warning";
  c6_info[38].dominantType = "char";
  c6_info[38].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c6_info[38].fileTimeLo = 1286786402U;
  c6_info[38].fileTimeHi = 0U;
  c6_info[38].mFileTimeLo = 0U;
  c6_info[38].mFileTimeHi = 0U;
  c6_info[39].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/bezConstraint.m";
  c6_info[39].name = "power";
  c6_info[39].dominantType = "double";
  c6_info[39].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c6_info[39].fileTimeLo = 1348163130U;
  c6_info[39].fileTimeHi = 0U;
  c6_info[39].mFileTimeLo = 0U;
  c6_info[39].mFileTimeHi = 0U;
  c6_info[40].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c6_info[40].name = "eml_scalar_eg";
  c6_info[40].dominantType = "double";
  c6_info[40].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[40].fileTimeLo = 1286786396U;
  c6_info[40].fileTimeHi = 0U;
  c6_info[40].mFileTimeLo = 0U;
  c6_info[40].mFileTimeHi = 0U;
  c6_info[41].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c6_info[41].name = "eml_scalexp_alloc";
  c6_info[41].dominantType = "double";
  c6_info[41].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c6_info[41].fileTimeLo = 1352388860U;
  c6_info[41].fileTimeHi = 0U;
  c6_info[41].mFileTimeLo = 0U;
  c6_info[41].mFileTimeHi = 0U;
  c6_info[42].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c6_info[42].name = "floor";
  c6_info[42].dominantType = "double";
  c6_info[42].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c6_info[42].fileTimeLo = 1343801580U;
  c6_info[42].fileTimeHi = 0U;
  c6_info[42].mFileTimeLo = 0U;
  c6_info[42].mFileTimeHi = 0U;
  c6_info[43].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c6_info[43].name = "eml_error";
  c6_info[43].dominantType = "char";
  c6_info[43].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c6_info[43].fileTimeLo = 1343801558U;
  c6_info[43].fileTimeHi = 0U;
  c6_info[43].mFileTimeLo = 0U;
  c6_info[43].mFileTimeHi = 0U;
  c6_info[44].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power";
  c6_info[44].name = "eml_scalar_eg";
  c6_info[44].dominantType = "double";
  c6_info[44].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[44].fileTimeLo = 1286786396U;
  c6_info[44].fileTimeHi = 0U;
  c6_info[44].mFileTimeLo = 0U;
  c6_info[44].mFileTimeHi = 0U;
  c6_info[45].context = "";
  c6_info[45].name = "actuated";
  c6_info[45].dominantType = "double";
  c6_info[45].resolved =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/actuated.m";
  c6_info[45].fileTimeLo = 1410873059U;
  c6_info[45].fileTimeHi = 0U;
  c6_info[45].mFileTimeLo = 0U;
  c6_info[45].mFileTimeHi = 0U;
  c6_info[46].context = "";
  c6_info[46].name = "mtimes";
  c6_info[46].dominantType = "double";
  c6_info[46].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[46].fileTimeLo = 1289483692U;
  c6_info[46].fileTimeHi = 0U;
  c6_info[46].mFileTimeLo = 0U;
  c6_info[46].mFileTimeHi = 0U;
  c6_info[47].context = "";
  c6_info[47].name = "mod";
  c6_info[47].dominantType = "double";
  c6_info[47].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  c6_info[47].fileTimeLo = 1343801582U;
  c6_info[47].fileTimeHi = 0U;
  c6_info[47].mFileTimeLo = 0U;
  c6_info[47].mFileTimeHi = 0U;
  c6_info[48].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  c6_info[48].name = "eml_scalar_eg";
  c6_info[48].dominantType = "double";
  c6_info[48].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[48].fileTimeLo = 1286786396U;
  c6_info[48].fileTimeHi = 0U;
  c6_info[48].mFileTimeLo = 0U;
  c6_info[48].mFileTimeHi = 0U;
  c6_info[49].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  c6_info[49].name = "eml_scalexp_alloc";
  c6_info[49].dominantType = "double";
  c6_info[49].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c6_info[49].fileTimeLo = 1352388860U;
  c6_info[49].fileTimeHi = 0U;
  c6_info[49].mFileTimeLo = 0U;
  c6_info[49].mFileTimeHi = 0U;
  c6_info[50].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m!floatmod";
  c6_info[50].name = "eml_scalar_eg";
  c6_info[50].dominantType = "double";
  c6_info[50].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[50].fileTimeLo = 1286786396U;
  c6_info[50].fileTimeHi = 0U;
  c6_info[50].mFileTimeLo = 0U;
  c6_info[50].mFileTimeHi = 0U;
  c6_info[51].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m!floatmod";
  c6_info[51].name = "eml_scalar_floor";
  c6_info[51].dominantType = "double";
  c6_info[51].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c6_info[51].fileTimeLo = 1286786326U;
  c6_info[51].fileTimeHi = 0U;
  c6_info[51].mFileTimeLo = 0U;
  c6_info[51].mFileTimeHi = 0U;
  c6_info[52].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m!floatmod";
  c6_info[52].name = "eml_scalar_round";
  c6_info[52].dominantType = "double";
  c6_info[52].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_round.m";
  c6_info[52].fileTimeLo = 1307622438U;
  c6_info[52].fileTimeHi = 0U;
  c6_info[52].mFileTimeLo = 0U;
  c6_info[52].mFileTimeHi = 0U;
  c6_info[53].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m!floatmod";
  c6_info[53].name = "eml_scalar_abs";
  c6_info[53].dominantType = "double";
  c6_info[53].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c6_info[53].fileTimeLo = 1286786312U;
  c6_info[53].fileTimeHi = 0U;
  c6_info[53].mFileTimeLo = 0U;
  c6_info[53].mFileTimeHi = 0U;
  c6_info[54].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m!floatmod";
  c6_info[54].name = "eps";
  c6_info[54].dominantType = "char";
  c6_info[54].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c6_info[54].fileTimeLo = 1326691996U;
  c6_info[54].fileTimeHi = 0U;
  c6_info[54].mFileTimeLo = 0U;
  c6_info[54].mFileTimeHi = 0U;
  c6_info[55].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c6_info[55].name = "eml_is_float_class";
  c6_info[55].dominantType = "char";
  c6_info[55].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c6_info[55].fileTimeLo = 1286786382U;
  c6_info[55].fileTimeHi = 0U;
  c6_info[55].mFileTimeLo = 0U;
  c6_info[55].mFileTimeHi = 0U;
  c6_info[56].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c6_info[56].name = "eml_eps";
  c6_info[56].dominantType = "char";
  c6_info[56].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c6_info[56].fileTimeLo = 1326691996U;
  c6_info[56].fileTimeHi = 0U;
  c6_info[56].mFileTimeLo = 0U;
  c6_info[56].mFileTimeHi = 0U;
  c6_info[57].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c6_info[57].name = "eml_float_model";
  c6_info[57].dominantType = "char";
  c6_info[57].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c6_info[57].fileTimeLo = 1326691996U;
  c6_info[57].fileTimeHi = 0U;
  c6_info[57].mFileTimeLo = 0U;
  c6_info[57].mFileTimeHi = 0U;
  c6_info[58].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m!floatmod";
  c6_info[58].name = "mtimes";
  c6_info[58].dominantType = "double";
  c6_info[58].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[58].fileTimeLo = 1289483692U;
  c6_info[58].fileTimeHi = 0U;
  c6_info[58].mFileTimeLo = 0U;
  c6_info[58].mFileTimeHi = 0U;
  c6_info[59].context = "";
  c6_info[59].name = "eml_li_find";
  c6_info[59].dominantType = "";
  c6_info[59].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m";
  c6_info[59].fileTimeLo = 1286786386U;
  c6_info[59].fileTimeHi = 0U;
  c6_info[59].mFileTimeLo = 0U;
  c6_info[59].mFileTimeHi = 0U;
}

static void c6_eml_li_find(SFc6_RobotSimInstanceStruct *chartInstance, boolean_T
  c6_x, int32_T c6_y_data[1], int32_T c6_y_sizes[2])
{
  boolean_T c6_b_x;
  int32_T c6_k;
  int32_T c6_tmp_sizes[2];
  int32_T c6_iv0[2];
  int32_T c6_i41;
  int32_T c6_i42;
  int32_T c6_loop_ub;
  int32_T c6_i43;
  int32_T c6_tmp_data[1];
  int32_T c6_i44;
  c6_b_x = c6_x;
  c6_k = 0;
  if (c6_b_x) {
    c6_k = 1;
  }

  c6_tmp_sizes[0] = 1;
  c6_iv0[0] = 1;
  c6_iv0[1] = c6_k;
  c6_tmp_sizes[1] = c6_iv0[1];
  c6_i41 = c6_tmp_sizes[0];
  c6_i42 = c6_tmp_sizes[1];
  c6_loop_ub = c6_k - 1;
  for (c6_i43 = 0; c6_i43 <= c6_loop_ub; c6_i43++) {
    c6_tmp_data[c6_i43] = 0;
  }

  for (c6_i44 = 0; c6_i44 < 2; c6_i44++) {
    c6_y_sizes[c6_i44] = c6_tmp_sizes[c6_i44];
  }

  if (c6_x) {
    _SFD_EML_ARRAY_BOUNDS_CHECK("", 1, 1, c6_y_sizes[1], 1, 0);
    c6_y_data[0] = 1;
  }
}

static void c6_eml_scalar_eg(SFc6_RobotSimInstanceStruct *chartInstance)
{
}

static void c6_eml_warning(SFc6_RobotSimInstanceStruct *chartInstance)
{
  int32_T c6_i45;
  static char_T c6_varargin_1[38] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 'n', 'c', 'h', 'o', 'o', 's', 'e', 'k', '_', 'L',
    'a', 'r', 'g', 'e', 'C', 'o', 'e', 'f', 'f', 'i', 'c', 'i', 'e', 'n', 't' };

  char_T c6_u[38];
  const mxArray *c6_y = NULL;
  for (c6_i45 = 0; c6_i45 < 38; c6_i45++) {
    c6_u[c6_i45] = c6_varargin_1[c6_i45];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 10, 0U, 1U, 0U, 2, 1, 38), FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U,
    14, c6_y));
}

static real_T c6_power(SFc6_RobotSimInstanceStruct *chartInstance, real_T c6_a,
  real_T c6_b)
{
  real_T c6_b_a;
  real_T c6_b_b;
  real_T c6_ak;
  real_T c6_bk;
  real_T c6_x;
  real_T c6_b_x;
  real_T c6_c_a;
  real_T c6_c_b;
  real_T c6_ar;
  real_T c6_br;
  c6_b_a = c6_a;
  c6_b_b = c6_b;
  c6_eml_scalar_eg(chartInstance);
  c6_ak = c6_b_a;
  c6_bk = c6_b_b;
  if (c6_ak < 0.0) {
    c6_x = c6_bk;
    c6_b_x = c6_x;
    c6_b_x = muDoubleScalarFloor(c6_b_x);
    if (c6_b_x != c6_bk) {
      c6_eml_error(chartInstance);
    }
  }

  c6_c_a = c6_ak;
  c6_c_b = c6_bk;
  c6_eml_scalar_eg(chartInstance);
  c6_ar = c6_c_a;
  c6_br = c6_c_b;
  return muDoubleScalarPower(c6_ar, c6_br);
}

static void c6_eml_error(SFc6_RobotSimInstanceStruct *chartInstance)
{
  int32_T c6_i46;
  static char_T c6_cv1[31] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'p', 'o', 'w', 'e', 'r', '_', 'd', 'o', 'm', 'a', 'i',
    'n', 'E', 'r', 'r', 'o', 'r' };

  char_T c6_u[31];
  const mxArray *c6_y = NULL;
  for (c6_i46 = 0; c6_i46 < 31; c6_i46++) {
    c6_u[c6_i46] = c6_cv1[c6_i46];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 10, 0U, 1U, 0U, 2, 1, 31), FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U, 14,
    c6_y));
}

static const mxArray *c6_d_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_u;
  const mxArray *c6_y = NULL;
  SFc6_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc6_RobotSimInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_u = *(int32_T *)c6_inData;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static int32_T c6_e_emlrt_marshallIn(SFc6_RobotSimInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  int32_T c6_y;
  int32_T c6_i47;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_i47, 1, 6, 0U, 0, 0U, 0);
  c6_y = c6_i47;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_b_sfEvent;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  int32_T c6_y;
  SFc6_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc6_RobotSimInstanceStruct *)chartInstanceVoid;
  c6_b_sfEvent = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_sfEvent),
    &c6_thisId);
  sf_mex_destroy(&c6_b_sfEvent);
  *(int32_T *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

static uint8_T c6_f_emlrt_marshallIn(SFc6_RobotSimInstanceStruct *chartInstance,
  const mxArray *c6_b_is_active_c6_RobotSim, const char_T *c6_identifier)
{
  uint8_T c6_y;
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_g_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c6_b_is_active_c6_RobotSim), &c6_thisId);
  sf_mex_destroy(&c6_b_is_active_c6_RobotSim);
  return c6_y;
}

static uint8_T c6_g_emlrt_marshallIn(SFc6_RobotSimInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  uint8_T c6_y;
  uint8_T c6_u0;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_u0, 1, 3, 0U, 0, 0U, 0);
  c6_y = c6_u0;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void init_dsm_address_info(SFc6_RobotSimInstanceStruct *chartInstance)
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

void sf_c6_RobotSim_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1675933881U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2249325385U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1973581209U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1727523466U);
}

mxArray *sf_c6_RobotSim_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("P7nI9SMXP7pmQVioToRzyF");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(5);
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
      pr[0] = (double)(1);
      pr[1] = (double)(5);
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

mxArray *sf_c6_RobotSim_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c6_RobotSim(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[6],T\"err\",},{M[8],M[0],T\"is_active_c6_RobotSim\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c6_RobotSim_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc6_RobotSimInstanceStruct *chartInstance;
    chartInstance = (SFc6_RobotSimInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _RobotSimMachineNumber_,
           6,
           1,
           1,
           5,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"theta_p");
          _SFD_SET_DATA_PROPS(1,1,1,0,"alpha_p");
          _SFD_SET_DATA_PROPS(2,1,1,0,"q");
          _SFD_SET_DATA_PROPS(3,1,1,0,"theta");
          _SFD_SET_DATA_PROPS(4,2,0,1,"err");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,375);
        _SFD_CV_INIT_SCRIPT(0,1,0,0,0,0,1,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(0,0,"bezConstraint",0,-1,517);
        _SFD_CV_INIT_SCRIPT_FOR(0,0,344,358,436);
        _SFD_CV_INIT_SCRIPT(1,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(1,0,"actuated",0,-1,175);
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
          dimVector[1]= 5;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 5;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c6_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c6_sf_marshallOut,(MexInFcnForType)c6_sf_marshallIn);

        {
          real_T *c6_theta;
          real_T *c6_err;
          real_T (*c6_theta_p)[5];
          real_T (*c6_alpha_p)[5];
          real_T (*c6_q)[2];
          c6_err = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          c6_theta = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c6_q = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 2);
          c6_alpha_p = (real_T (*)[5])ssGetInputPortSignal(chartInstance->S, 1);
          c6_theta_p = (real_T (*)[5])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c6_theta_p);
          _SFD_SET_DATA_VALUE_PTR(1U, *c6_alpha_p);
          _SFD_SET_DATA_VALUE_PTR(2U, *c6_q);
          _SFD_SET_DATA_VALUE_PTR(3U, c6_theta);
          _SFD_SET_DATA_VALUE_PTR(4U, c6_err);
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
  return "FJIHPk08zCS1HrSHV7uWwC";
}

static void sf_opaque_initialize_c6_RobotSim(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc6_RobotSimInstanceStruct*) chartInstanceVar)
    ->S,0);
  initialize_params_c6_RobotSim((SFc6_RobotSimInstanceStruct*) chartInstanceVar);
  initialize_c6_RobotSim((SFc6_RobotSimInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c6_RobotSim(void *chartInstanceVar)
{
  enable_c6_RobotSim((SFc6_RobotSimInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c6_RobotSim(void *chartInstanceVar)
{
  disable_c6_RobotSim((SFc6_RobotSimInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c6_RobotSim(void *chartInstanceVar)
{
  sf_c6_RobotSim((SFc6_RobotSimInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c6_RobotSim(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c6_RobotSim((SFc6_RobotSimInstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c6_RobotSim();/* state var info */
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

extern void sf_internal_set_sim_state_c6_RobotSim(SimStruct* S, const mxArray
  *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c6_RobotSim();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c6_RobotSim((SFc6_RobotSimInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c6_RobotSim(SimStruct* S)
{
  return sf_internal_get_sim_state_c6_RobotSim(S);
}

static void sf_opaque_set_sim_state_c6_RobotSim(SimStruct* S, const mxArray *st)
{
  sf_internal_set_sim_state_c6_RobotSim(S, st);
}

static void sf_opaque_terminate_c6_RobotSim(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc6_RobotSimInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_RobotSim_optimization_info();
    }

    finalize_c6_RobotSim((SFc6_RobotSimInstanceStruct*) chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc6_RobotSim((SFc6_RobotSimInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c6_RobotSim(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c6_RobotSim((SFc6_RobotSimInstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c6_RobotSim(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_RobotSim_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      6);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,6,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,6,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,6);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,6,4);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,6,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 4; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,6);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(702377102U));
  ssSetChecksum1(S,(4152130544U));
  ssSetChecksum2(S,(3902899615U));
  ssSetChecksum3(S,(4055793210U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c6_RobotSim(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c6_RobotSim(SimStruct *S)
{
  SFc6_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc6_RobotSimInstanceStruct *)utMalloc(sizeof
    (SFc6_RobotSimInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc6_RobotSimInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c6_RobotSim;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c6_RobotSim;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c6_RobotSim;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c6_RobotSim;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c6_RobotSim;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c6_RobotSim;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c6_RobotSim;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c6_RobotSim;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c6_RobotSim;
  chartInstance->chartInfo.mdlStart = mdlStart_c6_RobotSim;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c6_RobotSim;
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

void c6_RobotSim_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c6_RobotSim(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c6_RobotSim(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c6_RobotSim(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c6_RobotSim_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
