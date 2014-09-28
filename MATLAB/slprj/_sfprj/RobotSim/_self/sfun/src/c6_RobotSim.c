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

static const char * c6_b_debug_family_names[5] = { "H", "H0", "c", "nargin",
  "nargout" };

static const char * c6_c_debug_family_names[13] = { "s", "q_dep", "n", "i", "k",
  "H", "jj", "nargin", "nargout", "theta_p", "alpha_p", "theta", "q" };

static const char * c6_d_debug_family_names[5] = { "H0", "H", "c", "nargin",
  "nargout" };

static const char * c6_e_debug_family_names[6] = { "H0", "jj", "nargin",
  "nargout", "q", "q_act" };

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
  c6_theta_p[7], real_T c6_alpha_p[7], real_T c6_theta, real_T c6_q[2]);
static void c6_constrMatrices(SFc6_RobotSimInstanceStruct *chartInstance);
static real_T c6_actuated(SFc6_RobotSimInstanceStruct *chartInstance, real_T
  c6_q[2]);
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
static const mxArray *c6_d_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static const mxArray *c6_e_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_d_emlrt_marshallIn(SFc6_RobotSimInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId, real_T c6_y[4]);
static void c6_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static void c6_e_emlrt_marshallIn(SFc6_RobotSimInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId, real_T c6_y[7]);
static void c6_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static void c6_info_helper(c6_ResolvedFunctionInfo c6_info[79]);
static void c6_b_info_helper(c6_ResolvedFunctionInfo c6_info[79]);
static void c6_eml_li_find(SFc6_RobotSimInstanceStruct *chartInstance, boolean_T
  c6_x, int32_T c6_y_data[1], int32_T c6_y_sizes[2]);
static real_T c6_nchoosek(SFc6_RobotSimInstanceStruct *chartInstance, real_T
  c6_k);
static void c6_eml_scalar_eg(SFc6_RobotSimInstanceStruct *chartInstance);
static void c6_eml_warning(SFc6_RobotSimInstanceStruct *chartInstance);
static real_T c6_power(SFc6_RobotSimInstanceStruct *chartInstance, real_T c6_a,
  real_T c6_b);
static void c6_eml_error(SFc6_RobotSimInstanceStruct *chartInstance);
static void c6_b_eml_scalar_eg(SFc6_RobotSimInstanceStruct *chartInstance);
static void c6_c_eml_scalar_eg(SFc6_RobotSimInstanceStruct *chartInstance);
static const mxArray *c6_f_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static int32_T c6_f_emlrt_marshallIn(SFc6_RobotSimInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static uint8_T c6_g_emlrt_marshallIn(SFc6_RobotSimInstanceStruct *chartInstance,
  const mxArray *c6_b_is_active_c6_RobotSim, const char_T *c6_identifier);
static uint8_T c6_h_emlrt_marshallIn(SFc6_RobotSimInstanceStruct *chartInstance,
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
  chartInstance->c6_is_active_c6_RobotSim = c6_g_emlrt_marshallIn(chartInstance,
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
  real_T c6_theta_p[7];
  int32_T c6_i4;
  real_T c6_alpha_p[7];
  int32_T c6_i5;
  real_T c6_q[2];
  real_T c6_theta;
  uint32_T c6_debug_family_var_map[8];
  real_T c6_q_des[2];
  real_T c6_nargin = 4.0;
  real_T c6_nargout = 1.0;
  real_T c6_err;
  int32_T c6_i6;
  real_T c6_b_theta_p[7];
  int32_T c6_i7;
  real_T c6_b_alpha_p[7];
  real_T c6_dv0[2];
  int32_T c6_i8;
  int32_T c6_i9;
  real_T c6_b_q_des[2];
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
  real_T (*c6_b_q)[2];
  real_T (*c6_c_alpha_p)[7];
  real_T (*c6_c_theta_p)[7];
  c6_b_err = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c6_b_theta = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c6_b_q = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 2);
  c6_c_alpha_p = (real_T (*)[7])ssGetInputPortSignal(chartInstance->S, 1);
  c6_c_theta_p = (real_T (*)[7])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 3U, chartInstance->c6_sfEvent);
  for (c6_i0 = 0; c6_i0 < 7; c6_i0++) {
    _SFD_DATA_RANGE_CHECK((*c6_c_theta_p)[c6_i0], 0U);
  }

  for (c6_i1 = 0; c6_i1 < 7; c6_i1++) {
    _SFD_DATA_RANGE_CHECK((*c6_c_alpha_p)[c6_i1], 1U);
  }

  for (c6_i2 = 0; c6_i2 < 2; c6_i2++) {
    _SFD_DATA_RANGE_CHECK((*c6_b_q)[c6_i2], 2U);
  }

  _SFD_DATA_RANGE_CHECK(*c6_b_theta, 3U);
  _SFD_DATA_RANGE_CHECK(*c6_b_err, 4U);
  chartInstance->c6_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 3U, chartInstance->c6_sfEvent);
  c6_hoistedGlobal = *c6_b_theta;
  for (c6_i3 = 0; c6_i3 < 7; c6_i3++) {
    c6_theta_p[c6_i3] = (*c6_c_theta_p)[c6_i3];
  }

  for (c6_i4 = 0; c6_i4 < 7; c6_i4++) {
    c6_alpha_p[c6_i4] = (*c6_c_alpha_p)[c6_i4];
  }

  for (c6_i5 = 0; c6_i5 < 2; c6_i5++) {
    c6_q[c6_i5] = (*c6_b_q)[c6_i5];
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
  for (c6_i6 = 0; c6_i6 < 7; c6_i6++) {
    c6_b_theta_p[c6_i6] = c6_theta_p[c6_i6];
  }

  for (c6_i7 = 0; c6_i7 < 7; c6_i7++) {
    c6_b_alpha_p[c6_i7] = c6_alpha_p[c6_i7];
  }

  c6_bezConstraint(chartInstance, c6_b_theta_p, c6_b_alpha_p, c6_theta, c6_dv0);
  for (c6_i8 = 0; c6_i8 < 2; c6_i8++) {
    c6_q_des[c6_i8] = c6_dv0[c6_i8];
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 8);
  for (c6_i9 = 0; c6_i9 < 2; c6_i9++) {
    c6_b_q_des[c6_i9] = c6_q_des[c6_i9] - c6_q[c6_i9];
  }

  c6_err = c6_actuated(chartInstance, c6_b_q_des);
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
  c6_theta_p[7], real_T c6_alpha_p[7], real_T c6_theta, real_T c6_q[2])
{
  uint32_T c6_debug_family_var_map[13];
  real_T c6_s;
  real_T c6_q_dep;
  real_T c6_n;
  real_T c6_i;
  real_T c6_k;
  real_T c6_H[4];
  real_T c6_jj;
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
  int32_T c6_b_k;
  real_T c6_a;
  real_T c6_b;
  real_T c6_c_y;
  real_T c6_b_a;
  real_T c6_b_b;
  real_T c6_d_y;
  int32_T c6_i30;
  static real_T c6_c_a[4] = { 1.0, 0.0, 0.0, 1.0 };

  int32_T c6_i31;
  real_T c6_c_b[2];
  int32_T c6_i32;
  real_T c6_e_y[2];
  int32_T c6_i33;
  int32_T c6_i34;
  int32_T c6_i35;
  int32_T c6_i36;
  int32_T c6_i37;
  int32_T c6_i38;
  int32_T c6_i39;
  int32_T c6_i40;
  int32_T c6_i41;
  int32_T c6_i42;
  int32_T c6_i43;
  int32_T c6_i44;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 13U, 13U, c6_c_debug_family_names,
    c6_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_s, 0U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_q_dep, 1U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c6_n, 2U, c6_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c6_i, 3U, c6_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_k, 4U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c6_H, 5U, c6_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c6_jj, 6U, c6_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_nargin, 7U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_nargout, 8U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_theta_p, 9U, c6_c_sf_marshallOut,
    c6_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_alpha_p, 10U, c6_c_sf_marshallOut,
    c6_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_theta, 11U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_q, 12U, c6_b_sf_marshallOut,
    c6_b_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 5);
  c6_A = c6_theta - c6_theta_p[0];
  c6_B = c6_theta_p[6] - c6_theta_p[0];
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
  c6_q_dep = 0.0;
  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 9);
  c6_n = 6.0;
  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 11);
  c6_i = 1.0;
  CV_SCRIPT_FOR(0, 0, 1);
  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 12);
  c6_k = 0.0;
  c6_b_k = 0;
  while (c6_b_k < 7) {
    c6_k = (real_T)c6_b_k;
    CV_SCRIPT_FOR(0, 1, 1);
    _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 13);
    c6_a = c6_nchoosek(chartInstance, c6_k);
    c6_b = c6_power(chartInstance, 1.0 - c6_s, 6.0 - c6_k);
    c6_c_y = c6_a * c6_b;
    c6_b_a = c6_c_y * c6_power(chartInstance, c6_s, c6_k);
    c6_b_b = c6_alpha_p[(int32_T)(c6_k + 1.0) - 1];
    c6_d_y = c6_b_a * c6_b_b;
    c6_q_dep += c6_d_y;
    c6_b_k++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  CV_SCRIPT_FOR(0, 1, 0);
  CV_SCRIPT_FOR(0, 0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 18);
  c6_constrMatrices(chartInstance);
  for (c6_i30 = 0; c6_i30 < 4; c6_i30++) {
    c6_H[c6_i30] = c6_c_a[c6_i30];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 19);
  for (c6_i31 = 0; c6_i31 < 2; c6_i31++) {
    c6_q[c6_i31] = 0.0;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 20);
  c6_jj = 1.0;
  CV_SCRIPT_FOR(0, 2, 1);
  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 21);
  c6_c_b[0] = c6_q_dep;
  c6_c_b[1] = c6_theta;
  c6_b_eml_scalar_eg(chartInstance);
  c6_b_eml_scalar_eg(chartInstance);
  for (c6_i32 = 0; c6_i32 < 2; c6_i32++) {
    c6_e_y[c6_i32] = 0.0;
    c6_i33 = 0;
    for (c6_i34 = 0; c6_i34 < 2; c6_i34++) {
      c6_e_y[c6_i32] += c6_c_a[c6_i33 + c6_i32] * c6_c_b[c6_i34];
      c6_i33 += 2;
    }
  }

  for (c6_i35 = 0; c6_i35 < 2; c6_i35++) {
    c6_q[c6_i35] = c6_e_y[c6_i35];
  }

  CV_SCRIPT_FOR(0, 2, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 23);
  c6_c_b[0] = c6_q_dep;
  c6_c_b[1] = c6_theta;
  c6_b_eml_scalar_eg(chartInstance);
  c6_b_eml_scalar_eg(chartInstance);
  for (c6_i36 = 0; c6_i36 < 2; c6_i36++) {
    c6_q[c6_i36] = 0.0;
  }

  for (c6_i37 = 0; c6_i37 < 2; c6_i37++) {
    c6_q[c6_i37] = 0.0;
  }

  for (c6_i38 = 0; c6_i38 < 2; c6_i38++) {
    c6_e_y[c6_i38] = c6_q[c6_i38];
  }

  for (c6_i39 = 0; c6_i39 < 2; c6_i39++) {
    c6_q[c6_i39] = c6_e_y[c6_i39];
  }

  for (c6_i40 = 0; c6_i40 < 2; c6_i40++) {
    c6_e_y[c6_i40] = c6_q[c6_i40];
  }

  for (c6_i41 = 0; c6_i41 < 2; c6_i41++) {
    c6_q[c6_i41] = c6_e_y[c6_i41];
  }

  for (c6_i42 = 0; c6_i42 < 2; c6_i42++) {
    c6_q[c6_i42] = 0.0;
    c6_i43 = 0;
    for (c6_i44 = 0; c6_i44 < 2; c6_i44++) {
      c6_q[c6_i42] += c6_c_a[c6_i43 + c6_i42] * c6_c_b[c6_i44];
      c6_i43 += 2;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, -23);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c6_constrMatrices(SFc6_RobotSimInstanceStruct *chartInstance)
{
  uint32_T c6_debug_family_var_map[5];
  real_T c6_H[4];
  real_T c6_H0[2];
  real_T c6_c[2];
  real_T c6_nargin = 0.0;
  real_T c6_nargout = 1.0;
  int32_T c6_i45;
  int32_T c6_i46;
  int32_T c6_i47;
  static real_T c6_dv5[4] = { 1.0, 0.0, 0.0, 1.0 };

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 5U, 5U, c6_b_debug_family_names,
    c6_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_H, 0U, c6_e_sf_marshallOut,
    c6_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c6_H0, 1U, c6_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c6_c, 2U, c6_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_nargin, 3U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_nargout, 4U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  CV_SCRIPT_FCN(1, 0);
  _SFD_SCRIPT_CALL(1U, chartInstance->c6_sfEvent, 4);
  for (c6_i45 = 0; c6_i45 < 2; c6_i45++) {
    c6_H0[c6_i45] = 1.0 - (real_T)c6_i45;
  }

  _SFD_SCRIPT_CALL(1U, chartInstance->c6_sfEvent, 5);
  for (c6_i46 = 0; c6_i46 < 2; c6_i46++) {
    c6_c[c6_i46] = (real_T)c6_i46;
  }

  _SFD_SCRIPT_CALL(1U, chartInstance->c6_sfEvent, 6);
  for (c6_i47 = 0; c6_i47 < 4; c6_i47++) {
    c6_H[c6_i47] = c6_dv5[c6_i47];
  }

  _SFD_SCRIPT_CALL(1U, chartInstance->c6_sfEvent, -6);
  _SFD_SYMBOL_SCOPE_POP();
}

static real_T c6_actuated(SFc6_RobotSimInstanceStruct *chartInstance, real_T
  c6_q[2])
{
  real_T c6_q_act;
  uint32_T c6_debug_family_var_map[6];
  real_T c6_H0[2];
  real_T c6_jj;
  real_T c6_nargin = 1.0;
  real_T c6_nargout = 1.0;
  uint32_T c6_b_debug_family_var_map[5];
  real_T c6_b_H0[2];
  real_T c6_H[4];
  real_T c6_c[2];
  real_T c6_b_nargin = 0.0;
  real_T c6_b_nargout = 2.0;
  int32_T c6_i48;
  int32_T c6_i49;
  int32_T c6_i50;
  static real_T c6_unusedU0[4] = { 1.0, 0.0, 0.0, 1.0 };

  int32_T c6_i51;
  int32_T c6_i52;
  real_T c6_b[2];
  int32_T c6_k;
  int32_T c6_b_k;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 6U, 6U, c6_e_debug_family_names,
    c6_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(c6_H0, 0U, c6_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c6_jj, 1U, c6_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_nargin, 2U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_nargout, 3U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_q, 4U, c6_b_sf_marshallOut,
    c6_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_q_act, 5U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  CV_SCRIPT_FCN(2, 0);
  _SFD_SCRIPT_CALL(2U, chartInstance->c6_sfEvent, 3);
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 5U, 5U, c6_d_debug_family_names,
    c6_b_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(c6_b_H0, 0U, c6_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_H, 1U, c6_e_sf_marshallOut,
    c6_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c6_c, 2U, c6_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_b_nargin, 3U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_b_nargout, 4U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  CV_SCRIPT_FCN(1, 0);
  _SFD_SCRIPT_CALL(1U, chartInstance->c6_sfEvent, 4);
  for (c6_i48 = 0; c6_i48 < 2; c6_i48++) {
    c6_b_H0[c6_i48] = 1.0 - (real_T)c6_i48;
  }

  _SFD_SCRIPT_CALL(1U, chartInstance->c6_sfEvent, 5);
  for (c6_i49 = 0; c6_i49 < 2; c6_i49++) {
    c6_c[c6_i49] = (real_T)c6_i49;
  }

  _SFD_SCRIPT_CALL(1U, chartInstance->c6_sfEvent, 6);
  for (c6_i50 = 0; c6_i50 < 4; c6_i50++) {
    c6_H[c6_i50] = c6_unusedU0[c6_i50];
  }

  _SFD_SCRIPT_CALL(1U, chartInstance->c6_sfEvent, -6);
  _SFD_SYMBOL_SCOPE_POP();
  for (c6_i51 = 0; c6_i51 < 2; c6_i51++) {
    c6_H0[c6_i51] = 1.0 - (real_T)c6_i51;
  }

  _SFD_SCRIPT_CALL(2U, chartInstance->c6_sfEvent, 3);
  _SFD_SCRIPT_CALL(2U, chartInstance->c6_sfEvent, 4);
  c6_q_act = 0.0;
  _SFD_SCRIPT_CALL(2U, chartInstance->c6_sfEvent, 5);
  c6_jj = 1.0;
  CV_SCRIPT_FOR(2, 0, 1);
  _SFD_SCRIPT_CALL(2U, chartInstance->c6_sfEvent, 6);
  for (c6_i52 = 0; c6_i52 < 2; c6_i52++) {
    c6_b[c6_i52] = c6_q[c6_i52];
  }

  c6_c_eml_scalar_eg(chartInstance);
  c6_c_eml_scalar_eg(chartInstance);
  c6_q_act = 0.0;
  for (c6_k = 1; c6_k < 3; c6_k++) {
    c6_b_k = c6_k - 1;
    c6_q_act += (1.0 + -(real_T)c6_b_k) * c6_b[c6_b_k];
  }

  CV_SCRIPT_FOR(2, 0, 0);
  _SFD_SCRIPT_CALL(2U, chartInstance->c6_sfEvent, -6);
  _SFD_SYMBOL_SCOPE_POP();
  return c6_q_act;
}

static void init_script_number_translation(uint32_T c6_machineNumber, uint32_T
  c6_chartNumber)
{
  _SFD_SCRIPT_TRANSLATION(c6_chartNumber, 0U, sf_debug_get_script_id(
    "C:/Users/yak/My Documents/GitHub/thesis/MATLAB/bezConstraint.m"));
  _SFD_SCRIPT_TRANSLATION(c6_chartNumber, 1U, sf_debug_get_script_id(
    "C:/Users/yak/My Documents/GitHub/thesis/MATLAB/constrMatrices.m"));
  _SFD_SCRIPT_TRANSLATION(c6_chartNumber, 2U, sf_debug_get_script_id(
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
  int32_T c6_i53;
  real_T c6_b_inData[2];
  int32_T c6_i54;
  real_T c6_u[2];
  const mxArray *c6_y = NULL;
  SFc6_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc6_RobotSimInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i53 = 0; c6_i53 < 2; c6_i53++) {
    c6_b_inData[c6_i53] = (*(real_T (*)[2])c6_inData)[c6_i53];
  }

  for (c6_i54 = 0; c6_i54 < 2; c6_i54++) {
    c6_u[c6_i54] = c6_b_inData[c6_i54];
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
  int32_T c6_i55;
  real_T c6_b_inData[7];
  int32_T c6_i56;
  real_T c6_u[7];
  const mxArray *c6_y = NULL;
  SFc6_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc6_RobotSimInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i55 = 0; c6_i55 < 7; c6_i55++) {
    c6_b_inData[c6_i55] = (*(real_T (*)[7])c6_inData)[c6_i55];
  }

  for (c6_i56 = 0; c6_i56 < 7; c6_i56++) {
    c6_u[c6_i56] = c6_b_inData[c6_i56];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 1, 7), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static void c6_c_emlrt_marshallIn(SFc6_RobotSimInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId, real_T c6_y[2])
{
  real_T c6_dv6[2];
  int32_T c6_i57;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv6, 1, 0, 0U, 1, 0U, 1, 2);
  for (c6_i57 = 0; c6_i57 < 2; c6_i57++) {
    c6_y[c6_i57] = c6_dv6[c6_i57];
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
  int32_T c6_i58;
  SFc6_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc6_RobotSimInstanceStruct *)chartInstanceVoid;
  c6_q_des = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_q_des), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_q_des);
  for (c6_i58 = 0; c6_i58 < 2; c6_i58++) {
    (*(real_T (*)[2])c6_outData)[c6_i58] = c6_y[c6_i58];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_d_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i59;
  real_T c6_b_inData[2];
  int32_T c6_i60;
  real_T c6_u[2];
  const mxArray *c6_y = NULL;
  SFc6_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc6_RobotSimInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i59 = 0; c6_i59 < 2; c6_i59++) {
    c6_b_inData[c6_i59] = (*(real_T (*)[2])c6_inData)[c6_i59];
  }

  for (c6_i60 = 0; c6_i60 < 2; c6_i60++) {
    c6_u[c6_i60] = c6_b_inData[c6_i60];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 1, 2), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static const mxArray *c6_e_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i61;
  int32_T c6_i62;
  int32_T c6_i63;
  real_T c6_b_inData[4];
  int32_T c6_i64;
  int32_T c6_i65;
  int32_T c6_i66;
  real_T c6_u[4];
  const mxArray *c6_y = NULL;
  SFc6_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc6_RobotSimInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_i61 = 0;
  for (c6_i62 = 0; c6_i62 < 2; c6_i62++) {
    for (c6_i63 = 0; c6_i63 < 2; c6_i63++) {
      c6_b_inData[c6_i63 + c6_i61] = (*(real_T (*)[4])c6_inData)[c6_i63 + c6_i61];
    }

    c6_i61 += 2;
  }

  c6_i64 = 0;
  for (c6_i65 = 0; c6_i65 < 2; c6_i65++) {
    for (c6_i66 = 0; c6_i66 < 2; c6_i66++) {
      c6_u[c6_i66 + c6_i64] = c6_b_inData[c6_i66 + c6_i64];
    }

    c6_i64 += 2;
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 2, 2), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static void c6_d_emlrt_marshallIn(SFc6_RobotSimInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId, real_T c6_y[4])
{
  real_T c6_dv7[4];
  int32_T c6_i67;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv7, 1, 0, 0U, 1, 0U, 2, 2, 2);
  for (c6_i67 = 0; c6_i67 < 4; c6_i67++) {
    c6_y[c6_i67] = c6_dv7[c6_i67];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_H;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[4];
  int32_T c6_i68;
  int32_T c6_i69;
  int32_T c6_i70;
  SFc6_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc6_RobotSimInstanceStruct *)chartInstanceVoid;
  c6_H = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_H), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_H);
  c6_i68 = 0;
  for (c6_i69 = 0; c6_i69 < 2; c6_i69++) {
    for (c6_i70 = 0; c6_i70 < 2; c6_i70++) {
      (*(real_T (*)[4])c6_outData)[c6_i70 + c6_i68] = c6_y[c6_i70 + c6_i68];
    }

    c6_i68 += 2;
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static void c6_e_emlrt_marshallIn(SFc6_RobotSimInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId, real_T c6_y[7])
{
  real_T c6_dv8[7];
  int32_T c6_i71;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv8, 1, 0, 0U, 1, 0U, 2, 1, 7);
  for (c6_i71 = 0; c6_i71 < 7; c6_i71++) {
    c6_y[c6_i71] = c6_dv8[c6_i71];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_alpha_p;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[7];
  int32_T c6_i72;
  SFc6_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc6_RobotSimInstanceStruct *)chartInstanceVoid;
  c6_alpha_p = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_alpha_p), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_alpha_p);
  for (c6_i72 = 0; c6_i72 < 7; c6_i72++) {
    (*(real_T (*)[7])c6_outData)[c6_i72] = c6_y[c6_i72];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

const mxArray *sf_c6_RobotSim_get_eml_resolved_functions_info(void)
{
  const mxArray *c6_nameCaptureInfo;
  c6_ResolvedFunctionInfo c6_info[79];
  const mxArray *c6_m0 = NULL;
  int32_T c6_i73;
  c6_ResolvedFunctionInfo *c6_r0;
  c6_nameCaptureInfo = NULL;
  c6_nameCaptureInfo = NULL;
  c6_info_helper(c6_info);
  c6_b_info_helper(c6_info);
  sf_mex_assign(&c6_m0, sf_mex_createstruct("nameCaptureInfo", 1, 79), FALSE);
  for (c6_i73 = 0; c6_i73 < 79; c6_i73++) {
    c6_r0 = &c6_info[c6_i73];
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", c6_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c6_r0->context)), "context", "nameCaptureInfo",
                    c6_i73);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", c6_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c6_r0->name)), "name", "nameCaptureInfo", c6_i73);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", c6_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c6_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c6_i73);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", c6_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c6_r0->resolved)), "resolved", "nameCaptureInfo",
                    c6_i73);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", &c6_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c6_i73);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", &c6_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c6_i73);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", &c6_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c6_i73);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", &c6_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c6_i73);
  }

  sf_mex_assign(&c6_nameCaptureInfo, c6_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c6_nameCaptureInfo);
  return c6_nameCaptureInfo;
}

static void c6_info_helper(c6_ResolvedFunctionInfo c6_info[79])
{
  c6_info[0].context = "";
  c6_info[0].name = "bezConstraint";
  c6_info[0].dominantType = "double";
  c6_info[0].resolved =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/bezConstraint.m";
  c6_info[0].fileTimeLo = 1411536952U;
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
  c6_info[11].name = "length";
  c6_info[11].dominantType = "double";
  c6_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c6_info[11].fileTimeLo = 1303117406U;
  c6_info[11].fileTimeHi = 0U;
  c6_info[11].mFileTimeLo = 0U;
  c6_info[11].mFileTimeHi = 0U;
  c6_info[12].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/bezConstraint.m";
  c6_info[12].name = "nchoosek";
  c6_info[12].dominantType = "double";
  c6_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m";
  c6_info[12].fileTimeLo = 1352388862U;
  c6_info[12].fileTimeHi = 0U;
  c6_info[12].mFileTimeLo = 0U;
  c6_info[12].mFileTimeHi = 0U;
  c6_info[13].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m";
  c6_info[13].name = "floor";
  c6_info[13].dominantType = "double";
  c6_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c6_info[13].fileTimeLo = 1343801580U;
  c6_info[13].fileTimeHi = 0U;
  c6_info[13].mFileTimeLo = 0U;
  c6_info[13].mFileTimeHi = 0U;
  c6_info[14].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c6_info[14].name = "eml_scalar_floor";
  c6_info[14].dominantType = "double";
  c6_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c6_info[14].fileTimeLo = 1286786326U;
  c6_info[14].fileTimeHi = 0U;
  c6_info[14].mFileTimeLo = 0U;
  c6_info[14].mFileTimeHi = 0U;
  c6_info[15].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m";
  c6_info[15].name = "eml_error";
  c6_info[15].dominantType = "char";
  c6_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c6_info[15].fileTimeLo = 1343801558U;
  c6_info[15].fileTimeHi = 0U;
  c6_info[15].mFileTimeLo = 0U;
  c6_info[15].mFileTimeHi = 0U;
  c6_info[16].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m";
  c6_info[16].name = "eml_scalar_eg";
  c6_info[16].dominantType = "double";
  c6_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[16].fileTimeLo = 1286786396U;
  c6_info[16].fileTimeHi = 0U;
  c6_info[16].mFileTimeLo = 0U;
  c6_info[16].mFileTimeHi = 0U;
  c6_info[17].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m";
  c6_info[17].name = "eml_guarded_nan";
  c6_info[17].dominantType = "";
  c6_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m";
  c6_info[17].fileTimeLo = 1286786376U;
  c6_info[17].fileTimeHi = 0U;
  c6_info[17].mFileTimeLo = 0U;
  c6_info[17].mFileTimeHi = 0U;
  c6_info[18].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
  c6_info[18].name = "eml_scalar_eg";
  c6_info[18].dominantType = "double";
  c6_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[18].fileTimeLo = 1286786396U;
  c6_info[18].fileTimeHi = 0U;
  c6_info[18].mFileTimeLo = 0U;
  c6_info[18].mFileTimeHi = 0U;
  c6_info[19].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
  c6_info[19].name = "isfinite";
  c6_info[19].dominantType = "double";
  c6_info[19].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c6_info[19].fileTimeLo = 1286786358U;
  c6_info[19].fileTimeHi = 0U;
  c6_info[19].mFileTimeLo = 0U;
  c6_info[19].mFileTimeHi = 0U;
  c6_info[20].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c6_info[20].name = "isinf";
  c6_info[20].dominantType = "double";
  c6_info[20].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m";
  c6_info[20].fileTimeLo = 1286786360U;
  c6_info[20].fileTimeHi = 0U;
  c6_info[20].mFileTimeLo = 0U;
  c6_info[20].mFileTimeHi = 0U;
  c6_info[21].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c6_info[21].name = "isnan";
  c6_info[21].dominantType = "double";
  c6_info[21].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m";
  c6_info[21].fileTimeLo = 1286786360U;
  c6_info[21].fileTimeHi = 0U;
  c6_info[21].mFileTimeLo = 0U;
  c6_info[21].mFileTimeHi = 0U;
  c6_info[22].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
  c6_info[22].name = "eml_guarded_inf";
  c6_info[22].dominantType = "";
  c6_info[22].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_inf.m";
  c6_info[22].fileTimeLo = 1286786376U;
  c6_info[22].fileTimeHi = 0U;
  c6_info[22].mFileTimeLo = 0U;
  c6_info[22].mFileTimeHi = 0U;
  c6_info[23].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
  c6_info[23].name = "mtimes";
  c6_info[23].dominantType = "double";
  c6_info[23].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[23].fileTimeLo = 1289483692U;
  c6_info[23].fileTimeHi = 0U;
  c6_info[23].mFileTimeLo = 0U;
  c6_info[23].mFileTimeHi = 0U;
  c6_info[24].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
  c6_info[24].name = "round";
  c6_info[24].dominantType = "double";
  c6_info[24].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/round.m";
  c6_info[24].fileTimeLo = 1343801584U;
  c6_info[24].fileTimeHi = 0U;
  c6_info[24].mFileTimeLo = 0U;
  c6_info[24].mFileTimeHi = 0U;
  c6_info[25].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/round.m";
  c6_info[25].name = "eml_scalar_round";
  c6_info[25].dominantType = "double";
  c6_info[25].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_round.m";
  c6_info[25].fileTimeLo = 1307622438U;
  c6_info[25].fileTimeHi = 0U;
  c6_info[25].mFileTimeLo = 0U;
  c6_info[25].mFileTimeHi = 0U;
  c6_info[26].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
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
  c6_info[27].name = "flintmax";
  c6_info[27].dominantType = "char";
  c6_info[27].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/flintmax.m";
  c6_info[27].fileTimeLo = 1348163116U;
  c6_info[27].fileTimeHi = 0U;
  c6_info[27].mFileTimeLo = 0U;
  c6_info[27].mFileTimeHi = 0U;
  c6_info[28].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/flintmax.m";
  c6_info[28].name = "eml_float_model";
  c6_info[28].dominantType = "char";
  c6_info[28].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c6_info[28].fileTimeLo = 1326691996U;
  c6_info[28].fileTimeHi = 0U;
  c6_info[28].mFileTimeLo = 0U;
  c6_info[28].mFileTimeHi = 0U;
  c6_info[29].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
  c6_info[29].name = "eml_warning";
  c6_info[29].dominantType = "char";
  c6_info[29].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c6_info[29].fileTimeLo = 1286786402U;
  c6_info[29].fileTimeHi = 0U;
  c6_info[29].mFileTimeLo = 0U;
  c6_info[29].mFileTimeHi = 0U;
  c6_info[30].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/bezConstraint.m";
  c6_info[30].name = "power";
  c6_info[30].dominantType = "double";
  c6_info[30].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c6_info[30].fileTimeLo = 1348163130U;
  c6_info[30].fileTimeHi = 0U;
  c6_info[30].mFileTimeLo = 0U;
  c6_info[30].mFileTimeHi = 0U;
  c6_info[31].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c6_info[31].name = "eml_scalar_eg";
  c6_info[31].dominantType = "double";
  c6_info[31].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[31].fileTimeLo = 1286786396U;
  c6_info[31].fileTimeHi = 0U;
  c6_info[31].mFileTimeLo = 0U;
  c6_info[31].mFileTimeHi = 0U;
  c6_info[32].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c6_info[32].name = "eml_scalexp_alloc";
  c6_info[32].dominantType = "double";
  c6_info[32].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c6_info[32].fileTimeLo = 1352388860U;
  c6_info[32].fileTimeHi = 0U;
  c6_info[32].mFileTimeLo = 0U;
  c6_info[32].mFileTimeHi = 0U;
  c6_info[33].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c6_info[33].name = "floor";
  c6_info[33].dominantType = "double";
  c6_info[33].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c6_info[33].fileTimeLo = 1343801580U;
  c6_info[33].fileTimeHi = 0U;
  c6_info[33].mFileTimeLo = 0U;
  c6_info[33].mFileTimeHi = 0U;
  c6_info[34].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c6_info[34].name = "eml_error";
  c6_info[34].dominantType = "char";
  c6_info[34].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c6_info[34].fileTimeLo = 1343801558U;
  c6_info[34].fileTimeHi = 0U;
  c6_info[34].mFileTimeLo = 0U;
  c6_info[34].mFileTimeHi = 0U;
  c6_info[35].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power";
  c6_info[35].name = "eml_scalar_eg";
  c6_info[35].dominantType = "double";
  c6_info[35].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[35].fileTimeLo = 1286786396U;
  c6_info[35].fileTimeHi = 0U;
  c6_info[35].mFileTimeLo = 0U;
  c6_info[35].mFileTimeHi = 0U;
  c6_info[36].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/bezConstraint.m";
  c6_info[36].name = "mtimes";
  c6_info[36].dominantType = "double";
  c6_info[36].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[36].fileTimeLo = 1289483692U;
  c6_info[36].fileTimeHi = 0U;
  c6_info[36].mFileTimeLo = 0U;
  c6_info[36].mFileTimeHi = 0U;
  c6_info[37].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/bezConstraint.m";
  c6_info[37].name = "constrMatrices";
  c6_info[37].dominantType = "";
  c6_info[37].resolved =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/constrMatrices.m";
  c6_info[37].fileTimeLo = 1411047847U;
  c6_info[37].fileTimeHi = 0U;
  c6_info[37].mFileTimeLo = 0U;
  c6_info[37].mFileTimeHi = 0U;
  c6_info[38].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[38].name = "eml_index_class";
  c6_info[38].dominantType = "";
  c6_info[38].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[38].fileTimeLo = 1323134578U;
  c6_info[38].fileTimeHi = 0U;
  c6_info[38].mFileTimeLo = 0U;
  c6_info[38].mFileTimeHi = 0U;
  c6_info[39].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[39].name = "eml_scalar_eg";
  c6_info[39].dominantType = "double";
  c6_info[39].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[39].fileTimeLo = 1286786396U;
  c6_info[39].fileTimeHi = 0U;
  c6_info[39].mFileTimeLo = 0U;
  c6_info[39].mFileTimeHi = 0U;
  c6_info[40].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[40].name = "eml_xgemm";
  c6_info[40].dominantType = "char";
  c6_info[40].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c6_info[40].fileTimeLo = 1299040772U;
  c6_info[40].fileTimeHi = 0U;
  c6_info[40].mFileTimeLo = 0U;
  c6_info[40].mFileTimeHi = 0U;
  c6_info[41].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c6_info[41].name = "eml_blas_inline";
  c6_info[41].dominantType = "";
  c6_info[41].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c6_info[41].fileTimeLo = 1299040768U;
  c6_info[41].fileTimeHi = 0U;
  c6_info[41].mFileTimeLo = 0U;
  c6_info[41].mFileTimeHi = 0U;
  c6_info[42].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c6_info[42].name = "mtimes";
  c6_info[42].dominantType = "double";
  c6_info[42].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[42].fileTimeLo = 1289483692U;
  c6_info[42].fileTimeHi = 0U;
  c6_info[42].mFileTimeLo = 0U;
  c6_info[42].mFileTimeHi = 0U;
  c6_info[43].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c6_info[43].name = "eml_index_class";
  c6_info[43].dominantType = "";
  c6_info[43].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[43].fileTimeLo = 1323134578U;
  c6_info[43].fileTimeHi = 0U;
  c6_info[43].mFileTimeLo = 0U;
  c6_info[43].mFileTimeHi = 0U;
  c6_info[44].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c6_info[44].name = "eml_scalar_eg";
  c6_info[44].dominantType = "double";
  c6_info[44].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[44].fileTimeLo = 1286786396U;
  c6_info[44].fileTimeHi = 0U;
  c6_info[44].mFileTimeLo = 0U;
  c6_info[44].mFileTimeHi = 0U;
  c6_info[45].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c6_info[45].name = "eml_refblas_xgemm";
  c6_info[45].dominantType = "char";
  c6_info[45].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c6_info[45].fileTimeLo = 1299040774U;
  c6_info[45].fileTimeHi = 0U;
  c6_info[45].mFileTimeLo = 0U;
  c6_info[45].mFileTimeHi = 0U;
  c6_info[46].context = "";
  c6_info[46].name = "actuated";
  c6_info[46].dominantType = "double";
  c6_info[46].resolved =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/actuated.m";
  c6_info[46].fileTimeLo = 1411536958U;
  c6_info[46].fileTimeHi = 0U;
  c6_info[46].mFileTimeLo = 0U;
  c6_info[46].mFileTimeHi = 0U;
  c6_info[47].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/actuated.m";
  c6_info[47].name = "constrMatrices";
  c6_info[47].dominantType = "";
  c6_info[47].resolved =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/constrMatrices.m";
  c6_info[47].fileTimeLo = 1411047847U;
  c6_info[47].fileTimeHi = 0U;
  c6_info[47].mFileTimeLo = 0U;
  c6_info[47].mFileTimeHi = 0U;
  c6_info[48].context =
    "[E]C:/Users/yak/My Documents/GitHub/thesis/MATLAB/actuated.m";
  c6_info[48].name = "mtimes";
  c6_info[48].dominantType = "double";
  c6_info[48].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[48].fileTimeLo = 1289483692U;
  c6_info[48].fileTimeHi = 0U;
  c6_info[48].mFileTimeLo = 0U;
  c6_info[48].mFileTimeHi = 0U;
  c6_info[49].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[49].name = "eml_xdotu";
  c6_info[49].dominantType = "double";
  c6_info[49].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotu.m";
  c6_info[49].fileTimeLo = 1299040772U;
  c6_info[49].fileTimeHi = 0U;
  c6_info[49].mFileTimeLo = 0U;
  c6_info[49].mFileTimeHi = 0U;
  c6_info[50].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotu.m";
  c6_info[50].name = "eml_blas_inline";
  c6_info[50].dominantType = "";
  c6_info[50].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c6_info[50].fileTimeLo = 1299040768U;
  c6_info[50].fileTimeHi = 0U;
  c6_info[50].mFileTimeLo = 0U;
  c6_info[50].mFileTimeHi = 0U;
  c6_info[51].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotu.m";
  c6_info[51].name = "eml_xdot";
  c6_info[51].dominantType = "double";
  c6_info[51].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdot.m";
  c6_info[51].fileTimeLo = 1299040772U;
  c6_info[51].fileTimeHi = 0U;
  c6_info[51].mFileTimeLo = 0U;
  c6_info[51].mFileTimeHi = 0U;
  c6_info[52].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdot.m";
  c6_info[52].name = "eml_blas_inline";
  c6_info[52].dominantType = "";
  c6_info[52].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c6_info[52].fileTimeLo = 1299040768U;
  c6_info[52].fileTimeHi = 0U;
  c6_info[52].mFileTimeLo = 0U;
  c6_info[52].mFileTimeHi = 0U;
  c6_info[53].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xdot.m";
  c6_info[53].name = "eml_index_class";
  c6_info[53].dominantType = "";
  c6_info[53].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[53].fileTimeLo = 1323134578U;
  c6_info[53].fileTimeHi = 0U;
  c6_info[53].mFileTimeLo = 0U;
  c6_info[53].mFileTimeHi = 0U;
  c6_info[54].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xdot.m";
  c6_info[54].name = "eml_refblas_xdot";
  c6_info[54].dominantType = "double";
  c6_info[54].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdot.m";
  c6_info[54].fileTimeLo = 1299040772U;
  c6_info[54].fileTimeHi = 0U;
  c6_info[54].mFileTimeLo = 0U;
  c6_info[54].mFileTimeHi = 0U;
  c6_info[55].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdot.m";
  c6_info[55].name = "eml_refblas_xdotx";
  c6_info[55].dominantType = "char";
  c6_info[55].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c6_info[55].fileTimeLo = 1299040774U;
  c6_info[55].fileTimeHi = 0U;
  c6_info[55].mFileTimeLo = 0U;
  c6_info[55].mFileTimeHi = 0U;
  c6_info[56].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c6_info[56].name = "eml_scalar_eg";
  c6_info[56].dominantType = "double";
  c6_info[56].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[56].fileTimeLo = 1286786396U;
  c6_info[56].fileTimeHi = 0U;
  c6_info[56].mFileTimeLo = 0U;
  c6_info[56].mFileTimeHi = 0U;
  c6_info[57].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c6_info[57].name = "eml_index_class";
  c6_info[57].dominantType = "";
  c6_info[57].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[57].fileTimeLo = 1323134578U;
  c6_info[57].fileTimeHi = 0U;
  c6_info[57].mFileTimeLo = 0U;
  c6_info[57].mFileTimeHi = 0U;
  c6_info[58].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c6_info[58].name = "eml_index_minus";
  c6_info[58].dominantType = "double";
  c6_info[58].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c6_info[58].fileTimeLo = 1286786378U;
  c6_info[58].fileTimeHi = 0U;
  c6_info[58].mFileTimeLo = 0U;
  c6_info[58].mFileTimeHi = 0U;
  c6_info[59].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c6_info[59].name = "eml_index_class";
  c6_info[59].dominantType = "";
  c6_info[59].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[59].fileTimeLo = 1323134578U;
  c6_info[59].fileTimeHi = 0U;
  c6_info[59].mFileTimeLo = 0U;
  c6_info[59].mFileTimeHi = 0U;
  c6_info[60].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c6_info[60].name = "eml_index_times";
  c6_info[60].dominantType = "coder.internal.indexInt";
  c6_info[60].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c6_info[60].fileTimeLo = 1286786380U;
  c6_info[60].fileTimeHi = 0U;
  c6_info[60].mFileTimeLo = 0U;
  c6_info[60].mFileTimeHi = 0U;
  c6_info[61].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c6_info[61].name = "eml_index_class";
  c6_info[61].dominantType = "";
  c6_info[61].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[61].fileTimeLo = 1323134578U;
  c6_info[61].fileTimeHi = 0U;
  c6_info[61].mFileTimeLo = 0U;
  c6_info[61].mFileTimeHi = 0U;
  c6_info[62].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c6_info[62].name = "eml_index_plus";
  c6_info[62].dominantType = "coder.internal.indexInt";
  c6_info[62].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c6_info[62].fileTimeLo = 1286786378U;
  c6_info[62].fileTimeHi = 0U;
  c6_info[62].mFileTimeLo = 0U;
  c6_info[62].mFileTimeHi = 0U;
  c6_info[63].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c6_info[63].name = "eml_int_forloop_overflow_check";
  c6_info[63].dominantType = "";
  c6_info[63].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c6_info[63].fileTimeLo = 1346481540U;
  c6_info[63].fileTimeHi = 0U;
  c6_info[63].mFileTimeLo = 0U;
  c6_info[63].mFileTimeHi = 0U;
}

static void c6_b_info_helper(c6_ResolvedFunctionInfo c6_info[79])
{
  c6_info[64].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c6_info[64].name = "intmax";
  c6_info[64].dominantType = "char";
  c6_info[64].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c6_info[64].fileTimeLo = 1311226516U;
  c6_info[64].fileTimeHi = 0U;
  c6_info[64].mFileTimeLo = 0U;
  c6_info[64].mFileTimeHi = 0U;
  c6_info[65].context = "";
  c6_info[65].name = "mtimes";
  c6_info[65].dominantType = "double";
  c6_info[65].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[65].fileTimeLo = 1289483692U;
  c6_info[65].fileTimeHi = 0U;
  c6_info[65].mFileTimeLo = 0U;
  c6_info[65].mFileTimeHi = 0U;
  c6_info[66].context = "";
  c6_info[66].name = "mod";
  c6_info[66].dominantType = "double";
  c6_info[66].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  c6_info[66].fileTimeLo = 1343801582U;
  c6_info[66].fileTimeHi = 0U;
  c6_info[66].mFileTimeLo = 0U;
  c6_info[66].mFileTimeHi = 0U;
  c6_info[67].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  c6_info[67].name = "eml_scalar_eg";
  c6_info[67].dominantType = "double";
  c6_info[67].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[67].fileTimeLo = 1286786396U;
  c6_info[67].fileTimeHi = 0U;
  c6_info[67].mFileTimeLo = 0U;
  c6_info[67].mFileTimeHi = 0U;
  c6_info[68].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  c6_info[68].name = "eml_scalexp_alloc";
  c6_info[68].dominantType = "double";
  c6_info[68].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c6_info[68].fileTimeLo = 1352388860U;
  c6_info[68].fileTimeHi = 0U;
  c6_info[68].mFileTimeLo = 0U;
  c6_info[68].mFileTimeHi = 0U;
  c6_info[69].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m!floatmod";
  c6_info[69].name = "eml_scalar_eg";
  c6_info[69].dominantType = "double";
  c6_info[69].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[69].fileTimeLo = 1286786396U;
  c6_info[69].fileTimeHi = 0U;
  c6_info[69].mFileTimeLo = 0U;
  c6_info[69].mFileTimeHi = 0U;
  c6_info[70].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m!floatmod";
  c6_info[70].name = "eml_scalar_floor";
  c6_info[70].dominantType = "double";
  c6_info[70].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c6_info[70].fileTimeLo = 1286786326U;
  c6_info[70].fileTimeHi = 0U;
  c6_info[70].mFileTimeLo = 0U;
  c6_info[70].mFileTimeHi = 0U;
  c6_info[71].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m!floatmod";
  c6_info[71].name = "eml_scalar_round";
  c6_info[71].dominantType = "double";
  c6_info[71].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_round.m";
  c6_info[71].fileTimeLo = 1307622438U;
  c6_info[71].fileTimeHi = 0U;
  c6_info[71].mFileTimeLo = 0U;
  c6_info[71].mFileTimeHi = 0U;
  c6_info[72].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m!floatmod";
  c6_info[72].name = "eml_scalar_abs";
  c6_info[72].dominantType = "double";
  c6_info[72].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c6_info[72].fileTimeLo = 1286786312U;
  c6_info[72].fileTimeHi = 0U;
  c6_info[72].mFileTimeLo = 0U;
  c6_info[72].mFileTimeHi = 0U;
  c6_info[73].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m!floatmod";
  c6_info[73].name = "eps";
  c6_info[73].dominantType = "char";
  c6_info[73].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c6_info[73].fileTimeLo = 1326691996U;
  c6_info[73].fileTimeHi = 0U;
  c6_info[73].mFileTimeLo = 0U;
  c6_info[73].mFileTimeHi = 0U;
  c6_info[74].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c6_info[74].name = "eml_is_float_class";
  c6_info[74].dominantType = "char";
  c6_info[74].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c6_info[74].fileTimeLo = 1286786382U;
  c6_info[74].fileTimeHi = 0U;
  c6_info[74].mFileTimeLo = 0U;
  c6_info[74].mFileTimeHi = 0U;
  c6_info[75].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c6_info[75].name = "eml_eps";
  c6_info[75].dominantType = "char";
  c6_info[75].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c6_info[75].fileTimeLo = 1326691996U;
  c6_info[75].fileTimeHi = 0U;
  c6_info[75].mFileTimeLo = 0U;
  c6_info[75].mFileTimeHi = 0U;
  c6_info[76].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c6_info[76].name = "eml_float_model";
  c6_info[76].dominantType = "char";
  c6_info[76].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c6_info[76].fileTimeLo = 1326691996U;
  c6_info[76].fileTimeHi = 0U;
  c6_info[76].mFileTimeLo = 0U;
  c6_info[76].mFileTimeHi = 0U;
  c6_info[77].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m!floatmod";
  c6_info[77].name = "mtimes";
  c6_info[77].dominantType = "double";
  c6_info[77].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[77].fileTimeLo = 1289483692U;
  c6_info[77].fileTimeHi = 0U;
  c6_info[77].mFileTimeLo = 0U;
  c6_info[77].mFileTimeHi = 0U;
  c6_info[78].context = "";
  c6_info[78].name = "eml_li_find";
  c6_info[78].dominantType = "";
  c6_info[78].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m";
  c6_info[78].fileTimeLo = 1286786386U;
  c6_info[78].fileTimeHi = 0U;
  c6_info[78].mFileTimeLo = 0U;
  c6_info[78].mFileTimeHi = 0U;
}

static void c6_eml_li_find(SFc6_RobotSimInstanceStruct *chartInstance, boolean_T
  c6_x, int32_T c6_y_data[1], int32_T c6_y_sizes[2])
{
  boolean_T c6_b_x;
  int32_T c6_k;
  int32_T c6_tmp_sizes[2];
  int32_T c6_iv0[2];
  int32_T c6_i74;
  int32_T c6_i75;
  int32_T c6_loop_ub;
  int32_T c6_i76;
  int32_T c6_tmp_data[1];
  int32_T c6_i77;
  c6_b_x = c6_x;
  c6_k = 0;
  if (c6_b_x) {
    c6_k = 1;
  }

  c6_tmp_sizes[0] = 1;
  c6_iv0[0] = 1;
  c6_iv0[1] = c6_k;
  c6_tmp_sizes[1] = c6_iv0[1];
  c6_i74 = c6_tmp_sizes[0];
  c6_i75 = c6_tmp_sizes[1];
  c6_loop_ub = c6_k - 1;
  for (c6_i76 = 0; c6_i76 <= c6_loop_ub; c6_i76++) {
    c6_tmp_data[c6_i76] = 0;
  }

  for (c6_i77 = 0; c6_i77 < 2; c6_i77++) {
    c6_y_sizes[c6_i77] = c6_tmp_sizes[c6_i77];
  }

  if (c6_x) {
    _SFD_EML_ARRAY_BOUNDS_CHECK("", 1, 1, c6_y_sizes[1], 1, 0);
    c6_y_data[0] = 1;
  }
}

static real_T c6_nchoosek(SFc6_RobotSimInstanceStruct *chartInstance, real_T
  c6_k)
{
  real_T c6_y;
  real_T c6_x;
  real_T c6_b_x;
  boolean_T c6_b0;
  int32_T c6_i78;
  static char_T c6_cv0[27] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'n', 'c', 'h',
    'o', 'o', 's', 'e', 'k', ':', 'I', 'n', 'v', 'a', 'l', 'i', 'd', 'A', 'r',
    'g', '2' };

  char_T c6_u[27];
  const mxArray *c6_b_y = NULL;
  real_T c6_b_k;
  real_T c6_nmk;
  real_T c6_c_k;
  int32_T c6_i79;
  int32_T c6_j;
  real_T c6_b_j;
  real_T c6_a;
  real_T c6_b;
  c6_x = c6_k;
  c6_b_x = c6_x;
  c6_b_x = muDoubleScalarFloor(c6_b_x);
  c6_b0 = (c6_k == c6_b_x);
  if (c6_b0) {
  } else {
    for (c6_i78 = 0; c6_i78 < 27; c6_i78++) {
      c6_u[c6_i78] = c6_cv0[c6_i78];
    }

    c6_b_y = NULL;
    sf_mex_assign(&c6_b_y, sf_mex_create("y", c6_u, 10, 0U, 1U, 0U, 2, 1, 27),
                  FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U,
      14, c6_b_y));
  }

  c6_b_k = c6_k;
  c6_eml_scalar_eg(chartInstance);
  if (c6_b_k > 3.0) {
    c6_b_k = 6.0 - c6_b_k;
  }

  c6_y = 1.0;
  c6_nmk = 6.0 - c6_b_k;
  c6_c_k = c6_b_k;
  c6_i79 = (int32_T)c6_c_k - 1;
  for (c6_j = 0; c6_j <= c6_i79; c6_j++) {
    c6_b_j = 1.0 + (real_T)c6_j;
    c6_a = c6_y;
    c6_b = (c6_b_j + c6_nmk) / c6_b_j;
    c6_y = c6_a * c6_b;
  }

  c6_y = muDoubleScalarRound(c6_y);
  if (!(c6_y <= 9.007199254740992E+15)) {
    c6_eml_warning(chartInstance);
  }

  return c6_y;
}

static void c6_eml_scalar_eg(SFc6_RobotSimInstanceStruct *chartInstance)
{
}

static void c6_eml_warning(SFc6_RobotSimInstanceStruct *chartInstance)
{
  int32_T c6_i80;
  static char_T c6_varargin_1[38] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 'n', 'c', 'h', 'o', 'o', 's', 'e', 'k', '_', 'L',
    'a', 'r', 'g', 'e', 'C', 'o', 'e', 'f', 'f', 'i', 'c', 'i', 'e', 'n', 't' };

  char_T c6_u[38];
  const mxArray *c6_y = NULL;
  for (c6_i80 = 0; c6_i80 < 38; c6_i80++) {
    c6_u[c6_i80] = c6_varargin_1[c6_i80];
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
  int32_T c6_i81;
  static char_T c6_cv1[31] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'p', 'o', 'w', 'e', 'r', '_', 'd', 'o', 'm', 'a', 'i',
    'n', 'E', 'r', 'r', 'o', 'r' };

  char_T c6_u[31];
  const mxArray *c6_y = NULL;
  for (c6_i81 = 0; c6_i81 < 31; c6_i81++) {
    c6_u[c6_i81] = c6_cv1[c6_i81];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 10, 0U, 1U, 0U, 2, 1, 31), FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U, 14,
    c6_y));
}

static void c6_b_eml_scalar_eg(SFc6_RobotSimInstanceStruct *chartInstance)
{
}

static void c6_c_eml_scalar_eg(SFc6_RobotSimInstanceStruct *chartInstance)
{
}

static const mxArray *c6_f_sf_marshallOut(void *chartInstanceVoid, void
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

static int32_T c6_f_emlrt_marshallIn(SFc6_RobotSimInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  int32_T c6_y;
  int32_T c6_i82;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_i82, 1, 6, 0U, 0, 0U, 0);
  c6_y = c6_i82;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
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
  c6_y = c6_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_sfEvent),
    &c6_thisId);
  sf_mex_destroy(&c6_b_sfEvent);
  *(int32_T *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

static uint8_T c6_g_emlrt_marshallIn(SFc6_RobotSimInstanceStruct *chartInstance,
  const mxArray *c6_b_is_active_c6_RobotSim, const char_T *c6_identifier)
{
  uint8_T c6_y;
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_h_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c6_b_is_active_c6_RobotSim), &c6_thisId);
  sf_mex_destroy(&c6_b_is_active_c6_RobotSim);
  return c6_y;
}

static uint8_T c6_h_emlrt_marshallIn(SFc6_RobotSimInstanceStruct *chartInstance,
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
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1119229202U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3905970793U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3985949180U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3764425997U);
}

mxArray *sf_c6_RobotSim_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("QRwLNlHsnWIdMbkwLly82E");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(7);
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
      pr[1] = (double)(7);
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
        _SFD_CV_INIT_SCRIPT(0,1,0,0,0,0,3,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(0,0,"bezConstraint",0,-1,647);
        _SFD_CV_INIT_SCRIPT_FOR(0,0,324,352,484);
        _SFD_CV_INIT_SCRIPT_FOR(0,1,356,370,480);
        _SFD_CV_INIT_SCRIPT_FOR(0,2,551,579,626);
        _SFD_CV_INIT_SCRIPT(1,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(1,0,"constrMatrices",0,-1,92);
        _SFD_CV_INIT_SCRIPT(2,1,0,0,0,0,1,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(2,0,"actuated",0,-1,217);
        _SFD_CV_INIT_SCRIPT_FOR(2,0,156,180,213);
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
          dimVector[1]= 7;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 7;
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
          real_T (*c6_theta_p)[7];
          real_T (*c6_alpha_p)[7];
          real_T (*c6_q)[2];
          c6_err = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          c6_theta = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c6_q = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 2);
          c6_alpha_p = (real_T (*)[7])ssGetInputPortSignal(chartInstance->S, 1);
          c6_theta_p = (real_T (*)[7])ssGetInputPortSignal(chartInstance->S, 0);
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
  return "MLWSoyZZt0uoPhvhxFmO7D";
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
  ssSetChecksum0(S,(1684622333U));
  ssSetChecksum1(S,(1196942U));
  ssSetChecksum2(S,(1996420950U));
  ssSetChecksum3(S,(2035204813U));
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
