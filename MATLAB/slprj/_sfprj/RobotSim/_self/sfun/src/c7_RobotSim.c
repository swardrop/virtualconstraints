/* Include files */

#include <stddef.h>
#include "blas.h"
#include "RobotSim_sfun.h"
#include "c7_RobotSim.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "RobotSim_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c7_debug_family_names[12] = { "i", "ths", "us", "K_p", "K_d",
  "nargin", "nargout", "err", "errdot", "nom_torque", "theta", "u" };

/* Function Declarations */
static void initialize_c7_RobotSim(SFc7_RobotSimInstanceStruct *chartInstance);
static void initialize_params_c7_RobotSim(SFc7_RobotSimInstanceStruct
  *chartInstance);
static void enable_c7_RobotSim(SFc7_RobotSimInstanceStruct *chartInstance);
static void disable_c7_RobotSim(SFc7_RobotSimInstanceStruct *chartInstance);
static void c7_update_debugger_state_c7_RobotSim(SFc7_RobotSimInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c7_RobotSim(SFc7_RobotSimInstanceStruct
  *chartInstance);
static void set_sim_state_c7_RobotSim(SFc7_RobotSimInstanceStruct *chartInstance,
  const mxArray *c7_st);
static void finalize_c7_RobotSim(SFc7_RobotSimInstanceStruct *chartInstance);
static void sf_c7_RobotSim(SFc7_RobotSimInstanceStruct *chartInstance);
static void c7_chartstep_c7_RobotSim(SFc7_RobotSimInstanceStruct *chartInstance);
static void initSimStructsc7_RobotSim(SFc7_RobotSimInstanceStruct *chartInstance);
static void registerMessagesc7_RobotSim(SFc7_RobotSimInstanceStruct
  *chartInstance);
static void init_script_number_translation(uint32_T c7_machineNumber, uint32_T
  c7_chartNumber);
static const mxArray *c7_sf_marshallOut(void *chartInstanceVoid, void *c7_inData);
static real_T c7_emlrt_marshallIn(SFc7_RobotSimInstanceStruct *chartInstance,
  const mxArray *c7_u, const char_T *c7_identifier);
static real_T c7_b_emlrt_marshallIn(SFc7_RobotSimInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId);
static void c7_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static const mxArray *c7_b_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static const mxArray *c7_c_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static void c7_c_emlrt_marshallIn(SFc7_RobotSimInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId, real_T c7_y[2]);
static void c7_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static const mxArray *c7_d_sf_marshallOut(void *chartInstanceVoid, real_T
  c7_inData_data[1], int32_T c7_inData_sizes[2]);
static void c7_d_emlrt_marshallIn(SFc7_RobotSimInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId, real_T c7_y_data[1],
  int32_T c7_y_sizes[2]);
static void c7_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, real_T c7_outData_data[1],
  int32_T c7_outData_sizes[2]);
static void c7_info_helper(c7_ResolvedFunctionInfo c7_info[20]);
static const mxArray *c7_e_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static int32_T c7_e_emlrt_marshallIn(SFc7_RobotSimInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId);
static void c7_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static uint8_T c7_f_emlrt_marshallIn(SFc7_RobotSimInstanceStruct *chartInstance,
  const mxArray *c7_b_is_active_c7_RobotSim, const char_T *c7_identifier);
static uint8_T c7_g_emlrt_marshallIn(SFc7_RobotSimInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId);
static void init_dsm_address_info(SFc7_RobotSimInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c7_RobotSim(SFc7_RobotSimInstanceStruct *chartInstance)
{
  chartInstance->c7_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c7_is_active_c7_RobotSim = 0U;
}

static void initialize_params_c7_RobotSim(SFc7_RobotSimInstanceStruct
  *chartInstance)
{
}

static void enable_c7_RobotSim(SFc7_RobotSimInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c7_RobotSim(SFc7_RobotSimInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c7_update_debugger_state_c7_RobotSim(SFc7_RobotSimInstanceStruct
  *chartInstance)
{
}

static const mxArray *get_sim_state_c7_RobotSim(SFc7_RobotSimInstanceStruct
  *chartInstance)
{
  const mxArray *c7_st;
  const mxArray *c7_y = NULL;
  real_T c7_hoistedGlobal;
  real_T c7_u;
  const mxArray *c7_b_y = NULL;
  uint8_T c7_b_hoistedGlobal;
  uint8_T c7_b_u;
  const mxArray *c7_c_y = NULL;
  real_T *c7_c_u;
  c7_c_u = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c7_st = NULL;
  c7_st = NULL;
  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_createcellarray(2), FALSE);
  c7_hoistedGlobal = *c7_c_u;
  c7_u = c7_hoistedGlobal;
  c7_b_y = NULL;
  sf_mex_assign(&c7_b_y, sf_mex_create("y", &c7_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c7_y, 0, c7_b_y);
  c7_b_hoistedGlobal = chartInstance->c7_is_active_c7_RobotSim;
  c7_b_u = c7_b_hoistedGlobal;
  c7_c_y = NULL;
  sf_mex_assign(&c7_c_y, sf_mex_create("y", &c7_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c7_y, 1, c7_c_y);
  sf_mex_assign(&c7_st, c7_y, FALSE);
  return c7_st;
}

static void set_sim_state_c7_RobotSim(SFc7_RobotSimInstanceStruct *chartInstance,
  const mxArray *c7_st)
{
  const mxArray *c7_u;
  real_T *c7_b_u;
  c7_b_u = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c7_doneDoubleBufferReInit = TRUE;
  c7_u = sf_mex_dup(c7_st);
  *c7_b_u = c7_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c7_u, 0)),
    "u");
  chartInstance->c7_is_active_c7_RobotSim = c7_f_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c7_u, 1)), "is_active_c7_RobotSim");
  sf_mex_destroy(&c7_u);
  c7_update_debugger_state_c7_RobotSim(chartInstance);
  sf_mex_destroy(&c7_st);
}

static void finalize_c7_RobotSim(SFc7_RobotSimInstanceStruct *chartInstance)
{
}

static void sf_c7_RobotSim(SFc7_RobotSimInstanceStruct *chartInstance)
{
  int32_T c7_i0;
  real_T *c7_err;
  real_T *c7_errdot;
  real_T *c7_theta;
  real_T *c7_u;
  real_T (*c7_nom_torque)[100];
  c7_u = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c7_theta = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c7_nom_torque = (real_T (*)[100])ssGetInputPortSignal(chartInstance->S, 2);
  c7_errdot = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c7_err = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 3U, chartInstance->c7_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c7_err, 0U);
  _SFD_DATA_RANGE_CHECK(*c7_errdot, 1U);
  for (c7_i0 = 0; c7_i0 < 100; c7_i0++) {
    _SFD_DATA_RANGE_CHECK((*c7_nom_torque)[c7_i0], 2U);
  }

  _SFD_DATA_RANGE_CHECK(*c7_theta, 3U);
  _SFD_DATA_RANGE_CHECK(*c7_u, 4U);
  chartInstance->c7_sfEvent = CALL_EVENT;
  c7_chartstep_c7_RobotSim(chartInstance);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_RobotSimMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c7_chartstep_c7_RobotSim(SFc7_RobotSimInstanceStruct *chartInstance)
{
  real_T c7_hoistedGlobal;
  real_T c7_b_hoistedGlobal;
  real_T c7_c_hoistedGlobal;
  real_T c7_err;
  real_T c7_errdot;
  int32_T c7_i1;
  real_T c7_nom_torque[100];
  real_T c7_theta;
  uint32_T c7_debug_family_var_map[12];
  int32_T c7_i_sizes[2];
  real_T c7_i_data[1];
  real_T c7_ths[2];
  real_T c7_us[2];
  real_T c7_K_p;
  real_T c7_K_d;
  real_T c7_i;
  real_T c7_nargin = 4.0;
  real_T c7_nargout = 1.0;
  real_T c7_u;
  int32_T c7_i2;
  int32_T c7_i3;
  boolean_T c7_x[50];
  int32_T c7_idx;
  int32_T c7_i4;
  int32_T c7_ii_sizes[2];
  int32_T c7_ii;
  int32_T c7_b_ii;
  int32_T c7_ii_data[1];
  int32_T c7_c_ii;
  int32_T c7_d_ii;
  int32_T c7_b_i;
  int32_T c7_c_i;
  int32_T c7_loop_ub;
  int32_T c7_i5;
  boolean_T c7_b0;
  int32_T c7_i6;
  int32_T c7_i7;
  real_T c7_A;
  real_T c7_B;
  real_T c7_b_x;
  real_T c7_y;
  real_T c7_c_x;
  real_T c7_b_y;
  real_T c7_c_y;
  real_T c7_a;
  real_T c7_b;
  real_T c7_d_y;
  real_T c7_b_b;
  real_T c7_e_y;
  real_T c7_c_b;
  real_T c7_f_y;
  real_T *c7_b_theta;
  real_T *c7_b_errdot;
  real_T *c7_b_err;
  real_T *c7_b_u;
  real_T (*c7_b_nom_torque)[100];
  boolean_T exitg1;
  c7_b_u = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c7_b_theta = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c7_b_nom_torque = (real_T (*)[100])ssGetInputPortSignal(chartInstance->S, 2);
  c7_b_errdot = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c7_b_err = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 3U, chartInstance->c7_sfEvent);
  c7_hoistedGlobal = *c7_b_err;
  c7_b_hoistedGlobal = *c7_b_errdot;
  c7_c_hoistedGlobal = *c7_b_theta;
  c7_err = c7_hoistedGlobal;
  c7_errdot = c7_b_hoistedGlobal;
  for (c7_i1 = 0; c7_i1 < 100; c7_i1++) {
    c7_nom_torque[c7_i1] = (*c7_b_nom_torque)[c7_i1];
  }

  c7_theta = c7_c_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 12U, 13U, c7_debug_family_names,
    c7_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c7_i_data, (const int32_T *)
    &c7_i_sizes, NULL, 0, -1, (void *)c7_d_sf_marshallOut, (void *)
    c7_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_ths, 1U, c7_c_sf_marshallOut,
    c7_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_us, 2U, c7_c_sf_marshallOut,
    c7_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c7_K_p, 3U, c7_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c7_K_d, 4U, c7_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_i, MAX_uint32_T, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_nargin, 5U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_nargout, 6U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c7_err, 7U, c7_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c7_errdot, 8U, c7_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c7_nom_torque, 9U, c7_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c7_theta, 10U, c7_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_u, 11U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 6);
  c7_i2 = 0;
  for (c7_i3 = 0; c7_i3 < 50; c7_i3++) {
    c7_x[c7_i3] = (c7_theta >= c7_nom_torque[c7_i2]);
    c7_i2 += 2;
  }

  c7_idx = 0;
  for (c7_i4 = 0; c7_i4 < 2; c7_i4++) {
    c7_ii_sizes[c7_i4] = 1;
  }

  c7_ii = 50;
  exitg1 = FALSE;
  while ((exitg1 == FALSE) && (c7_ii > 0)) {
    c7_b_ii = c7_ii;
    if (c7_x[c7_b_ii - 1]) {
      c7_idx = 1;
      _SFD_EML_ARRAY_BOUNDS_CHECK("", 1, 1, c7_ii_sizes[1], 1, 0);
      c7_ii_data[0] = c7_b_ii;
      exitg1 = TRUE;
    } else {
      c7_ii--;
    }
  }

  if (c7_idx == 0) {
    c7_ii_sizes[0] = 1;
    c7_ii_sizes[1] = 0;
    c7_c_ii = c7_ii_sizes[0];
    c7_d_ii = c7_ii_sizes[1];
  }

  c7_i_sizes[0] = 1;
  c7_i_sizes[1] = c7_ii_sizes[1];
  c7_b_i = c7_i_sizes[0];
  c7_c_i = c7_i_sizes[1];
  c7_loop_ub = c7_ii_sizes[0] * c7_ii_sizes[1] - 1;
  for (c7_i5 = 0; c7_i5 <= c7_loop_ub; c7_i5++) {
    c7_i_data[c7_i5] = (real_T)c7_ii_data[c7_i5];
  }

  _SFD_SYMBOL_SWITCH(0U, 0U);
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 7);
  c7_b0 = (c7_i_sizes[1] == 0);
  if (CV_EML_IF(0, 1, 0, c7_b0)) {
    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 8);
    c7_u = 0.0;
  } else {
    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 10);
    (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("i", 1, 1, c7_i_sizes[1], 1, 0);
    c7_i = c7_i_data[0];
    _SFD_SYMBOL_SWITCH(0U, 5U);
    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 11);
    if (CV_EML_IF(0, 1, 1, c7_i < 50.0)) {
      _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 12);
      for (c7_i6 = 0; c7_i6 < 2; c7_i6++) {
        c7_ths[c7_i6] = c7_nom_torque[((int32_T)(c7_i + (real_T)c7_i6) - 1) << 1];
      }

      _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 13);
      for (c7_i7 = 0; c7_i7 < 2; c7_i7++) {
        c7_us[c7_i7] = c7_nom_torque[1 + (((int32_T)(c7_i + (real_T)c7_i7) - 1) <<
          1)];
      }

      _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 14);
      c7_A = c7_us[1] - c7_us[0];
      c7_B = c7_ths[1] - c7_ths[0];
      c7_b_x = c7_A;
      c7_y = c7_B;
      c7_c_x = c7_b_x;
      c7_b_y = c7_y;
      c7_c_y = c7_c_x / c7_b_y;
      c7_a = c7_c_y;
      c7_b = c7_theta - c7_ths[0];
      c7_d_y = c7_a * c7_b;
      c7_u = c7_d_y + c7_us[0];
    } else {
      _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 16);
      c7_u = 0.0;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 19);
  c7_u = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 23);
  c7_K_p = 1000.0;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 24);
  c7_K_d = 100.0;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 26);
  c7_b_b = c7_err;
  c7_e_y = 1000.0 * c7_b_b;
  c7_c_b = c7_errdot;
  c7_f_y = 100.0 * c7_c_b;
  c7_u = c7_e_y + c7_f_y;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, -26);
  _SFD_SYMBOL_SCOPE_POP();
  *c7_b_u = c7_u;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 3U, chartInstance->c7_sfEvent);
}

static void initSimStructsc7_RobotSim(SFc7_RobotSimInstanceStruct *chartInstance)
{
}

static void registerMessagesc7_RobotSim(SFc7_RobotSimInstanceStruct
  *chartInstance)
{
}

static void init_script_number_translation(uint32_T c7_machineNumber, uint32_T
  c7_chartNumber)
{
}

static const mxArray *c7_sf_marshallOut(void *chartInstanceVoid, void *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  real_T c7_u;
  const mxArray *c7_y = NULL;
  SFc7_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc7_RobotSimInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  c7_u = *(real_T *)c7_inData;
  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", &c7_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, FALSE);
  return c7_mxArrayOutData;
}

static real_T c7_emlrt_marshallIn(SFc7_RobotSimInstanceStruct *chartInstance,
  const mxArray *c7_u, const char_T *c7_identifier)
{
  real_T c7_y;
  emlrtMsgIdentifier c7_thisId;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_y = c7_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_u), &c7_thisId);
  sf_mex_destroy(&c7_u);
  return c7_y;
}

static real_T c7_b_emlrt_marshallIn(SFc7_RobotSimInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId)
{
  real_T c7_y;
  real_T c7_d0;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), &c7_d0, 1, 0, 0U, 0, 0U, 0);
  c7_y = c7_d0;
  sf_mex_destroy(&c7_u);
  return c7_y;
}

static void c7_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_u;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  real_T c7_y;
  SFc7_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc7_RobotSimInstanceStruct *)chartInstanceVoid;
  c7_u = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_y = c7_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_u), &c7_thisId);
  sf_mex_destroy(&c7_u);
  *(real_T *)c7_outData = c7_y;
  sf_mex_destroy(&c7_mxArrayInData);
}

static const mxArray *c7_b_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_i8;
  int32_T c7_i9;
  int32_T c7_i10;
  real_T c7_b_inData[100];
  int32_T c7_i11;
  int32_T c7_i12;
  int32_T c7_i13;
  real_T c7_u[100];
  const mxArray *c7_y = NULL;
  SFc7_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc7_RobotSimInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  c7_i8 = 0;
  for (c7_i9 = 0; c7_i9 < 50; c7_i9++) {
    for (c7_i10 = 0; c7_i10 < 2; c7_i10++) {
      c7_b_inData[c7_i10 + c7_i8] = (*(real_T (*)[100])c7_inData)[c7_i10 + c7_i8];
    }

    c7_i8 += 2;
  }

  c7_i11 = 0;
  for (c7_i12 = 0; c7_i12 < 50; c7_i12++) {
    for (c7_i13 = 0; c7_i13 < 2; c7_i13++) {
      c7_u[c7_i13 + c7_i11] = c7_b_inData[c7_i13 + c7_i11];
    }

    c7_i11 += 2;
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 2, 2, 50), FALSE);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, FALSE);
  return c7_mxArrayOutData;
}

static const mxArray *c7_c_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_i14;
  real_T c7_b_inData[2];
  int32_T c7_i15;
  real_T c7_u[2];
  const mxArray *c7_y = NULL;
  SFc7_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc7_RobotSimInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  for (c7_i14 = 0; c7_i14 < 2; c7_i14++) {
    c7_b_inData[c7_i14] = (*(real_T (*)[2])c7_inData)[c7_i14];
  }

  for (c7_i15 = 0; c7_i15 < 2; c7_i15++) {
    c7_u[c7_i15] = c7_b_inData[c7_i15];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 2, 1, 2), FALSE);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, FALSE);
  return c7_mxArrayOutData;
}

static void c7_c_emlrt_marshallIn(SFc7_RobotSimInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId, real_T c7_y[2])
{
  real_T c7_dv0[2];
  int32_T c7_i16;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), c7_dv0, 1, 0, 0U, 1, 0U, 2, 1, 2);
  for (c7_i16 = 0; c7_i16 < 2; c7_i16++) {
    c7_y[c7_i16] = c7_dv0[c7_i16];
  }

  sf_mex_destroy(&c7_u);
}

static void c7_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_us;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  real_T c7_y[2];
  int32_T c7_i17;
  SFc7_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc7_RobotSimInstanceStruct *)chartInstanceVoid;
  c7_us = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_us), &c7_thisId, c7_y);
  sf_mex_destroy(&c7_us);
  for (c7_i17 = 0; c7_i17 < 2; c7_i17++) {
    (*(real_T (*)[2])c7_outData)[c7_i17] = c7_y[c7_i17];
  }

  sf_mex_destroy(&c7_mxArrayInData);
}

static const mxArray *c7_d_sf_marshallOut(void *chartInstanceVoid, real_T
  c7_inData_data[1], int32_T c7_inData_sizes[2])
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_b_inData_sizes[2];
  int32_T c7_loop_ub;
  int32_T c7_i18;
  real_T c7_b_inData_data[1];
  int32_T c7_u_sizes[2];
  int32_T c7_b_loop_ub;
  int32_T c7_i19;
  real_T c7_u_data[1];
  const mxArray *c7_y = NULL;
  SFc7_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc7_RobotSimInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  c7_b_inData_sizes[0] = 1;
  c7_b_inData_sizes[1] = c7_inData_sizes[1];
  c7_loop_ub = c7_inData_sizes[1] - 1;
  for (c7_i18 = 0; c7_i18 <= c7_loop_ub; c7_i18++) {
    c7_b_inData_data[c7_b_inData_sizes[0] * c7_i18] =
      c7_inData_data[c7_inData_sizes[0] * c7_i18];
  }

  c7_u_sizes[0] = 1;
  c7_u_sizes[1] = c7_b_inData_sizes[1];
  c7_b_loop_ub = c7_b_inData_sizes[1] - 1;
  for (c7_i19 = 0; c7_i19 <= c7_b_loop_ub; c7_i19++) {
    c7_u_data[c7_u_sizes[0] * c7_i19] = c7_b_inData_data[c7_b_inData_sizes[0] *
      c7_i19];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u_data, 0, 0U, 1U, 0U, 2,
    c7_u_sizes[0], c7_u_sizes[1]), FALSE);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, FALSE);
  return c7_mxArrayOutData;
}

static void c7_d_emlrt_marshallIn(SFc7_RobotSimInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId, real_T c7_y_data[1],
  int32_T c7_y_sizes[2])
{
  int32_T c7_i20;
  uint32_T c7_uv0[2];
  int32_T c7_i21;
  static boolean_T c7_bv0[2] = { FALSE, TRUE };

  boolean_T c7_bv1[2];
  int32_T c7_tmp_sizes[2];
  real_T c7_tmp_data[1];
  int32_T c7_y;
  int32_T c7_b_y;
  int32_T c7_loop_ub;
  int32_T c7_i22;
  for (c7_i20 = 0; c7_i20 < 2; c7_i20++) {
    c7_uv0[c7_i20] = 1U;
  }

  for (c7_i21 = 0; c7_i21 < 2; c7_i21++) {
    c7_bv1[c7_i21] = c7_bv0[c7_i21];
  }

  sf_mex_import_vs(c7_parentId, sf_mex_dup(c7_u), c7_tmp_data, 1, 0, 0U, 1, 0U,
                   2, c7_bv1, c7_uv0, c7_tmp_sizes);
  c7_y_sizes[0] = 1;
  c7_y_sizes[1] = c7_tmp_sizes[1];
  c7_y = c7_y_sizes[0];
  c7_b_y = c7_y_sizes[1];
  c7_loop_ub = c7_tmp_sizes[0] * c7_tmp_sizes[1] - 1;
  for (c7_i22 = 0; c7_i22 <= c7_loop_ub; c7_i22++) {
    c7_y_data[c7_i22] = c7_tmp_data[c7_i22];
  }

  sf_mex_destroy(&c7_u);
}

static void c7_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, real_T c7_outData_data[1],
  int32_T c7_outData_sizes[2])
{
  const mxArray *c7_i;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  int32_T c7_y_sizes[2];
  real_T c7_y_data[1];
  int32_T c7_loop_ub;
  int32_T c7_i23;
  SFc7_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc7_RobotSimInstanceStruct *)chartInstanceVoid;
  c7_i = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_i), &c7_thisId, c7_y_data,
                        c7_y_sizes);
  sf_mex_destroy(&c7_i);
  c7_outData_sizes[0] = 1;
  c7_outData_sizes[1] = c7_y_sizes[1];
  c7_loop_ub = c7_y_sizes[1] - 1;
  for (c7_i23 = 0; c7_i23 <= c7_loop_ub; c7_i23++) {
    c7_outData_data[c7_outData_sizes[0] * c7_i23] = c7_y_data[c7_y_sizes[0] *
      c7_i23];
  }

  sf_mex_destroy(&c7_mxArrayInData);
}

const mxArray *sf_c7_RobotSim_get_eml_resolved_functions_info(void)
{
  const mxArray *c7_nameCaptureInfo;
  c7_ResolvedFunctionInfo c7_info[20];
  const mxArray *c7_m0 = NULL;
  int32_T c7_i24;
  c7_ResolvedFunctionInfo *c7_r0;
  c7_nameCaptureInfo = NULL;
  c7_nameCaptureInfo = NULL;
  c7_info_helper(c7_info);
  sf_mex_assign(&c7_m0, sf_mex_createstruct("nameCaptureInfo", 1, 20), FALSE);
  for (c7_i24 = 0; c7_i24 < 20; c7_i24++) {
    c7_r0 = &c7_info[c7_i24];
    sf_mex_addfield(c7_m0, sf_mex_create("nameCaptureInfo", c7_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c7_r0->context)), "context", "nameCaptureInfo",
                    c7_i24);
    sf_mex_addfield(c7_m0, sf_mex_create("nameCaptureInfo", c7_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c7_r0->name)), "name", "nameCaptureInfo", c7_i24);
    sf_mex_addfield(c7_m0, sf_mex_create("nameCaptureInfo", c7_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c7_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c7_i24);
    sf_mex_addfield(c7_m0, sf_mex_create("nameCaptureInfo", c7_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c7_r0->resolved)), "resolved", "nameCaptureInfo",
                    c7_i24);
    sf_mex_addfield(c7_m0, sf_mex_create("nameCaptureInfo", &c7_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c7_i24);
    sf_mex_addfield(c7_m0, sf_mex_create("nameCaptureInfo", &c7_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c7_i24);
    sf_mex_addfield(c7_m0, sf_mex_create("nameCaptureInfo", &c7_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c7_i24);
    sf_mex_addfield(c7_m0, sf_mex_create("nameCaptureInfo", &c7_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c7_i24);
  }

  sf_mex_assign(&c7_nameCaptureInfo, c7_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c7_nameCaptureInfo);
  return c7_nameCaptureInfo;
}

static void c7_info_helper(c7_ResolvedFunctionInfo c7_info[20])
{
  c7_info[0].context = "";
  c7_info[0].name = "find";
  c7_info[0].dominantType = "double";
  c7_info[0].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m";
  c7_info[0].fileTimeLo = 1303117406U;
  c7_info[0].fileTimeHi = 0U;
  c7_info[0].mFileTimeLo = 0U;
  c7_info[0].mFileTimeHi = 0U;
  c7_info[1].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m!eml_find";
  c7_info[1].name = "eml_index_class";
  c7_info[1].dominantType = "";
  c7_info[1].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[1].fileTimeLo = 1323134578U;
  c7_info[1].fileTimeHi = 0U;
  c7_info[1].mFileTimeLo = 0U;
  c7_info[1].mFileTimeHi = 0U;
  c7_info[2].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m!eml_find";
  c7_info[2].name = "eml_scalar_eg";
  c7_info[2].dominantType = "logical";
  c7_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c7_info[2].fileTimeLo = 1286786396U;
  c7_info[2].fileTimeHi = 0U;
  c7_info[2].mFileTimeLo = 0U;
  c7_info[2].mFileTimeHi = 0U;
  c7_info[3].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m!eml_find";
  c7_info[3].name = "floor";
  c7_info[3].dominantType = "double";
  c7_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c7_info[3].fileTimeLo = 1343801580U;
  c7_info[3].fileTimeHi = 0U;
  c7_info[3].mFileTimeLo = 0U;
  c7_info[3].mFileTimeHi = 0U;
  c7_info[4].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c7_info[4].name = "eml_scalar_floor";
  c7_info[4].dominantType = "double";
  c7_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c7_info[4].fileTimeLo = 1286786326U;
  c7_info[4].fileTimeHi = 0U;
  c7_info[4].mFileTimeLo = 0U;
  c7_info[4].mFileTimeHi = 0U;
  c7_info[5].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m!eml_find";
  c7_info[5].name = "min";
  c7_info[5].dominantType = "coder.internal.indexInt";
  c7_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c7_info[5].fileTimeLo = 1311226518U;
  c7_info[5].fileTimeHi = 0U;
  c7_info[5].mFileTimeLo = 0U;
  c7_info[5].mFileTimeHi = 0U;
  c7_info[6].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c7_info[6].name = "eml_min_or_max";
  c7_info[6].dominantType = "char";
  c7_info[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c7_info[6].fileTimeLo = 1334042690U;
  c7_info[6].fileTimeHi = 0U;
  c7_info[6].mFileTimeLo = 0U;
  c7_info[6].mFileTimeHi = 0U;
  c7_info[7].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c7_info[7].name = "eml_scalar_eg";
  c7_info[7].dominantType = "coder.internal.indexInt";
  c7_info[7].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c7_info[7].fileTimeLo = 1286786396U;
  c7_info[7].fileTimeHi = 0U;
  c7_info[7].mFileTimeLo = 0U;
  c7_info[7].mFileTimeHi = 0U;
  c7_info[8].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c7_info[8].name = "eml_scalexp_alloc";
  c7_info[8].dominantType = "coder.internal.indexInt";
  c7_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c7_info[8].fileTimeLo = 1352388860U;
  c7_info[8].fileTimeHi = 0U;
  c7_info[8].mFileTimeLo = 0U;
  c7_info[8].mFileTimeHi = 0U;
  c7_info[9].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c7_info[9].name = "eml_index_class";
  c7_info[9].dominantType = "";
  c7_info[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[9].fileTimeLo = 1323134578U;
  c7_info[9].fileTimeHi = 0U;
  c7_info[9].mFileTimeLo = 0U;
  c7_info[9].mFileTimeHi = 0U;
  c7_info[10].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c7_info[10].name = "eml_scalar_eg";
  c7_info[10].dominantType = "coder.internal.indexInt";
  c7_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c7_info[10].fileTimeLo = 1286786396U;
  c7_info[10].fileTimeHi = 0U;
  c7_info[10].mFileTimeLo = 0U;
  c7_info[10].mFileTimeHi = 0U;
  c7_info[11].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m!eml_find";
  c7_info[11].name = "eml_int_forloop_overflow_check";
  c7_info[11].dominantType = "";
  c7_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c7_info[11].fileTimeLo = 1346481540U;
  c7_info[11].fileTimeHi = 0U;
  c7_info[11].mFileTimeLo = 0U;
  c7_info[11].mFileTimeHi = 0U;
  c7_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c7_info[12].name = "intmin";
  c7_info[12].dominantType = "char";
  c7_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c7_info[12].fileTimeLo = 1311226518U;
  c7_info[12].fileTimeHi = 0U;
  c7_info[12].mFileTimeLo = 0U;
  c7_info[12].mFileTimeHi = 0U;
  c7_info[13].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m!eml_find";
  c7_info[13].name = "eml_index_plus";
  c7_info[13].dominantType = "double";
  c7_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c7_info[13].fileTimeLo = 1286786378U;
  c7_info[13].fileTimeHi = 0U;
  c7_info[13].mFileTimeLo = 0U;
  c7_info[13].mFileTimeHi = 0U;
  c7_info[14].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c7_info[14].name = "eml_index_class";
  c7_info[14].dominantType = "";
  c7_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[14].fileTimeLo = 1323134578U;
  c7_info[14].fileTimeHi = 0U;
  c7_info[14].mFileTimeLo = 0U;
  c7_info[14].mFileTimeHi = 0U;
  c7_info[15].context = "";
  c7_info[15].name = "mrdivide";
  c7_info[15].dominantType = "double";
  c7_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c7_info[15].fileTimeLo = 1357915548U;
  c7_info[15].fileTimeHi = 0U;
  c7_info[15].mFileTimeLo = 1319697566U;
  c7_info[15].mFileTimeHi = 0U;
  c7_info[16].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c7_info[16].name = "rdivide";
  c7_info[16].dominantType = "double";
  c7_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c7_info[16].fileTimeLo = 1346481588U;
  c7_info[16].fileTimeHi = 0U;
  c7_info[16].mFileTimeLo = 0U;
  c7_info[16].mFileTimeHi = 0U;
  c7_info[17].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c7_info[17].name = "eml_scalexp_compatible";
  c7_info[17].dominantType = "double";
  c7_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m";
  c7_info[17].fileTimeLo = 1286786396U;
  c7_info[17].fileTimeHi = 0U;
  c7_info[17].mFileTimeLo = 0U;
  c7_info[17].mFileTimeHi = 0U;
  c7_info[18].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c7_info[18].name = "eml_div";
  c7_info[18].dominantType = "double";
  c7_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c7_info[18].fileTimeLo = 1313319010U;
  c7_info[18].fileTimeHi = 0U;
  c7_info[18].mFileTimeLo = 0U;
  c7_info[18].mFileTimeHi = 0U;
  c7_info[19].context = "";
  c7_info[19].name = "mtimes";
  c7_info[19].dominantType = "double";
  c7_info[19].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c7_info[19].fileTimeLo = 1289483692U;
  c7_info[19].fileTimeHi = 0U;
  c7_info[19].mFileTimeLo = 0U;
  c7_info[19].mFileTimeHi = 0U;
}

static const mxArray *c7_e_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_u;
  const mxArray *c7_y = NULL;
  SFc7_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc7_RobotSimInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  c7_u = *(int32_T *)c7_inData;
  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", &c7_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, FALSE);
  return c7_mxArrayOutData;
}

static int32_T c7_e_emlrt_marshallIn(SFc7_RobotSimInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId)
{
  int32_T c7_y;
  int32_T c7_i25;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), &c7_i25, 1, 6, 0U, 0, 0U, 0);
  c7_y = c7_i25;
  sf_mex_destroy(&c7_u);
  return c7_y;
}

static void c7_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_b_sfEvent;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  int32_T c7_y;
  SFc7_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc7_RobotSimInstanceStruct *)chartInstanceVoid;
  c7_b_sfEvent = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_y = c7_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_b_sfEvent),
    &c7_thisId);
  sf_mex_destroy(&c7_b_sfEvent);
  *(int32_T *)c7_outData = c7_y;
  sf_mex_destroy(&c7_mxArrayInData);
}

static uint8_T c7_f_emlrt_marshallIn(SFc7_RobotSimInstanceStruct *chartInstance,
  const mxArray *c7_b_is_active_c7_RobotSim, const char_T *c7_identifier)
{
  uint8_T c7_y;
  emlrtMsgIdentifier c7_thisId;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_y = c7_g_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c7_b_is_active_c7_RobotSim), &c7_thisId);
  sf_mex_destroy(&c7_b_is_active_c7_RobotSim);
  return c7_y;
}

static uint8_T c7_g_emlrt_marshallIn(SFc7_RobotSimInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId)
{
  uint8_T c7_y;
  uint8_T c7_u0;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), &c7_u0, 1, 3, 0U, 0, 0U, 0);
  c7_y = c7_u0;
  sf_mex_destroy(&c7_u);
  return c7_y;
}

static void init_dsm_address_info(SFc7_RobotSimInstanceStruct *chartInstance)
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

void sf_c7_RobotSim_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(451915789U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(223678804U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1931130197U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1213839455U);
}

mxArray *sf_c7_RobotSim_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("AkFr78uRJxOSghJe4T99xD");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(2);
      pr[1] = (double)(50);
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

mxArray *sf_c7_RobotSim_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c7_RobotSim(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"u\",},{M[8],M[0],T\"is_active_c7_RobotSim\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c7_RobotSim_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc7_RobotSimInstanceStruct *chartInstance;
    chartInstance = (SFc7_RobotSimInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _RobotSimMachineNumber_,
           7,
           1,
           1,
           5,
           0,
           0,
           0,
           0,
           0,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"err");
          _SFD_SET_DATA_PROPS(1,1,1,0,"errdot");
          _SFD_SET_DATA_PROPS(2,1,1,0,"nom_torque");
          _SFD_SET_DATA_PROPS(3,1,1,0,"theta");
          _SFD_SET_DATA_PROPS(4,2,0,1,"u");
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
        _SFD_CV_INIT_EML(0,1,1,2,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,642);
        _SFD_CV_INIT_EML_IF(0,1,0,193,206,233,478);
        _SFD_CV_INIT_EML_IF(0,1,1,257,282,432,474);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c7_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c7_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[2];
          dimVector[0]= 2;
          dimVector[1]= 50;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c7_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c7_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c7_sf_marshallOut,(MexInFcnForType)c7_sf_marshallIn);

        {
          real_T *c7_err;
          real_T *c7_errdot;
          real_T *c7_theta;
          real_T *c7_u;
          real_T (*c7_nom_torque)[100];
          c7_u = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          c7_theta = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c7_nom_torque = (real_T (*)[100])ssGetInputPortSignal(chartInstance->S,
            2);
          c7_errdot = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c7_err = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c7_err);
          _SFD_SET_DATA_VALUE_PTR(1U, c7_errdot);
          _SFD_SET_DATA_VALUE_PTR(2U, *c7_nom_torque);
          _SFD_SET_DATA_VALUE_PTR(3U, c7_theta);
          _SFD_SET_DATA_VALUE_PTR(4U, c7_u);
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
  return "hOKVJMcblS8OCPUqsNvvVG";
}

static void sf_opaque_initialize_c7_RobotSim(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc7_RobotSimInstanceStruct*) chartInstanceVar)
    ->S,0);
  initialize_params_c7_RobotSim((SFc7_RobotSimInstanceStruct*) chartInstanceVar);
  initialize_c7_RobotSim((SFc7_RobotSimInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c7_RobotSim(void *chartInstanceVar)
{
  enable_c7_RobotSim((SFc7_RobotSimInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c7_RobotSim(void *chartInstanceVar)
{
  disable_c7_RobotSim((SFc7_RobotSimInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c7_RobotSim(void *chartInstanceVar)
{
  sf_c7_RobotSim((SFc7_RobotSimInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c7_RobotSim(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c7_RobotSim((SFc7_RobotSimInstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c7_RobotSim();/* state var info */
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

extern void sf_internal_set_sim_state_c7_RobotSim(SimStruct* S, const mxArray
  *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c7_RobotSim();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c7_RobotSim((SFc7_RobotSimInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c7_RobotSim(SimStruct* S)
{
  return sf_internal_get_sim_state_c7_RobotSim(S);
}

static void sf_opaque_set_sim_state_c7_RobotSim(SimStruct* S, const mxArray *st)
{
  sf_internal_set_sim_state_c7_RobotSim(S, st);
}

static void sf_opaque_terminate_c7_RobotSim(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc7_RobotSimInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_RobotSim_optimization_info();
    }

    finalize_c7_RobotSim((SFc7_RobotSimInstanceStruct*) chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc7_RobotSim((SFc7_RobotSimInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c7_RobotSim(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c7_RobotSim((SFc7_RobotSimInstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c7_RobotSim(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_RobotSim_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      7);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,7,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,7,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,7);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,7,4);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,7,1);
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

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,7);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(3097494989U));
  ssSetChecksum1(S,(3119990425U));
  ssSetChecksum2(S,(4246176976U));
  ssSetChecksum3(S,(1372100205U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c7_RobotSim(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c7_RobotSim(SimStruct *S)
{
  SFc7_RobotSimInstanceStruct *chartInstance;
  chartInstance = (SFc7_RobotSimInstanceStruct *)utMalloc(sizeof
    (SFc7_RobotSimInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc7_RobotSimInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c7_RobotSim;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c7_RobotSim;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c7_RobotSim;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c7_RobotSim;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c7_RobotSim;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c7_RobotSim;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c7_RobotSim;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c7_RobotSim;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c7_RobotSim;
  chartInstance->chartInfo.mdlStart = mdlStart_c7_RobotSim;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c7_RobotSim;
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

void c7_RobotSim_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c7_RobotSim(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c7_RobotSim(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c7_RobotSim(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c7_RobotSim_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
