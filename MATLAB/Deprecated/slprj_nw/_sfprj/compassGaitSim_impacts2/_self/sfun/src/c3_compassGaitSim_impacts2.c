/* Include files */

#include <stddef.h>
#include "blas.h"
#include "compassGaitSim_impacts2_sfun.h"
#include "c3_compassGaitSim_impacts2.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "compassGaitSim_impacts2_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c3_debug_family_names[8] = { "t", "n", "i", "nargin",
  "nargout", "th1", "points", "th2_des" };

/* Function Declarations */
static void initialize_c3_compassGaitSim_impacts2
  (SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance);
static void initialize_params_c3_compassGaitSim_impacts2
  (SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance);
static void enable_c3_compassGaitSim_impacts2
  (SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance);
static void disable_c3_compassGaitSim_impacts2
  (SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance);
static void c3_update_debugger_state_c3_compassGaitSim_impacts2
  (SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance);
static const mxArray *get_sim_state_c3_compassGaitSim_impacts2
  (SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance);
static void set_sim_state_c3_compassGaitSim_impacts2
  (SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance, const mxArray
   *c3_st);
static void finalize_c3_compassGaitSim_impacts2
  (SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance);
static void sf_c3_compassGaitSim_impacts2
  (SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance);
static void c3_chartstep_c3_compassGaitSim_impacts2
  (SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance);
static void initSimStructsc3_compassGaitSim_impacts2
  (SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance);
static void registerMessagesc3_compassGaitSim_impacts2
  (SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber);
static const mxArray *c3_sf_marshallOut(void *chartInstanceVoid, void *c3_inData);
static real_T c3_emlrt_marshallIn(SFc3_compassGaitSim_impacts2InstanceStruct
  *chartInstance, const mxArray *c3_th2_des, const char_T *c3_identifier);
static real_T c3_b_emlrt_marshallIn(SFc3_compassGaitSim_impacts2InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_b_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static void c3_info_helper(c3_ResolvedFunctionInfo c3_info[30]);
static void c3_eml_error(SFc3_compassGaitSim_impacts2InstanceStruct
  *chartInstance);
static void c3_eml_scalar_eg(SFc3_compassGaitSim_impacts2InstanceStruct
  *chartInstance);
static void c3_eml_warning(SFc3_compassGaitSim_impacts2InstanceStruct
  *chartInstance);
static real_T c3_mpower(SFc3_compassGaitSim_impacts2InstanceStruct
  *chartInstance, real_T c3_a, real_T c3_b);
static void c3_b_eml_error(SFc3_compassGaitSim_impacts2InstanceStruct
  *chartInstance);
static const mxArray *c3_c_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static int32_T c3_c_emlrt_marshallIn(SFc3_compassGaitSim_impacts2InstanceStruct *
  chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static uint8_T c3_d_emlrt_marshallIn(SFc3_compassGaitSim_impacts2InstanceStruct *
  chartInstance, const mxArray *c3_b_is_active_c3_compassGaitSim_impacts2, const
  char_T *c3_identifier);
static uint8_T c3_e_emlrt_marshallIn(SFc3_compassGaitSim_impacts2InstanceStruct *
  chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void init_dsm_address_info(SFc3_compassGaitSim_impacts2InstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c3_compassGaitSim_impacts2
  (SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance)
{
  chartInstance->c3_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c3_is_active_c3_compassGaitSim_impacts2 = 0U;
}

static void initialize_params_c3_compassGaitSim_impacts2
  (SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance)
{
}

static void enable_c3_compassGaitSim_impacts2
  (SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c3_compassGaitSim_impacts2
  (SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c3_update_debugger_state_c3_compassGaitSim_impacts2
  (SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c3_compassGaitSim_impacts2
  (SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance)
{
  const mxArray *c3_st;
  const mxArray *c3_y = NULL;
  real_T c3_hoistedGlobal;
  real_T c3_u;
  const mxArray *c3_b_y = NULL;
  uint8_T c3_b_hoistedGlobal;
  uint8_T c3_b_u;
  const mxArray *c3_c_y = NULL;
  real_T *c3_th2_des;
  c3_th2_des = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c3_st = NULL;
  c3_st = NULL;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_createcellarray(2), FALSE);
  c3_hoistedGlobal = *c3_th2_des;
  c3_u = c3_hoistedGlobal;
  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", &c3_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c3_y, 0, c3_b_y);
  c3_b_hoistedGlobal = chartInstance->c3_is_active_c3_compassGaitSim_impacts2;
  c3_b_u = c3_b_hoistedGlobal;
  c3_c_y = NULL;
  sf_mex_assign(&c3_c_y, sf_mex_create("y", &c3_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c3_y, 1, c3_c_y);
  sf_mex_assign(&c3_st, c3_y, FALSE);
  return c3_st;
}

static void set_sim_state_c3_compassGaitSim_impacts2
  (SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance, const mxArray
   *c3_st)
{
  const mxArray *c3_u;
  real_T *c3_th2_des;
  c3_th2_des = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c3_doneDoubleBufferReInit = TRUE;
  c3_u = sf_mex_dup(c3_st);
  *c3_th2_des = c3_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c3_u, 0)), "th2_des");
  chartInstance->c3_is_active_c3_compassGaitSim_impacts2 = c3_d_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 1)),
     "is_active_c3_compassGaitSim_impacts2");
  sf_mex_destroy(&c3_u);
  c3_update_debugger_state_c3_compassGaitSim_impacts2(chartInstance);
  sf_mex_destroy(&c3_st);
}

static void finalize_c3_compassGaitSim_impacts2
  (SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance)
{
}

static void sf_c3_compassGaitSim_impacts2
  (SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance)
{
  int32_T c3_i0;
  real_T *c3_th1;
  real_T *c3_th2_des;
  real_T (*c3_points)[12];
  c3_th2_des = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c3_points = (real_T (*)[12])ssGetInputPortSignal(chartInstance->S, 1);
  c3_th1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c3_th1, 0U);
  for (c3_i0 = 0; c3_i0 < 12; c3_i0++) {
    _SFD_DATA_RANGE_CHECK((*c3_points)[c3_i0], 1U);
  }

  _SFD_DATA_RANGE_CHECK(*c3_th2_des, 2U);
  chartInstance->c3_sfEvent = CALL_EVENT;
  c3_chartstep_c3_compassGaitSim_impacts2(chartInstance);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_compassGaitSim_impacts2MachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c3_chartstep_c3_compassGaitSim_impacts2
  (SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance)
{
  real_T c3_hoistedGlobal;
  real_T c3_th1;
  int32_T c3_i1;
  real_T c3_points[12];
  uint32_T c3_debug_family_var_map[8];
  real_T c3_t;
  real_T c3_n;
  real_T c3_i;
  real_T c3_nargin = 2.0;
  real_T c3_nargout = 1.0;
  real_T c3_th2_des;
  real_T c3_A;
  real_T c3_B;
  real_T c3_x;
  real_T c3_y;
  real_T c3_b_x;
  real_T c3_b_y;
  int32_T c3_b_i;
  real_T c3_k;
  real_T c3_c_x;
  real_T c3_d_x;
  boolean_T c3_b0;
  int32_T c3_i2;
  static char_T c3_cv0[27] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'n', 'c', 'h',
    'o', 'o', 's', 'e', 'k', ':', 'I', 'n', 'v', 'a', 'l', 'i', 'd', 'A', 'r',
    'g', '2' };

  char_T c3_u[27];
  const mxArray *c3_c_y = NULL;
  real_T c3_d_y;
  real_T c3_b_k;
  real_T c3_e_x;
  real_T c3_f_x;
  boolean_T c3_b;
  boolean_T c3_b1;
  real_T c3_g_x;
  boolean_T c3_b_b;
  boolean_T c3_b2;
  boolean_T c3_c_b;
  real_T c3_nmk;
  real_T c3_c_k;
  int32_T c3_i3;
  int32_T c3_j;
  real_T c3_b_j;
  real_T c3_a;
  real_T c3_d_b;
  real_T c3_b_a;
  real_T c3_e_b;
  real_T c3_e_y;
  real_T c3_c_a;
  real_T c3_f_b;
  real_T c3_f_y;
  real_T c3_d_a;
  real_T c3_g_b;
  real_T c3_g_y;
  real_T *c3_b_th2_des;
  real_T *c3_b_th1;
  real_T (*c3_b_points)[12];
  boolean_T guard1 = FALSE;
  c3_b_th2_des = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c3_b_points = (real_T (*)[12])ssGetInputPortSignal(chartInstance->S, 1);
  c3_b_th1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
  c3_hoistedGlobal = *c3_b_th1;
  c3_th1 = c3_hoistedGlobal;
  for (c3_i1 = 0; c3_i1 < 12; c3_i1++) {
    c3_points[c3_i1] = (*c3_b_points)[c3_i1];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 8U, 8U, c3_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_t, 0U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_n, 1U, c3_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_i, 2U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 3U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 4U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_th1, 5U, c3_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c3_points, 6U, c3_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_th2_des, 7U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 4);
  c3_A = c3_th1 - c3_points[0];
  c3_B = c3_points[5] - c3_points[0];
  c3_x = c3_A;
  c3_y = c3_B;
  c3_b_x = c3_x;
  c3_b_y = c3_y;
  c3_t = c3_b_x / c3_b_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 6);
  if (CV_EML_IF(0, 1, 0, c3_t > 1.0)) {
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 7);
    c3_t = 1.0;
  } else {
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 8);
    if (CV_EML_IF(0, 1, 1, c3_t < 0.0)) {
      _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 9);
      c3_t = 0.0;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 12);
  c3_th2_des = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 13);
  c3_n = 5.0;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 14);
  c3_i = 0.0;
  c3_b_i = 0;
  while (c3_b_i < 6) {
    c3_i = (real_T)c3_b_i;
    CV_EML_FOR(0, 1, 0, 1);
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 15);
    c3_k = c3_i;
    guard1 = FALSE;
    if (c3_k >= 0.0) {
      c3_c_x = c3_k;
      c3_d_x = c3_c_x;
      c3_d_x = muDoubleScalarFloor(c3_d_x);
      if (c3_k == c3_d_x) {
        c3_b0 = TRUE;
      } else {
        guard1 = TRUE;
      }
    } else {
      guard1 = TRUE;
    }

    if (guard1 == TRUE) {
      c3_b0 = FALSE;
    }

    if (c3_b0) {
    } else {
      for (c3_i2 = 0; c3_i2 < 27; c3_i2++) {
        c3_u[c3_i2] = c3_cv0[c3_i2];
      }

      c3_c_y = NULL;
      sf_mex_assign(&c3_c_y, sf_mex_create("y", c3_u, 10, 0U, 1U, 0U, 2, 1, 27),
                    FALSE);
      sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U,
        14, c3_c_y));
    }

    if (c3_k > 5.0) {
      c3_eml_error(chartInstance);
      c3_eml_scalar_eg(chartInstance);
      c3_d_y = rtNaN;
    } else {
      c3_b_k = c3_k;
      c3_eml_scalar_eg(chartInstance);
      c3_e_x = c3_b_k;
      c3_f_x = c3_e_x;
      c3_b = muDoubleScalarIsInf(c3_f_x);
      c3_b1 = !c3_b;
      c3_g_x = c3_e_x;
      c3_b_b = muDoubleScalarIsNaN(c3_g_x);
      c3_b2 = !c3_b_b;
      c3_c_b = (c3_b1 && c3_b2);
      if (c3_c_b) {
        if (c3_b_k > 2.5) {
          c3_b_k = 5.0 - c3_b_k;
        }

        if (c3_b_k > 1000.0) {
          c3_d_y = rtInf;
        } else {
          c3_d_y = 1.0;
          c3_nmk = 5.0 - c3_b_k;
          c3_c_k = c3_b_k;
          c3_i3 = (int32_T)c3_c_k;
          _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c3_c_k, mxDOUBLE_CLASS, c3_i3);
          for (c3_j = 0; c3_j < c3_i3; c3_j++) {
            c3_b_j = 1.0 + (real_T)c3_j;
            c3_a = c3_d_y;
            c3_d_b = (c3_b_j + c3_nmk) / c3_b_j;
            c3_d_y = c3_a * c3_d_b;
          }

          c3_d_y = muDoubleScalarRound(c3_d_y);
        }
      } else {
        c3_d_y = rtNaN;
      }

      if (!(c3_d_y <= 9.007199254740992E+15)) {
        c3_eml_warning(chartInstance);
      }
    }

    c3_b_a = c3_d_y;
    c3_e_b = c3_mpower(chartInstance, 1.0 - c3_t, c3_n - c3_i);
    c3_e_y = c3_b_a * c3_e_b;
    c3_c_a = c3_e_y;
    c3_f_b = c3_mpower(chartInstance, c3_t, c3_i);
    c3_f_y = c3_c_a * c3_f_b;
    c3_d_a = c3_f_y;
    c3_g_b = c3_points[(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("points",
      (int32_T)_SFD_INTEGER_CHECK("i+1", c3_i + 1.0), 1, 6, 1, 0) + 5];
    c3_g_y = c3_d_a * c3_g_b;
    c3_th2_des += c3_g_y;
    c3_b_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 0, 0);
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, -15);
  _SFD_SYMBOL_SCOPE_POP();
  *c3_b_th2_des = c3_th2_des;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
}

static void initSimStructsc3_compassGaitSim_impacts2
  (SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance)
{
}

static void registerMessagesc3_compassGaitSim_impacts2
  (SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber)
{
}

static const mxArray *c3_sf_marshallOut(void *chartInstanceVoid, void *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  real_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance;
  chartInstance = (SFc3_compassGaitSim_impacts2InstanceStruct *)
    chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(real_T *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, FALSE);
  return c3_mxArrayOutData;
}

static real_T c3_emlrt_marshallIn(SFc3_compassGaitSim_impacts2InstanceStruct
  *chartInstance, const mxArray *c3_th2_des, const char_T *c3_identifier)
{
  real_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_th2_des), &c3_thisId);
  sf_mex_destroy(&c3_th2_des);
  return c3_y;
}

static real_T c3_b_emlrt_marshallIn(SFc3_compassGaitSim_impacts2InstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  real_T c3_y;
  real_T c3_d0;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_d0, 1, 0, 0U, 0, 0U, 0);
  c3_y = c3_d0;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_th2_des;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y;
  SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance;
  chartInstance = (SFc3_compassGaitSim_impacts2InstanceStruct *)
    chartInstanceVoid;
  c3_th2_des = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_th2_des), &c3_thisId);
  sf_mex_destroy(&c3_th2_des);
  *(real_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_b_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i4;
  int32_T c3_i5;
  int32_T c3_i6;
  real_T c3_b_inData[12];
  int32_T c3_i7;
  int32_T c3_i8;
  int32_T c3_i9;
  real_T c3_u[12];
  const mxArray *c3_y = NULL;
  SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance;
  chartInstance = (SFc3_compassGaitSim_impacts2InstanceStruct *)
    chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_i4 = 0;
  for (c3_i5 = 0; c3_i5 < 2; c3_i5++) {
    for (c3_i6 = 0; c3_i6 < 6; c3_i6++) {
      c3_b_inData[c3_i6 + c3_i4] = (*(real_T (*)[12])c3_inData)[c3_i6 + c3_i4];
    }

    c3_i4 += 6;
  }

  c3_i7 = 0;
  for (c3_i8 = 0; c3_i8 < 2; c3_i8++) {
    for (c3_i9 = 0; c3_i9 < 6; c3_i9++) {
      c3_u[c3_i9 + c3_i7] = c3_b_inData[c3_i9 + c3_i7];
    }

    c3_i7 += 6;
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 2, 6, 2), FALSE);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, FALSE);
  return c3_mxArrayOutData;
}

const mxArray *sf_c3_compassGaitSim_impacts2_get_eml_resolved_functions_info
  (void)
{
  const mxArray *c3_nameCaptureInfo;
  c3_ResolvedFunctionInfo c3_info[30];
  const mxArray *c3_m0 = NULL;
  int32_T c3_i10;
  c3_ResolvedFunctionInfo *c3_r0;
  c3_nameCaptureInfo = NULL;
  c3_nameCaptureInfo = NULL;
  c3_info_helper(c3_info);
  sf_mex_assign(&c3_m0, sf_mex_createstruct("nameCaptureInfo", 1, 30), FALSE);
  for (c3_i10 = 0; c3_i10 < 30; c3_i10++) {
    c3_r0 = &c3_info[c3_i10];
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", c3_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c3_r0->context)), "context", "nameCaptureInfo",
                    c3_i10);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", c3_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c3_r0->name)), "name", "nameCaptureInfo", c3_i10);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", c3_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c3_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c3_i10);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", c3_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c3_r0->resolved)), "resolved", "nameCaptureInfo",
                    c3_i10);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", &c3_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c3_i10);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", &c3_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c3_i10);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", &c3_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c3_i10);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", &c3_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c3_i10);
  }

  sf_mex_assign(&c3_nameCaptureInfo, c3_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c3_nameCaptureInfo);
  return c3_nameCaptureInfo;
}

static void c3_info_helper(c3_ResolvedFunctionInfo c3_info[30])
{
  c3_info[0].context = "";
  c3_info[0].name = "mrdivide";
  c3_info[0].dominantType = "double";
  c3_info[0].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c3_info[0].fileTimeLo = 1357915548U;
  c3_info[0].fileTimeHi = 0U;
  c3_info[0].mFileTimeLo = 1319697566U;
  c3_info[0].mFileTimeHi = 0U;
  c3_info[1].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c3_info[1].name = "rdivide";
  c3_info[1].dominantType = "double";
  c3_info[1].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c3_info[1].fileTimeLo = 1346481588U;
  c3_info[1].fileTimeHi = 0U;
  c3_info[1].mFileTimeLo = 0U;
  c3_info[1].mFileTimeHi = 0U;
  c3_info[2].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c3_info[2].name = "eml_scalexp_compatible";
  c3_info[2].dominantType = "double";
  c3_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m";
  c3_info[2].fileTimeLo = 1286786396U;
  c3_info[2].fileTimeHi = 0U;
  c3_info[2].mFileTimeLo = 0U;
  c3_info[2].mFileTimeHi = 0U;
  c3_info[3].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c3_info[3].name = "eml_div";
  c3_info[3].dominantType = "double";
  c3_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c3_info[3].fileTimeLo = 1313319010U;
  c3_info[3].fileTimeHi = 0U;
  c3_info[3].mFileTimeLo = 0U;
  c3_info[3].mFileTimeHi = 0U;
  c3_info[4].context = "";
  c3_info[4].name = "nchoosek";
  c3_info[4].dominantType = "double";
  c3_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m";
  c3_info[4].fileTimeLo = 1352388862U;
  c3_info[4].fileTimeHi = 0U;
  c3_info[4].mFileTimeLo = 0U;
  c3_info[4].mFileTimeHi = 0U;
  c3_info[5].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m";
  c3_info[5].name = "floor";
  c3_info[5].dominantType = "double";
  c3_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c3_info[5].fileTimeLo = 1343801580U;
  c3_info[5].fileTimeHi = 0U;
  c3_info[5].mFileTimeLo = 0U;
  c3_info[5].mFileTimeHi = 0U;
  c3_info[6].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c3_info[6].name = "eml_scalar_floor";
  c3_info[6].dominantType = "double";
  c3_info[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c3_info[6].fileTimeLo = 1286786326U;
  c3_info[6].fileTimeHi = 0U;
  c3_info[6].mFileTimeLo = 0U;
  c3_info[6].mFileTimeHi = 0U;
  c3_info[7].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m";
  c3_info[7].name = "eml_error";
  c3_info[7].dominantType = "char";
  c3_info[7].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c3_info[7].fileTimeLo = 1343801558U;
  c3_info[7].fileTimeHi = 0U;
  c3_info[7].mFileTimeLo = 0U;
  c3_info[7].mFileTimeHi = 0U;
  c3_info[8].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m";
  c3_info[8].name = "eml_scalar_eg";
  c3_info[8].dominantType = "double";
  c3_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c3_info[8].fileTimeLo = 1286786396U;
  c3_info[8].fileTimeHi = 0U;
  c3_info[8].mFileTimeLo = 0U;
  c3_info[8].mFileTimeHi = 0U;
  c3_info[9].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m";
  c3_info[9].name = "eml_guarded_nan";
  c3_info[9].dominantType = "";
  c3_info[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m";
  c3_info[9].fileTimeLo = 1286786376U;
  c3_info[9].fileTimeHi = 0U;
  c3_info[9].mFileTimeLo = 0U;
  c3_info[9].mFileTimeHi = 0U;
  c3_info[10].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
  c3_info[10].name = "eml_scalar_eg";
  c3_info[10].dominantType = "double";
  c3_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c3_info[10].fileTimeLo = 1286786396U;
  c3_info[10].fileTimeHi = 0U;
  c3_info[10].mFileTimeLo = 0U;
  c3_info[10].mFileTimeHi = 0U;
  c3_info[11].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
  c3_info[11].name = "isfinite";
  c3_info[11].dominantType = "double";
  c3_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c3_info[11].fileTimeLo = 1286786358U;
  c3_info[11].fileTimeHi = 0U;
  c3_info[11].mFileTimeLo = 0U;
  c3_info[11].mFileTimeHi = 0U;
  c3_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c3_info[12].name = "isinf";
  c3_info[12].dominantType = "double";
  c3_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m";
  c3_info[12].fileTimeLo = 1286786360U;
  c3_info[12].fileTimeHi = 0U;
  c3_info[12].mFileTimeLo = 0U;
  c3_info[12].mFileTimeHi = 0U;
  c3_info[13].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c3_info[13].name = "isnan";
  c3_info[13].dominantType = "double";
  c3_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m";
  c3_info[13].fileTimeLo = 1286786360U;
  c3_info[13].fileTimeHi = 0U;
  c3_info[13].mFileTimeLo = 0U;
  c3_info[13].mFileTimeHi = 0U;
  c3_info[14].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
  c3_info[14].name = "eml_guarded_inf";
  c3_info[14].dominantType = "";
  c3_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_inf.m";
  c3_info[14].fileTimeLo = 1286786376U;
  c3_info[14].fileTimeHi = 0U;
  c3_info[14].mFileTimeLo = 0U;
  c3_info[14].mFileTimeHi = 0U;
  c3_info[15].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
  c3_info[15].name = "mtimes";
  c3_info[15].dominantType = "double";
  c3_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c3_info[15].fileTimeLo = 1289483692U;
  c3_info[15].fileTimeHi = 0U;
  c3_info[15].mFileTimeLo = 0U;
  c3_info[15].mFileTimeHi = 0U;
  c3_info[16].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
  c3_info[16].name = "round";
  c3_info[16].dominantType = "double";
  c3_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/round.m";
  c3_info[16].fileTimeLo = 1343801584U;
  c3_info[16].fileTimeHi = 0U;
  c3_info[16].mFileTimeLo = 0U;
  c3_info[16].mFileTimeHi = 0U;
  c3_info[17].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/round.m";
  c3_info[17].name = "eml_scalar_round";
  c3_info[17].dominantType = "double";
  c3_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_round.m";
  c3_info[17].fileTimeLo = 1307622438U;
  c3_info[17].fileTimeHi = 0U;
  c3_info[17].mFileTimeLo = 0U;
  c3_info[17].mFileTimeHi = 0U;
  c3_info[18].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
  c3_info[18].name = "eml_guarded_nan";
  c3_info[18].dominantType = "";
  c3_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m";
  c3_info[18].fileTimeLo = 1286786376U;
  c3_info[18].fileTimeHi = 0U;
  c3_info[18].mFileTimeLo = 0U;
  c3_info[18].mFileTimeHi = 0U;
  c3_info[19].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
  c3_info[19].name = "flintmax";
  c3_info[19].dominantType = "char";
  c3_info[19].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/flintmax.m";
  c3_info[19].fileTimeLo = 1348163116U;
  c3_info[19].fileTimeHi = 0U;
  c3_info[19].mFileTimeLo = 0U;
  c3_info[19].mFileTimeHi = 0U;
  c3_info[20].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/flintmax.m";
  c3_info[20].name = "eml_float_model";
  c3_info[20].dominantType = "char";
  c3_info[20].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c3_info[20].fileTimeLo = 1326691996U;
  c3_info[20].fileTimeHi = 0U;
  c3_info[20].mFileTimeLo = 0U;
  c3_info[20].mFileTimeHi = 0U;
  c3_info[21].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
  c3_info[21].name = "eml_warning";
  c3_info[21].dominantType = "char";
  c3_info[21].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c3_info[21].fileTimeLo = 1286786402U;
  c3_info[21].fileTimeHi = 0U;
  c3_info[21].mFileTimeLo = 0U;
  c3_info[21].mFileTimeHi = 0U;
  c3_info[22].context = "";
  c3_info[22].name = "mpower";
  c3_info[22].dominantType = "double";
  c3_info[22].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c3_info[22].fileTimeLo = 1286786442U;
  c3_info[22].fileTimeHi = 0U;
  c3_info[22].mFileTimeLo = 0U;
  c3_info[22].mFileTimeHi = 0U;
  c3_info[23].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c3_info[23].name = "power";
  c3_info[23].dominantType = "double";
  c3_info[23].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c3_info[23].fileTimeLo = 1348163130U;
  c3_info[23].fileTimeHi = 0U;
  c3_info[23].mFileTimeLo = 0U;
  c3_info[23].mFileTimeHi = 0U;
  c3_info[24].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c3_info[24].name = "eml_scalar_eg";
  c3_info[24].dominantType = "double";
  c3_info[24].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c3_info[24].fileTimeLo = 1286786396U;
  c3_info[24].fileTimeHi = 0U;
  c3_info[24].mFileTimeLo = 0U;
  c3_info[24].mFileTimeHi = 0U;
  c3_info[25].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c3_info[25].name = "eml_scalexp_alloc";
  c3_info[25].dominantType = "double";
  c3_info[25].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c3_info[25].fileTimeLo = 1352388860U;
  c3_info[25].fileTimeHi = 0U;
  c3_info[25].mFileTimeLo = 0U;
  c3_info[25].mFileTimeHi = 0U;
  c3_info[26].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c3_info[26].name = "floor";
  c3_info[26].dominantType = "double";
  c3_info[26].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c3_info[26].fileTimeLo = 1343801580U;
  c3_info[26].fileTimeHi = 0U;
  c3_info[26].mFileTimeLo = 0U;
  c3_info[26].mFileTimeHi = 0U;
  c3_info[27].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c3_info[27].name = "eml_error";
  c3_info[27].dominantType = "char";
  c3_info[27].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c3_info[27].fileTimeLo = 1343801558U;
  c3_info[27].fileTimeHi = 0U;
  c3_info[27].mFileTimeLo = 0U;
  c3_info[27].mFileTimeHi = 0U;
  c3_info[28].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power";
  c3_info[28].name = "eml_scalar_eg";
  c3_info[28].dominantType = "double";
  c3_info[28].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c3_info[28].fileTimeLo = 1286786396U;
  c3_info[28].fileTimeHi = 0U;
  c3_info[28].mFileTimeLo = 0U;
  c3_info[28].mFileTimeHi = 0U;
  c3_info[29].context = "";
  c3_info[29].name = "mtimes";
  c3_info[29].dominantType = "double";
  c3_info[29].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c3_info[29].fileTimeLo = 1289483692U;
  c3_info[29].fileTimeHi = 0U;
  c3_info[29].mFileTimeLo = 0U;
  c3_info[29].mFileTimeHi = 0U;
}

static void c3_eml_error(SFc3_compassGaitSim_impacts2InstanceStruct
  *chartInstance)
{
  int32_T c3_i11;
  static char_T c3_cv1[27] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'n', 'c', 'h',
    'o', 'o', 's', 'e', 'k', ':', 'K', 'O', 'u', 't', 'O', 'f', 'R', 'a', 'n',
    'g', 'e' };

  char_T c3_u[27];
  const mxArray *c3_y = NULL;
  for (c3_i11 = 0; c3_i11 < 27; c3_i11++) {
    c3_u[c3_i11] = c3_cv1[c3_i11];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 10, 0U, 1U, 0U, 2, 1, 27), FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U, 14,
    c3_y));
}

static void c3_eml_scalar_eg(SFc3_compassGaitSim_impacts2InstanceStruct
  *chartInstance)
{
}

static void c3_eml_warning(SFc3_compassGaitSim_impacts2InstanceStruct
  *chartInstance)
{
  int32_T c3_i12;
  static char_T c3_varargin_1[38] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 'n', 'c', 'h', 'o', 'o', 's', 'e', 'k', '_', 'L',
    'a', 'r', 'g', 'e', 'C', 'o', 'e', 'f', 'f', 'i', 'c', 'i', 'e', 'n', 't' };

  char_T c3_u[38];
  const mxArray *c3_y = NULL;
  for (c3_i12 = 0; c3_i12 < 38; c3_i12++) {
    c3_u[c3_i12] = c3_varargin_1[c3_i12];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 10, 0U, 1U, 0U, 2, 1, 38), FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U,
    14, c3_y));
}

static real_T c3_mpower(SFc3_compassGaitSim_impacts2InstanceStruct
  *chartInstance, real_T c3_a, real_T c3_b)
{
  real_T c3_b_a;
  real_T c3_b_b;
  real_T c3_c_a;
  real_T c3_c_b;
  real_T c3_ak;
  real_T c3_bk;
  real_T c3_x;
  real_T c3_b_x;
  real_T c3_d_a;
  real_T c3_d_b;
  real_T c3_ar;
  real_T c3_br;
  c3_b_a = c3_a;
  c3_b_b = c3_b;
  c3_c_a = c3_b_a;
  c3_c_b = c3_b_b;
  c3_eml_scalar_eg(chartInstance);
  c3_ak = c3_c_a;
  c3_bk = c3_c_b;
  if (c3_ak < 0.0) {
    c3_x = c3_bk;
    c3_b_x = c3_x;
    c3_b_x = muDoubleScalarFloor(c3_b_x);
    if (c3_b_x != c3_bk) {
      c3_b_eml_error(chartInstance);
    }
  }

  c3_d_a = c3_ak;
  c3_d_b = c3_bk;
  c3_eml_scalar_eg(chartInstance);
  c3_ar = c3_d_a;
  c3_br = c3_d_b;
  return muDoubleScalarPower(c3_ar, c3_br);
}

static void c3_b_eml_error(SFc3_compassGaitSim_impacts2InstanceStruct
  *chartInstance)
{
  int32_T c3_i13;
  static char_T c3_cv2[31] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'p', 'o', 'w', 'e', 'r', '_', 'd', 'o', 'm', 'a', 'i',
    'n', 'E', 'r', 'r', 'o', 'r' };

  char_T c3_u[31];
  const mxArray *c3_y = NULL;
  for (c3_i13 = 0; c3_i13 < 31; c3_i13++) {
    c3_u[c3_i13] = c3_cv2[c3_i13];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 10, 0U, 1U, 0U, 2, 1, 31), FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U, 14,
    c3_y));
}

static const mxArray *c3_c_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance;
  chartInstance = (SFc3_compassGaitSim_impacts2InstanceStruct *)
    chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(int32_T *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, FALSE);
  return c3_mxArrayOutData;
}

static int32_T c3_c_emlrt_marshallIn(SFc3_compassGaitSim_impacts2InstanceStruct *
  chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  int32_T c3_y;
  int32_T c3_i14;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_i14, 1, 6, 0U, 0, 0U, 0);
  c3_y = c3_i14;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_sfEvent;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  int32_T c3_y;
  SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance;
  chartInstance = (SFc3_compassGaitSim_impacts2InstanceStruct *)
    chartInstanceVoid;
  c3_b_sfEvent = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_sfEvent),
    &c3_thisId);
  sf_mex_destroy(&c3_b_sfEvent);
  *(int32_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static uint8_T c3_d_emlrt_marshallIn(SFc3_compassGaitSim_impacts2InstanceStruct *
  chartInstance, const mxArray *c3_b_is_active_c3_compassGaitSim_impacts2, const
  char_T *c3_identifier)
{
  uint8_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c3_b_is_active_c3_compassGaitSim_impacts2), &c3_thisId);
  sf_mex_destroy(&c3_b_is_active_c3_compassGaitSim_impacts2);
  return c3_y;
}

static uint8_T c3_e_emlrt_marshallIn(SFc3_compassGaitSim_impacts2InstanceStruct *
  chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  uint8_T c3_y;
  uint8_T c3_u0;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_u0, 1, 3, 0U, 0, 0U, 0);
  c3_y = c3_u0;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void init_dsm_address_info(SFc3_compassGaitSim_impacts2InstanceStruct
  *chartInstance)
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

void sf_c3_compassGaitSim_impacts2_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3576928417U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2817531326U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1660929767U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(4206020146U);
}

mxArray *sf_c3_compassGaitSim_impacts2_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("i97ULWosFKoJMk0NQRscGH");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
      pr[1] = (double)(2);
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

mxArray *sf_c3_compassGaitSim_impacts2_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c3_compassGaitSim_impacts2(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"th2_des\",},{M[8],M[0],T\"is_active_c3_compassGaitSim_impacts2\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c3_compassGaitSim_impacts2_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance;
    chartInstance = (SFc3_compassGaitSim_impacts2InstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _compassGaitSim_impacts2MachineNumber_,
           3,
           1,
           1,
           3,
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
          init_script_number_translation(_compassGaitSim_impacts2MachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_compassGaitSim_impacts2MachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _compassGaitSim_impacts2MachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"th1");
          _SFD_SET_DATA_PROPS(1,1,1,0,"points");
          _SFD_SET_DATA_PROPS(2,2,0,1,"th2_des");
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
        _SFD_CV_INIT_EML(0,1,1,2,0,0,0,1,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,305);
        _SFD_CV_INIT_EML_IF(0,1,0,121,129,141,153);
        _SFD_CV_INIT_EML_IF(0,1,1,141,153,-1,153);
        _SFD_CV_INIT_EML_FOR(0,1,0,208,222,300);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[2];
          dimVector[0]= 6;
          dimVector[1]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c3_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)c3_sf_marshallIn);

        {
          real_T *c3_th1;
          real_T *c3_th2_des;
          real_T (*c3_points)[12];
          c3_th2_des = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          c3_points = (real_T (*)[12])ssGetInputPortSignal(chartInstance->S, 1);
          c3_th1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c3_th1);
          _SFD_SET_DATA_VALUE_PTR(1U, *c3_points);
          _SFD_SET_DATA_VALUE_PTR(2U, c3_th2_des);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _compassGaitSim_impacts2MachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "3oLs4EcRG4TSjdagR3hNXB";
}

static void sf_opaque_initialize_c3_compassGaitSim_impacts2(void
  *chartInstanceVar)
{
  chart_debug_initialization(((SFc3_compassGaitSim_impacts2InstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c3_compassGaitSim_impacts2
    ((SFc3_compassGaitSim_impacts2InstanceStruct*) chartInstanceVar);
  initialize_c3_compassGaitSim_impacts2
    ((SFc3_compassGaitSim_impacts2InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c3_compassGaitSim_impacts2(void *chartInstanceVar)
{
  enable_c3_compassGaitSim_impacts2((SFc3_compassGaitSim_impacts2InstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c3_compassGaitSim_impacts2(void *chartInstanceVar)
{
  disable_c3_compassGaitSim_impacts2((SFc3_compassGaitSim_impacts2InstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c3_compassGaitSim_impacts2(void *chartInstanceVar)
{
  sf_c3_compassGaitSim_impacts2((SFc3_compassGaitSim_impacts2InstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c3_compassGaitSim_impacts2
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c3_compassGaitSim_impacts2
    ((SFc3_compassGaitSim_impacts2InstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c3_compassGaitSim_impacts2();/* state var info */
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

extern void sf_internal_set_sim_state_c3_compassGaitSim_impacts2(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c3_compassGaitSim_impacts2();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c3_compassGaitSim_impacts2
    ((SFc3_compassGaitSim_impacts2InstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c3_compassGaitSim_impacts2
  (SimStruct* S)
{
  return sf_internal_get_sim_state_c3_compassGaitSim_impacts2(S);
}

static void sf_opaque_set_sim_state_c3_compassGaitSim_impacts2(SimStruct* S,
  const mxArray *st)
{
  sf_internal_set_sim_state_c3_compassGaitSim_impacts2(S, st);
}

static void sf_opaque_terminate_c3_compassGaitSim_impacts2(void
  *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc3_compassGaitSim_impacts2InstanceStruct*)
                    chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_compassGaitSim_impacts2_optimization_info();
    }

    finalize_c3_compassGaitSim_impacts2
      ((SFc3_compassGaitSim_impacts2InstanceStruct*) chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc3_compassGaitSim_impacts2
    ((SFc3_compassGaitSim_impacts2InstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c3_compassGaitSim_impacts2(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c3_compassGaitSim_impacts2
      ((SFc3_compassGaitSim_impacts2InstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c3_compassGaitSim_impacts2(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_compassGaitSim_impacts2_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      3);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,3,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,3,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,3);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,3,2);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,3,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 2; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,3);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(4208159763U));
  ssSetChecksum1(S,(405768544U));
  ssSetChecksum2(S,(987478650U));
  ssSetChecksum3(S,(721789568U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c3_compassGaitSim_impacts2(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c3_compassGaitSim_impacts2(SimStruct *S)
{
  SFc3_compassGaitSim_impacts2InstanceStruct *chartInstance;
  chartInstance = (SFc3_compassGaitSim_impacts2InstanceStruct *)utMalloc(sizeof
    (SFc3_compassGaitSim_impacts2InstanceStruct));
  memset(chartInstance, 0, sizeof(SFc3_compassGaitSim_impacts2InstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c3_compassGaitSim_impacts2;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c3_compassGaitSim_impacts2;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c3_compassGaitSim_impacts2;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c3_compassGaitSim_impacts2;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c3_compassGaitSim_impacts2;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c3_compassGaitSim_impacts2;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c3_compassGaitSim_impacts2;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c3_compassGaitSim_impacts2;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c3_compassGaitSim_impacts2;
  chartInstance->chartInfo.mdlStart = mdlStart_c3_compassGaitSim_impacts2;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c3_compassGaitSim_impacts2;
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

void c3_compassGaitSim_impacts2_method_dispatcher(SimStruct *S, int_T method,
  void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c3_compassGaitSim_impacts2(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c3_compassGaitSim_impacts2(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c3_compassGaitSim_impacts2(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c3_compassGaitSim_impacts2_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
