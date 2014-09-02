/* Include files */

#include <stddef.h>
#include "blas.h"
#include "compassGaitSim_impacts_sfun.h"
#include "c4_compassGaitSim_impacts.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "compassGaitSim_impacts_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c4_debug_family_names[19] = { "HORIZ_THRESH",
  "EXCEPTION_THRESH", "delta", "isOnHoriz", "isShorter", "nargin", "nargout",
  "t2", "t2d", "t1", "t1d", "I1", "I2", "prev_impact", "t2d_", "t2_", "t1d_",
  "t1_", "impact" };

/* Function Declarations */
static void initialize_c4_compassGaitSim_impacts
  (SFc4_compassGaitSim_impactsInstanceStruct *chartInstance);
static void initialize_params_c4_compassGaitSim_impacts
  (SFc4_compassGaitSim_impactsInstanceStruct *chartInstance);
static void enable_c4_compassGaitSim_impacts
  (SFc4_compassGaitSim_impactsInstanceStruct *chartInstance);
static void disable_c4_compassGaitSim_impacts
  (SFc4_compassGaitSim_impactsInstanceStruct *chartInstance);
static void c4_update_debugger_state_c4_compassGaitSim_impacts
  (SFc4_compassGaitSim_impactsInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c4_compassGaitSim_impacts
  (SFc4_compassGaitSim_impactsInstanceStruct *chartInstance);
static void set_sim_state_c4_compassGaitSim_impacts
  (SFc4_compassGaitSim_impactsInstanceStruct *chartInstance, const mxArray
   *c4_st);
static void finalize_c4_compassGaitSim_impacts
  (SFc4_compassGaitSim_impactsInstanceStruct *chartInstance);
static void sf_c4_compassGaitSim_impacts
  (SFc4_compassGaitSim_impactsInstanceStruct *chartInstance);
static void c4_chartstep_c4_compassGaitSim_impacts
  (SFc4_compassGaitSim_impactsInstanceStruct *chartInstance);
static void initSimStructsc4_compassGaitSim_impacts
  (SFc4_compassGaitSim_impactsInstanceStruct *chartInstance);
static void registerMessagesc4_compassGaitSim_impacts
  (SFc4_compassGaitSim_impactsInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c4_machineNumber, uint32_T
  c4_chartNumber);
static const mxArray *c4_sf_marshallOut(void *chartInstanceVoid, void *c4_inData);
static boolean_T c4_emlrt_marshallIn(SFc4_compassGaitSim_impactsInstanceStruct
  *chartInstance, const mxArray *c4_impact, const char_T *c4_identifier);
static boolean_T c4_b_emlrt_marshallIn(SFc4_compassGaitSim_impactsInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void c4_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static const mxArray *c4_b_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static real_T c4_c_emlrt_marshallIn(SFc4_compassGaitSim_impactsInstanceStruct
  *chartInstance, const mxArray *c4_t1_, const char_T *c4_identifier);
static real_T c4_d_emlrt_marshallIn(SFc4_compassGaitSim_impactsInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void c4_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static const mxArray *c4_c_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static void c4_info_helper(c4_ResolvedFunctionInfo c4_info[19]);
static const mxArray *c4_d_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static int32_T c4_e_emlrt_marshallIn(SFc4_compassGaitSim_impactsInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void c4_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static uint8_T c4_f_emlrt_marshallIn(SFc4_compassGaitSim_impactsInstanceStruct
  *chartInstance, const mxArray *c4_b_is_active_c4_compassGaitSim_impacts, const
  char_T *c4_identifier);
static uint8_T c4_g_emlrt_marshallIn(SFc4_compassGaitSim_impactsInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void init_dsm_address_info(SFc4_compassGaitSim_impactsInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c4_compassGaitSim_impacts
  (SFc4_compassGaitSim_impactsInstanceStruct *chartInstance)
{
  chartInstance->c4_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c4_is_active_c4_compassGaitSim_impacts = 0U;
}

static void initialize_params_c4_compassGaitSim_impacts
  (SFc4_compassGaitSim_impactsInstanceStruct *chartInstance)
{
}

static void enable_c4_compassGaitSim_impacts
  (SFc4_compassGaitSim_impactsInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c4_compassGaitSim_impacts
  (SFc4_compassGaitSim_impactsInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c4_update_debugger_state_c4_compassGaitSim_impacts
  (SFc4_compassGaitSim_impactsInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c4_compassGaitSim_impacts
  (SFc4_compassGaitSim_impactsInstanceStruct *chartInstance)
{
  const mxArray *c4_st;
  const mxArray *c4_y = NULL;
  boolean_T c4_hoistedGlobal;
  boolean_T c4_u;
  const mxArray *c4_b_y = NULL;
  real_T c4_b_hoistedGlobal;
  real_T c4_b_u;
  const mxArray *c4_c_y = NULL;
  real_T c4_c_hoistedGlobal;
  real_T c4_c_u;
  const mxArray *c4_d_y = NULL;
  real_T c4_d_hoistedGlobal;
  real_T c4_d_u;
  const mxArray *c4_e_y = NULL;
  real_T c4_e_hoistedGlobal;
  real_T c4_e_u;
  const mxArray *c4_f_y = NULL;
  uint8_T c4_f_hoistedGlobal;
  uint8_T c4_f_u;
  const mxArray *c4_g_y = NULL;
  boolean_T *c4_impact;
  real_T *c4_t1_;
  real_T *c4_t1d_;
  real_T *c4_t2_;
  real_T *c4_t2d_;
  c4_impact = (boolean_T *)ssGetOutputPortSignal(chartInstance->S, 5);
  c4_t1_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c4_t1d_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c4_t2_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c4_t2d_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c4_st = NULL;
  c4_st = NULL;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_createcellarray(6), FALSE);
  c4_hoistedGlobal = *c4_impact;
  c4_u = c4_hoistedGlobal;
  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", &c4_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c4_y, 0, c4_b_y);
  c4_b_hoistedGlobal = *c4_t1_;
  c4_b_u = c4_b_hoistedGlobal;
  c4_c_y = NULL;
  sf_mex_assign(&c4_c_y, sf_mex_create("y", &c4_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c4_y, 1, c4_c_y);
  c4_c_hoistedGlobal = *c4_t1d_;
  c4_c_u = c4_c_hoistedGlobal;
  c4_d_y = NULL;
  sf_mex_assign(&c4_d_y, sf_mex_create("y", &c4_c_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c4_y, 2, c4_d_y);
  c4_d_hoistedGlobal = *c4_t2_;
  c4_d_u = c4_d_hoistedGlobal;
  c4_e_y = NULL;
  sf_mex_assign(&c4_e_y, sf_mex_create("y", &c4_d_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c4_y, 3, c4_e_y);
  c4_e_hoistedGlobal = *c4_t2d_;
  c4_e_u = c4_e_hoistedGlobal;
  c4_f_y = NULL;
  sf_mex_assign(&c4_f_y, sf_mex_create("y", &c4_e_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c4_y, 4, c4_f_y);
  c4_f_hoistedGlobal = chartInstance->c4_is_active_c4_compassGaitSim_impacts;
  c4_f_u = c4_f_hoistedGlobal;
  c4_g_y = NULL;
  sf_mex_assign(&c4_g_y, sf_mex_create("y", &c4_f_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c4_y, 5, c4_g_y);
  sf_mex_assign(&c4_st, c4_y, FALSE);
  return c4_st;
}

static void set_sim_state_c4_compassGaitSim_impacts
  (SFc4_compassGaitSim_impactsInstanceStruct *chartInstance, const mxArray
   *c4_st)
{
  const mxArray *c4_u;
  boolean_T *c4_impact;
  real_T *c4_t1_;
  real_T *c4_t1d_;
  real_T *c4_t2_;
  real_T *c4_t2d_;
  c4_impact = (boolean_T *)ssGetOutputPortSignal(chartInstance->S, 5);
  c4_t1_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c4_t1d_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c4_t2_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c4_t2d_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c4_doneDoubleBufferReInit = TRUE;
  c4_u = sf_mex_dup(c4_st);
  *c4_impact = c4_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c4_u,
    0)), "impact");
  *c4_t1_ = c4_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c4_u,
    1)), "t1_");
  *c4_t1d_ = c4_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c4_u,
    2)), "t1d_");
  *c4_t2_ = c4_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c4_u,
    3)), "t2_");
  *c4_t2d_ = c4_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c4_u,
    4)), "t2d_");
  chartInstance->c4_is_active_c4_compassGaitSim_impacts = c4_f_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c4_u, 5)),
     "is_active_c4_compassGaitSim_impacts");
  sf_mex_destroy(&c4_u);
  c4_update_debugger_state_c4_compassGaitSim_impacts(chartInstance);
  sf_mex_destroy(&c4_st);
}

static void finalize_c4_compassGaitSim_impacts
  (SFc4_compassGaitSim_impactsInstanceStruct *chartInstance)
{
}

static void sf_c4_compassGaitSim_impacts
  (SFc4_compassGaitSim_impactsInstanceStruct *chartInstance)
{
  int32_T c4_i0;
  real_T *c4_t2d_;
  real_T *c4_t2_;
  real_T *c4_t1d_;
  real_T *c4_t1_;
  real_T *c4_t2;
  real_T *c4_t2d;
  real_T *c4_t1;
  real_T *c4_t1d;
  real_T *c4_I1;
  real_T *c4_I2;
  boolean_T *c4_impact;
  real_T (*c4_prev_impact)[10];
  c4_prev_impact = (real_T (*)[10])ssGetInputPortSignal(chartInstance->S, 6);
  c4_impact = (boolean_T *)ssGetOutputPortSignal(chartInstance->S, 5);
  c4_I2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
  c4_I1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c4_t1d = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c4_t1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c4_t2d = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c4_t2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  c4_t1_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c4_t1d_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c4_t2_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c4_t2d_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 3U, chartInstance->c4_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c4_t2d_, 0U);
  _SFD_DATA_RANGE_CHECK(*c4_t2_, 1U);
  _SFD_DATA_RANGE_CHECK(*c4_t1d_, 2U);
  _SFD_DATA_RANGE_CHECK(*c4_t1_, 3U);
  _SFD_DATA_RANGE_CHECK(*c4_t2, 4U);
  _SFD_DATA_RANGE_CHECK(*c4_t2d, 5U);
  _SFD_DATA_RANGE_CHECK(*c4_t1, 6U);
  _SFD_DATA_RANGE_CHECK(*c4_t1d, 7U);
  _SFD_DATA_RANGE_CHECK(*c4_I1, 8U);
  _SFD_DATA_RANGE_CHECK(*c4_I2, 9U);
  _SFD_DATA_RANGE_CHECK((real_T)*c4_impact, 10U);
  for (c4_i0 = 0; c4_i0 < 10; c4_i0++) {
    _SFD_DATA_RANGE_CHECK((*c4_prev_impact)[c4_i0], 11U);
  }

  chartInstance->c4_sfEvent = CALL_EVENT;
  c4_chartstep_c4_compassGaitSim_impacts(chartInstance);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_compassGaitSim_impactsMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c4_chartstep_c4_compassGaitSim_impacts
  (SFc4_compassGaitSim_impactsInstanceStruct *chartInstance)
{
  real_T c4_hoistedGlobal;
  real_T c4_b_hoistedGlobal;
  real_T c4_c_hoistedGlobal;
  real_T c4_d_hoistedGlobal;
  real_T c4_e_hoistedGlobal;
  real_T c4_f_hoistedGlobal;
  real_T c4_t2;
  real_T c4_t2d;
  real_T c4_t1;
  real_T c4_t1d;
  real_T c4_I1;
  real_T c4_I2;
  int32_T c4_i1;
  real_T c4_prev_impact[10];
  uint32_T c4_debug_family_var_map[19];
  real_T c4_HORIZ_THRESH;
  real_T c4_EXCEPTION_THRESH;
  real_T c4_delta;
  boolean_T c4_isOnHoriz;
  boolean_T c4_isShorter;
  real_T c4_nargin = 7.0;
  real_T c4_nargout = 5.0;
  real_T c4_t2d_;
  real_T c4_t2_;
  real_T c4_t1d_;
  real_T c4_t1_;
  boolean_T c4_impact;
  real_T c4_b;
  real_T c4_y;
  real_T c4_x;
  real_T c4_b_x;
  real_T c4_b_y;
  int32_T c4_i2;
  boolean_T c4_varargin_1[10];
  boolean_T c4_mtmp;
  int32_T c4_ix;
  int32_T c4_b_ix;
  boolean_T c4_a;
  boolean_T c4_b_b;
  boolean_T c4_p;
  boolean_T c4_b_mtmp;
  boolean_T c4_minval;
  real_T c4_b_a;
  real_T c4_c_b;
  real_T c4_c_y;
  real_T c4_c_a;
  real_T c4_d_b;
  real_T c4_d_y;
  real_T c4_e_b;
  real_T c4_e_y;
  real_T c4_d_a;
  real_T c4_f_b;
  real_T c4_f_y;
  real_T c4_e_a;
  real_T c4_g_b;
  real_T c4_g_y;
  real_T c4_A;
  real_T c4_B;
  real_T c4_c_x;
  real_T c4_h_y;
  real_T c4_d_x;
  real_T c4_i_y;
  real_T c4_f_a;
  real_T c4_h_b;
  real_T c4_g_a;
  real_T c4_i_b;
  boolean_T *c4_b_impact;
  real_T *c4_b_t1_;
  real_T *c4_b_t1d_;
  real_T *c4_b_t2_;
  real_T *c4_b_t2d_;
  real_T *c4_b_I2;
  real_T *c4_b_I1;
  real_T *c4_b_t1d;
  real_T *c4_b_t1;
  real_T *c4_b_t2d;
  real_T *c4_b_t2;
  real_T (*c4_b_prev_impact)[10];
  boolean_T guard1 = FALSE;
  boolean_T guard2 = FALSE;
  c4_b_prev_impact = (real_T (*)[10])ssGetInputPortSignal(chartInstance->S, 6);
  c4_b_impact = (boolean_T *)ssGetOutputPortSignal(chartInstance->S, 5);
  c4_b_I2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
  c4_b_I1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c4_b_t1d = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c4_b_t1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c4_b_t2d = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c4_b_t2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  c4_b_t1_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c4_b_t1d_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c4_b_t2_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c4_b_t2d_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 3U, chartInstance->c4_sfEvent);
  c4_hoistedGlobal = *c4_b_t2;
  c4_b_hoistedGlobal = *c4_b_t2d;
  c4_c_hoistedGlobal = *c4_b_t1;
  c4_d_hoistedGlobal = *c4_b_t1d;
  c4_e_hoistedGlobal = *c4_b_I1;
  c4_f_hoistedGlobal = *c4_b_I2;
  c4_t2 = c4_hoistedGlobal;
  c4_t2d = c4_b_hoistedGlobal;
  c4_t1 = c4_c_hoistedGlobal;
  c4_t1d = c4_d_hoistedGlobal;
  c4_I1 = c4_e_hoistedGlobal;
  c4_I2 = c4_f_hoistedGlobal;
  for (c4_i1 = 0; c4_i1 < 10; c4_i1++) {
    c4_prev_impact[c4_i1] = (*c4_b_prev_impact)[c4_i1];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 19U, 19U, c4_debug_family_names,
    c4_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_HORIZ_THRESH, 0U, c4_b_sf_marshallOut,
    c4_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_EXCEPTION_THRESH, 1U,
    c4_b_sf_marshallOut, c4_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_delta, 2U, c4_b_sf_marshallOut,
    c4_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_isOnHoriz, 3U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_isShorter, 4U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_nargin, 5U, c4_b_sf_marshallOut,
    c4_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_nargout, 6U, c4_b_sf_marshallOut,
    c4_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c4_t2, 7U, c4_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c4_t2d, 8U, c4_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c4_t1, 9U, c4_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c4_t1d, 10U, c4_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c4_I1, 11U, c4_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c4_I2, 12U, c4_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c4_prev_impact, 13U, c4_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_t2d_, 14U, c4_b_sf_marshallOut,
    c4_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_t2_, 15U, c4_b_sf_marshallOut,
    c4_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_t1d_, 16U, c4_b_sf_marshallOut,
    c4_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_t1_, 17U, c4_b_sf_marshallOut,
    c4_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_impact, 18U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 5);
  c4_HORIZ_THRESH = 0.03;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 6);
  c4_EXCEPTION_THRESH = 0.1;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 7);
  c4_delta = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 9);
  c4_b = c4_t1;
  c4_y = 2.0 * c4_b;
  c4_x = c4_t2 + c4_y;
  c4_b_x = c4_x;
  c4_b_y = muDoubleScalarAbs(c4_b_x);
  c4_isOnHoriz = (c4_b_y < c4_HORIZ_THRESH);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 11);
  c4_isShorter = (c4_t2 + 3.1415926535897931 < c4_EXCEPTION_THRESH);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 14);
  guard1 = FALSE;
  guard2 = FALSE;
  if (CV_EML_COND(0, 1, 0, c4_isOnHoriz)) {
    if (!CV_EML_COND(0, 1, 1, c4_isShorter)) {
      for (c4_i2 = 0; c4_i2 < 10; c4_i2++) {
        c4_varargin_1[c4_i2] = !(c4_prev_impact[c4_i2] != 0.0);
      }

      c4_mtmp = c4_varargin_1[0];
      for (c4_ix = 2; c4_ix < 11; c4_ix++) {
        c4_b_ix = c4_ix;
        c4_a = c4_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c4_b_ix), 1, 10, 1, 0) - 1];
        c4_b_b = c4_mtmp;
        c4_p = ((int32_T)c4_a < (int32_T)c4_b_b);
        if (c4_p) {
          c4_mtmp = c4_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c4_b_ix), 1, 10, 1, 0) - 1];
        }
      }

      c4_b_mtmp = c4_mtmp;
      c4_minval = c4_b_mtmp;
      if (CV_EML_COND(0, 1, 2, c4_minval)) {
        CV_EML_MCDC(0, 1, 0, TRUE);
        CV_EML_IF(0, 1, 0, TRUE);
        _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 15);
        c4_t1_ = 3.1415926535897931 - c4_t1;
        _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 16);
        c4_t2_ = -6.2831853071795862 - c4_t2;
        _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 18);
        c4_b_a = c4_t1d;
        c4_c_b = c4_I1 + c4_I2;
        c4_c_y = c4_b_a * c4_c_b;
        c4_c_a = c4_t2d;
        c4_d_b = c4_I2;
        c4_d_y = c4_c_a * c4_d_b;
        c4_e_b = c4_I2;
        c4_e_y = 2.0 * c4_e_b;
        c4_d_a = c4_t1d;
        c4_f_b = c4_I1 + c4_e_y;
        c4_f_y = c4_d_a * c4_f_b;
        c4_e_a = c4_t2d;
        c4_g_b = c4_I1 + c4_I2;
        c4_g_y = c4_e_a * c4_g_b;
        c4_A = c4_c_y + c4_d_y;
        c4_B = c4_f_y + c4_g_y;
        c4_c_x = c4_A;
        c4_h_y = c4_B;
        c4_d_x = c4_c_x;
        c4_i_y = c4_h_y;
        c4_delta = c4_d_x / c4_i_y;
        _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 19);
        c4_f_a = c4_delta;
        c4_h_b = c4_t1d;
        c4_t1d = c4_f_a * c4_h_b;
        _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 20);
        c4_g_a = c4_delta;
        c4_i_b = c4_t2d;
        c4_t2d = c4_g_a * c4_i_b;
        _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 22);
        c4_t1d_ = c4_t1d + c4_t2d;
        _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 23);
        c4_t2d_ = c4_t2d;
        _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 24);
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
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 26);
    c4_t1_ = c4_t1;
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 27);
    c4_t2_ = c4_t2;
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 28);
    c4_t1d_ = c4_t1d;
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 29);
    c4_t2d_ = c4_t2d;
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 30);
    c4_impact = FALSE;
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, -30);
  _SFD_SYMBOL_SCOPE_POP();
  *c4_b_t2d_ = c4_t2d_;
  *c4_b_t2_ = c4_t2_;
  *c4_b_t1d_ = c4_t1d_;
  *c4_b_t1_ = c4_t1_;
  *c4_b_impact = c4_impact;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 3U, chartInstance->c4_sfEvent);
}

static void initSimStructsc4_compassGaitSim_impacts
  (SFc4_compassGaitSim_impactsInstanceStruct *chartInstance)
{
}

static void registerMessagesc4_compassGaitSim_impacts
  (SFc4_compassGaitSim_impactsInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c4_machineNumber, uint32_T
  c4_chartNumber)
{
}

static const mxArray *c4_sf_marshallOut(void *chartInstanceVoid, void *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  boolean_T c4_u;
  const mxArray *c4_y = NULL;
  SFc4_compassGaitSim_impactsInstanceStruct *chartInstance;
  chartInstance = (SFc4_compassGaitSim_impactsInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_u = *(boolean_T *)c4_inData;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", &c4_u, 11, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, FALSE);
  return c4_mxArrayOutData;
}

static boolean_T c4_emlrt_marshallIn(SFc4_compassGaitSim_impactsInstanceStruct
  *chartInstance, const mxArray *c4_impact, const char_T *c4_identifier)
{
  boolean_T c4_y;
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_impact), &c4_thisId);
  sf_mex_destroy(&c4_impact);
  return c4_y;
}

static boolean_T c4_b_emlrt_marshallIn(SFc4_compassGaitSim_impactsInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  boolean_T c4_y;
  boolean_T c4_b0;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_b0, 1, 11, 0U, 0, 0U, 0);
  c4_y = c4_b0;
  sf_mex_destroy(&c4_u);
  return c4_y;
}

static void c4_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_impact;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  boolean_T c4_y;
  SFc4_compassGaitSim_impactsInstanceStruct *chartInstance;
  chartInstance = (SFc4_compassGaitSim_impactsInstanceStruct *)chartInstanceVoid;
  c4_impact = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_impact), &c4_thisId);
  sf_mex_destroy(&c4_impact);
  *(boolean_T *)c4_outData = c4_y;
  sf_mex_destroy(&c4_mxArrayInData);
}

static const mxArray *c4_b_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  real_T c4_u;
  const mxArray *c4_y = NULL;
  SFc4_compassGaitSim_impactsInstanceStruct *chartInstance;
  chartInstance = (SFc4_compassGaitSim_impactsInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_u = *(real_T *)c4_inData;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", &c4_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, FALSE);
  return c4_mxArrayOutData;
}

static real_T c4_c_emlrt_marshallIn(SFc4_compassGaitSim_impactsInstanceStruct
  *chartInstance, const mxArray *c4_t1_, const char_T *c4_identifier)
{
  real_T c4_y;
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_t1_), &c4_thisId);
  sf_mex_destroy(&c4_t1_);
  return c4_y;
}

static real_T c4_d_emlrt_marshallIn(SFc4_compassGaitSim_impactsInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  real_T c4_y;
  real_T c4_d0;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_d0, 1, 0, 0U, 0, 0U, 0);
  c4_y = c4_d0;
  sf_mex_destroy(&c4_u);
  return c4_y;
}

static void c4_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_t1_;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_y;
  SFc4_compassGaitSim_impactsInstanceStruct *chartInstance;
  chartInstance = (SFc4_compassGaitSim_impactsInstanceStruct *)chartInstanceVoid;
  c4_t1_ = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_t1_), &c4_thisId);
  sf_mex_destroy(&c4_t1_);
  *(real_T *)c4_outData = c4_y;
  sf_mex_destroy(&c4_mxArrayInData);
}

static const mxArray *c4_c_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i3;
  real_T c4_b_inData[10];
  int32_T c4_i4;
  real_T c4_u[10];
  const mxArray *c4_y = NULL;
  SFc4_compassGaitSim_impactsInstanceStruct *chartInstance;
  chartInstance = (SFc4_compassGaitSim_impactsInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  for (c4_i3 = 0; c4_i3 < 10; c4_i3++) {
    c4_b_inData[c4_i3] = (*(real_T (*)[10])c4_inData)[c4_i3];
  }

  for (c4_i4 = 0; c4_i4 < 10; c4_i4++) {
    c4_u[c4_i4] = c4_b_inData[c4_i4];
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 1, 10), FALSE);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, FALSE);
  return c4_mxArrayOutData;
}

const mxArray *sf_c4_compassGaitSim_impacts_get_eml_resolved_functions_info(void)
{
  const mxArray *c4_nameCaptureInfo;
  c4_ResolvedFunctionInfo c4_info[19];
  const mxArray *c4_m0 = NULL;
  int32_T c4_i5;
  c4_ResolvedFunctionInfo *c4_r0;
  c4_nameCaptureInfo = NULL;
  c4_nameCaptureInfo = NULL;
  c4_info_helper(c4_info);
  sf_mex_assign(&c4_m0, sf_mex_createstruct("nameCaptureInfo", 1, 19), FALSE);
  for (c4_i5 = 0; c4_i5 < 19; c4_i5++) {
    c4_r0 = &c4_info[c4_i5];
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", c4_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c4_r0->context)), "context", "nameCaptureInfo",
                    c4_i5);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", c4_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c4_r0->name)), "name", "nameCaptureInfo", c4_i5);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", c4_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c4_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c4_i5);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", c4_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c4_r0->resolved)), "resolved", "nameCaptureInfo",
                    c4_i5);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", &c4_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c4_i5);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", &c4_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c4_i5);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", &c4_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c4_i5);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", &c4_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c4_i5);
  }

  sf_mex_assign(&c4_nameCaptureInfo, c4_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c4_nameCaptureInfo);
  return c4_nameCaptureInfo;
}

static void c4_info_helper(c4_ResolvedFunctionInfo c4_info[19])
{
  c4_info[0].context = "";
  c4_info[0].name = "mtimes";
  c4_info[0].dominantType = "double";
  c4_info[0].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c4_info[0].fileTimeLo = 1289483692U;
  c4_info[0].fileTimeHi = 0U;
  c4_info[0].mFileTimeLo = 0U;
  c4_info[0].mFileTimeHi = 0U;
  c4_info[1].context = "";
  c4_info[1].name = "abs";
  c4_info[1].dominantType = "double";
  c4_info[1].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c4_info[1].fileTimeLo = 1343801566U;
  c4_info[1].fileTimeHi = 0U;
  c4_info[1].mFileTimeLo = 0U;
  c4_info[1].mFileTimeHi = 0U;
  c4_info[2].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c4_info[2].name = "eml_scalar_abs";
  c4_info[2].dominantType = "double";
  c4_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c4_info[2].fileTimeLo = 1286786312U;
  c4_info[2].fileTimeHi = 0U;
  c4_info[2].mFileTimeLo = 0U;
  c4_info[2].mFileTimeHi = 0U;
  c4_info[3].context = "";
  c4_info[3].name = "min";
  c4_info[3].dominantType = "logical";
  c4_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c4_info[3].fileTimeLo = 1311226518U;
  c4_info[3].fileTimeHi = 0U;
  c4_info[3].mFileTimeLo = 0U;
  c4_info[3].mFileTimeHi = 0U;
  c4_info[4].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c4_info[4].name = "eml_min_or_max";
  c4_info[4].dominantType = "char";
  c4_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c4_info[4].fileTimeLo = 1334042690U;
  c4_info[4].fileTimeHi = 0U;
  c4_info[4].mFileTimeLo = 0U;
  c4_info[4].mFileTimeHi = 0U;
  c4_info[5].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum";
  c4_info[5].name = "eml_const_nonsingleton_dim";
  c4_info[5].dominantType = "logical";
  c4_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m";
  c4_info[5].fileTimeLo = 1286786296U;
  c4_info[5].fileTimeHi = 0U;
  c4_info[5].mFileTimeLo = 0U;
  c4_info[5].mFileTimeHi = 0U;
  c4_info[6].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum";
  c4_info[6].name = "eml_scalar_eg";
  c4_info[6].dominantType = "logical";
  c4_info[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[6].fileTimeLo = 1286786396U;
  c4_info[6].fileTimeHi = 0U;
  c4_info[6].mFileTimeLo = 0U;
  c4_info[6].mFileTimeHi = 0U;
  c4_info[7].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum";
  c4_info[7].name = "eml_index_class";
  c4_info[7].dominantType = "";
  c4_info[7].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[7].fileTimeLo = 1323134578U;
  c4_info[7].fileTimeHi = 0U;
  c4_info[7].mFileTimeLo = 0U;
  c4_info[7].mFileTimeHi = 0U;
  c4_info[8].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub";
  c4_info[8].name = "eml_index_class";
  c4_info[8].dominantType = "";
  c4_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[8].fileTimeLo = 1323134578U;
  c4_info[8].fileTimeHi = 0U;
  c4_info[8].mFileTimeLo = 0U;
  c4_info[8].mFileTimeHi = 0U;
  c4_info[9].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub";
  c4_info[9].name = "isnan";
  c4_info[9].dominantType = "logical";
  c4_info[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m";
  c4_info[9].fileTimeLo = 1286786360U;
  c4_info[9].fileTimeHi = 0U;
  c4_info[9].mFileTimeLo = 0U;
  c4_info[9].mFileTimeHi = 0U;
  c4_info[10].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub";
  c4_info[10].name = "eml_index_plus";
  c4_info[10].dominantType = "coder.internal.indexInt";
  c4_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c4_info[10].fileTimeLo = 1286786378U;
  c4_info[10].fileTimeHi = 0U;
  c4_info[10].mFileTimeLo = 0U;
  c4_info[10].mFileTimeHi = 0U;
  c4_info[11].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c4_info[11].name = "eml_index_class";
  c4_info[11].dominantType = "";
  c4_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[11].fileTimeLo = 1323134578U;
  c4_info[11].fileTimeHi = 0U;
  c4_info[11].mFileTimeLo = 0U;
  c4_info[11].mFileTimeHi = 0U;
  c4_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub";
  c4_info[12].name = "eml_int_forloop_overflow_check";
  c4_info[12].dominantType = "";
  c4_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c4_info[12].fileTimeLo = 1346481540U;
  c4_info[12].fileTimeHi = 0U;
  c4_info[12].mFileTimeLo = 0U;
  c4_info[12].mFileTimeHi = 0U;
  c4_info[13].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c4_info[13].name = "intmax";
  c4_info[13].dominantType = "char";
  c4_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c4_info[13].fileTimeLo = 1311226516U;
  c4_info[13].fileTimeHi = 0U;
  c4_info[13].mFileTimeLo = 0U;
  c4_info[13].mFileTimeHi = 0U;
  c4_info[14].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub";
  c4_info[14].name = "eml_relop";
  c4_info[14].dominantType = "function_handle";
  c4_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_relop.m";
  c4_info[14].fileTimeLo = 1342422382U;
  c4_info[14].fileTimeHi = 0U;
  c4_info[14].mFileTimeLo = 0U;
  c4_info[14].mFileTimeHi = 0U;
  c4_info[15].context = "";
  c4_info[15].name = "mrdivide";
  c4_info[15].dominantType = "double";
  c4_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c4_info[15].fileTimeLo = 1357915548U;
  c4_info[15].fileTimeHi = 0U;
  c4_info[15].mFileTimeLo = 1319697566U;
  c4_info[15].mFileTimeHi = 0U;
  c4_info[16].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c4_info[16].name = "rdivide";
  c4_info[16].dominantType = "double";
  c4_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c4_info[16].fileTimeLo = 1346481588U;
  c4_info[16].fileTimeHi = 0U;
  c4_info[16].mFileTimeLo = 0U;
  c4_info[16].mFileTimeHi = 0U;
  c4_info[17].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c4_info[17].name = "eml_scalexp_compatible";
  c4_info[17].dominantType = "double";
  c4_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m";
  c4_info[17].fileTimeLo = 1286786396U;
  c4_info[17].fileTimeHi = 0U;
  c4_info[17].mFileTimeLo = 0U;
  c4_info[17].mFileTimeHi = 0U;
  c4_info[18].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c4_info[18].name = "eml_div";
  c4_info[18].dominantType = "double";
  c4_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c4_info[18].fileTimeLo = 1313319010U;
  c4_info[18].fileTimeHi = 0U;
  c4_info[18].mFileTimeLo = 0U;
  c4_info[18].mFileTimeHi = 0U;
}

static const mxArray *c4_d_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_u;
  const mxArray *c4_y = NULL;
  SFc4_compassGaitSim_impactsInstanceStruct *chartInstance;
  chartInstance = (SFc4_compassGaitSim_impactsInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_u = *(int32_T *)c4_inData;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", &c4_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, FALSE);
  return c4_mxArrayOutData;
}

static int32_T c4_e_emlrt_marshallIn(SFc4_compassGaitSim_impactsInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  int32_T c4_y;
  int32_T c4_i6;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_i6, 1, 6, 0U, 0, 0U, 0);
  c4_y = c4_i6;
  sf_mex_destroy(&c4_u);
  return c4_y;
}

static void c4_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_b_sfEvent;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  int32_T c4_y;
  SFc4_compassGaitSim_impactsInstanceStruct *chartInstance;
  chartInstance = (SFc4_compassGaitSim_impactsInstanceStruct *)chartInstanceVoid;
  c4_b_sfEvent = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_sfEvent),
    &c4_thisId);
  sf_mex_destroy(&c4_b_sfEvent);
  *(int32_T *)c4_outData = c4_y;
  sf_mex_destroy(&c4_mxArrayInData);
}

static uint8_T c4_f_emlrt_marshallIn(SFc4_compassGaitSim_impactsInstanceStruct
  *chartInstance, const mxArray *c4_b_is_active_c4_compassGaitSim_impacts, const
  char_T *c4_identifier)
{
  uint8_T c4_y;
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_g_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c4_b_is_active_c4_compassGaitSim_impacts), &c4_thisId);
  sf_mex_destroy(&c4_b_is_active_c4_compassGaitSim_impacts);
  return c4_y;
}

static uint8_T c4_g_emlrt_marshallIn(SFc4_compassGaitSim_impactsInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  uint8_T c4_y;
  uint8_T c4_u0;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_u0, 1, 3, 0U, 0, 0U, 0);
  c4_y = c4_u0;
  sf_mex_destroy(&c4_u);
  return c4_y;
}

static void init_dsm_address_info(SFc4_compassGaitSim_impactsInstanceStruct
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

void sf_c4_compassGaitSim_impacts_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(484527078U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1985557446U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3079358366U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(780981859U);
}

mxArray *sf_c4_compassGaitSim_impacts_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("WUnAwg5FyTdUPttqTyJXAH");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,7,3,dataFields);

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
      pr[0] = (double)(1);
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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,5,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(10);
      pr[1] = (double)(1);
      mxSetField(mxData,6,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,6,"type",mxType);
    }

    mxSetField(mxData,6,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,5,3,dataFields);

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
      pr[0] = (double)(1);
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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(1));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c4_compassGaitSim_impacts_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c4_compassGaitSim_impacts(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x6'type','srcId','name','auxInfo'{{M[1],M[12],T\"impact\",},{M[1],M[9],T\"t1_\",},{M[1],M[5],T\"t1d_\",},{M[1],M[10],T\"t2_\",},{M[1],M[11],T\"t2d_\",},{M[8],M[0],T\"is_active_c4_compassGaitSim_impacts\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 6, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c4_compassGaitSim_impacts_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc4_compassGaitSim_impactsInstanceStruct *chartInstance;
    chartInstance = (SFc4_compassGaitSim_impactsInstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _compassGaitSim_impactsMachineNumber_,
           4,
           1,
           1,
           12,
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
          init_script_number_translation(_compassGaitSim_impactsMachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_compassGaitSim_impactsMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _compassGaitSim_impactsMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,2,0,1,"t2d_");
          _SFD_SET_DATA_PROPS(1,2,0,1,"t2_");
          _SFD_SET_DATA_PROPS(2,2,0,1,"t1d_");
          _SFD_SET_DATA_PROPS(3,2,0,1,"t1_");
          _SFD_SET_DATA_PROPS(4,1,1,0,"t2");
          _SFD_SET_DATA_PROPS(5,1,1,0,"t2d");
          _SFD_SET_DATA_PROPS(6,1,1,0,"t1");
          _SFD_SET_DATA_PROPS(7,1,1,0,"t1d");
          _SFD_SET_DATA_PROPS(8,1,1,0,"I1");
          _SFD_SET_DATA_PROPS(9,1,1,0,"I2");
          _SFD_SET_DATA_PROPS(10,2,0,1,"impact");
          _SFD_SET_DATA_PROPS(11,1,1,0,"prev_impact");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,861);
        _SFD_CV_INIT_EML_IF(0,1,0,421,468,768,856);

        {
          static int condStart[] = { 424, 438, 451 };

          static int condEnd[] = { 433, 447, 468 };

          static int pfixExpr[] = { 0, 1, -1, -3, 2, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,0,424,468,3,0,&(condStart[0]),&(condEnd[0]),
                                6,&(pfixExpr[0]));
        }

        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_b_sf_marshallOut,(MexInFcnForType)c4_b_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_b_sf_marshallOut,(MexInFcnForType)c4_b_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_b_sf_marshallOut,(MexInFcnForType)c4_b_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_b_sf_marshallOut,(MexInFcnForType)c4_b_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(10,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_sf_marshallOut,(MexInFcnForType)c4_sf_marshallIn);

        {
          unsigned int dimVector[1];
          dimVector[0]= 10;
          _SFD_SET_DATA_COMPILED_PROPS(11,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c4_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          real_T *c4_t2d_;
          real_T *c4_t2_;
          real_T *c4_t1d_;
          real_T *c4_t1_;
          real_T *c4_t2;
          real_T *c4_t2d;
          real_T *c4_t1;
          real_T *c4_t1d;
          real_T *c4_I1;
          real_T *c4_I2;
          boolean_T *c4_impact;
          real_T (*c4_prev_impact)[10];
          c4_prev_impact = (real_T (*)[10])ssGetInputPortSignal(chartInstance->S,
            6);
          c4_impact = (boolean_T *)ssGetOutputPortSignal(chartInstance->S, 5);
          c4_I2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
          c4_I1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
          c4_t1d = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c4_t1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c4_t2d = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c4_t2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          c4_t1_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
          c4_t1d_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
          c4_t2_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
          c4_t2d_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          _SFD_SET_DATA_VALUE_PTR(0U, c4_t2d_);
          _SFD_SET_DATA_VALUE_PTR(1U, c4_t2_);
          _SFD_SET_DATA_VALUE_PTR(2U, c4_t1d_);
          _SFD_SET_DATA_VALUE_PTR(3U, c4_t1_);
          _SFD_SET_DATA_VALUE_PTR(4U, c4_t2);
          _SFD_SET_DATA_VALUE_PTR(5U, c4_t2d);
          _SFD_SET_DATA_VALUE_PTR(6U, c4_t1);
          _SFD_SET_DATA_VALUE_PTR(7U, c4_t1d);
          _SFD_SET_DATA_VALUE_PTR(8U, c4_I1);
          _SFD_SET_DATA_VALUE_PTR(9U, c4_I2);
          _SFD_SET_DATA_VALUE_PTR(10U, c4_impact);
          _SFD_SET_DATA_VALUE_PTR(11U, *c4_prev_impact);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _compassGaitSim_impactsMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "Gy9A926fLTkmxaFMKOpwaD";
}

static void sf_opaque_initialize_c4_compassGaitSim_impacts(void
  *chartInstanceVar)
{
  chart_debug_initialization(((SFc4_compassGaitSim_impactsInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c4_compassGaitSim_impacts
    ((SFc4_compassGaitSim_impactsInstanceStruct*) chartInstanceVar);
  initialize_c4_compassGaitSim_impacts
    ((SFc4_compassGaitSim_impactsInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c4_compassGaitSim_impacts(void *chartInstanceVar)
{
  enable_c4_compassGaitSim_impacts((SFc4_compassGaitSim_impactsInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c4_compassGaitSim_impacts(void *chartInstanceVar)
{
  disable_c4_compassGaitSim_impacts((SFc4_compassGaitSim_impactsInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c4_compassGaitSim_impacts(void *chartInstanceVar)
{
  sf_c4_compassGaitSim_impacts((SFc4_compassGaitSim_impactsInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c4_compassGaitSim_impacts
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c4_compassGaitSim_impacts
    ((SFc4_compassGaitSim_impactsInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c4_compassGaitSim_impacts();/* state var info */
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

extern void sf_internal_set_sim_state_c4_compassGaitSim_impacts(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c4_compassGaitSim_impacts();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c4_compassGaitSim_impacts
    ((SFc4_compassGaitSim_impactsInstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c4_compassGaitSim_impacts
  (SimStruct* S)
{
  return sf_internal_get_sim_state_c4_compassGaitSim_impacts(S);
}

static void sf_opaque_set_sim_state_c4_compassGaitSim_impacts(SimStruct* S,
  const mxArray *st)
{
  sf_internal_set_sim_state_c4_compassGaitSim_impacts(S, st);
}

static void sf_opaque_terminate_c4_compassGaitSim_impacts(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc4_compassGaitSim_impactsInstanceStruct*)
                    chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_compassGaitSim_impacts_optimization_info();
    }

    finalize_c4_compassGaitSim_impacts
      ((SFc4_compassGaitSim_impactsInstanceStruct*) chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc4_compassGaitSim_impacts
    ((SFc4_compassGaitSim_impactsInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c4_compassGaitSim_impacts(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c4_compassGaitSim_impacts
      ((SFc4_compassGaitSim_impactsInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c4_compassGaitSim_impacts(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_compassGaitSim_impacts_optimization_info();
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
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 6, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,4,7);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,4,5);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=5; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 7; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,4);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(3035472432U));
  ssSetChecksum1(S,(985712123U));
  ssSetChecksum2(S,(2295717686U));
  ssSetChecksum3(S,(1229522715U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c4_compassGaitSim_impacts(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c4_compassGaitSim_impacts(SimStruct *S)
{
  SFc4_compassGaitSim_impactsInstanceStruct *chartInstance;
  chartInstance = (SFc4_compassGaitSim_impactsInstanceStruct *)utMalloc(sizeof
    (SFc4_compassGaitSim_impactsInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc4_compassGaitSim_impactsInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c4_compassGaitSim_impacts;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c4_compassGaitSim_impacts;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c4_compassGaitSim_impacts;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c4_compassGaitSim_impacts;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c4_compassGaitSim_impacts;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c4_compassGaitSim_impacts;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c4_compassGaitSim_impacts;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c4_compassGaitSim_impacts;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c4_compassGaitSim_impacts;
  chartInstance->chartInfo.mdlStart = mdlStart_c4_compassGaitSim_impacts;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c4_compassGaitSim_impacts;
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

void c4_compassGaitSim_impacts_method_dispatcher(SimStruct *S, int_T method,
  void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c4_compassGaitSim_impacts(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c4_compassGaitSim_impacts(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c4_compassGaitSim_impacts(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c4_compassGaitSim_impacts_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
