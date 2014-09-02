/* Include files */

#include <stddef.h>
#include "blas.h"
#include "compassGaitSim_impacts1_sfun.h"
#include "c5_compassGaitSim_impacts1.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "compassGaitSim_impacts1_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c5_debug_family_names[13] = { "delta", "nargin", "nargout",
  "t2", "t2d", "t1", "t1d", "I1", "I2", "t2d_", "t2_", "t1d_", "t1_" };

/* Function Declarations */
static void initialize_c5_compassGaitSim_impacts1
  (SFc5_compassGaitSim_impacts1InstanceStruct *chartInstance);
static void initialize_params_c5_compassGaitSim_impacts1
  (SFc5_compassGaitSim_impacts1InstanceStruct *chartInstance);
static void enable_c5_compassGaitSim_impacts1
  (SFc5_compassGaitSim_impacts1InstanceStruct *chartInstance);
static void disable_c5_compassGaitSim_impacts1
  (SFc5_compassGaitSim_impacts1InstanceStruct *chartInstance);
static void c5_update_debugger_state_c5_compassGaitSim_impacts1
  (SFc5_compassGaitSim_impacts1InstanceStruct *chartInstance);
static const mxArray *get_sim_state_c5_compassGaitSim_impacts1
  (SFc5_compassGaitSim_impacts1InstanceStruct *chartInstance);
static void set_sim_state_c5_compassGaitSim_impacts1
  (SFc5_compassGaitSim_impacts1InstanceStruct *chartInstance, const mxArray
   *c5_st);
static void finalize_c5_compassGaitSim_impacts1
  (SFc5_compassGaitSim_impacts1InstanceStruct *chartInstance);
static void sf_c5_compassGaitSim_impacts1
  (SFc5_compassGaitSim_impacts1InstanceStruct *chartInstance);
static void initSimStructsc5_compassGaitSim_impacts1
  (SFc5_compassGaitSim_impacts1InstanceStruct *chartInstance);
static void registerMessagesc5_compassGaitSim_impacts1
  (SFc5_compassGaitSim_impacts1InstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c5_machineNumber, uint32_T
  c5_chartNumber);
static const mxArray *c5_sf_marshallOut(void *chartInstanceVoid, void *c5_inData);
static real_T c5_emlrt_marshallIn(SFc5_compassGaitSim_impacts1InstanceStruct
  *chartInstance, const mxArray *c5_t1_, const char_T *c5_identifier);
static real_T c5_b_emlrt_marshallIn(SFc5_compassGaitSim_impacts1InstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void c5_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static const mxArray *c5_b_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static int32_T c5_c_emlrt_marshallIn(SFc5_compassGaitSim_impacts1InstanceStruct *
  chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void c5_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static uint8_T c5_d_emlrt_marshallIn(SFc5_compassGaitSim_impacts1InstanceStruct *
  chartInstance, const mxArray *c5_b_is_active_c5_compassGaitSim_impacts1, const
  char_T *c5_identifier);
static uint8_T c5_e_emlrt_marshallIn(SFc5_compassGaitSim_impacts1InstanceStruct *
  chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void init_dsm_address_info(SFc5_compassGaitSim_impacts1InstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c5_compassGaitSim_impacts1
  (SFc5_compassGaitSim_impacts1InstanceStruct *chartInstance)
{
  chartInstance->c5_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c5_is_active_c5_compassGaitSim_impacts1 = 0U;
}

static void initialize_params_c5_compassGaitSim_impacts1
  (SFc5_compassGaitSim_impacts1InstanceStruct *chartInstance)
{
}

static void enable_c5_compassGaitSim_impacts1
  (SFc5_compassGaitSim_impacts1InstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c5_compassGaitSim_impacts1
  (SFc5_compassGaitSim_impacts1InstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c5_update_debugger_state_c5_compassGaitSim_impacts1
  (SFc5_compassGaitSim_impacts1InstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c5_compassGaitSim_impacts1
  (SFc5_compassGaitSim_impacts1InstanceStruct *chartInstance)
{
  const mxArray *c5_st;
  const mxArray *c5_y = NULL;
  real_T c5_hoistedGlobal;
  real_T c5_u;
  const mxArray *c5_b_y = NULL;
  real_T c5_b_hoistedGlobal;
  real_T c5_b_u;
  const mxArray *c5_c_y = NULL;
  real_T c5_c_hoistedGlobal;
  real_T c5_c_u;
  const mxArray *c5_d_y = NULL;
  real_T c5_d_hoistedGlobal;
  real_T c5_d_u;
  const mxArray *c5_e_y = NULL;
  uint8_T c5_e_hoistedGlobal;
  uint8_T c5_e_u;
  const mxArray *c5_f_y = NULL;
  real_T *c5_t1_;
  real_T *c5_t1d_;
  real_T *c5_t2_;
  real_T *c5_t2d_;
  c5_t1_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c5_t1d_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c5_t2_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c5_t2d_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c5_st = NULL;
  c5_st = NULL;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_createcellarray(5), FALSE);
  c5_hoistedGlobal = *c5_t1_;
  c5_u = c5_hoistedGlobal;
  c5_b_y = NULL;
  sf_mex_assign(&c5_b_y, sf_mex_create("y", &c5_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c5_y, 0, c5_b_y);
  c5_b_hoistedGlobal = *c5_t1d_;
  c5_b_u = c5_b_hoistedGlobal;
  c5_c_y = NULL;
  sf_mex_assign(&c5_c_y, sf_mex_create("y", &c5_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c5_y, 1, c5_c_y);
  c5_c_hoistedGlobal = *c5_t2_;
  c5_c_u = c5_c_hoistedGlobal;
  c5_d_y = NULL;
  sf_mex_assign(&c5_d_y, sf_mex_create("y", &c5_c_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c5_y, 2, c5_d_y);
  c5_d_hoistedGlobal = *c5_t2d_;
  c5_d_u = c5_d_hoistedGlobal;
  c5_e_y = NULL;
  sf_mex_assign(&c5_e_y, sf_mex_create("y", &c5_d_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c5_y, 3, c5_e_y);
  c5_e_hoistedGlobal = chartInstance->c5_is_active_c5_compassGaitSim_impacts1;
  c5_e_u = c5_e_hoistedGlobal;
  c5_f_y = NULL;
  sf_mex_assign(&c5_f_y, sf_mex_create("y", &c5_e_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c5_y, 4, c5_f_y);
  sf_mex_assign(&c5_st, c5_y, FALSE);
  return c5_st;
}

static void set_sim_state_c5_compassGaitSim_impacts1
  (SFc5_compassGaitSim_impacts1InstanceStruct *chartInstance, const mxArray
   *c5_st)
{
  const mxArray *c5_u;
  real_T *c5_t1_;
  real_T *c5_t1d_;
  real_T *c5_t2_;
  real_T *c5_t2d_;
  c5_t1_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c5_t1d_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c5_t2_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c5_t2d_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c5_doneDoubleBufferReInit = TRUE;
  c5_u = sf_mex_dup(c5_st);
  *c5_t1_ = c5_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c5_u, 0)),
    "t1_");
  *c5_t1d_ = c5_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c5_u,
    1)), "t1d_");
  *c5_t2_ = c5_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c5_u, 2)),
    "t2_");
  *c5_t2d_ = c5_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c5_u,
    3)), "t2d_");
  chartInstance->c5_is_active_c5_compassGaitSim_impacts1 = c5_d_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c5_u, 4)),
     "is_active_c5_compassGaitSim_impacts1");
  sf_mex_destroy(&c5_u);
  c5_update_debugger_state_c5_compassGaitSim_impacts1(chartInstance);
  sf_mex_destroy(&c5_st);
}

static void finalize_c5_compassGaitSim_impacts1
  (SFc5_compassGaitSim_impacts1InstanceStruct *chartInstance)
{
}

static void sf_c5_compassGaitSim_impacts1
  (SFc5_compassGaitSim_impacts1InstanceStruct *chartInstance)
{
  real_T c5_hoistedGlobal;
  real_T c5_b_hoistedGlobal;
  real_T c5_c_hoistedGlobal;
  real_T c5_d_hoistedGlobal;
  real_T c5_e_hoistedGlobal;
  real_T c5_f_hoistedGlobal;
  real_T c5_t2;
  real_T c5_t2d;
  real_T c5_t1;
  real_T c5_t1d;
  real_T c5_I1;
  real_T c5_I2;
  uint32_T c5_debug_family_var_map[13];
  real_T c5_delta;
  real_T c5_nargin = 6.0;
  real_T c5_nargout = 4.0;
  real_T c5_t2d_;
  real_T c5_t2_;
  real_T c5_t1d_;
  real_T c5_t1_;
  real_T c5_a;
  real_T c5_b;
  real_T c5_y;
  real_T c5_b_a;
  real_T c5_b_b;
  real_T c5_b_y;
  real_T c5_c_a;
  real_T c5_c_b;
  real_T c5_c_y;
  real_T c5_d_a;
  real_T c5_d_b;
  real_T c5_d_y;
  real_T c5_A;
  real_T c5_B;
  real_T c5_x;
  real_T c5_e_y;
  real_T c5_b_x;
  real_T c5_f_y;
  real_T c5_e_a;
  real_T c5_e_b;
  real_T c5_f_a;
  real_T c5_f_b;
  int32_T c5_i0;
  static char_T c5_cv0[8] = { 'T', 'h', 'i', 's', ' ', 'w', 'a', 'y' };

  char_T c5_u[8];
  const mxArray *c5_g_y = NULL;
  real_T c5_b_u;
  const mxArray *c5_h_y = NULL;
  real_T *c5_b_t1_;
  real_T *c5_b_t1d_;
  real_T *c5_b_t2_;
  real_T *c5_b_t2d_;
  real_T *c5_b_I2;
  real_T *c5_b_I1;
  real_T *c5_b_t1d;
  real_T *c5_b_t1;
  real_T *c5_b_t2d;
  real_T *c5_b_t2;
  c5_b_I2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
  c5_b_I1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c5_b_t1d = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c5_b_t1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c5_b_t2d = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c5_b_t2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  c5_b_t1_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c5_b_t1d_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c5_b_t2_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c5_b_t2d_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 3U, chartInstance->c5_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c5_b_t2d_, 0U);
  _SFD_DATA_RANGE_CHECK(*c5_b_t2_, 1U);
  _SFD_DATA_RANGE_CHECK(*c5_b_t1d_, 2U);
  _SFD_DATA_RANGE_CHECK(*c5_b_t1_, 3U);
  _SFD_DATA_RANGE_CHECK(*c5_b_t2, 4U);
  _SFD_DATA_RANGE_CHECK(*c5_b_t2d, 5U);
  _SFD_DATA_RANGE_CHECK(*c5_b_t1, 6U);
  _SFD_DATA_RANGE_CHECK(*c5_b_t1d, 7U);
  _SFD_DATA_RANGE_CHECK(*c5_b_I1, 8U);
  _SFD_DATA_RANGE_CHECK(*c5_b_I2, 9U);
  chartInstance->c5_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 3U, chartInstance->c5_sfEvent);
  c5_hoistedGlobal = *c5_b_t2;
  c5_b_hoistedGlobal = *c5_b_t2d;
  c5_c_hoistedGlobal = *c5_b_t1;
  c5_d_hoistedGlobal = *c5_b_t1d;
  c5_e_hoistedGlobal = *c5_b_I1;
  c5_f_hoistedGlobal = *c5_b_I2;
  c5_t2 = c5_hoistedGlobal;
  c5_t2d = c5_b_hoistedGlobal;
  c5_t1 = c5_c_hoistedGlobal;
  c5_t1d = c5_d_hoistedGlobal;
  c5_I1 = c5_e_hoistedGlobal;
  c5_I2 = c5_f_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 13U, 13U, c5_debug_family_names,
    c5_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_delta, 0U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargin, 1U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargout, 2U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c5_t2, 3U, c5_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c5_t2d, 4U, c5_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c5_t1, 5U, c5_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c5_t1d, 6U, c5_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c5_I1, 7U, c5_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c5_I2, 8U, c5_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_t2d_, 9U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_t2_, 10U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_t1d_, 11U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_t1_, 12U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 5);
  c5_t1_ = 3.1415926535897931 - c5_t1;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 6);
  c5_t2_ = -6.2831853071795862 - c5_t2;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 8);
  c5_a = c5_t1d;
  c5_b = c5_I1 + c5_I2;
  c5_y = c5_a * c5_b;
  c5_b_a = c5_t2d;
  c5_b_b = c5_I2;
  c5_b_y = c5_b_a * c5_b_b;
  c5_c_a = c5_t1d;
  c5_c_b = c5_I1 + c5_I2;
  c5_c_y = c5_c_a * c5_c_b;
  c5_d_a = c5_t2d;
  c5_d_b = c5_I1;
  c5_d_y = c5_d_a * c5_d_b;
  c5_A = c5_y + c5_b_y;
  c5_B = c5_c_y + c5_d_y;
  c5_x = c5_A;
  c5_e_y = c5_B;
  c5_b_x = c5_x;
  c5_f_y = c5_e_y;
  c5_delta = c5_b_x / c5_f_y;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 9);
  c5_e_a = c5_delta;
  c5_e_b = c5_t1d;
  c5_t1d = c5_e_a * c5_e_b;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 10);
  c5_f_a = c5_delta;
  c5_f_b = c5_t2d;
  c5_t2d = c5_f_a * c5_f_b;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 12);
  c5_t1d_ = c5_t1d + c5_t2d;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 13);
  c5_t2d_ = c5_t2d;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 15);
  for (c5_i0 = 0; c5_i0 < 8; c5_i0++) {
    c5_u[c5_i0] = c5_cv0[c5_i0];
  }

  c5_g_y = NULL;
  sf_mex_assign(&c5_g_y, sf_mex_create("y", c5_u, 10, 0U, 1U, 0U, 2, 1, 8),
                FALSE);
  sf_mex_call_debug("disp", 0U, 1U, 14, c5_g_y);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 16);
  c5_b_u = c5_t1_;
  c5_h_y = NULL;
  sf_mex_assign(&c5_h_y, sf_mex_create("y", &c5_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_call_debug("disp", 0U, 1U, 14, c5_h_y);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, -16);
  _SFD_SYMBOL_SCOPE_POP();
  *c5_b_t2d_ = c5_t2d_;
  *c5_b_t2_ = c5_t2_;
  *c5_b_t1d_ = c5_t1d_;
  *c5_b_t1_ = c5_t1_;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 3U, chartInstance->c5_sfEvent);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_compassGaitSim_impacts1MachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc5_compassGaitSim_impacts1
  (SFc5_compassGaitSim_impacts1InstanceStruct *chartInstance)
{
}

static void registerMessagesc5_compassGaitSim_impacts1
  (SFc5_compassGaitSim_impacts1InstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c5_machineNumber, uint32_T
  c5_chartNumber)
{
}

static const mxArray *c5_sf_marshallOut(void *chartInstanceVoid, void *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  real_T c5_u;
  const mxArray *c5_y = NULL;
  SFc5_compassGaitSim_impacts1InstanceStruct *chartInstance;
  chartInstance = (SFc5_compassGaitSim_impacts1InstanceStruct *)
    chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_u = *(real_T *)c5_inData;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, FALSE);
  return c5_mxArrayOutData;
}

static real_T c5_emlrt_marshallIn(SFc5_compassGaitSim_impacts1InstanceStruct
  *chartInstance, const mxArray *c5_t1_, const char_T *c5_identifier)
{
  real_T c5_y;
  emlrtMsgIdentifier c5_thisId;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_t1_), &c5_thisId);
  sf_mex_destroy(&c5_t1_);
  return c5_y;
}

static real_T c5_b_emlrt_marshallIn(SFc5_compassGaitSim_impacts1InstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
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
  const mxArray *c5_t1_;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real_T c5_y;
  SFc5_compassGaitSim_impacts1InstanceStruct *chartInstance;
  chartInstance = (SFc5_compassGaitSim_impacts1InstanceStruct *)
    chartInstanceVoid;
  c5_t1_ = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_t1_), &c5_thisId);
  sf_mex_destroy(&c5_t1_);
  *(real_T *)c5_outData = c5_y;
  sf_mex_destroy(&c5_mxArrayInData);
}

const mxArray *sf_c5_compassGaitSim_impacts1_get_eml_resolved_functions_info
  (void)
{
  const mxArray *c5_nameCaptureInfo;
  c5_ResolvedFunctionInfo c5_info[7];
  c5_ResolvedFunctionInfo (*c5_b_info)[7];
  const mxArray *c5_m0 = NULL;
  int32_T c5_i1;
  c5_ResolvedFunctionInfo *c5_r0;
  c5_nameCaptureInfo = NULL;
  c5_nameCaptureInfo = NULL;
  c5_b_info = (c5_ResolvedFunctionInfo (*)[7])c5_info;
  (*c5_b_info)[0].context = "";
  (*c5_b_info)[0].name = "mtimes";
  (*c5_b_info)[0].dominantType = "double";
  (*c5_b_info)[0].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  (*c5_b_info)[0].fileTimeLo = 1289483692U;
  (*c5_b_info)[0].fileTimeHi = 0U;
  (*c5_b_info)[0].mFileTimeLo = 0U;
  (*c5_b_info)[0].mFileTimeHi = 0U;
  (*c5_b_info)[1].context = "";
  (*c5_b_info)[1].name = "mrdivide";
  (*c5_b_info)[1].dominantType = "double";
  (*c5_b_info)[1].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  (*c5_b_info)[1].fileTimeLo = 1357915548U;
  (*c5_b_info)[1].fileTimeHi = 0U;
  (*c5_b_info)[1].mFileTimeLo = 1319697566U;
  (*c5_b_info)[1].mFileTimeHi = 0U;
  (*c5_b_info)[2].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  (*c5_b_info)[2].name = "rdivide";
  (*c5_b_info)[2].dominantType = "double";
  (*c5_b_info)[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  (*c5_b_info)[2].fileTimeLo = 1346481588U;
  (*c5_b_info)[2].fileTimeHi = 0U;
  (*c5_b_info)[2].mFileTimeLo = 0U;
  (*c5_b_info)[2].mFileTimeHi = 0U;
  (*c5_b_info)[3].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  (*c5_b_info)[3].name = "eml_scalexp_compatible";
  (*c5_b_info)[3].dominantType = "double";
  (*c5_b_info)[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m";
  (*c5_b_info)[3].fileTimeLo = 1286786396U;
  (*c5_b_info)[3].fileTimeHi = 0U;
  (*c5_b_info)[3].mFileTimeLo = 0U;
  (*c5_b_info)[3].mFileTimeHi = 0U;
  (*c5_b_info)[4].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  (*c5_b_info)[4].name = "eml_div";
  (*c5_b_info)[4].dominantType = "double";
  (*c5_b_info)[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  (*c5_b_info)[4].fileTimeLo = 1313319010U;
  (*c5_b_info)[4].fileTimeHi = 0U;
  (*c5_b_info)[4].mFileTimeLo = 0U;
  (*c5_b_info)[4].mFileTimeHi = 0U;
  (*c5_b_info)[5].context = "";
  (*c5_b_info)[5].name = "disp";
  (*c5_b_info)[5].dominantType = "char";
  (*c5_b_info)[5].resolved = "[IXMB]$matlabroot$/toolbox/matlab/lang/disp";
  (*c5_b_info)[5].fileTimeLo = MAX_uint32_T;
  (*c5_b_info)[5].fileTimeHi = MAX_uint32_T;
  (*c5_b_info)[5].mFileTimeLo = MAX_uint32_T;
  (*c5_b_info)[5].mFileTimeHi = MAX_uint32_T;
  (*c5_b_info)[6].context = "";
  (*c5_b_info)[6].name = "disp";
  (*c5_b_info)[6].dominantType = "double";
  (*c5_b_info)[6].resolved = "[IXMB]$matlabroot$/toolbox/matlab/lang/disp";
  (*c5_b_info)[6].fileTimeLo = MAX_uint32_T;
  (*c5_b_info)[6].fileTimeHi = MAX_uint32_T;
  (*c5_b_info)[6].mFileTimeLo = MAX_uint32_T;
  (*c5_b_info)[6].mFileTimeHi = MAX_uint32_T;
  sf_mex_assign(&c5_m0, sf_mex_createstruct("nameCaptureInfo", 1, 7), FALSE);
  for (c5_i1 = 0; c5_i1 < 7; c5_i1++) {
    c5_r0 = &c5_info[c5_i1];
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", c5_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c5_r0->context)), "context", "nameCaptureInfo",
                    c5_i1);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", c5_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c5_r0->name)), "name", "nameCaptureInfo", c5_i1);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", c5_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c5_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c5_i1);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", c5_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c5_r0->resolved)), "resolved", "nameCaptureInfo",
                    c5_i1);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", &c5_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c5_i1);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", &c5_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c5_i1);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", &c5_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c5_i1);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", &c5_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c5_i1);
  }

  sf_mex_assign(&c5_nameCaptureInfo, c5_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c5_nameCaptureInfo);
  return c5_nameCaptureInfo;
}

static const mxArray *c5_b_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_u;
  const mxArray *c5_y = NULL;
  SFc5_compassGaitSim_impacts1InstanceStruct *chartInstance;
  chartInstance = (SFc5_compassGaitSim_impacts1InstanceStruct *)
    chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_u = *(int32_T *)c5_inData;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, FALSE);
  return c5_mxArrayOutData;
}

static int32_T c5_c_emlrt_marshallIn(SFc5_compassGaitSim_impacts1InstanceStruct *
  chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  int32_T c5_y;
  int32_T c5_i2;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_i2, 1, 6, 0U, 0, 0U, 0);
  c5_y = c5_i2;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void c5_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_b_sfEvent;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  int32_T c5_y;
  SFc5_compassGaitSim_impacts1InstanceStruct *chartInstance;
  chartInstance = (SFc5_compassGaitSim_impacts1InstanceStruct *)
    chartInstanceVoid;
  c5_b_sfEvent = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_b_sfEvent),
    &c5_thisId);
  sf_mex_destroy(&c5_b_sfEvent);
  *(int32_T *)c5_outData = c5_y;
  sf_mex_destroy(&c5_mxArrayInData);
}

static uint8_T c5_d_emlrt_marshallIn(SFc5_compassGaitSim_impacts1InstanceStruct *
  chartInstance, const mxArray *c5_b_is_active_c5_compassGaitSim_impacts1, const
  char_T *c5_identifier)
{
  uint8_T c5_y;
  emlrtMsgIdentifier c5_thisId;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c5_b_is_active_c5_compassGaitSim_impacts1), &c5_thisId);
  sf_mex_destroy(&c5_b_is_active_c5_compassGaitSim_impacts1);
  return c5_y;
}

static uint8_T c5_e_emlrt_marshallIn(SFc5_compassGaitSim_impacts1InstanceStruct *
  chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  uint8_T c5_y;
  uint8_T c5_u0;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_u0, 1, 3, 0U, 0, 0U, 0);
  c5_y = c5_u0;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void init_dsm_address_info(SFc5_compassGaitSim_impacts1InstanceStruct
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

void sf_c5_compassGaitSim_impacts1_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(172516234U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3732390422U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(49021058U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(850378953U);
}

mxArray *sf_c5_compassGaitSim_impacts1_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("HoeIdHJpoCWflQR5RpvyOC");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,6,3,dataFields);

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
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c5_compassGaitSim_impacts1_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c5_compassGaitSim_impacts1(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x5'type','srcId','name','auxInfo'{{M[1],M[9],T\"t1_\",},{M[1],M[5],T\"t1d_\",},{M[1],M[10],T\"t2_\",},{M[1],M[11],T\"t2d_\",},{M[8],M[0],T\"is_active_c5_compassGaitSim_impacts1\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 5, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c5_compassGaitSim_impacts1_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc5_compassGaitSim_impacts1InstanceStruct *chartInstance;
    chartInstance = (SFc5_compassGaitSim_impacts1InstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _compassGaitSim_impacts1MachineNumber_,
           5,
           1,
           1,
           10,
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
          init_script_number_translation(_compassGaitSim_impacts1MachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_compassGaitSim_impacts1MachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _compassGaitSim_impacts1MachineNumber_,
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,356);
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
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)c5_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)c5_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)c5_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)NULL);

        {
          real_T *c5_t2d_;
          real_T *c5_t2_;
          real_T *c5_t1d_;
          real_T *c5_t1_;
          real_T *c5_t2;
          real_T *c5_t2d;
          real_T *c5_t1;
          real_T *c5_t1d;
          real_T *c5_I1;
          real_T *c5_I2;
          c5_I2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
          c5_I1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
          c5_t1d = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c5_t1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c5_t2d = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c5_t2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          c5_t1_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
          c5_t1d_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
          c5_t2_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
          c5_t2d_ = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          _SFD_SET_DATA_VALUE_PTR(0U, c5_t2d_);
          _SFD_SET_DATA_VALUE_PTR(1U, c5_t2_);
          _SFD_SET_DATA_VALUE_PTR(2U, c5_t1d_);
          _SFD_SET_DATA_VALUE_PTR(3U, c5_t1_);
          _SFD_SET_DATA_VALUE_PTR(4U, c5_t2);
          _SFD_SET_DATA_VALUE_PTR(5U, c5_t2d);
          _SFD_SET_DATA_VALUE_PTR(6U, c5_t1);
          _SFD_SET_DATA_VALUE_PTR(7U, c5_t1d);
          _SFD_SET_DATA_VALUE_PTR(8U, c5_I1);
          _SFD_SET_DATA_VALUE_PTR(9U, c5_I2);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _compassGaitSim_impacts1MachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "PQvIScs2mzsgoA7EkVxuyB";
}

static void sf_opaque_initialize_c5_compassGaitSim_impacts1(void
  *chartInstanceVar)
{
  chart_debug_initialization(((SFc5_compassGaitSim_impacts1InstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c5_compassGaitSim_impacts1
    ((SFc5_compassGaitSim_impacts1InstanceStruct*) chartInstanceVar);
  initialize_c5_compassGaitSim_impacts1
    ((SFc5_compassGaitSim_impacts1InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c5_compassGaitSim_impacts1(void *chartInstanceVar)
{
  enable_c5_compassGaitSim_impacts1((SFc5_compassGaitSim_impacts1InstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c5_compassGaitSim_impacts1(void *chartInstanceVar)
{
  disable_c5_compassGaitSim_impacts1((SFc5_compassGaitSim_impacts1InstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c5_compassGaitSim_impacts1(void *chartInstanceVar)
{
  sf_c5_compassGaitSim_impacts1((SFc5_compassGaitSim_impacts1InstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c5_compassGaitSim_impacts1
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c5_compassGaitSim_impacts1
    ((SFc5_compassGaitSim_impacts1InstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c5_compassGaitSim_impacts1();/* state var info */
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

extern void sf_internal_set_sim_state_c5_compassGaitSim_impacts1(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c5_compassGaitSim_impacts1();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c5_compassGaitSim_impacts1
    ((SFc5_compassGaitSim_impacts1InstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c5_compassGaitSim_impacts1
  (SimStruct* S)
{
  return sf_internal_get_sim_state_c5_compassGaitSim_impacts1(S);
}

static void sf_opaque_set_sim_state_c5_compassGaitSim_impacts1(SimStruct* S,
  const mxArray *st)
{
  sf_internal_set_sim_state_c5_compassGaitSim_impacts1(S, st);
}

static void sf_opaque_terminate_c5_compassGaitSim_impacts1(void
  *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc5_compassGaitSim_impacts1InstanceStruct*)
                    chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_compassGaitSim_impacts1_optimization_info();
    }

    finalize_c5_compassGaitSim_impacts1
      ((SFc5_compassGaitSim_impacts1InstanceStruct*) chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc5_compassGaitSim_impacts1
    ((SFc5_compassGaitSim_impacts1InstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c5_compassGaitSim_impacts1(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c5_compassGaitSim_impacts1
      ((SFc5_compassGaitSim_impacts1InstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c5_compassGaitSim_impacts1(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_compassGaitSim_impacts1_optimization_info();
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
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,5,6);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,5,4);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=4; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 6; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,5);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1340424618U));
  ssSetChecksum1(S,(1750407163U));
  ssSetChecksum2(S,(2522895515U));
  ssSetChecksum3(S,(2901508533U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c5_compassGaitSim_impacts1(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c5_compassGaitSim_impacts1(SimStruct *S)
{
  SFc5_compassGaitSim_impacts1InstanceStruct *chartInstance;
  chartInstance = (SFc5_compassGaitSim_impacts1InstanceStruct *)utMalloc(sizeof
    (SFc5_compassGaitSim_impacts1InstanceStruct));
  memset(chartInstance, 0, sizeof(SFc5_compassGaitSim_impacts1InstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c5_compassGaitSim_impacts1;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c5_compassGaitSim_impacts1;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c5_compassGaitSim_impacts1;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c5_compassGaitSim_impacts1;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c5_compassGaitSim_impacts1;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c5_compassGaitSim_impacts1;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c5_compassGaitSim_impacts1;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c5_compassGaitSim_impacts1;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c5_compassGaitSim_impacts1;
  chartInstance->chartInfo.mdlStart = mdlStart_c5_compassGaitSim_impacts1;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c5_compassGaitSim_impacts1;
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

void c5_compassGaitSim_impacts1_method_dispatcher(SimStruct *S, int_T method,
  void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c5_compassGaitSim_impacts1(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c5_compassGaitSim_impacts1(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c5_compassGaitSim_impacts1(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c5_compassGaitSim_impacts1_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
