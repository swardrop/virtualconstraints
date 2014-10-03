/* Include files */

#include <stddef.h>
#include "blas.h"
#include "compassGaitSim_sfun.h"
#include "c1_compassGaitSim.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "compassGaitSim_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c1_debug_family_names[19] = { "i", "ths", "us", "K_p", "K_d",
  "t", "t2_des", "n", "t2ed", "nargin", "nargout", "nom_torque_table", "points",
  "t1", "t2", "t_step", "t2err_p", "T2", "t2err" };

/* Function Declarations */
static void initialize_c1_compassGaitSim(SFc1_compassGaitSimInstanceStruct
  *chartInstance);
static void initialize_params_c1_compassGaitSim
  (SFc1_compassGaitSimInstanceStruct *chartInstance);
static void enable_c1_compassGaitSim(SFc1_compassGaitSimInstanceStruct
  *chartInstance);
static void disable_c1_compassGaitSim(SFc1_compassGaitSimInstanceStruct
  *chartInstance);
static void c1_update_debugger_state_c1_compassGaitSim
  (SFc1_compassGaitSimInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c1_compassGaitSim
  (SFc1_compassGaitSimInstanceStruct *chartInstance);
static void set_sim_state_c1_compassGaitSim(SFc1_compassGaitSimInstanceStruct
  *chartInstance, const mxArray *c1_st);
static void finalize_c1_compassGaitSim(SFc1_compassGaitSimInstanceStruct
  *chartInstance);
static void sf_c1_compassGaitSim(SFc1_compassGaitSimInstanceStruct
  *chartInstance);
static void c1_chartstep_c1_compassGaitSim(SFc1_compassGaitSimInstanceStruct
  *chartInstance);
static void initSimStructsc1_compassGaitSim(SFc1_compassGaitSimInstanceStruct
  *chartInstance);
static void registerMessagesc1_compassGaitSim(SFc1_compassGaitSimInstanceStruct *
  chartInstance);
static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber);
static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData);
static real_T c1_emlrt_marshallIn(SFc1_compassGaitSimInstanceStruct
  *chartInstance, const mxArray *c1_t2err, const char_T *c1_identifier);
static real_T c1_b_emlrt_marshallIn(SFc1_compassGaitSimInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_c_emlrt_marshallIn(SFc1_compassGaitSimInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_y[2]);
static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_e_sf_marshallOut(void *chartInstanceVoid, real_T
  c1_inData_data[1], int32_T c1_inData_sizes[1]);
static void c1_d_emlrt_marshallIn(SFc1_compassGaitSimInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_y_data[1], int32_T c1_y_sizes[1]);
static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, real_T c1_outData_data[1],
  int32_T c1_outData_sizes[1]);
static void c1_info_helper(c1_ResolvedFunctionInfo c1_info[61]);
static void c1_eml_li_find(SFc1_compassGaitSimInstanceStruct *chartInstance,
  boolean_T c1_x, int32_T c1_y_data[1], int32_T c1_y_sizes[2]);
static void c1_eml_scalar_eg(SFc1_compassGaitSimInstanceStruct *chartInstance);
static void c1_eml_warning(SFc1_compassGaitSimInstanceStruct *chartInstance);
static real_T c1_mpower(SFc1_compassGaitSimInstanceStruct *chartInstance, real_T
  c1_a, real_T c1_b);
static void c1_eml_error(SFc1_compassGaitSimInstanceStruct *chartInstance);
static const mxArray *c1_f_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static int32_T c1_e_emlrt_marshallIn(SFc1_compassGaitSimInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static uint8_T c1_f_emlrt_marshallIn(SFc1_compassGaitSimInstanceStruct
  *chartInstance, const mxArray *c1_b_is_active_c1_compassGaitSim, const char_T *
  c1_identifier);
static uint8_T c1_g_emlrt_marshallIn(SFc1_compassGaitSimInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void init_dsm_address_info(SFc1_compassGaitSimInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c1_compassGaitSim(SFc1_compassGaitSimInstanceStruct
  *chartInstance)
{
  chartInstance->c1_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c1_is_active_c1_compassGaitSim = 0U;
}

static void initialize_params_c1_compassGaitSim
  (SFc1_compassGaitSimInstanceStruct *chartInstance)
{
}

static void enable_c1_compassGaitSim(SFc1_compassGaitSimInstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c1_compassGaitSim(SFc1_compassGaitSimInstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c1_update_debugger_state_c1_compassGaitSim
  (SFc1_compassGaitSimInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c1_compassGaitSim
  (SFc1_compassGaitSimInstanceStruct *chartInstance)
{
  const mxArray *c1_st;
  const mxArray *c1_y = NULL;
  real_T c1_hoistedGlobal;
  real_T c1_u;
  const mxArray *c1_b_y = NULL;
  real_T c1_b_hoistedGlobal;
  real_T c1_b_u;
  const mxArray *c1_c_y = NULL;
  uint8_T c1_c_hoistedGlobal;
  uint8_T c1_c_u;
  const mxArray *c1_d_y = NULL;
  real_T *c1_T2;
  real_T *c1_t2err;
  c1_t2err = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_T2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c1_st = NULL;
  c1_st = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_createcellarray(3), FALSE);
  c1_hoistedGlobal = *c1_T2;
  c1_u = c1_hoistedGlobal;
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 0, c1_b_y);
  c1_b_hoistedGlobal = *c1_t2err;
  c1_b_u = c1_b_hoistedGlobal;
  c1_c_y = NULL;
  sf_mex_assign(&c1_c_y, sf_mex_create("y", &c1_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 1, c1_c_y);
  c1_c_hoistedGlobal = chartInstance->c1_is_active_c1_compassGaitSim;
  c1_c_u = c1_c_hoistedGlobal;
  c1_d_y = NULL;
  sf_mex_assign(&c1_d_y, sf_mex_create("y", &c1_c_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 2, c1_d_y);
  sf_mex_assign(&c1_st, c1_y, FALSE);
  return c1_st;
}

static void set_sim_state_c1_compassGaitSim(SFc1_compassGaitSimInstanceStruct
  *chartInstance, const mxArray *c1_st)
{
  const mxArray *c1_u;
  real_T *c1_T2;
  real_T *c1_t2err;
  c1_t2err = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_T2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c1_doneDoubleBufferReInit = TRUE;
  c1_u = sf_mex_dup(c1_st);
  *c1_T2 = c1_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 0)),
    "T2");
  *c1_t2err = c1_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u,
    1)), "t2err");
  chartInstance->c1_is_active_c1_compassGaitSim = c1_f_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 2)),
     "is_active_c1_compassGaitSim");
  sf_mex_destroy(&c1_u);
  c1_update_debugger_state_c1_compassGaitSim(chartInstance);
  sf_mex_destroy(&c1_st);
}

static void finalize_c1_compassGaitSim(SFc1_compassGaitSimInstanceStruct
  *chartInstance)
{
}

static void sf_c1_compassGaitSim(SFc1_compassGaitSimInstanceStruct
  *chartInstance)
{
  int32_T c1_i0;
  int32_T c1_i1;
  real_T *c1_t1;
  real_T *c1_t2;
  real_T *c1_t_step;
  real_T *c1_T2;
  real_T *c1_t2err;
  real_T *c1_t2err_p;
  real_T (*c1_points)[10];
  real_T (*c1_nom_torque_table)[100];
  c1_t2err_p = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
  c1_t2err = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_T2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c1_t_step = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c1_t2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c1_t1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c1_points = (real_T (*)[10])ssGetInputPortSignal(chartInstance->S, 1);
  c1_nom_torque_table = (real_T (*)[100])ssGetInputPortSignal(chartInstance->S,
    0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  for (c1_i0 = 0; c1_i0 < 100; c1_i0++) {
    _SFD_DATA_RANGE_CHECK((*c1_nom_torque_table)[c1_i0], 0U);
  }

  for (c1_i1 = 0; c1_i1 < 10; c1_i1++) {
    _SFD_DATA_RANGE_CHECK((*c1_points)[c1_i1], 1U);
  }

  _SFD_DATA_RANGE_CHECK(*c1_t1, 2U);
  _SFD_DATA_RANGE_CHECK(*c1_t2, 3U);
  _SFD_DATA_RANGE_CHECK(*c1_t_step, 4U);
  _SFD_DATA_RANGE_CHECK(*c1_T2, 5U);
  _SFD_DATA_RANGE_CHECK(*c1_t2err, 6U);
  _SFD_DATA_RANGE_CHECK(*c1_t2err_p, 7U);
  chartInstance->c1_sfEvent = CALL_EVENT;
  c1_chartstep_c1_compassGaitSim(chartInstance);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_compassGaitSimMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c1_chartstep_c1_compassGaitSim(SFc1_compassGaitSimInstanceStruct
  *chartInstance)
{
  real_T c1_hoistedGlobal;
  real_T c1_b_hoistedGlobal;
  real_T c1_c_hoistedGlobal;
  real_T c1_d_hoistedGlobal;
  int32_T c1_i2;
  real_T c1_nom_torque_table[100];
  int32_T c1_i3;
  real_T c1_points[10];
  real_T c1_t1;
  real_T c1_t2;
  real_T c1_t_step;
  real_T c1_t2err_p;
  uint32_T c1_debug_family_var_map[19];
  int32_T c1_i_sizes;
  real_T c1_i_data[1];
  real_T c1_ths[2];
  real_T c1_us[2];
  real_T c1_K_p;
  real_T c1_K_d;
  real_T c1_t;
  real_T c1_t2_des;
  real_T c1_n;
  real_T c1_t2ed;
  real_T c1_i;
  real_T c1_nargin = 6.0;
  real_T c1_nargout = 2.0;
  real_T c1_T2;
  real_T c1_t2err;
  int32_T c1_i4;
  boolean_T c1_x[50];
  int32_T c1_idx;
  static int32_T c1_iv0[1] = { 1 };

  int32_T c1_ii_sizes;
  int32_T c1_ii;
  int32_T c1_b_ii;
  int32_T c1_ii_data[1];
  int32_T c1_loop_ub;
  int32_T c1_i5;
  int32_T c1_i6;
  int32_T c1_i7;
  real_T c1_A;
  real_T c1_B;
  real_T c1_b_x;
  real_T c1_y;
  real_T c1_c_x;
  real_T c1_b_y;
  real_T c1_c_y;
  real_T c1_a;
  real_T c1_b;
  real_T c1_d_y;
  real_T c1_b_A;
  real_T c1_b_B;
  real_T c1_d_x;
  real_T c1_e_y;
  real_T c1_e_x;
  real_T c1_f_y;
  real_T c1_dv0[1];
  int32_T c1_tmp_sizes[2];
  int32_T c1_tmp_data[1];
  int32_T c1_b_loop_ub;
  int32_T c1_i8;
  real_T c1_dv1[1];
  int32_T c1_c_loop_ub;
  int32_T c1_i9;
  int32_T c1_b_i;
  real_T c1_k;
  real_T c1_f_x;
  real_T c1_g_x;
  boolean_T c1_b0;
  int32_T c1_i10;
  static char_T c1_cv0[27] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'n', 'c', 'h',
    'o', 'o', 's', 'e', 'k', ':', 'I', 'n', 'v', 'a', 'l', 'i', 'd', 'A', 'r',
    'g', '2' };

  char_T c1_u[27];
  const mxArray *c1_g_y = NULL;
  real_T c1_b_k;
  real_T c1_h_y;
  real_T c1_nmk;
  real_T c1_c_k;
  int32_T c1_i11;
  int32_T c1_j;
  real_T c1_b_j;
  real_T c1_b_a;
  real_T c1_b_b;
  real_T c1_c_a;
  real_T c1_c_b;
  real_T c1_i_y;
  real_T c1_d_a;
  real_T c1_d_b;
  real_T c1_j_y;
  real_T c1_e_a;
  real_T c1_e_b;
  real_T c1_k_y;
  real_T c1_h_x;
  real_T c1_xk;
  real_T c1_i_x;
  real_T c1_j_x;
  real_T c1_k_x;
  real_T c1_l_x;
  real_T c1_l_y;
  real_T c1_m_x;
  real_T c1_m_y;
  real_T c1_f_b;
  real_T c1_n_y;
  real_T c1_n_x;
  real_T c1_o_x;
  real_T c1_dv2[1];
  int32_T c1_d_loop_ub;
  int32_T c1_i12;
  real_T c1_dv3[1];
  int32_T c1_e_loop_ub;
  int32_T c1_i13;
  real_T c1_c_A;
  real_T c1_c_B;
  real_T c1_p_x;
  real_T c1_o_y;
  real_T c1_q_x;
  real_T c1_p_y;
  real_T c1_g_b;
  real_T c1_q_y;
  real_T c1_h_b;
  real_T c1_r_y;
  real_T *c1_b_t1;
  real_T *c1_b_t2;
  real_T *c1_b_t_step;
  real_T *c1_b_t2err_p;
  real_T *c1_b_T2;
  real_T *c1_b_t2err;
  real_T (*c1_b_points)[10];
  real_T (*c1_b_nom_torque_table)[100];
  boolean_T exitg1;
  c1_b_t2err_p = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
  c1_b_t2err = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_b_T2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c1_b_t_step = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c1_b_t2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c1_b_t1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c1_b_points = (real_T (*)[10])ssGetInputPortSignal(chartInstance->S, 1);
  c1_b_nom_torque_table = (real_T (*)[100])ssGetInputPortSignal(chartInstance->S,
    0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  c1_hoistedGlobal = *c1_b_t1;
  c1_b_hoistedGlobal = *c1_b_t2;
  c1_c_hoistedGlobal = *c1_b_t_step;
  c1_d_hoistedGlobal = *c1_b_t2err_p;
  for (c1_i2 = 0; c1_i2 < 100; c1_i2++) {
    c1_nom_torque_table[c1_i2] = (*c1_b_nom_torque_table)[c1_i2];
  }

  for (c1_i3 = 0; c1_i3 < 10; c1_i3++) {
    c1_points[c1_i3] = (*c1_b_points)[c1_i3];
  }

  c1_t1 = c1_hoistedGlobal;
  c1_t2 = c1_b_hoistedGlobal;
  c1_t_step = c1_c_hoistedGlobal;
  c1_t2err_p = c1_d_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 19U, 20U, c1_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c1_i_data, (const int32_T *)
    &c1_i_sizes, NULL, 0, -1, (void *)c1_e_sf_marshallOut, (void *)
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_ths, 1U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_us, 2U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_K_p, 3U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_K_d, 4U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_t, 5U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_t2_des, 6U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_n, 7U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_t2ed, 8U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_i, MAX_uint32_T, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 9U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 10U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c1_nom_torque_table, 11U, c1_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c1_points, 12U, c1_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_t1, 13U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_t2, 14U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_t_step, 15U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_t2err_p, 16U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_T2, 17U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_t2err, 18U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 7);
  for (c1_i4 = 0; c1_i4 < 50; c1_i4++) {
    c1_x[c1_i4] = (c1_t1 <= c1_nom_torque_table[c1_i4]);
  }

  c1_idx = 0;
  c1_ii_sizes = c1_iv0[0];
  c1_ii = 50;
  exitg1 = FALSE;
  while ((exitg1 == FALSE) && (c1_ii > 0)) {
    c1_b_ii = c1_ii;
    if (c1_x[c1_b_ii - 1]) {
      c1_idx = 1;
      _SFD_EML_ARRAY_BOUNDS_CHECK("", 1, 1, c1_ii_sizes, 1, 0);
      c1_ii_data[0] = c1_b_ii;
      exitg1 = TRUE;
    } else {
      c1_ii--;
    }
  }

  if (c1_idx == 0) {
    c1_ii_sizes = 0;
  }

  c1_i_sizes = c1_ii_sizes;
  c1_loop_ub = c1_ii_sizes - 1;
  for (c1_i5 = 0; c1_i5 <= c1_loop_ub; c1_i5++) {
    c1_i_data[c1_i5] = (real_T)c1_ii_data[c1_i5];
  }

  _SFD_SYMBOL_SWITCH(0U, 0U);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 8);
  if (CV_EML_IF(0, 1, 0, c1_i_sizes == 0)) {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 9);
    c1_T2 = 0.0;
  } else {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 11);
    c1_i = c1_i_data[0];
    _SFD_SYMBOL_SWITCH(0U, 9U);
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 12);
    if (CV_EML_IF(0, 1, 1, c1_i < 50.0)) {
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 13);
      for (c1_i6 = 0; c1_i6 < 2; c1_i6++) {
        c1_ths[c1_i6] = c1_nom_torque_table[(int32_T)(c1_i + (real_T)c1_i6) - 1];
      }

      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 14);
      for (c1_i7 = 0; c1_i7 < 2; c1_i7++) {
        c1_us[c1_i7] = c1_nom_torque_table[(int32_T)(c1_i + (real_T)c1_i7) + 49];
      }

      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 15);
      c1_A = c1_us[1] - c1_us[0];
      c1_B = c1_ths[1] - c1_ths[0];
      c1_b_x = c1_A;
      c1_y = c1_B;
      c1_c_x = c1_b_x;
      c1_b_y = c1_y;
      c1_c_y = c1_c_x / c1_b_y;
      c1_a = c1_c_y;
      c1_b = c1_t1 - c1_ths[0];
      c1_d_y = c1_a * c1_b;
      c1_T2 = c1_d_y + c1_us[0];
    } else {
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 17);
      c1_T2 = 0.0;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 23);
  c1_K_p = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 24);
  c1_K_d = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 27);
  c1_b_A = c1_t1 - c1_points[0];
  c1_b_B = c1_points[4] - c1_points[0];
  c1_d_x = c1_b_A;
  c1_e_y = c1_b_B;
  c1_e_x = c1_d_x;
  c1_f_y = c1_e_y;
  c1_t = c1_e_x / c1_f_y;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 28);
  c1_dv0[0] = c1_t;
  c1_eml_li_find(chartInstance, c1_t > 1.0, c1_tmp_data, c1_tmp_sizes);
  c1_b_loop_ub = c1_tmp_sizes[0] * c1_tmp_sizes[1] - 1;
  for (c1_i8 = 0; c1_i8 <= c1_b_loop_ub; c1_i8++) {
    c1_dv0[c1_tmp_data[c1_i8] - 1] = 1.0;
  }

  c1_t = c1_dv0[0];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 29);
  c1_dv1[0] = c1_t;
  c1_eml_li_find(chartInstance, c1_t < 0.0, c1_tmp_data, c1_tmp_sizes);
  c1_c_loop_ub = c1_tmp_sizes[0] * c1_tmp_sizes[1] - 1;
  for (c1_i9 = 0; c1_i9 <= c1_c_loop_ub; c1_i9++) {
    c1_dv1[c1_tmp_data[c1_i9] - 1] = 0.0;
  }

  c1_t = c1_dv1[0];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 30);
  c1_t2_des = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 31);
  c1_n = 4.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 32);
  c1_i = 0.0;
  _SFD_SYMBOL_SWITCH(0U, 9U);
  c1_b_i = 0;
  while (c1_b_i < 5) {
    c1_i = (real_T)c1_b_i;
    _SFD_SYMBOL_SWITCH(0U, 9U);
    CV_EML_FOR(0, 1, 0, 1);
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 33);
    c1_k = c1_i;
    c1_f_x = c1_k;
    c1_g_x = c1_f_x;
    c1_g_x = muDoubleScalarFloor(c1_g_x);
    c1_b0 = (c1_k == c1_g_x);
    if (c1_b0) {
    } else {
      for (c1_i10 = 0; c1_i10 < 27; c1_i10++) {
        c1_u[c1_i10] = c1_cv0[c1_i10];
      }

      c1_g_y = NULL;
      sf_mex_assign(&c1_g_y, sf_mex_create("y", c1_u, 10, 0U, 1U, 0U, 2, 1, 27),
                    FALSE);
      sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U,
        14, c1_g_y));
    }

    c1_b_k = c1_k;
    c1_eml_scalar_eg(chartInstance);
    if (c1_b_k > 2.0) {
      c1_b_k = 4.0 - c1_b_k;
    }

    c1_h_y = 1.0;
    c1_nmk = 4.0 - c1_b_k;
    c1_c_k = c1_b_k;
    c1_i11 = (int32_T)c1_c_k - 1;
    for (c1_j = 0; c1_j <= c1_i11; c1_j++) {
      c1_b_j = 1.0 + (real_T)c1_j;
      c1_b_a = c1_h_y;
      c1_b_b = (c1_b_j + c1_nmk) / c1_b_j;
      c1_h_y = c1_b_a * c1_b_b;
    }

    c1_h_y = muDoubleScalarRound(c1_h_y);
    if (!(c1_h_y <= 9.007199254740992E+15)) {
      c1_eml_warning(chartInstance);
    }

    c1_c_a = c1_h_y;
    c1_c_b = c1_mpower(chartInstance, 1.0 - c1_t, 4.0 - c1_i);
    c1_i_y = c1_c_a * c1_c_b;
    c1_d_a = c1_i_y;
    c1_d_b = c1_mpower(chartInstance, c1_t, c1_i);
    c1_j_y = c1_d_a * c1_d_b;
    c1_e_a = c1_j_y;
    c1_e_b = c1_points[(int32_T)(c1_i + 1.0) + 4];
    c1_k_y = c1_e_a * c1_e_b;
    c1_t2_des += c1_k_y;
    c1_b_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 0, 0);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 36);
  c1_t2err = c1_t2_des - c1_t2;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 38);
  c1_h_x = c1_t2err;
  c1_eml_scalar_eg(chartInstance);
  c1_xk = c1_h_x;
  c1_i_x = c1_xk;
  c1_eml_scalar_eg(chartInstance);
  c1_t2err = c1_i_x / 6.2831853071795862;
  c1_j_x = c1_t2err;
  c1_k_x = c1_j_x;
  c1_k_x = muDoubleScalarRound(c1_k_x);
  c1_l_x = c1_t2err - c1_k_x;
  c1_l_y = muDoubleScalarAbs(c1_l_x);
  c1_m_x = c1_t2err;
  c1_m_y = muDoubleScalarAbs(c1_m_x);
  c1_f_b = c1_m_y;
  c1_n_y = 2.2204460492503131E-16 * c1_f_b;
  if (c1_l_y <= c1_n_y) {
    c1_t2err = 0.0;
  } else {
    c1_n_x = c1_t2err;
    c1_o_x = c1_n_x;
    c1_o_x = muDoubleScalarFloor(c1_o_x);
    c1_t2err = (c1_t2err - c1_o_x) * 6.2831853071795862;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 40);
  if (CV_EML_IF(0, 1, 2, c1_t2err > 3.1415926535897931)) {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 41);
    c1_t2err = -(6.2831853071795862 - c1_t2err);
  } else {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 42);
    if (CV_EML_IF(0, 1, 3, c1_t2err < -3.1415926535897931)) {
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 43);
      c1_t2err = 6.2831853071795862 - c1_t2err;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 46);
  c1_t2ed = c1_t2err - c1_t2err_p;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 48);
  c1_dv2[0] = c1_t2ed;
  c1_eml_li_find(chartInstance, c1_t2ed > 3.1415926535897931, c1_tmp_data,
                 c1_tmp_sizes);
  c1_d_loop_ub = c1_tmp_sizes[0] * c1_tmp_sizes[1] - 1;
  for (c1_i12 = 0; c1_i12 <= c1_d_loop_ub; c1_i12++) {
    c1_dv2[c1_tmp_data[c1_i12] - 1] = c1_t2ed - 3.1415926535897931;
  }

  c1_t2ed = c1_dv2[0];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 49);
  c1_dv3[0] = c1_t2ed;
  c1_eml_li_find(chartInstance, c1_t2ed < -3.1415926535897931, c1_tmp_data,
                 c1_tmp_sizes);
  c1_e_loop_ub = c1_tmp_sizes[0] * c1_tmp_sizes[1] - 1;
  for (c1_i13 = 0; c1_i13 <= c1_e_loop_ub; c1_i13++) {
    c1_dv3[c1_tmp_data[c1_i13] - 1] = c1_t2ed + 3.1415926535897931;
  }

  c1_t2ed = c1_dv3[0];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 50);
  c1_c_A = c1_t2ed;
  c1_c_B = c1_t_step;
  c1_p_x = c1_c_A;
  c1_o_y = c1_c_B;
  c1_q_x = c1_p_x;
  c1_p_y = c1_o_y;
  c1_t2ed = c1_q_x / c1_p_y;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 52);
  c1_g_b = c1_t2err;
  c1_q_y = 0.0 * c1_g_b;
  c1_h_b = c1_t2ed;
  c1_r_y = 0.0 * c1_h_b;
  c1_T2 = (c1_T2 + c1_q_y) + c1_r_y;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, -52);
  _SFD_SYMBOL_SCOPE_POP();
  *c1_b_T2 = c1_T2;
  *c1_b_t2err = c1_t2err;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
}

static void initSimStructsc1_compassGaitSim(SFc1_compassGaitSimInstanceStruct
  *chartInstance)
{
}

static void registerMessagesc1_compassGaitSim(SFc1_compassGaitSimInstanceStruct *
  chartInstance)
{
}

static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber)
{
}

static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_compassGaitSimInstanceStruct *chartInstance;
  chartInstance = (SFc1_compassGaitSimInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static real_T c1_emlrt_marshallIn(SFc1_compassGaitSimInstanceStruct
  *chartInstance, const mxArray *c1_t2err, const char_T *c1_identifier)
{
  real_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_t2err), &c1_thisId);
  sf_mex_destroy(&c1_t2err);
  return c1_y;
}

static real_T c1_b_emlrt_marshallIn(SFc1_compassGaitSimInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_y;
  real_T c1_d0;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d0, 1, 0, 0U, 0, 0U, 0);
  c1_y = c1_d0;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_t2err;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_compassGaitSimInstanceStruct *chartInstance;
  chartInstance = (SFc1_compassGaitSimInstanceStruct *)chartInstanceVoid;
  c1_t2err = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_t2err), &c1_thisId);
  sf_mex_destroy(&c1_t2err);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i14;
  int32_T c1_i15;
  int32_T c1_i16;
  real_T c1_b_inData[10];
  int32_T c1_i17;
  int32_T c1_i18;
  int32_T c1_i19;
  real_T c1_u[10];
  const mxArray *c1_y = NULL;
  SFc1_compassGaitSimInstanceStruct *chartInstance;
  chartInstance = (SFc1_compassGaitSimInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_i14 = 0;
  for (c1_i15 = 0; c1_i15 < 2; c1_i15++) {
    for (c1_i16 = 0; c1_i16 < 5; c1_i16++) {
      c1_b_inData[c1_i16 + c1_i14] = (*(real_T (*)[10])c1_inData)[c1_i16 +
        c1_i14];
    }

    c1_i14 += 5;
  }

  c1_i17 = 0;
  for (c1_i18 = 0; c1_i18 < 2; c1_i18++) {
    for (c1_i19 = 0; c1_i19 < 5; c1_i19++) {
      c1_u[c1_i19 + c1_i17] = c1_b_inData[c1_i19 + c1_i17];
    }

    c1_i17 += 5;
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 2, 5, 2), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i20;
  int32_T c1_i21;
  int32_T c1_i22;
  real_T c1_b_inData[100];
  int32_T c1_i23;
  int32_T c1_i24;
  int32_T c1_i25;
  real_T c1_u[100];
  const mxArray *c1_y = NULL;
  SFc1_compassGaitSimInstanceStruct *chartInstance;
  chartInstance = (SFc1_compassGaitSimInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_i20 = 0;
  for (c1_i21 = 0; c1_i21 < 2; c1_i21++) {
    for (c1_i22 = 0; c1_i22 < 50; c1_i22++) {
      c1_b_inData[c1_i22 + c1_i20] = (*(real_T (*)[100])c1_inData)[c1_i22 +
        c1_i20];
    }

    c1_i20 += 50;
  }

  c1_i23 = 0;
  for (c1_i24 = 0; c1_i24 < 2; c1_i24++) {
    for (c1_i25 = 0; c1_i25 < 50; c1_i25++) {
      c1_u[c1_i25 + c1_i23] = c1_b_inData[c1_i25 + c1_i23];
    }

    c1_i23 += 50;
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 2, 50, 2), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i26;
  real_T c1_b_inData[2];
  int32_T c1_i27;
  real_T c1_u[2];
  const mxArray *c1_y = NULL;
  SFc1_compassGaitSimInstanceStruct *chartInstance;
  chartInstance = (SFc1_compassGaitSimInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  for (c1_i26 = 0; c1_i26 < 2; c1_i26++) {
    c1_b_inData[c1_i26] = (*(real_T (*)[2])c1_inData)[c1_i26];
  }

  for (c1_i27 = 0; c1_i27 < 2; c1_i27++) {
    c1_u[c1_i27] = c1_b_inData[c1_i27];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 1, 2), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static void c1_c_emlrt_marshallIn(SFc1_compassGaitSimInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_y[2])
{
  real_T c1_dv4[2];
  int32_T c1_i28;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_dv4, 1, 0, 0U, 1, 0U, 1, 2);
  for (c1_i28 = 0; c1_i28 < 2; c1_i28++) {
    c1_y[c1_i28] = c1_dv4[c1_i28];
  }

  sf_mex_destroy(&c1_u);
}

static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_us;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y[2];
  int32_T c1_i29;
  SFc1_compassGaitSimInstanceStruct *chartInstance;
  chartInstance = (SFc1_compassGaitSimInstanceStruct *)chartInstanceVoid;
  c1_us = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_us), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_us);
  for (c1_i29 = 0; c1_i29 < 2; c1_i29++) {
    (*(real_T (*)[2])c1_outData)[c1_i29] = c1_y[c1_i29];
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_e_sf_marshallOut(void *chartInstanceVoid, real_T
  c1_inData_data[1], int32_T c1_inData_sizes[1])
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_b_inData_sizes;
  int32_T c1_loop_ub;
  int32_T c1_i30;
  real_T c1_b_inData_data[1];
  int32_T c1_u_sizes;
  int32_T c1_b_loop_ub;
  int32_T c1_i31;
  real_T c1_u_data[1];
  const mxArray *c1_y = NULL;
  SFc1_compassGaitSimInstanceStruct *chartInstance;
  chartInstance = (SFc1_compassGaitSimInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_b_inData_sizes = c1_inData_sizes[0];
  c1_loop_ub = c1_inData_sizes[0] - 1;
  for (c1_i30 = 0; c1_i30 <= c1_loop_ub; c1_i30++) {
    c1_b_inData_data[c1_i30] = c1_inData_data[c1_i30];
  }

  c1_u_sizes = c1_b_inData_sizes;
  c1_b_loop_ub = c1_b_inData_sizes - 1;
  for (c1_i31 = 0; c1_i31 <= c1_b_loop_ub; c1_i31++) {
    c1_u_data[c1_i31] = c1_b_inData_data[c1_i31];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u_data, 0, 0U, 1U, 0U, 1,
    c1_u_sizes), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static void c1_d_emlrt_marshallIn(SFc1_compassGaitSimInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_y_data[1], int32_T c1_y_sizes[1])
{
  static uint32_T c1_uv0[1] = { 1U };

  uint32_T c1_uv1[1];
  static boolean_T c1_bv0[1] = { TRUE };

  boolean_T c1_bv1[1];
  int32_T c1_tmp_sizes;
  real_T c1_tmp_data[1];
  int32_T c1_loop_ub;
  int32_T c1_i32;
  c1_uv1[0] = c1_uv0[0];
  c1_bv1[0] = c1_bv0[0];
  sf_mex_import_vs(c1_parentId, sf_mex_dup(c1_u), c1_tmp_data, 1, 0, 0U, 1, 0U,
                   1, c1_bv1, c1_uv1, &c1_tmp_sizes);
  c1_y_sizes[0] = c1_tmp_sizes;
  c1_loop_ub = c1_tmp_sizes - 1;
  for (c1_i32 = 0; c1_i32 <= c1_loop_ub; c1_i32++) {
    c1_y_data[c1_i32] = c1_tmp_data[c1_i32];
  }

  sf_mex_destroy(&c1_u);
}

static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, real_T c1_outData_data[1],
  int32_T c1_outData_sizes[1])
{
  const mxArray *c1_i;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  int32_T c1_y_sizes;
  real_T c1_y_data[1];
  int32_T c1_loop_ub;
  int32_T c1_i33;
  SFc1_compassGaitSimInstanceStruct *chartInstance;
  chartInstance = (SFc1_compassGaitSimInstanceStruct *)chartInstanceVoid;
  c1_i = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_i), &c1_thisId, c1_y_data, *
                        (int32_T (*)[1])&c1_y_sizes);
  sf_mex_destroy(&c1_i);
  c1_outData_sizes[0] = c1_y_sizes;
  c1_loop_ub = c1_y_sizes - 1;
  for (c1_i33 = 0; c1_i33 <= c1_loop_ub; c1_i33++) {
    c1_outData_data[c1_i33] = c1_y_data[c1_i33];
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

const mxArray *sf_c1_compassGaitSim_get_eml_resolved_functions_info(void)
{
  const mxArray *c1_nameCaptureInfo;
  c1_ResolvedFunctionInfo c1_info[61];
  const mxArray *c1_m0 = NULL;
  int32_T c1_i34;
  c1_ResolvedFunctionInfo *c1_r0;
  c1_nameCaptureInfo = NULL;
  c1_nameCaptureInfo = NULL;
  c1_info_helper(c1_info);
  sf_mex_assign(&c1_m0, sf_mex_createstruct("nameCaptureInfo", 1, 61), FALSE);
  for (c1_i34 = 0; c1_i34 < 61; c1_i34++) {
    c1_r0 = &c1_info[c1_i34];
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c1_r0->context)), "context", "nameCaptureInfo",
                    c1_i34);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c1_r0->name)), "name", "nameCaptureInfo", c1_i34);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c1_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c1_i34);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c1_r0->resolved)), "resolved", "nameCaptureInfo",
                    c1_i34);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c1_i34);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c1_i34);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c1_i34);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c1_i34);
  }

  sf_mex_assign(&c1_nameCaptureInfo, c1_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c1_nameCaptureInfo);
  return c1_nameCaptureInfo;
}

static void c1_info_helper(c1_ResolvedFunctionInfo c1_info[61])
{
  c1_info[0].context = "";
  c1_info[0].name = "find";
  c1_info[0].dominantType = "double";
  c1_info[0].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m";
  c1_info[0].fileTimeLo = 1303117406U;
  c1_info[0].fileTimeHi = 0U;
  c1_info[0].mFileTimeLo = 0U;
  c1_info[0].mFileTimeHi = 0U;
  c1_info[1].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m!eml_find";
  c1_info[1].name = "eml_index_class";
  c1_info[1].dominantType = "";
  c1_info[1].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c1_info[1].fileTimeLo = 1323134578U;
  c1_info[1].fileTimeHi = 0U;
  c1_info[1].mFileTimeLo = 0U;
  c1_info[1].mFileTimeHi = 0U;
  c1_info[2].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m!eml_find";
  c1_info[2].name = "eml_scalar_eg";
  c1_info[2].dominantType = "logical";
  c1_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c1_info[2].fileTimeLo = 1286786396U;
  c1_info[2].fileTimeHi = 0U;
  c1_info[2].mFileTimeLo = 0U;
  c1_info[2].mFileTimeHi = 0U;
  c1_info[3].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m!eml_find";
  c1_info[3].name = "floor";
  c1_info[3].dominantType = "double";
  c1_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c1_info[3].fileTimeLo = 1343801580U;
  c1_info[3].fileTimeHi = 0U;
  c1_info[3].mFileTimeLo = 0U;
  c1_info[3].mFileTimeHi = 0U;
  c1_info[4].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c1_info[4].name = "eml_scalar_floor";
  c1_info[4].dominantType = "double";
  c1_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c1_info[4].fileTimeLo = 1286786326U;
  c1_info[4].fileTimeHi = 0U;
  c1_info[4].mFileTimeLo = 0U;
  c1_info[4].mFileTimeHi = 0U;
  c1_info[5].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m!eml_find";
  c1_info[5].name = "min";
  c1_info[5].dominantType = "coder.internal.indexInt";
  c1_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c1_info[5].fileTimeLo = 1311226518U;
  c1_info[5].fileTimeHi = 0U;
  c1_info[5].mFileTimeLo = 0U;
  c1_info[5].mFileTimeHi = 0U;
  c1_info[6].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c1_info[6].name = "eml_min_or_max";
  c1_info[6].dominantType = "char";
  c1_info[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c1_info[6].fileTimeLo = 1334042690U;
  c1_info[6].fileTimeHi = 0U;
  c1_info[6].mFileTimeLo = 0U;
  c1_info[6].mFileTimeHi = 0U;
  c1_info[7].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c1_info[7].name = "eml_scalar_eg";
  c1_info[7].dominantType = "coder.internal.indexInt";
  c1_info[7].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c1_info[7].fileTimeLo = 1286786396U;
  c1_info[7].fileTimeHi = 0U;
  c1_info[7].mFileTimeLo = 0U;
  c1_info[7].mFileTimeHi = 0U;
  c1_info[8].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c1_info[8].name = "eml_scalexp_alloc";
  c1_info[8].dominantType = "coder.internal.indexInt";
  c1_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c1_info[8].fileTimeLo = 1352388860U;
  c1_info[8].fileTimeHi = 0U;
  c1_info[8].mFileTimeLo = 0U;
  c1_info[8].mFileTimeHi = 0U;
  c1_info[9].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c1_info[9].name = "eml_index_class";
  c1_info[9].dominantType = "";
  c1_info[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c1_info[9].fileTimeLo = 1323134578U;
  c1_info[9].fileTimeHi = 0U;
  c1_info[9].mFileTimeLo = 0U;
  c1_info[9].mFileTimeHi = 0U;
  c1_info[10].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c1_info[10].name = "eml_scalar_eg";
  c1_info[10].dominantType = "coder.internal.indexInt";
  c1_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c1_info[10].fileTimeLo = 1286786396U;
  c1_info[10].fileTimeHi = 0U;
  c1_info[10].mFileTimeLo = 0U;
  c1_info[10].mFileTimeHi = 0U;
  c1_info[11].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m!eml_find";
  c1_info[11].name = "eml_int_forloop_overflow_check";
  c1_info[11].dominantType = "";
  c1_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c1_info[11].fileTimeLo = 1346481540U;
  c1_info[11].fileTimeHi = 0U;
  c1_info[11].mFileTimeLo = 0U;
  c1_info[11].mFileTimeHi = 0U;
  c1_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c1_info[12].name = "intmin";
  c1_info[12].dominantType = "char";
  c1_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c1_info[12].fileTimeLo = 1311226518U;
  c1_info[12].fileTimeHi = 0U;
  c1_info[12].mFileTimeLo = 0U;
  c1_info[12].mFileTimeHi = 0U;
  c1_info[13].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m!eml_find";
  c1_info[13].name = "eml_index_plus";
  c1_info[13].dominantType = "double";
  c1_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c1_info[13].fileTimeLo = 1286786378U;
  c1_info[13].fileTimeHi = 0U;
  c1_info[13].mFileTimeLo = 0U;
  c1_info[13].mFileTimeHi = 0U;
  c1_info[14].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c1_info[14].name = "eml_index_class";
  c1_info[14].dominantType = "";
  c1_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c1_info[14].fileTimeLo = 1323134578U;
  c1_info[14].fileTimeHi = 0U;
  c1_info[14].mFileTimeLo = 0U;
  c1_info[14].mFileTimeHi = 0U;
  c1_info[15].context = "";
  c1_info[15].name = "mrdivide";
  c1_info[15].dominantType = "double";
  c1_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c1_info[15].fileTimeLo = 1357915548U;
  c1_info[15].fileTimeHi = 0U;
  c1_info[15].mFileTimeLo = 1319697566U;
  c1_info[15].mFileTimeHi = 0U;
  c1_info[16].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c1_info[16].name = "rdivide";
  c1_info[16].dominantType = "double";
  c1_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c1_info[16].fileTimeLo = 1346481588U;
  c1_info[16].fileTimeHi = 0U;
  c1_info[16].mFileTimeLo = 0U;
  c1_info[16].mFileTimeHi = 0U;
  c1_info[17].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c1_info[17].name = "eml_scalexp_compatible";
  c1_info[17].dominantType = "double";
  c1_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m";
  c1_info[17].fileTimeLo = 1286786396U;
  c1_info[17].fileTimeHi = 0U;
  c1_info[17].mFileTimeLo = 0U;
  c1_info[17].mFileTimeHi = 0U;
  c1_info[18].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c1_info[18].name = "eml_div";
  c1_info[18].dominantType = "double";
  c1_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c1_info[18].fileTimeLo = 1313319010U;
  c1_info[18].fileTimeHi = 0U;
  c1_info[18].mFileTimeLo = 0U;
  c1_info[18].mFileTimeHi = 0U;
  c1_info[19].context = "";
  c1_info[19].name = "mtimes";
  c1_info[19].dominantType = "double";
  c1_info[19].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c1_info[19].fileTimeLo = 1289483692U;
  c1_info[19].fileTimeHi = 0U;
  c1_info[19].mFileTimeLo = 0U;
  c1_info[19].mFileTimeHi = 0U;
  c1_info[20].context = "";
  c1_info[20].name = "eml_li_find";
  c1_info[20].dominantType = "";
  c1_info[20].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m";
  c1_info[20].fileTimeLo = 1286786386U;
  c1_info[20].fileTimeHi = 0U;
  c1_info[20].mFileTimeLo = 0U;
  c1_info[20].mFileTimeHi = 0U;
  c1_info[21].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m";
  c1_info[21].name = "eml_index_class";
  c1_info[21].dominantType = "";
  c1_info[21].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c1_info[21].fileTimeLo = 1323134578U;
  c1_info[21].fileTimeHi = 0U;
  c1_info[21].mFileTimeLo = 0U;
  c1_info[21].mFileTimeHi = 0U;
  c1_info[22].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m!compute_nones";
  c1_info[22].name = "eml_index_class";
  c1_info[22].dominantType = "";
  c1_info[22].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c1_info[22].fileTimeLo = 1323134578U;
  c1_info[22].fileTimeHi = 0U;
  c1_info[22].mFileTimeLo = 0U;
  c1_info[22].mFileTimeHi = 0U;
  c1_info[23].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m!compute_nones";
  c1_info[23].name = "eml_index_plus";
  c1_info[23].dominantType = "double";
  c1_info[23].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c1_info[23].fileTimeLo = 1286786378U;
  c1_info[23].fileTimeHi = 0U;
  c1_info[23].mFileTimeLo = 0U;
  c1_info[23].mFileTimeHi = 0U;
  c1_info[24].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m";
  c1_info[24].name = "eml_index_plus";
  c1_info[24].dominantType = "double";
  c1_info[24].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c1_info[24].fileTimeLo = 1286786378U;
  c1_info[24].fileTimeHi = 0U;
  c1_info[24].mFileTimeLo = 0U;
  c1_info[24].mFileTimeHi = 0U;
  c1_info[25].context = "";
  c1_info[25].name = "nchoosek";
  c1_info[25].dominantType = "double";
  c1_info[25].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m";
  c1_info[25].fileTimeLo = 1352388862U;
  c1_info[25].fileTimeHi = 0U;
  c1_info[25].mFileTimeLo = 0U;
  c1_info[25].mFileTimeHi = 0U;
  c1_info[26].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m";
  c1_info[26].name = "floor";
  c1_info[26].dominantType = "double";
  c1_info[26].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c1_info[26].fileTimeLo = 1343801580U;
  c1_info[26].fileTimeHi = 0U;
  c1_info[26].mFileTimeLo = 0U;
  c1_info[26].mFileTimeHi = 0U;
  c1_info[27].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m";
  c1_info[27].name = "eml_error";
  c1_info[27].dominantType = "char";
  c1_info[27].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c1_info[27].fileTimeLo = 1343801558U;
  c1_info[27].fileTimeHi = 0U;
  c1_info[27].mFileTimeLo = 0U;
  c1_info[27].mFileTimeHi = 0U;
  c1_info[28].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m";
  c1_info[28].name = "eml_scalar_eg";
  c1_info[28].dominantType = "double";
  c1_info[28].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c1_info[28].fileTimeLo = 1286786396U;
  c1_info[28].fileTimeHi = 0U;
  c1_info[28].mFileTimeLo = 0U;
  c1_info[28].mFileTimeHi = 0U;
  c1_info[29].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m";
  c1_info[29].name = "eml_guarded_nan";
  c1_info[29].dominantType = "";
  c1_info[29].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m";
  c1_info[29].fileTimeLo = 1286786376U;
  c1_info[29].fileTimeHi = 0U;
  c1_info[29].mFileTimeLo = 0U;
  c1_info[29].mFileTimeHi = 0U;
  c1_info[30].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
  c1_info[30].name = "eml_scalar_eg";
  c1_info[30].dominantType = "double";
  c1_info[30].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c1_info[30].fileTimeLo = 1286786396U;
  c1_info[30].fileTimeHi = 0U;
  c1_info[30].mFileTimeLo = 0U;
  c1_info[30].mFileTimeHi = 0U;
  c1_info[31].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
  c1_info[31].name = "isfinite";
  c1_info[31].dominantType = "double";
  c1_info[31].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c1_info[31].fileTimeLo = 1286786358U;
  c1_info[31].fileTimeHi = 0U;
  c1_info[31].mFileTimeLo = 0U;
  c1_info[31].mFileTimeHi = 0U;
  c1_info[32].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c1_info[32].name = "isinf";
  c1_info[32].dominantType = "double";
  c1_info[32].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m";
  c1_info[32].fileTimeLo = 1286786360U;
  c1_info[32].fileTimeHi = 0U;
  c1_info[32].mFileTimeLo = 0U;
  c1_info[32].mFileTimeHi = 0U;
  c1_info[33].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c1_info[33].name = "isnan";
  c1_info[33].dominantType = "double";
  c1_info[33].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m";
  c1_info[33].fileTimeLo = 1286786360U;
  c1_info[33].fileTimeHi = 0U;
  c1_info[33].mFileTimeLo = 0U;
  c1_info[33].mFileTimeHi = 0U;
  c1_info[34].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
  c1_info[34].name = "eml_guarded_inf";
  c1_info[34].dominantType = "";
  c1_info[34].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_inf.m";
  c1_info[34].fileTimeLo = 1286786376U;
  c1_info[34].fileTimeHi = 0U;
  c1_info[34].mFileTimeLo = 0U;
  c1_info[34].mFileTimeHi = 0U;
  c1_info[35].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
  c1_info[35].name = "mtimes";
  c1_info[35].dominantType = "double";
  c1_info[35].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c1_info[35].fileTimeLo = 1289483692U;
  c1_info[35].fileTimeHi = 0U;
  c1_info[35].mFileTimeLo = 0U;
  c1_info[35].mFileTimeHi = 0U;
  c1_info[36].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
  c1_info[36].name = "round";
  c1_info[36].dominantType = "double";
  c1_info[36].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/round.m";
  c1_info[36].fileTimeLo = 1343801584U;
  c1_info[36].fileTimeHi = 0U;
  c1_info[36].mFileTimeLo = 0U;
  c1_info[36].mFileTimeHi = 0U;
  c1_info[37].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/round.m";
  c1_info[37].name = "eml_scalar_round";
  c1_info[37].dominantType = "double";
  c1_info[37].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_round.m";
  c1_info[37].fileTimeLo = 1307622438U;
  c1_info[37].fileTimeHi = 0U;
  c1_info[37].mFileTimeLo = 0U;
  c1_info[37].mFileTimeHi = 0U;
  c1_info[38].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
  c1_info[38].name = "eml_guarded_nan";
  c1_info[38].dominantType = "";
  c1_info[38].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m";
  c1_info[38].fileTimeLo = 1286786376U;
  c1_info[38].fileTimeHi = 0U;
  c1_info[38].mFileTimeLo = 0U;
  c1_info[38].mFileTimeHi = 0U;
  c1_info[39].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
  c1_info[39].name = "flintmax";
  c1_info[39].dominantType = "char";
  c1_info[39].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/flintmax.m";
  c1_info[39].fileTimeLo = 1348163116U;
  c1_info[39].fileTimeHi = 0U;
  c1_info[39].mFileTimeLo = 0U;
  c1_info[39].mFileTimeHi = 0U;
  c1_info[40].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/flintmax.m";
  c1_info[40].name = "eml_float_model";
  c1_info[40].dominantType = "char";
  c1_info[40].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c1_info[40].fileTimeLo = 1326691996U;
  c1_info[40].fileTimeHi = 0U;
  c1_info[40].mFileTimeLo = 0U;
  c1_info[40].mFileTimeHi = 0U;
  c1_info[41].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/nchoosek.m!nCk";
  c1_info[41].name = "eml_warning";
  c1_info[41].dominantType = "char";
  c1_info[41].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c1_info[41].fileTimeLo = 1286786402U;
  c1_info[41].fileTimeHi = 0U;
  c1_info[41].mFileTimeLo = 0U;
  c1_info[41].mFileTimeHi = 0U;
  c1_info[42].context = "";
  c1_info[42].name = "mpower";
  c1_info[42].dominantType = "double";
  c1_info[42].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c1_info[42].fileTimeLo = 1286786442U;
  c1_info[42].fileTimeHi = 0U;
  c1_info[42].mFileTimeLo = 0U;
  c1_info[42].mFileTimeHi = 0U;
  c1_info[43].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c1_info[43].name = "power";
  c1_info[43].dominantType = "double";
  c1_info[43].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c1_info[43].fileTimeLo = 1348163130U;
  c1_info[43].fileTimeHi = 0U;
  c1_info[43].mFileTimeLo = 0U;
  c1_info[43].mFileTimeHi = 0U;
  c1_info[44].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c1_info[44].name = "eml_scalar_eg";
  c1_info[44].dominantType = "double";
  c1_info[44].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c1_info[44].fileTimeLo = 1286786396U;
  c1_info[44].fileTimeHi = 0U;
  c1_info[44].mFileTimeLo = 0U;
  c1_info[44].mFileTimeHi = 0U;
  c1_info[45].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c1_info[45].name = "eml_scalexp_alloc";
  c1_info[45].dominantType = "double";
  c1_info[45].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c1_info[45].fileTimeLo = 1352388860U;
  c1_info[45].fileTimeHi = 0U;
  c1_info[45].mFileTimeLo = 0U;
  c1_info[45].mFileTimeHi = 0U;
  c1_info[46].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c1_info[46].name = "floor";
  c1_info[46].dominantType = "double";
  c1_info[46].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c1_info[46].fileTimeLo = 1343801580U;
  c1_info[46].fileTimeHi = 0U;
  c1_info[46].mFileTimeLo = 0U;
  c1_info[46].mFileTimeHi = 0U;
  c1_info[47].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c1_info[47].name = "eml_error";
  c1_info[47].dominantType = "char";
  c1_info[47].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c1_info[47].fileTimeLo = 1343801558U;
  c1_info[47].fileTimeHi = 0U;
  c1_info[47].mFileTimeLo = 0U;
  c1_info[47].mFileTimeHi = 0U;
  c1_info[48].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power";
  c1_info[48].name = "eml_scalar_eg";
  c1_info[48].dominantType = "double";
  c1_info[48].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c1_info[48].fileTimeLo = 1286786396U;
  c1_info[48].fileTimeHi = 0U;
  c1_info[48].mFileTimeLo = 0U;
  c1_info[48].mFileTimeHi = 0U;
  c1_info[49].context = "";
  c1_info[49].name = "mod";
  c1_info[49].dominantType = "double";
  c1_info[49].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  c1_info[49].fileTimeLo = 1343801582U;
  c1_info[49].fileTimeHi = 0U;
  c1_info[49].mFileTimeLo = 0U;
  c1_info[49].mFileTimeHi = 0U;
  c1_info[50].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  c1_info[50].name = "eml_scalar_eg";
  c1_info[50].dominantType = "double";
  c1_info[50].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c1_info[50].fileTimeLo = 1286786396U;
  c1_info[50].fileTimeHi = 0U;
  c1_info[50].mFileTimeLo = 0U;
  c1_info[50].mFileTimeHi = 0U;
  c1_info[51].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  c1_info[51].name = "eml_scalexp_alloc";
  c1_info[51].dominantType = "double";
  c1_info[51].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c1_info[51].fileTimeLo = 1352388860U;
  c1_info[51].fileTimeHi = 0U;
  c1_info[51].mFileTimeLo = 0U;
  c1_info[51].mFileTimeHi = 0U;
  c1_info[52].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m!floatmod";
  c1_info[52].name = "eml_scalar_eg";
  c1_info[52].dominantType = "double";
  c1_info[52].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c1_info[52].fileTimeLo = 1286786396U;
  c1_info[52].fileTimeHi = 0U;
  c1_info[52].mFileTimeLo = 0U;
  c1_info[52].mFileTimeHi = 0U;
  c1_info[53].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m!floatmod";
  c1_info[53].name = "eml_scalar_floor";
  c1_info[53].dominantType = "double";
  c1_info[53].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c1_info[53].fileTimeLo = 1286786326U;
  c1_info[53].fileTimeHi = 0U;
  c1_info[53].mFileTimeLo = 0U;
  c1_info[53].mFileTimeHi = 0U;
  c1_info[54].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m!floatmod";
  c1_info[54].name = "eml_scalar_round";
  c1_info[54].dominantType = "double";
  c1_info[54].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_round.m";
  c1_info[54].fileTimeLo = 1307622438U;
  c1_info[54].fileTimeHi = 0U;
  c1_info[54].mFileTimeLo = 0U;
  c1_info[54].mFileTimeHi = 0U;
  c1_info[55].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m!floatmod";
  c1_info[55].name = "eml_scalar_abs";
  c1_info[55].dominantType = "double";
  c1_info[55].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c1_info[55].fileTimeLo = 1286786312U;
  c1_info[55].fileTimeHi = 0U;
  c1_info[55].mFileTimeLo = 0U;
  c1_info[55].mFileTimeHi = 0U;
  c1_info[56].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m!floatmod";
  c1_info[56].name = "eps";
  c1_info[56].dominantType = "char";
  c1_info[56].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c1_info[56].fileTimeLo = 1326691996U;
  c1_info[56].fileTimeHi = 0U;
  c1_info[56].mFileTimeLo = 0U;
  c1_info[56].mFileTimeHi = 0U;
  c1_info[57].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c1_info[57].name = "eml_is_float_class";
  c1_info[57].dominantType = "char";
  c1_info[57].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c1_info[57].fileTimeLo = 1286786382U;
  c1_info[57].fileTimeHi = 0U;
  c1_info[57].mFileTimeLo = 0U;
  c1_info[57].mFileTimeHi = 0U;
  c1_info[58].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c1_info[58].name = "eml_eps";
  c1_info[58].dominantType = "char";
  c1_info[58].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c1_info[58].fileTimeLo = 1326691996U;
  c1_info[58].fileTimeHi = 0U;
  c1_info[58].mFileTimeLo = 0U;
  c1_info[58].mFileTimeHi = 0U;
  c1_info[59].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c1_info[59].name = "eml_float_model";
  c1_info[59].dominantType = "char";
  c1_info[59].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c1_info[59].fileTimeLo = 1326691996U;
  c1_info[59].fileTimeHi = 0U;
  c1_info[59].mFileTimeLo = 0U;
  c1_info[59].mFileTimeHi = 0U;
  c1_info[60].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m!floatmod";
  c1_info[60].name = "mtimes";
  c1_info[60].dominantType = "double";
  c1_info[60].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c1_info[60].fileTimeLo = 1289483692U;
  c1_info[60].fileTimeHi = 0U;
  c1_info[60].mFileTimeLo = 0U;
  c1_info[60].mFileTimeHi = 0U;
}

static void c1_eml_li_find(SFc1_compassGaitSimInstanceStruct *chartInstance,
  boolean_T c1_x, int32_T c1_y_data[1], int32_T c1_y_sizes[2])
{
  boolean_T c1_b_x;
  int32_T c1_k;
  int32_T c1_tmp_sizes[2];
  int32_T c1_iv1[2];
  int32_T c1_i35;
  int32_T c1_i36;
  int32_T c1_loop_ub;
  int32_T c1_i37;
  int32_T c1_tmp_data[1];
  int32_T c1_i38;
  c1_b_x = c1_x;
  c1_k = 0;
  if (c1_b_x) {
    c1_k = 1;
  }

  c1_tmp_sizes[0] = 1;
  c1_iv1[0] = 1;
  c1_iv1[1] = c1_k;
  c1_tmp_sizes[1] = c1_iv1[1];
  c1_i35 = c1_tmp_sizes[0];
  c1_i36 = c1_tmp_sizes[1];
  c1_loop_ub = c1_k - 1;
  for (c1_i37 = 0; c1_i37 <= c1_loop_ub; c1_i37++) {
    c1_tmp_data[c1_i37] = 0;
  }

  for (c1_i38 = 0; c1_i38 < 2; c1_i38++) {
    c1_y_sizes[c1_i38] = c1_tmp_sizes[c1_i38];
  }

  if (c1_x) {
    _SFD_EML_ARRAY_BOUNDS_CHECK("", 1, 1, c1_y_sizes[1], 1, 0);
    c1_y_data[0] = 1;
  }
}

static void c1_eml_scalar_eg(SFc1_compassGaitSimInstanceStruct *chartInstance)
{
}

static void c1_eml_warning(SFc1_compassGaitSimInstanceStruct *chartInstance)
{
  int32_T c1_i39;
  static char_T c1_varargin_1[38] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 'n', 'c', 'h', 'o', 'o', 's', 'e', 'k', '_', 'L',
    'a', 'r', 'g', 'e', 'C', 'o', 'e', 'f', 'f', 'i', 'c', 'i', 'e', 'n', 't' };

  char_T c1_u[38];
  const mxArray *c1_y = NULL;
  for (c1_i39 = 0; c1_i39 < 38; c1_i39++) {
    c1_u[c1_i39] = c1_varargin_1[c1_i39];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 10, 0U, 1U, 0U, 2, 1, 38), FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U,
    14, c1_y));
}

static real_T c1_mpower(SFc1_compassGaitSimInstanceStruct *chartInstance, real_T
  c1_a, real_T c1_b)
{
  real_T c1_b_a;
  real_T c1_b_b;
  real_T c1_c_a;
  real_T c1_c_b;
  real_T c1_ak;
  real_T c1_bk;
  real_T c1_x;
  real_T c1_b_x;
  real_T c1_d_a;
  real_T c1_d_b;
  real_T c1_ar;
  real_T c1_br;
  c1_b_a = c1_a;
  c1_b_b = c1_b;
  c1_c_a = c1_b_a;
  c1_c_b = c1_b_b;
  c1_eml_scalar_eg(chartInstance);
  c1_ak = c1_c_a;
  c1_bk = c1_c_b;
  if (c1_ak < 0.0) {
    c1_x = c1_bk;
    c1_b_x = c1_x;
    c1_b_x = muDoubleScalarFloor(c1_b_x);
    if (c1_b_x != c1_bk) {
      c1_eml_error(chartInstance);
    }
  }

  c1_d_a = c1_ak;
  c1_d_b = c1_bk;
  c1_eml_scalar_eg(chartInstance);
  c1_ar = c1_d_a;
  c1_br = c1_d_b;
  return muDoubleScalarPower(c1_ar, c1_br);
}

static void c1_eml_error(SFc1_compassGaitSimInstanceStruct *chartInstance)
{
  int32_T c1_i40;
  static char_T c1_cv1[31] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'p', 'o', 'w', 'e', 'r', '_', 'd', 'o', 'm', 'a', 'i',
    'n', 'E', 'r', 'r', 'o', 'r' };

  char_T c1_u[31];
  const mxArray *c1_y = NULL;
  for (c1_i40 = 0; c1_i40 < 31; c1_i40++) {
    c1_u[c1_i40] = c1_cv1[c1_i40];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 10, 0U, 1U, 0U, 2, 1, 31), FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U, 14,
    c1_y));
}

static const mxArray *c1_f_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_compassGaitSimInstanceStruct *chartInstance;
  chartInstance = (SFc1_compassGaitSimInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(int32_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static int32_T c1_e_emlrt_marshallIn(SFc1_compassGaitSimInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  int32_T c1_y;
  int32_T c1_i41;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_i41, 1, 6, 0U, 0, 0U, 0);
  c1_y = c1_i41;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_sfEvent;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  int32_T c1_y;
  SFc1_compassGaitSimInstanceStruct *chartInstance;
  chartInstance = (SFc1_compassGaitSimInstanceStruct *)chartInstanceVoid;
  c1_b_sfEvent = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_sfEvent),
    &c1_thisId);
  sf_mex_destroy(&c1_b_sfEvent);
  *(int32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static uint8_T c1_f_emlrt_marshallIn(SFc1_compassGaitSimInstanceStruct
  *chartInstance, const mxArray *c1_b_is_active_c1_compassGaitSim, const char_T *
  c1_identifier)
{
  uint8_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_g_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c1_b_is_active_c1_compassGaitSim), &c1_thisId);
  sf_mex_destroy(&c1_b_is_active_c1_compassGaitSim);
  return c1_y;
}

static uint8_T c1_g_emlrt_marshallIn(SFc1_compassGaitSimInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  uint8_T c1_y;
  uint8_T c1_u0;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_u0, 1, 3, 0U, 0, 0U, 0);
  c1_y = c1_u0;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void init_dsm_address_info(SFc1_compassGaitSimInstanceStruct
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

void sf_c1_compassGaitSim_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3783000148U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(982395708U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3451195823U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1966011742U);
}

mxArray *sf_c1_compassGaitSim_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("4dIntng0ZH92Qk9Kc5W2XF");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,6,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(50);
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
      pr[0] = (double)(5);
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

mxArray *sf_c1_compassGaitSim_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c1_compassGaitSim(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x3'type','srcId','name','auxInfo'{{M[1],M[5],T\"T2\",},{M[1],M[4],T\"t2err\",},{M[8],M[0],T\"is_active_c1_compassGaitSim\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 3, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c1_compassGaitSim_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_compassGaitSimInstanceStruct *chartInstance;
    chartInstance = (SFc1_compassGaitSimInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _compassGaitSimMachineNumber_,
           1,
           1,
           1,
           8,
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
          init_script_number_translation(_compassGaitSimMachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_compassGaitSimMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _compassGaitSimMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"nom_torque_table");
          _SFD_SET_DATA_PROPS(1,1,1,0,"points");
          _SFD_SET_DATA_PROPS(2,1,1,0,"t1");
          _SFD_SET_DATA_PROPS(3,1,1,0,"t2");
          _SFD_SET_DATA_PROPS(4,1,1,0,"t_step");
          _SFD_SET_DATA_PROPS(5,2,0,1,"T2");
          _SFD_SET_DATA_PROPS(6,2,0,1,"t2err");
          _SFD_SET_DATA_PROPS(7,1,1,0,"t2err_p");
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
        _SFD_CV_INIT_EML(0,1,1,4,0,0,0,1,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1280);
        _SFD_CV_INIT_EML_IF(0,1,0,233,246,259,498);
        _SFD_CV_INIT_EML_IF(0,1,1,283,314,466,494);
        _SFD_CV_INIT_EML_IF(0,1,2,987,1000,1030,1048);
        _SFD_CV_INIT_EML_IF(0,1,3,1030,1048,-1,1048);
        _SFD_CV_INIT_EML_FOR(0,1,0,769,783,859);
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
          dimVector[0]= 50;
          dimVector[1]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 5;
          dimVector[1]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);

        {
          real_T *c1_t1;
          real_T *c1_t2;
          real_T *c1_t_step;
          real_T *c1_T2;
          real_T *c1_t2err;
          real_T *c1_t2err_p;
          real_T (*c1_nom_torque_table)[100];
          real_T (*c1_points)[10];
          c1_t2err_p = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
          c1_t2err = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
          c1_T2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          c1_t_step = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
          c1_t2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c1_t1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c1_points = (real_T (*)[10])ssGetInputPortSignal(chartInstance->S, 1);
          c1_nom_torque_table = (real_T (*)[100])ssGetInputPortSignal
            (chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c1_nom_torque_table);
          _SFD_SET_DATA_VALUE_PTR(1U, *c1_points);
          _SFD_SET_DATA_VALUE_PTR(2U, c1_t1);
          _SFD_SET_DATA_VALUE_PTR(3U, c1_t2);
          _SFD_SET_DATA_VALUE_PTR(4U, c1_t_step);
          _SFD_SET_DATA_VALUE_PTR(5U, c1_T2);
          _SFD_SET_DATA_VALUE_PTR(6U, c1_t2err);
          _SFD_SET_DATA_VALUE_PTR(7U, c1_t2err_p);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _compassGaitSimMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "bxOtQQQBh6CCPnjnn0WiPC";
}

static void sf_opaque_initialize_c1_compassGaitSim(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc1_compassGaitSimInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c1_compassGaitSim((SFc1_compassGaitSimInstanceStruct*)
    chartInstanceVar);
  initialize_c1_compassGaitSim((SFc1_compassGaitSimInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c1_compassGaitSim(void *chartInstanceVar)
{
  enable_c1_compassGaitSim((SFc1_compassGaitSimInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c1_compassGaitSim(void *chartInstanceVar)
{
  disable_c1_compassGaitSim((SFc1_compassGaitSimInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c1_compassGaitSim(void *chartInstanceVar)
{
  sf_c1_compassGaitSim((SFc1_compassGaitSimInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c1_compassGaitSim(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c1_compassGaitSim
    ((SFc1_compassGaitSimInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c1_compassGaitSim();/* state var info */
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

extern void sf_internal_set_sim_state_c1_compassGaitSim(SimStruct* S, const
  mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c1_compassGaitSim();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c1_compassGaitSim((SFc1_compassGaitSimInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c1_compassGaitSim(SimStruct* S)
{
  return sf_internal_get_sim_state_c1_compassGaitSim(S);
}

static void sf_opaque_set_sim_state_c1_compassGaitSim(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c1_compassGaitSim(S, st);
}

static void sf_opaque_terminate_c1_compassGaitSim(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc1_compassGaitSimInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_compassGaitSim_optimization_info();
    }

    finalize_c1_compassGaitSim((SFc1_compassGaitSimInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc1_compassGaitSim((SFc1_compassGaitSimInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c1_compassGaitSim(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c1_compassGaitSim((SFc1_compassGaitSimInstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c1_compassGaitSim(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_compassGaitSim_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      1);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,1,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,1,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,1);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,1,6);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,1,2);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=2; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 6; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,1);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2946507761U));
  ssSetChecksum1(S,(3341058137U));
  ssSetChecksum2(S,(1538379854U));
  ssSetChecksum3(S,(2696810271U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c1_compassGaitSim(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c1_compassGaitSim(SimStruct *S)
{
  SFc1_compassGaitSimInstanceStruct *chartInstance;
  chartInstance = (SFc1_compassGaitSimInstanceStruct *)utMalloc(sizeof
    (SFc1_compassGaitSimInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc1_compassGaitSimInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c1_compassGaitSim;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c1_compassGaitSim;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c1_compassGaitSim;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c1_compassGaitSim;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c1_compassGaitSim;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c1_compassGaitSim;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c1_compassGaitSim;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c1_compassGaitSim;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c1_compassGaitSim;
  chartInstance->chartInfo.mdlStart = mdlStart_c1_compassGaitSim;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c1_compassGaitSim;
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

void c1_compassGaitSim_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c1_compassGaitSim(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c1_compassGaitSim(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c1_compassGaitSim(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c1_compassGaitSim_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
