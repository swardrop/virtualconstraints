/* Include files */

#include <stddef.h>
#include "blas.h"
#include "compassGaitSim_impacts1_sfun.h"
#include "c2_compassGaitSim_impacts1.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "compassGaitSim_impacts1_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c2_debug_family_names[21] = { "c1", "c2", "s2", "c12",
  "nargin", "nargout", "T1", "T2", "t1d", "t2d", "t1", "t2", "I1", "l1", "m1",
  "I2", "l2", "m2", "g", "t1dd", "t2dd" };

/* Function Declarations */
static void initialize_c2_compassGaitSim_impacts1
  (SFc2_compassGaitSim_impacts1InstanceStruct *chartInstance);
static void initialize_params_c2_compassGaitSim_impacts1
  (SFc2_compassGaitSim_impacts1InstanceStruct *chartInstance);
static void enable_c2_compassGaitSim_impacts1
  (SFc2_compassGaitSim_impacts1InstanceStruct *chartInstance);
static void disable_c2_compassGaitSim_impacts1
  (SFc2_compassGaitSim_impacts1InstanceStruct *chartInstance);
static void c2_update_debugger_state_c2_compassGaitSim_impacts1
  (SFc2_compassGaitSim_impacts1InstanceStruct *chartInstance);
static const mxArray *get_sim_state_c2_compassGaitSim_impacts1
  (SFc2_compassGaitSim_impacts1InstanceStruct *chartInstance);
static void set_sim_state_c2_compassGaitSim_impacts1
  (SFc2_compassGaitSim_impacts1InstanceStruct *chartInstance, const mxArray
   *c2_st);
static void finalize_c2_compassGaitSim_impacts1
  (SFc2_compassGaitSim_impacts1InstanceStruct *chartInstance);
static void sf_c2_compassGaitSim_impacts1
  (SFc2_compassGaitSim_impacts1InstanceStruct *chartInstance);
static void c2_chartstep_c2_compassGaitSim_impacts1
  (SFc2_compassGaitSim_impacts1InstanceStruct *chartInstance);
static void initSimStructsc2_compassGaitSim_impacts1
  (SFc2_compassGaitSim_impacts1InstanceStruct *chartInstance);
static void registerMessagesc2_compassGaitSim_impacts1
  (SFc2_compassGaitSim_impacts1InstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static real_T c2_emlrt_marshallIn(SFc2_compassGaitSim_impacts1InstanceStruct
  *chartInstance, const mxArray *c2_t2dd, const char_T *c2_identifier);
static real_T c2_b_emlrt_marshallIn(SFc2_compassGaitSim_impacts1InstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[17]);
static real_T c2_mpower(SFc2_compassGaitSim_impacts1InstanceStruct
  *chartInstance, real_T c2_a);
static void c2_eml_scalar_eg(SFc2_compassGaitSim_impacts1InstanceStruct
  *chartInstance);
static real_T c2_b_mpower(SFc2_compassGaitSim_impacts1InstanceStruct
  *chartInstance, real_T c2_a);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_c_emlrt_marshallIn(SFc2_compassGaitSim_impacts1InstanceStruct *
  chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static uint8_T c2_d_emlrt_marshallIn(SFc2_compassGaitSim_impacts1InstanceStruct *
  chartInstance, const mxArray *c2_b_is_active_c2_compassGaitSim_impacts1, const
  char_T *c2_identifier);
static uint8_T c2_e_emlrt_marshallIn(SFc2_compassGaitSim_impacts1InstanceStruct *
  chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void init_dsm_address_info(SFc2_compassGaitSim_impacts1InstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c2_compassGaitSim_impacts1
  (SFc2_compassGaitSim_impacts1InstanceStruct *chartInstance)
{
  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c2_is_active_c2_compassGaitSim_impacts1 = 0U;
}

static void initialize_params_c2_compassGaitSim_impacts1
  (SFc2_compassGaitSim_impacts1InstanceStruct *chartInstance)
{
}

static void enable_c2_compassGaitSim_impacts1
  (SFc2_compassGaitSim_impacts1InstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c2_compassGaitSim_impacts1
  (SFc2_compassGaitSim_impacts1InstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c2_update_debugger_state_c2_compassGaitSim_impacts1
  (SFc2_compassGaitSim_impacts1InstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c2_compassGaitSim_impacts1
  (SFc2_compassGaitSim_impacts1InstanceStruct *chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  real_T c2_hoistedGlobal;
  real_T c2_u;
  const mxArray *c2_b_y = NULL;
  real_T c2_b_hoistedGlobal;
  real_T c2_b_u;
  const mxArray *c2_c_y = NULL;
  uint8_T c2_c_hoistedGlobal;
  uint8_T c2_c_u;
  const mxArray *c2_d_y = NULL;
  real_T *c2_t1dd;
  real_T *c2_t2dd;
  c2_t2dd = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_t1dd = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellarray(3), FALSE);
  c2_hoistedGlobal = *c2_t1dd;
  c2_u = c2_hoistedGlobal;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_b_hoistedGlobal = *c2_t2dd;
  c2_b_u = c2_b_hoistedGlobal;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  c2_c_hoistedGlobal = chartInstance->c2_is_active_c2_compassGaitSim_impacts1;
  c2_c_u = c2_c_hoistedGlobal;
  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_c_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 2, c2_d_y);
  sf_mex_assign(&c2_st, c2_y, FALSE);
  return c2_st;
}

static void set_sim_state_c2_compassGaitSim_impacts1
  (SFc2_compassGaitSim_impacts1InstanceStruct *chartInstance, const mxArray
   *c2_st)
{
  const mxArray *c2_u;
  real_T *c2_t1dd;
  real_T *c2_t2dd;
  c2_t2dd = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_t1dd = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c2_doneDoubleBufferReInit = TRUE;
  c2_u = sf_mex_dup(c2_st);
  *c2_t1dd = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u,
    0)), "t1dd");
  *c2_t2dd = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u,
    1)), "t2dd");
  chartInstance->c2_is_active_c2_compassGaitSim_impacts1 = c2_d_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 2)),
     "is_active_c2_compassGaitSim_impacts1");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_compassGaitSim_impacts1(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_compassGaitSim_impacts1
  (SFc2_compassGaitSim_impacts1InstanceStruct *chartInstance)
{
}

static void sf_c2_compassGaitSim_impacts1
  (SFc2_compassGaitSim_impacts1InstanceStruct *chartInstance)
{
  real_T *c2_T1;
  real_T *c2_t1dd;
  real_T *c2_T2;
  real_T *c2_t1d;
  real_T *c2_t2d;
  real_T *c2_t1;
  real_T *c2_t2;
  real_T *c2_t2dd;
  real_T *c2_I1;
  real_T *c2_l1;
  real_T *c2_m1;
  real_T *c2_I2;
  real_T *c2_l2;
  real_T *c2_m2;
  real_T *c2_g;
  c2_g = (real_T *)ssGetInputPortSignal(chartInstance->S, 12);
  c2_m2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 11);
  c2_l2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 10);
  c2_I2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 9);
  c2_m1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 8);
  c2_l1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 7);
  c2_I1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 6);
  c2_t2dd = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_t2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
  c2_t1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c2_t2d = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c2_t1d = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c2_T2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c2_t1dd = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c2_T1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c2_T1, 0U);
  _SFD_DATA_RANGE_CHECK(*c2_t1dd, 1U);
  _SFD_DATA_RANGE_CHECK(*c2_T2, 2U);
  _SFD_DATA_RANGE_CHECK(*c2_t1d, 3U);
  _SFD_DATA_RANGE_CHECK(*c2_t2d, 4U);
  _SFD_DATA_RANGE_CHECK(*c2_t1, 5U);
  _SFD_DATA_RANGE_CHECK(*c2_t2, 6U);
  _SFD_DATA_RANGE_CHECK(*c2_t2dd, 7U);
  _SFD_DATA_RANGE_CHECK(*c2_I1, 8U);
  _SFD_DATA_RANGE_CHECK(*c2_l1, 9U);
  _SFD_DATA_RANGE_CHECK(*c2_m1, 10U);
  _SFD_DATA_RANGE_CHECK(*c2_I2, 11U);
  _SFD_DATA_RANGE_CHECK(*c2_l2, 12U);
  _SFD_DATA_RANGE_CHECK(*c2_m2, 13U);
  _SFD_DATA_RANGE_CHECK(*c2_g, 14U);
  chartInstance->c2_sfEvent = CALL_EVENT;
  c2_chartstep_c2_compassGaitSim_impacts1(chartInstance);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_compassGaitSim_impacts1MachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c2_chartstep_c2_compassGaitSim_impacts1
  (SFc2_compassGaitSim_impacts1InstanceStruct *chartInstance)
{
  real_T c2_hoistedGlobal;
  real_T c2_b_hoistedGlobal;
  real_T c2_c_hoistedGlobal;
  real_T c2_d_hoistedGlobal;
  real_T c2_e_hoistedGlobal;
  real_T c2_f_hoistedGlobal;
  real_T c2_g_hoistedGlobal;
  real_T c2_h_hoistedGlobal;
  real_T c2_i_hoistedGlobal;
  real_T c2_j_hoistedGlobal;
  real_T c2_k_hoistedGlobal;
  real_T c2_l_hoistedGlobal;
  real_T c2_m_hoistedGlobal;
  real_T c2_T1;
  real_T c2_T2;
  real_T c2_t1d;
  real_T c2_t2d;
  real_T c2_t1;
  real_T c2_t2;
  real_T c2_I1;
  real_T c2_l1;
  real_T c2_m1;
  real_T c2_I2;
  real_T c2_l2;
  real_T c2_m2;
  real_T c2_g;
  uint32_T c2_debug_family_var_map[21];
  real_T c2_c1;
  real_T c2_c2;
  real_T c2_s2;
  real_T c2_c12;
  real_T c2_nargin = 13.0;
  real_T c2_nargout = 2.0;
  real_T c2_t1dd;
  real_T c2_t2dd;
  real_T c2_x;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_d_x;
  real_T c2_e_x;
  real_T c2_f_x;
  real_T c2_g_x;
  real_T c2_h_x;
  real_T c2_b;
  real_T c2_y;
  real_T c2_a;
  real_T c2_b_b;
  real_T c2_b_y;
  real_T c2_c_b;
  real_T c2_c_y;
  real_T c2_b_a;
  real_T c2_d_b;
  real_T c2_d_y;
  real_T c2_c_a;
  real_T c2_e_b;
  real_T c2_e_y;
  real_T c2_d_a;
  real_T c2_f_b;
  real_T c2_f_y;
  real_T c2_e_a;
  real_T c2_g_b;
  real_T c2_g_y;
  real_T c2_f_a;
  real_T c2_h_b;
  real_T c2_h_y;
  real_T c2_g_a;
  real_T c2_i_b;
  real_T c2_i_y;
  real_T c2_h_a;
  real_T c2_j_b;
  real_T c2_j_y;
  real_T c2_i_a;
  real_T c2_k_b;
  real_T c2_k_y;
  real_T c2_j_a;
  real_T c2_l_b;
  real_T c2_l_y;
  real_T c2_m_b;
  real_T c2_m_y;
  real_T c2_k_a;
  real_T c2_n_b;
  real_T c2_n_y;
  real_T c2_l_a;
  real_T c2_o_b;
  real_T c2_o_y;
  real_T c2_m_a;
  real_T c2_p_b;
  real_T c2_p_y;
  real_T c2_n_a;
  real_T c2_q_b;
  real_T c2_q_y;
  real_T c2_r_b;
  real_T c2_r_y;
  real_T c2_o_a;
  real_T c2_s_b;
  real_T c2_s_y;
  real_T c2_p_a;
  real_T c2_t_b;
  real_T c2_t_y;
  real_T c2_q_a;
  real_T c2_u_b;
  real_T c2_u_y;
  real_T c2_r_a;
  real_T c2_v_b;
  real_T c2_v_y;
  real_T c2_w_b;
  real_T c2_w_y;
  real_T c2_s_a;
  real_T c2_x_b;
  real_T c2_x_y;
  real_T c2_t_a;
  real_T c2_y_b;
  real_T c2_y_y;
  real_T c2_u_a;
  real_T c2_ab_b;
  real_T c2_ab_y;
  real_T c2_v_a;
  real_T c2_bb_b;
  real_T c2_bb_y;
  real_T c2_w_a;
  real_T c2_cb_b;
  real_T c2_cb_y;
  real_T c2_x_a;
  real_T c2_db_b;
  real_T c2_db_y;
  real_T c2_y_a;
  real_T c2_eb_b;
  real_T c2_eb_y;
  real_T c2_ab_a;
  real_T c2_fb_b;
  real_T c2_fb_y;
  real_T c2_A;
  real_T c2_i_x;
  real_T c2_j_x;
  real_T c2_gb_y;
  real_T c2_bb_a;
  real_T c2_gb_b;
  real_T c2_hb_y;
  real_T c2_cb_a;
  real_T c2_hb_b;
  real_T c2_ib_y;
  real_T c2_db_a;
  real_T c2_ib_b;
  real_T c2_jb_y;
  real_T c2_eb_a;
  real_T c2_jb_b;
  real_T c2_kb_y;
  real_T c2_b_A;
  real_T c2_k_x;
  real_T c2_l_x;
  real_T c2_lb_y;
  real_T c2_kb_b;
  real_T c2_mb_y;
  real_T c2_fb_a;
  real_T c2_lb_b;
  real_T c2_nb_y;
  real_T c2_gb_a;
  real_T c2_mb_b;
  real_T c2_ob_y;
  real_T c2_hb_a;
  real_T c2_nb_b;
  real_T c2_pb_y;
  real_T c2_ib_a;
  real_T c2_ob_b;
  real_T c2_qb_y;
  real_T c2_jb_a;
  real_T c2_pb_b;
  real_T c2_rb_y;
  real_T c2_qb_b;
  real_T c2_sb_y;
  real_T c2_kb_a;
  real_T c2_rb_b;
  real_T c2_tb_y;
  real_T c2_lb_a;
  real_T c2_sb_b;
  real_T c2_ub_y;
  real_T c2_mb_a;
  real_T c2_tb_b;
  real_T c2_vb_y;
  real_T c2_nb_a;
  real_T c2_ub_b;
  real_T c2_wb_y;
  real_T c2_ob_a;
  real_T c2_vb_b;
  real_T c2_xb_y;
  real_T c2_pb_a;
  real_T c2_wb_b;
  real_T c2_yb_y;
  real_T c2_qb_a;
  real_T c2_xb_b;
  real_T c2_ac_y;
  real_T c2_rb_a;
  real_T c2_yb_b;
  real_T c2_bc_y;
  real_T c2_sb_a;
  real_T c2_ac_b;
  real_T c2_cc_y;
  real_T c2_tb_a;
  real_T c2_bc_b;
  real_T c2_dc_y;
  real_T c2_c_A;
  real_T c2_m_x;
  real_T c2_n_x;
  real_T c2_ec_y;
  real_T c2_ub_a;
  real_T c2_cc_b;
  real_T c2_fc_y;
  real_T c2_vb_a;
  real_T c2_dc_b;
  real_T c2_gc_y;
  real_T c2_wb_a;
  real_T c2_ec_b;
  real_T c2_hc_y;
  real_T c2_xb_a;
  real_T c2_fc_b;
  real_T c2_ic_y;
  real_T c2_yb_a;
  real_T c2_gc_b;
  real_T c2_jc_y;
  real_T c2_ac_a;
  real_T c2_hc_b;
  real_T c2_kc_y;
  real_T c2_bc_a;
  real_T c2_ic_b;
  real_T c2_lc_y;
  real_T c2_cc_a;
  real_T c2_jc_b;
  real_T c2_mc_y;
  real_T c2_dc_a;
  real_T c2_kc_b;
  real_T c2_nc_y;
  real_T c2_ec_a;
  real_T c2_lc_b;
  real_T c2_oc_y;
  real_T c2_fc_a;
  real_T c2_mc_b;
  real_T c2_pc_y;
  real_T c2_gc_a;
  real_T c2_nc_b;
  real_T c2_qc_y;
  real_T c2_hc_a;
  real_T c2_oc_b;
  real_T c2_rc_y;
  real_T c2_ic_a;
  real_T c2_pc_b;
  real_T c2_sc_y;
  real_T c2_jc_a;
  real_T c2_qc_b;
  real_T c2_tc_y;
  real_T c2_rc_b;
  real_T c2_uc_y;
  real_T c2_kc_a;
  real_T c2_sc_b;
  real_T c2_vc_y;
  real_T c2_lc_a;
  real_T c2_tc_b;
  real_T c2_wc_y;
  real_T c2_mc_a;
  real_T c2_uc_b;
  real_T c2_xc_y;
  real_T c2_nc_a;
  real_T c2_vc_b;
  real_T c2_yc_y;
  real_T c2_oc_a;
  real_T c2_wc_b;
  real_T c2_ad_y;
  real_T c2_pc_a;
  real_T c2_xc_b;
  real_T c2_bd_y;
  real_T c2_qc_a;
  real_T c2_yc_b;
  real_T c2_cd_y;
  real_T c2_rc_a;
  real_T c2_ad_b;
  real_T c2_dd_y;
  real_T c2_sc_a;
  real_T c2_bd_b;
  real_T c2_ed_y;
  real_T c2_tc_a;
  real_T c2_cd_b;
  real_T c2_fd_y;
  real_T c2_uc_a;
  real_T c2_dd_b;
  real_T c2_gd_y;
  real_T c2_ed_b;
  real_T c2_hd_y;
  real_T c2_vc_a;
  real_T c2_fd_b;
  real_T c2_id_y;
  real_T c2_wc_a;
  real_T c2_gd_b;
  real_T c2_jd_y;
  real_T c2_xc_a;
  real_T c2_hd_b;
  real_T c2_kd_y;
  real_T c2_yc_a;
  real_T c2_id_b;
  real_T c2_ld_y;
  real_T c2_jd_b;
  real_T c2_md_y;
  real_T c2_ad_a;
  real_T c2_kd_b;
  real_T c2_nd_y;
  real_T c2_d_A;
  real_T c2_B;
  real_T c2_o_x;
  real_T c2_od_y;
  real_T c2_p_x;
  real_T c2_pd_y;
  real_T c2_ld_b;
  real_T c2_qd_y;
  real_T c2_bd_a;
  real_T c2_md_b;
  real_T c2_rd_y;
  real_T c2_nd_b;
  real_T c2_sd_y;
  real_T c2_cd_a;
  real_T c2_od_b;
  real_T c2_td_y;
  real_T c2_pd_b;
  real_T c2_ud_y;
  real_T c2_dd_a;
  real_T c2_qd_b;
  real_T c2_vd_y;
  real_T c2_ed_a;
  real_T c2_rd_b;
  real_T c2_wd_y;
  real_T c2_fd_a;
  real_T c2_sd_b;
  real_T c2_xd_y;
  real_T c2_td_b;
  real_T c2_yd_y;
  real_T c2_gd_a;
  real_T c2_ud_b;
  real_T c2_ae_y;
  real_T c2_hd_a;
  real_T c2_vd_b;
  real_T c2_be_y;
  real_T c2_id_a;
  real_T c2_wd_b;
  real_T c2_ce_y;
  real_T c2_jd_a;
  real_T c2_xd_b;
  real_T c2_de_y;
  real_T c2_kd_a;
  real_T c2_yd_b;
  real_T c2_ee_y;
  real_T c2_ld_a;
  real_T c2_ae_b;
  real_T c2_fe_y;
  real_T c2_md_a;
  real_T c2_be_b;
  real_T c2_ge_y;
  real_T c2_nd_a;
  real_T c2_ce_b;
  real_T c2_he_y;
  real_T c2_de_b;
  real_T c2_ie_y;
  real_T c2_od_a;
  real_T c2_ee_b;
  real_T c2_je_y;
  real_T c2_pd_a;
  real_T c2_fe_b;
  real_T c2_ke_y;
  real_T c2_qd_a;
  real_T c2_ge_b;
  real_T c2_le_y;
  real_T c2_rd_a;
  real_T c2_he_b;
  real_T c2_me_y;
  real_T c2_ie_b;
  real_T c2_ne_y;
  real_T c2_sd_a;
  real_T c2_je_b;
  real_T c2_oe_y;
  real_T c2_td_a;
  real_T c2_ke_b;
  real_T c2_pe_y;
  real_T c2_ud_a;
  real_T c2_le_b;
  real_T c2_qe_y;
  real_T c2_vd_a;
  real_T c2_me_b;
  real_T c2_re_y;
  real_T c2_ne_b;
  real_T c2_se_y;
  real_T c2_wd_a;
  real_T c2_oe_b;
  real_T c2_te_y;
  real_T c2_xd_a;
  real_T c2_pe_b;
  real_T c2_ue_y;
  real_T c2_yd_a;
  real_T c2_qe_b;
  real_T c2_ve_y;
  real_T c2_ae_a;
  real_T c2_re_b;
  real_T c2_we_y;
  real_T c2_se_b;
  real_T c2_xe_y;
  real_T c2_be_a;
  real_T c2_te_b;
  real_T c2_ye_y;
  real_T c2_ce_a;
  real_T c2_ue_b;
  real_T c2_af_y;
  real_T c2_de_a;
  real_T c2_ve_b;
  real_T c2_bf_y;
  real_T c2_ee_a;
  real_T c2_we_b;
  real_T c2_cf_y;
  real_T c2_xe_b;
  real_T c2_df_y;
  real_T c2_fe_a;
  real_T c2_ye_b;
  real_T c2_ef_y;
  real_T c2_ge_a;
  real_T c2_af_b;
  real_T c2_ff_y;
  real_T c2_he_a;
  real_T c2_bf_b;
  real_T c2_gf_y;
  real_T c2_ie_a;
  real_T c2_cf_b;
  real_T c2_hf_y;
  real_T c2_df_b;
  real_T c2_if_y;
  real_T c2_je_a;
  real_T c2_ef_b;
  real_T c2_jf_y;
  real_T c2_ke_a;
  real_T c2_ff_b;
  real_T c2_kf_y;
  real_T c2_le_a;
  real_T c2_gf_b;
  real_T c2_lf_y;
  real_T c2_me_a;
  real_T c2_hf_b;
  real_T c2_mf_y;
  real_T c2_ne_a;
  real_T c2_if_b;
  real_T c2_nf_y;
  real_T c2_oe_a;
  real_T c2_jf_b;
  real_T c2_of_y;
  real_T c2_pe_a;
  real_T c2_kf_b;
  real_T c2_pf_y;
  real_T c2_qe_a;
  real_T c2_lf_b;
  real_T c2_qf_y;
  real_T c2_e_A;
  real_T c2_q_x;
  real_T c2_r_x;
  real_T c2_rf_y;
  real_T c2_mf_b;
  real_T c2_sf_y;
  real_T c2_re_a;
  real_T c2_nf_b;
  real_T c2_tf_y;
  real_T c2_se_a;
  real_T c2_of_b;
  real_T c2_uf_y;
  real_T c2_te_a;
  real_T c2_pf_b;
  real_T c2_vf_y;
  real_T c2_ue_a;
  real_T c2_qf_b;
  real_T c2_wf_y;
  real_T c2_ve_a;
  real_T c2_rf_b;
  real_T c2_xf_y;
  real_T c2_we_a;
  real_T c2_sf_b;
  real_T c2_yf_y;
  real_T c2_xe_a;
  real_T c2_tf_b;
  real_T c2_ag_y;
  real_T c2_ye_a;
  real_T c2_uf_b;
  real_T c2_bg_y;
  real_T c2_f_A;
  real_T c2_s_x;
  real_T c2_t_x;
  real_T c2_cg_y;
  real_T c2_vf_b;
  real_T c2_dg_y;
  real_T c2_af_a;
  real_T c2_wf_b;
  real_T c2_eg_y;
  real_T c2_bf_a;
  real_T c2_xf_b;
  real_T c2_fg_y;
  real_T c2_cf_a;
  real_T c2_yf_b;
  real_T c2_gg_y;
  real_T c2_df_a;
  real_T c2_ag_b;
  real_T c2_hg_y;
  real_T c2_ef_a;
  real_T c2_bg_b;
  real_T c2_ig_y;
  real_T c2_cg_b;
  real_T c2_jg_y;
  real_T c2_ff_a;
  real_T c2_dg_b;
  real_T c2_kg_y;
  real_T c2_gf_a;
  real_T c2_eg_b;
  real_T c2_lg_y;
  real_T c2_hf_a;
  real_T c2_fg_b;
  real_T c2_mg_y;
  real_T c2_if_a;
  real_T c2_gg_b;
  real_T c2_ng_y;
  real_T c2_jf_a;
  real_T c2_hg_b;
  real_T c2_og_y;
  real_T c2_ig_b;
  real_T c2_pg_y;
  real_T c2_kf_a;
  real_T c2_jg_b;
  real_T c2_qg_y;
  real_T c2_lf_a;
  real_T c2_kg_b;
  real_T c2_rg_y;
  real_T c2_mf_a;
  real_T c2_lg_b;
  real_T c2_sg_y;
  real_T c2_nf_a;
  real_T c2_mg_b;
  real_T c2_tg_y;
  real_T c2_of_a;
  real_T c2_ng_b;
  real_T c2_ug_y;
  real_T c2_pf_a;
  real_T c2_og_b;
  real_T c2_vg_y;
  real_T c2_qf_a;
  real_T c2_pg_b;
  real_T c2_wg_y;
  real_T c2_rf_a;
  real_T c2_qg_b;
  real_T c2_xg_y;
  real_T c2_sf_a;
  real_T c2_rg_b;
  real_T c2_yg_y;
  real_T c2_tf_a;
  real_T c2_sg_b;
  real_T c2_ah_y;
  real_T c2_g_A;
  real_T c2_u_x;
  real_T c2_v_x;
  real_T c2_bh_y;
  real_T c2_tg_b;
  real_T c2_ch_y;
  real_T c2_uf_a;
  real_T c2_ug_b;
  real_T c2_dh_y;
  real_T c2_vf_a;
  real_T c2_vg_b;
  real_T c2_eh_y;
  real_T c2_wf_a;
  real_T c2_wg_b;
  real_T c2_fh_y;
  real_T c2_xf_a;
  real_T c2_xg_b;
  real_T c2_gh_y;
  real_T c2_yf_a;
  real_T c2_yg_b;
  real_T c2_hh_y;
  real_T c2_ag_a;
  real_T c2_ah_b;
  real_T c2_ih_y;
  real_T c2_bg_a;
  real_T c2_bh_b;
  real_T c2_jh_y;
  real_T c2_cg_a;
  real_T c2_ch_b;
  real_T c2_kh_y;
  real_T c2_dg_a;
  real_T c2_dh_b;
  real_T c2_lh_y;
  real_T c2_eg_a;
  real_T c2_eh_b;
  real_T c2_mh_y;
  real_T c2_fh_b;
  real_T c2_nh_y;
  real_T c2_fg_a;
  real_T c2_gh_b;
  real_T c2_oh_y;
  real_T c2_gg_a;
  real_T c2_hh_b;
  real_T c2_ph_y;
  real_T c2_hg_a;
  real_T c2_ih_b;
  real_T c2_qh_y;
  real_T c2_ig_a;
  real_T c2_jh_b;
  real_T c2_rh_y;
  real_T c2_jg_a;
  real_T c2_kh_b;
  real_T c2_sh_y;
  real_T c2_kg_a;
  real_T c2_lh_b;
  real_T c2_th_y;
  real_T c2_lg_a;
  real_T c2_mh_b;
  real_T c2_uh_y;
  real_T c2_mg_a;
  real_T c2_nh_b;
  real_T c2_vh_y;
  real_T c2_ng_a;
  real_T c2_oh_b;
  real_T c2_wh_y;
  real_T c2_og_a;
  real_T c2_ph_b;
  real_T c2_xh_y;
  real_T c2_pg_a;
  real_T c2_qh_b;
  real_T c2_yh_y;
  real_T c2_qg_a;
  real_T c2_rh_b;
  real_T c2_ai_y;
  real_T c2_rg_a;
  real_T c2_sh_b;
  real_T c2_bi_y;
  real_T c2_sg_a;
  real_T c2_th_b;
  real_T c2_ci_y;
  real_T c2_tg_a;
  real_T c2_uh_b;
  real_T c2_di_y;
  real_T c2_vh_b;
  real_T c2_ei_y;
  real_T c2_ug_a;
  real_T c2_wh_b;
  real_T c2_fi_y;
  real_T c2_vg_a;
  real_T c2_xh_b;
  real_T c2_gi_y;
  real_T c2_wg_a;
  real_T c2_yh_b;
  real_T c2_hi_y;
  real_T c2_xg_a;
  real_T c2_ai_b;
  real_T c2_ii_y;
  real_T c2_yg_a;
  real_T c2_bi_b;
  real_T c2_ji_y;
  real_T c2_ah_a;
  real_T c2_ci_b;
  real_T c2_ki_y;
  real_T c2_di_b;
  real_T c2_li_y;
  real_T c2_bh_a;
  real_T c2_ei_b;
  real_T c2_mi_y;
  real_T c2_ch_a;
  real_T c2_fi_b;
  real_T c2_ni_y;
  real_T c2_dh_a;
  real_T c2_gi_b;
  real_T c2_oi_y;
  real_T c2_eh_a;
  real_T c2_hi_b;
  real_T c2_pi_y;
  real_T c2_fh_a;
  real_T c2_ii_b;
  real_T c2_qi_y;
  real_T c2_gh_a;
  real_T c2_ji_b;
  real_T c2_ri_y;
  real_T c2_hh_a;
  real_T c2_ki_b;
  real_T c2_si_y;
  real_T c2_ih_a;
  real_T c2_li_b;
  real_T c2_ti_y;
  real_T c2_jh_a;
  real_T c2_mi_b;
  real_T c2_ui_y;
  real_T c2_kh_a;
  real_T c2_ni_b;
  real_T c2_vi_y;
  real_T c2_lh_a;
  real_T c2_oi_b;
  real_T c2_wi_y;
  real_T c2_mh_a;
  real_T c2_pi_b;
  real_T c2_xi_y;
  real_T c2_nh_a;
  real_T c2_qi_b;
  real_T c2_yi_y;
  real_T c2_oh_a;
  real_T c2_ri_b;
  real_T c2_aj_y;
  real_T c2_ph_a;
  real_T c2_si_b;
  real_T c2_bj_y;
  real_T c2_qh_a;
  real_T c2_ti_b;
  real_T c2_cj_y;
  real_T c2_rh_a;
  real_T c2_ui_b;
  real_T c2_dj_y;
  real_T c2_vi_b;
  real_T c2_ej_y;
  real_T c2_sh_a;
  real_T c2_wi_b;
  real_T c2_fj_y;
  real_T c2_th_a;
  real_T c2_xi_b;
  real_T c2_gj_y;
  real_T c2_uh_a;
  real_T c2_yi_b;
  real_T c2_hj_y;
  real_T c2_vh_a;
  real_T c2_aj_b;
  real_T c2_ij_y;
  real_T c2_bj_b;
  real_T c2_jj_y;
  real_T c2_wh_a;
  real_T c2_cj_b;
  real_T c2_kj_y;
  real_T c2_h_A;
  real_T c2_b_B;
  real_T c2_w_x;
  real_T c2_lj_y;
  real_T c2_x_x;
  real_T c2_mj_y;
  real_T *c2_b_t2dd;
  real_T *c2_b_t1dd;
  real_T *c2_b_g;
  real_T *c2_b_m2;
  real_T *c2_b_l2;
  real_T *c2_b_I2;
  real_T *c2_b_m1;
  real_T *c2_b_l1;
  real_T *c2_b_I1;
  real_T *c2_b_t2;
  real_T *c2_b_t1;
  real_T *c2_b_t2d;
  real_T *c2_b_t1d;
  real_T *c2_b_T2;
  real_T *c2_b_T1;
  c2_b_g = (real_T *)ssGetInputPortSignal(chartInstance->S, 12);
  c2_b_m2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 11);
  c2_b_l2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 10);
  c2_b_I2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 9);
  c2_b_m1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 8);
  c2_b_l1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 7);
  c2_b_I1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 6);
  c2_b_t2dd = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_b_t2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
  c2_b_t1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c2_b_t2d = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c2_b_t1d = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c2_b_T2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c2_b_t1dd = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c2_b_T1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  c2_hoistedGlobal = *c2_b_T1;
  c2_b_hoistedGlobal = *c2_b_T2;
  c2_c_hoistedGlobal = *c2_b_t1d;
  c2_d_hoistedGlobal = *c2_b_t2d;
  c2_e_hoistedGlobal = *c2_b_t1;
  c2_f_hoistedGlobal = *c2_b_t2;
  c2_g_hoistedGlobal = *c2_b_I1;
  c2_h_hoistedGlobal = *c2_b_l1;
  c2_i_hoistedGlobal = *c2_b_m1;
  c2_j_hoistedGlobal = *c2_b_I2;
  c2_k_hoistedGlobal = *c2_b_l2;
  c2_l_hoistedGlobal = *c2_b_m2;
  c2_m_hoistedGlobal = *c2_b_g;
  c2_T1 = c2_hoistedGlobal;
  c2_T2 = c2_b_hoistedGlobal;
  c2_t1d = c2_c_hoistedGlobal;
  c2_t2d = c2_d_hoistedGlobal;
  c2_t1 = c2_e_hoistedGlobal;
  c2_t2 = c2_f_hoistedGlobal;
  c2_I1 = c2_g_hoistedGlobal;
  c2_l1 = c2_h_hoistedGlobal;
  c2_m1 = c2_i_hoistedGlobal;
  c2_I2 = c2_j_hoistedGlobal;
  c2_l2 = c2_k_hoistedGlobal;
  c2_m2 = c2_l_hoistedGlobal;
  c2_g = c2_m_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 21U, 21U, c2_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_c1, 0U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_c2, 1U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_s2, 2U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_c12, 3U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 4U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 5U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_T1, 6U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_T2, 7U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_t1d, 8U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_t2d, 9U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_t1, 10U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_t2, 11U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_I1, 12U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_l1, 13U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_m1, 14U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_I2, 15U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_l2, 16U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_m2, 17U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_g, 18U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_t1dd, 19U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_t2dd, 20U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 5);
  c2_x = c2_t1;
  c2_c1 = c2_x;
  c2_b_x = c2_c1;
  c2_c1 = c2_b_x;
  c2_c1 = muDoubleScalarCos(c2_c1);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 6);
  c2_c_x = c2_t2;
  c2_c2 = c2_c_x;
  c2_d_x = c2_c2;
  c2_c2 = c2_d_x;
  c2_c2 = muDoubleScalarCos(c2_c2);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 7);
  c2_e_x = c2_t2;
  c2_s2 = c2_e_x;
  c2_f_x = c2_s2;
  c2_s2 = c2_f_x;
  c2_s2 = muDoubleScalarSin(c2_s2);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 8);
  c2_g_x = c2_t1 + c2_t2;
  c2_c12 = c2_g_x;
  c2_h_x = c2_c12;
  c2_c12 = c2_h_x;
  c2_c12 = muDoubleScalarCos(c2_c12);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 13);
  c2_b = c2_I2;
  c2_y = 4.0 * c2_b;
  c2_a = c2_y;
  c2_b_b = c2_T1;
  c2_b_y = c2_a * c2_b_b;
  c2_c_b = c2_I2;
  c2_c_y = 4.0 * c2_c_b;
  c2_b_a = c2_c_y;
  c2_d_b = c2_T2;
  c2_d_y = c2_b_a * c2_d_b;
  c2_c_a = c2_T1;
  c2_e_b = c2_mpower(chartInstance, c2_l2);
  c2_e_y = c2_c_a * c2_e_b;
  c2_d_a = c2_e_y;
  c2_f_b = c2_m2;
  c2_f_y = c2_d_a * c2_f_b;
  c2_e_a = c2_T2;
  c2_g_b = c2_mpower(chartInstance, c2_l2);
  c2_g_y = c2_e_a * c2_g_b;
  c2_f_a = c2_g_y;
  c2_h_b = c2_m2;
  c2_h_y = c2_f_a * c2_h_b;
  c2_g_a = c2_c1;
  c2_i_b = c2_g;
  c2_i_y = c2_g_a * c2_i_b;
  c2_h_a = c2_i_y;
  c2_j_b = c2_l1;
  c2_j_y = c2_h_a * c2_j_b;
  c2_i_a = c2_j_y;
  c2_k_b = c2_mpower(chartInstance, c2_l2);
  c2_k_y = c2_i_a * c2_k_b;
  c2_j_a = c2_k_y;
  c2_l_b = c2_mpower(chartInstance, c2_m2);
  c2_l_y = c2_j_a * c2_l_b;
  c2_m_b = c2_I2;
  c2_m_y = 2.0 * c2_m_b;
  c2_k_a = c2_m_y;
  c2_n_b = c2_c1;
  c2_n_y = c2_k_a * c2_n_b;
  c2_l_a = c2_n_y;
  c2_o_b = c2_g;
  c2_o_y = c2_l_a * c2_o_b;
  c2_m_a = c2_o_y;
  c2_p_b = c2_l1;
  c2_p_y = c2_m_a * c2_p_b;
  c2_n_a = c2_p_y;
  c2_q_b = c2_m1;
  c2_q_y = c2_n_a * c2_q_b;
  c2_r_b = c2_I2;
  c2_r_y = 4.0 * c2_r_b;
  c2_o_a = c2_r_y;
  c2_s_b = c2_c1;
  c2_s_y = c2_o_a * c2_s_b;
  c2_p_a = c2_s_y;
  c2_t_b = c2_g;
  c2_t_y = c2_p_a * c2_t_b;
  c2_q_a = c2_t_y;
  c2_u_b = c2_l1;
  c2_u_y = c2_q_a * c2_u_b;
  c2_r_a = c2_u_y;
  c2_v_b = c2_m2;
  c2_v_y = c2_r_a * c2_v_b;
  c2_w_b = c2_T2;
  c2_w_y = 2.0 * c2_w_b;
  c2_s_a = c2_w_y;
  c2_x_b = c2_c2;
  c2_x_y = c2_s_a * c2_x_b;
  c2_t_a = c2_x_y;
  c2_y_b = c2_l1;
  c2_y_y = c2_t_a * c2_y_b;
  c2_u_a = c2_y_y;
  c2_ab_b = c2_l2;
  c2_ab_y = c2_u_a * c2_ab_b;
  c2_v_a = c2_ab_y;
  c2_bb_b = c2_m2;
  c2_bb_y = c2_v_a * c2_bb_b;
  c2_w_a = c2_l1;
  c2_cb_b = c2_b_mpower(chartInstance, c2_l2);
  c2_cb_y = c2_w_a * c2_cb_b;
  c2_x_a = c2_cb_y;
  c2_db_b = c2_mpower(chartInstance, c2_m2);
  c2_db_y = c2_x_a * c2_db_b;
  c2_y_a = c2_db_y;
  c2_eb_b = c2_s2;
  c2_eb_y = c2_y_a * c2_eb_b;
  c2_ab_a = c2_eb_y;
  c2_fb_b = c2_mpower(chartInstance, c2_t1d);
  c2_fb_y = c2_ab_a * c2_fb_b;
  c2_A = c2_fb_y;
  c2_i_x = c2_A;
  c2_j_x = c2_i_x;
  c2_gb_y = c2_j_x / 2.0;
  c2_bb_a = c2_l1;
  c2_gb_b = c2_b_mpower(chartInstance, c2_l2);
  c2_hb_y = c2_bb_a * c2_gb_b;
  c2_cb_a = c2_hb_y;
  c2_hb_b = c2_mpower(chartInstance, c2_m2);
  c2_ib_y = c2_cb_a * c2_hb_b;
  c2_db_a = c2_ib_y;
  c2_ib_b = c2_s2;
  c2_jb_y = c2_db_a * c2_ib_b;
  c2_eb_a = c2_jb_y;
  c2_jb_b = c2_mpower(chartInstance, c2_t2d);
  c2_kb_y = c2_eb_a * c2_jb_b;
  c2_b_A = c2_kb_y;
  c2_k_x = c2_b_A;
  c2_l_x = c2_k_x;
  c2_lb_y = c2_l_x / 2.0;
  c2_kb_b = c2_I2;
  c2_mb_y = 2.0 * c2_kb_b;
  c2_fb_a = c2_mb_y;
  c2_lb_b = c2_l1;
  c2_nb_y = c2_fb_a * c2_lb_b;
  c2_gb_a = c2_nb_y;
  c2_mb_b = c2_l2;
  c2_ob_y = c2_gb_a * c2_mb_b;
  c2_hb_a = c2_ob_y;
  c2_nb_b = c2_m2;
  c2_pb_y = c2_hb_a * c2_nb_b;
  c2_ib_a = c2_pb_y;
  c2_ob_b = c2_s2;
  c2_qb_y = c2_ib_a * c2_ob_b;
  c2_jb_a = c2_qb_y;
  c2_pb_b = c2_mpower(chartInstance, c2_t1d);
  c2_rb_y = c2_jb_a * c2_pb_b;
  c2_qb_b = c2_I2;
  c2_sb_y = 2.0 * c2_qb_b;
  c2_kb_a = c2_sb_y;
  c2_rb_b = c2_l1;
  c2_tb_y = c2_kb_a * c2_rb_b;
  c2_lb_a = c2_tb_y;
  c2_sb_b = c2_l2;
  c2_ub_y = c2_lb_a * c2_sb_b;
  c2_mb_a = c2_ub_y;
  c2_tb_b = c2_m2;
  c2_vb_y = c2_mb_a * c2_tb_b;
  c2_nb_a = c2_vb_y;
  c2_ub_b = c2_s2;
  c2_wb_y = c2_nb_a * c2_ub_b;
  c2_ob_a = c2_wb_y;
  c2_vb_b = c2_mpower(chartInstance, c2_t2d);
  c2_xb_y = c2_ob_a * c2_vb_b;
  c2_pb_a = c2_c1;
  c2_wb_b = c2_g;
  c2_yb_y = c2_pb_a * c2_wb_b;
  c2_qb_a = c2_yb_y;
  c2_xb_b = c2_l1;
  c2_ac_y = c2_qb_a * c2_xb_b;
  c2_rb_a = c2_ac_y;
  c2_yb_b = c2_mpower(chartInstance, c2_l2);
  c2_bc_y = c2_rb_a * c2_yb_b;
  c2_sb_a = c2_bc_y;
  c2_ac_b = c2_m1;
  c2_cc_y = c2_sb_a * c2_ac_b;
  c2_tb_a = c2_cc_y;
  c2_bc_b = c2_m2;
  c2_dc_y = c2_tb_a * c2_bc_b;
  c2_c_A = c2_dc_y;
  c2_m_x = c2_c_A;
  c2_n_x = c2_m_x;
  c2_ec_y = c2_n_x / 2.0;
  c2_ub_a = c2_c2;
  c2_cc_b = c2_mpower(chartInstance, c2_l1);
  c2_fc_y = c2_ub_a * c2_cc_b;
  c2_vb_a = c2_fc_y;
  c2_dc_b = c2_mpower(chartInstance, c2_l2);
  c2_gc_y = c2_vb_a * c2_dc_b;
  c2_wb_a = c2_gc_y;
  c2_ec_b = c2_mpower(chartInstance, c2_m2);
  c2_hc_y = c2_wb_a * c2_ec_b;
  c2_xb_a = c2_hc_y;
  c2_fc_b = c2_s2;
  c2_ic_y = c2_xb_a * c2_fc_b;
  c2_yb_a = c2_ic_y;
  c2_gc_b = c2_mpower(chartInstance, c2_t1d);
  c2_jc_y = c2_yb_a * c2_gc_b;
  c2_ac_a = c2_c2;
  c2_hc_b = c2_c12;
  c2_kc_y = c2_ac_a * c2_hc_b;
  c2_bc_a = c2_kc_y;
  c2_ic_b = c2_g;
  c2_lc_y = c2_bc_a * c2_ic_b;
  c2_cc_a = c2_lc_y;
  c2_jc_b = c2_l1;
  c2_mc_y = c2_cc_a * c2_jc_b;
  c2_dc_a = c2_mc_y;
  c2_kc_b = c2_mpower(chartInstance, c2_l2);
  c2_nc_y = c2_dc_a * c2_kc_b;
  c2_ec_a = c2_nc_y;
  c2_lc_b = c2_mpower(chartInstance, c2_m2);
  c2_oc_y = c2_ec_a * c2_lc_b;
  c2_fc_a = c2_l1;
  c2_mc_b = c2_b_mpower(chartInstance, c2_l2);
  c2_pc_y = c2_fc_a * c2_mc_b;
  c2_gc_a = c2_pc_y;
  c2_nc_b = c2_mpower(chartInstance, c2_m2);
  c2_qc_y = c2_gc_a * c2_nc_b;
  c2_hc_a = c2_qc_y;
  c2_oc_b = c2_s2;
  c2_rc_y = c2_hc_a * c2_oc_b;
  c2_ic_a = c2_rc_y;
  c2_pc_b = c2_t1d;
  c2_sc_y = c2_ic_a * c2_pc_b;
  c2_jc_a = c2_sc_y;
  c2_qc_b = c2_t2d;
  c2_tc_y = c2_jc_a * c2_qc_b;
  c2_rc_b = c2_I2;
  c2_uc_y = 4.0 * c2_rc_b;
  c2_kc_a = c2_uc_y;
  c2_sc_b = c2_l1;
  c2_vc_y = c2_kc_a * c2_sc_b;
  c2_lc_a = c2_vc_y;
  c2_tc_b = c2_l2;
  c2_wc_y = c2_lc_a * c2_tc_b;
  c2_mc_a = c2_wc_y;
  c2_uc_b = c2_m2;
  c2_xc_y = c2_mc_a * c2_uc_b;
  c2_nc_a = c2_xc_y;
  c2_vc_b = c2_s2;
  c2_yc_y = c2_nc_a * c2_vc_b;
  c2_oc_a = c2_yc_y;
  c2_wc_b = c2_t1d;
  c2_ad_y = c2_oc_a * c2_wc_b;
  c2_pc_a = c2_ad_y;
  c2_xc_b = c2_t2d;
  c2_bd_y = c2_pc_a * c2_xc_b;
  c2_qc_a = -c2_mpower(chartInstance, c2_c2);
  c2_yc_b = c2_mpower(chartInstance, c2_l1);
  c2_cd_y = c2_qc_a * c2_yc_b;
  c2_rc_a = c2_cd_y;
  c2_ad_b = c2_mpower(chartInstance, c2_l2);
  c2_dd_y = c2_rc_a * c2_ad_b;
  c2_sc_a = c2_dd_y;
  c2_bd_b = c2_mpower(chartInstance, c2_m2);
  c2_ed_y = c2_sc_a * c2_bd_b;
  c2_tc_a = c2_mpower(chartInstance, c2_l1);
  c2_cd_b = c2_mpower(chartInstance, c2_l2);
  c2_fd_y = c2_tc_a * c2_cd_b;
  c2_uc_a = c2_fd_y;
  c2_dd_b = c2_mpower(chartInstance, c2_m2);
  c2_gd_y = c2_uc_a * c2_dd_b;
  c2_ed_b = c2_I2;
  c2_hd_y = 4.0 * c2_ed_b;
  c2_vc_a = c2_hd_y;
  c2_fd_b = c2_mpower(chartInstance, c2_l1);
  c2_id_y = c2_vc_a * c2_fd_b;
  c2_wc_a = c2_id_y;
  c2_gd_b = c2_m2;
  c2_jd_y = c2_wc_a * c2_gd_b;
  c2_xc_a = c2_I1;
  c2_hd_b = c2_mpower(chartInstance, c2_l2);
  c2_kd_y = c2_xc_a * c2_hd_b;
  c2_yc_a = c2_kd_y;
  c2_id_b = c2_m2;
  c2_ld_y = c2_yc_a * c2_id_b;
  c2_jd_b = c2_I1;
  c2_md_y = 4.0 * c2_jd_b;
  c2_ad_a = c2_md_y;
  c2_kd_b = c2_I2;
  c2_nd_y = c2_ad_a * c2_kd_b;
  c2_d_A = (((((((((((((((c2_b_y - c2_d_y) + c2_f_y) - c2_h_y) - c2_l_y) -
                      c2_q_y) - c2_v_y) - c2_bb_y) + c2_gb_y) + c2_lb_y) +
                 c2_rb_y) + c2_xb_y) - c2_ec_y) + c2_jc_y) + c2_oc_y) + c2_tc_y)
    + c2_bd_y;
  c2_B = (((c2_ed_y + c2_gd_y) + c2_jd_y) + c2_ld_y) + c2_nd_y;
  c2_o_x = c2_d_A;
  c2_od_y = c2_B;
  c2_p_x = c2_o_x;
  c2_pd_y = c2_od_y;
  c2_t1dd = c2_p_x / c2_pd_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 23);
  c2_ld_b = c2_I2;
  c2_qd_y = 4.0 * c2_ld_b;
  c2_bd_a = c2_qd_y;
  c2_md_b = c2_T1;
  c2_rd_y = c2_bd_a * c2_md_b;
  c2_nd_b = c2_I1;
  c2_sd_y = 4.0 * c2_nd_b;
  c2_cd_a = c2_sd_y;
  c2_od_b = c2_T2;
  c2_td_y = c2_cd_a * c2_od_b;
  c2_pd_b = c2_I2;
  c2_ud_y = 4.0 * c2_pd_b;
  c2_dd_a = c2_ud_y;
  c2_qd_b = c2_T2;
  c2_vd_y = c2_dd_a * c2_qd_b;
  c2_ed_a = c2_T1;
  c2_rd_b = c2_mpower(chartInstance, c2_l2);
  c2_wd_y = c2_ed_a * c2_rd_b;
  c2_fd_a = c2_wd_y;
  c2_sd_b = c2_m2;
  c2_xd_y = c2_fd_a * c2_sd_b;
  c2_td_b = c2_T2;
  c2_yd_y = 4.0 * c2_td_b;
  c2_gd_a = c2_yd_y;
  c2_ud_b = c2_mpower(chartInstance, c2_l1);
  c2_ae_y = c2_gd_a * c2_ud_b;
  c2_hd_a = c2_ae_y;
  c2_vd_b = c2_m2;
  c2_be_y = c2_hd_a * c2_vd_b;
  c2_id_a = c2_T2;
  c2_wd_b = c2_mpower(chartInstance, c2_l2);
  c2_ce_y = c2_id_a * c2_wd_b;
  c2_jd_a = c2_ce_y;
  c2_xd_b = c2_m2;
  c2_de_y = c2_jd_a * c2_xd_b;
  c2_kd_a = c2_c1;
  c2_yd_b = c2_g;
  c2_ee_y = c2_kd_a * c2_yd_b;
  c2_ld_a = c2_ee_y;
  c2_ae_b = c2_l1;
  c2_fe_y = c2_ld_a * c2_ae_b;
  c2_md_a = c2_fe_y;
  c2_be_b = c2_mpower(chartInstance, c2_l2);
  c2_ge_y = c2_md_a * c2_be_b;
  c2_nd_a = c2_ge_y;
  c2_ce_b = c2_mpower(chartInstance, c2_m2);
  c2_he_y = c2_nd_a * c2_ce_b;
  c2_de_b = c2_c12;
  c2_ie_y = 2.0 * c2_de_b;
  c2_od_a = c2_ie_y;
  c2_ee_b = c2_g;
  c2_je_y = c2_od_a * c2_ee_b;
  c2_pd_a = c2_je_y;
  c2_fe_b = c2_mpower(chartInstance, c2_l1);
  c2_ke_y = c2_pd_a * c2_fe_b;
  c2_qd_a = c2_ke_y;
  c2_ge_b = c2_l2;
  c2_le_y = c2_qd_a * c2_ge_b;
  c2_rd_a = c2_le_y;
  c2_he_b = c2_mpower(chartInstance, c2_m2);
  c2_me_y = c2_rd_a * c2_he_b;
  c2_ie_b = c2_I2;
  c2_ne_y = 2.0 * c2_ie_b;
  c2_sd_a = c2_ne_y;
  c2_je_b = c2_c1;
  c2_oe_y = c2_sd_a * c2_je_b;
  c2_td_a = c2_oe_y;
  c2_ke_b = c2_g;
  c2_pe_y = c2_td_a * c2_ke_b;
  c2_ud_a = c2_pe_y;
  c2_le_b = c2_l1;
  c2_qe_y = c2_ud_a * c2_le_b;
  c2_vd_a = c2_qe_y;
  c2_me_b = c2_m1;
  c2_re_y = c2_vd_a * c2_me_b;
  c2_ne_b = c2_I2;
  c2_se_y = 4.0 * c2_ne_b;
  c2_wd_a = c2_se_y;
  c2_oe_b = c2_c1;
  c2_te_y = c2_wd_a * c2_oe_b;
  c2_xd_a = c2_te_y;
  c2_pe_b = c2_g;
  c2_ue_y = c2_xd_a * c2_pe_b;
  c2_yd_a = c2_ue_y;
  c2_qe_b = c2_l1;
  c2_ve_y = c2_yd_a * c2_qe_b;
  c2_ae_a = c2_ve_y;
  c2_re_b = c2_m2;
  c2_we_y = c2_ae_a * c2_re_b;
  c2_se_b = c2_I1;
  c2_xe_y = 2.0 * c2_se_b;
  c2_be_a = c2_xe_y;
  c2_te_b = c2_c12;
  c2_ye_y = c2_be_a * c2_te_b;
  c2_ce_a = c2_ye_y;
  c2_ue_b = c2_g;
  c2_af_y = c2_ce_a * c2_ue_b;
  c2_de_a = c2_af_y;
  c2_ve_b = c2_l2;
  c2_bf_y = c2_de_a * c2_ve_b;
  c2_ee_a = c2_bf_y;
  c2_we_b = c2_m2;
  c2_cf_y = c2_ee_a * c2_we_b;
  c2_xe_b = c2_T1;
  c2_df_y = 2.0 * c2_xe_b;
  c2_fe_a = c2_df_y;
  c2_ye_b = c2_c2;
  c2_ef_y = c2_fe_a * c2_ye_b;
  c2_ge_a = c2_ef_y;
  c2_af_b = c2_l1;
  c2_ff_y = c2_ge_a * c2_af_b;
  c2_he_a = c2_ff_y;
  c2_bf_b = c2_l2;
  c2_gf_y = c2_he_a * c2_bf_b;
  c2_ie_a = c2_gf_y;
  c2_cf_b = c2_m2;
  c2_hf_y = c2_ie_a * c2_cf_b;
  c2_df_b = c2_T2;
  c2_if_y = 4.0 * c2_df_b;
  c2_je_a = c2_if_y;
  c2_ef_b = c2_c2;
  c2_jf_y = c2_je_a * c2_ef_b;
  c2_ke_a = c2_jf_y;
  c2_ff_b = c2_l1;
  c2_kf_y = c2_ke_a * c2_ff_b;
  c2_le_a = c2_kf_y;
  c2_gf_b = c2_l2;
  c2_lf_y = c2_le_a * c2_gf_b;
  c2_me_a = c2_lf_y;
  c2_hf_b = c2_m2;
  c2_mf_y = c2_me_a * c2_hf_b;
  c2_ne_a = c2_l1;
  c2_if_b = c2_b_mpower(chartInstance, c2_l2);
  c2_nf_y = c2_ne_a * c2_if_b;
  c2_oe_a = c2_nf_y;
  c2_jf_b = c2_mpower(chartInstance, c2_m2);
  c2_of_y = c2_oe_a * c2_jf_b;
  c2_pe_a = c2_of_y;
  c2_kf_b = c2_s2;
  c2_pf_y = c2_pe_a * c2_kf_b;
  c2_qe_a = c2_pf_y;
  c2_lf_b = c2_mpower(chartInstance, c2_t1d);
  c2_qf_y = c2_qe_a * c2_lf_b;
  c2_e_A = c2_qf_y;
  c2_q_x = c2_e_A;
  c2_r_x = c2_q_x;
  c2_rf_y = c2_r_x / 2.0;
  c2_mf_b = c2_b_mpower(chartInstance, c2_l1);
  c2_sf_y = 2.0 * c2_mf_b;
  c2_re_a = c2_sf_y;
  c2_nf_b = c2_l2;
  c2_tf_y = c2_re_a * c2_nf_b;
  c2_se_a = c2_tf_y;
  c2_of_b = c2_mpower(chartInstance, c2_m2);
  c2_uf_y = c2_se_a * c2_of_b;
  c2_te_a = c2_uf_y;
  c2_pf_b = c2_s2;
  c2_vf_y = c2_te_a * c2_pf_b;
  c2_ue_a = c2_vf_y;
  c2_qf_b = c2_mpower(chartInstance, c2_t1d);
  c2_wf_y = c2_ue_a * c2_qf_b;
  c2_ve_a = c2_l1;
  c2_rf_b = c2_b_mpower(chartInstance, c2_l2);
  c2_xf_y = c2_ve_a * c2_rf_b;
  c2_we_a = c2_xf_y;
  c2_sf_b = c2_mpower(chartInstance, c2_m2);
  c2_yf_y = c2_we_a * c2_sf_b;
  c2_xe_a = c2_yf_y;
  c2_tf_b = c2_s2;
  c2_ag_y = c2_xe_a * c2_tf_b;
  c2_ye_a = c2_ag_y;
  c2_uf_b = c2_mpower(chartInstance, c2_t2d);
  c2_bg_y = c2_ye_a * c2_uf_b;
  c2_f_A = c2_bg_y;
  c2_s_x = c2_f_A;
  c2_t_x = c2_s_x;
  c2_cg_y = c2_t_x / 2.0;
  c2_vf_b = c2_I1;
  c2_dg_y = 2.0 * c2_vf_b;
  c2_af_a = c2_dg_y;
  c2_wf_b = c2_l1;
  c2_eg_y = c2_af_a * c2_wf_b;
  c2_bf_a = c2_eg_y;
  c2_xf_b = c2_l2;
  c2_fg_y = c2_bf_a * c2_xf_b;
  c2_cf_a = c2_fg_y;
  c2_yf_b = c2_m2;
  c2_gg_y = c2_cf_a * c2_yf_b;
  c2_df_a = c2_gg_y;
  c2_ag_b = c2_s2;
  c2_hg_y = c2_df_a * c2_ag_b;
  c2_ef_a = c2_hg_y;
  c2_bg_b = c2_mpower(chartInstance, c2_t1d);
  c2_ig_y = c2_ef_a * c2_bg_b;
  c2_cg_b = c2_I2;
  c2_jg_y = 2.0 * c2_cg_b;
  c2_ff_a = c2_jg_y;
  c2_dg_b = c2_l1;
  c2_kg_y = c2_ff_a * c2_dg_b;
  c2_gf_a = c2_kg_y;
  c2_eg_b = c2_l2;
  c2_lg_y = c2_gf_a * c2_eg_b;
  c2_hf_a = c2_lg_y;
  c2_fg_b = c2_m2;
  c2_mg_y = c2_hf_a * c2_fg_b;
  c2_if_a = c2_mg_y;
  c2_gg_b = c2_s2;
  c2_ng_y = c2_if_a * c2_gg_b;
  c2_jf_a = c2_ng_y;
  c2_hg_b = c2_mpower(chartInstance, c2_t1d);
  c2_og_y = c2_jf_a * c2_hg_b;
  c2_ig_b = c2_I2;
  c2_pg_y = 2.0 * c2_ig_b;
  c2_kf_a = c2_pg_y;
  c2_jg_b = c2_l1;
  c2_qg_y = c2_kf_a * c2_jg_b;
  c2_lf_a = c2_qg_y;
  c2_kg_b = c2_l2;
  c2_rg_y = c2_lf_a * c2_kg_b;
  c2_mf_a = c2_rg_y;
  c2_lg_b = c2_m2;
  c2_sg_y = c2_mf_a * c2_lg_b;
  c2_nf_a = c2_sg_y;
  c2_mg_b = c2_s2;
  c2_tg_y = c2_nf_a * c2_mg_b;
  c2_of_a = c2_tg_y;
  c2_ng_b = c2_mpower(chartInstance, c2_t2d);
  c2_ug_y = c2_of_a * c2_ng_b;
  c2_pf_a = c2_c1;
  c2_og_b = c2_g;
  c2_vg_y = c2_pf_a * c2_og_b;
  c2_qf_a = c2_vg_y;
  c2_pg_b = c2_l1;
  c2_wg_y = c2_qf_a * c2_pg_b;
  c2_rf_a = c2_wg_y;
  c2_qg_b = c2_mpower(chartInstance, c2_l2);
  c2_xg_y = c2_rf_a * c2_qg_b;
  c2_sf_a = c2_xg_y;
  c2_rg_b = c2_m1;
  c2_yg_y = c2_sf_a * c2_rg_b;
  c2_tf_a = c2_yg_y;
  c2_sg_b = c2_m2;
  c2_ah_y = c2_tf_a * c2_sg_b;
  c2_g_A = c2_ah_y;
  c2_u_x = c2_g_A;
  c2_v_x = c2_u_x;
  c2_bh_y = c2_v_x / 2.0;
  c2_tg_b = c2_c2;
  c2_ch_y = 2.0 * c2_tg_b;
  c2_uf_a = c2_ch_y;
  c2_ug_b = c2_mpower(chartInstance, c2_l1);
  c2_dh_y = c2_uf_a * c2_ug_b;
  c2_vf_a = c2_dh_y;
  c2_vg_b = c2_mpower(chartInstance, c2_l2);
  c2_eh_y = c2_vf_a * c2_vg_b;
  c2_wf_a = c2_eh_y;
  c2_wg_b = c2_mpower(chartInstance, c2_m2);
  c2_fh_y = c2_wf_a * c2_wg_b;
  c2_xf_a = c2_fh_y;
  c2_xg_b = c2_s2;
  c2_gh_y = c2_xf_a * c2_xg_b;
  c2_yf_a = c2_gh_y;
  c2_yg_b = c2_mpower(chartInstance, c2_t1d);
  c2_hh_y = c2_yf_a * c2_yg_b;
  c2_ag_a = c2_c2;
  c2_ah_b = c2_mpower(chartInstance, c2_l1);
  c2_ih_y = c2_ag_a * c2_ah_b;
  c2_bg_a = c2_ih_y;
  c2_bh_b = c2_mpower(chartInstance, c2_l2);
  c2_jh_y = c2_bg_a * c2_bh_b;
  c2_cg_a = c2_jh_y;
  c2_ch_b = c2_mpower(chartInstance, c2_m2);
  c2_kh_y = c2_cg_a * c2_ch_b;
  c2_dg_a = c2_kh_y;
  c2_dh_b = c2_s2;
  c2_lh_y = c2_dg_a * c2_dh_b;
  c2_eg_a = c2_lh_y;
  c2_eh_b = c2_mpower(chartInstance, c2_t2d);
  c2_mh_y = c2_eg_a * c2_eh_b;
  c2_fh_b = c2_c1;
  c2_nh_y = 2.0 * c2_fh_b;
  c2_fg_a = c2_nh_y;
  c2_gh_b = c2_c2;
  c2_oh_y = c2_fg_a * c2_gh_b;
  c2_gg_a = c2_oh_y;
  c2_hh_b = c2_g;
  c2_ph_y = c2_gg_a * c2_hh_b;
  c2_hg_a = c2_ph_y;
  c2_ih_b = c2_mpower(chartInstance, c2_l1);
  c2_qh_y = c2_hg_a * c2_ih_b;
  c2_ig_a = c2_qh_y;
  c2_jh_b = c2_l2;
  c2_rh_y = c2_ig_a * c2_jh_b;
  c2_jg_a = c2_rh_y;
  c2_kh_b = c2_mpower(chartInstance, c2_m2);
  c2_sh_y = c2_jg_a * c2_kh_b;
  c2_kg_a = c2_c2;
  c2_lh_b = c2_c12;
  c2_th_y = c2_kg_a * c2_lh_b;
  c2_lg_a = c2_th_y;
  c2_mh_b = c2_g;
  c2_uh_y = c2_lg_a * c2_mh_b;
  c2_mg_a = c2_uh_y;
  c2_nh_b = c2_l1;
  c2_vh_y = c2_mg_a * c2_nh_b;
  c2_ng_a = c2_vh_y;
  c2_oh_b = c2_mpower(chartInstance, c2_l2);
  c2_wh_y = c2_ng_a * c2_oh_b;
  c2_og_a = c2_wh_y;
  c2_ph_b = c2_mpower(chartInstance, c2_m2);
  c2_xh_y = c2_og_a * c2_ph_b;
  c2_pg_a = c2_l1;
  c2_qh_b = c2_b_mpower(chartInstance, c2_l2);
  c2_yh_y = c2_pg_a * c2_qh_b;
  c2_qg_a = c2_yh_y;
  c2_rh_b = c2_mpower(chartInstance, c2_m2);
  c2_ai_y = c2_qg_a * c2_rh_b;
  c2_rg_a = c2_ai_y;
  c2_sh_b = c2_s2;
  c2_bi_y = c2_rg_a * c2_sh_b;
  c2_sg_a = c2_bi_y;
  c2_th_b = c2_t1d;
  c2_ci_y = c2_sg_a * c2_th_b;
  c2_tg_a = c2_ci_y;
  c2_uh_b = c2_t2d;
  c2_di_y = c2_tg_a * c2_uh_b;
  c2_vh_b = c2_I2;
  c2_ei_y = 4.0 * c2_vh_b;
  c2_ug_a = c2_ei_y;
  c2_wh_b = c2_l1;
  c2_fi_y = c2_ug_a * c2_wh_b;
  c2_vg_a = c2_fi_y;
  c2_xh_b = c2_l2;
  c2_gi_y = c2_vg_a * c2_xh_b;
  c2_wg_a = c2_gi_y;
  c2_yh_b = c2_m2;
  c2_hi_y = c2_wg_a * c2_yh_b;
  c2_xg_a = c2_hi_y;
  c2_ai_b = c2_s2;
  c2_ii_y = c2_xg_a * c2_ai_b;
  c2_yg_a = c2_ii_y;
  c2_bi_b = c2_t1d;
  c2_ji_y = c2_yg_a * c2_bi_b;
  c2_ah_a = c2_ji_y;
  c2_ci_b = c2_t2d;
  c2_ki_y = c2_ah_a * c2_ci_b;
  c2_di_b = c2_c2;
  c2_li_y = 2.0 * c2_di_b;
  c2_bh_a = c2_li_y;
  c2_ei_b = c2_mpower(chartInstance, c2_l1);
  c2_mi_y = c2_bh_a * c2_ei_b;
  c2_ch_a = c2_mi_y;
  c2_fi_b = c2_mpower(chartInstance, c2_l2);
  c2_ni_y = c2_ch_a * c2_fi_b;
  c2_dh_a = c2_ni_y;
  c2_gi_b = c2_mpower(chartInstance, c2_m2);
  c2_oi_y = c2_dh_a * c2_gi_b;
  c2_eh_a = c2_oi_y;
  c2_hi_b = c2_s2;
  c2_pi_y = c2_eh_a * c2_hi_b;
  c2_fh_a = c2_pi_y;
  c2_ii_b = c2_t1d;
  c2_qi_y = c2_fh_a * c2_ii_b;
  c2_gh_a = c2_qi_y;
  c2_ji_b = c2_t2d;
  c2_ri_y = c2_gh_a * c2_ji_b;
  c2_hh_a = c2_c1;
  c2_ki_b = c2_c2;
  c2_si_y = c2_hh_a * c2_ki_b;
  c2_ih_a = c2_si_y;
  c2_li_b = c2_g;
  c2_ti_y = c2_ih_a * c2_li_b;
  c2_jh_a = c2_ti_y;
  c2_mi_b = c2_mpower(chartInstance, c2_l1);
  c2_ui_y = c2_jh_a * c2_mi_b;
  c2_kh_a = c2_ui_y;
  c2_ni_b = c2_l2;
  c2_vi_y = c2_kh_a * c2_ni_b;
  c2_lh_a = c2_vi_y;
  c2_oi_b = c2_m1;
  c2_wi_y = c2_lh_a * c2_oi_b;
  c2_mh_a = c2_wi_y;
  c2_pi_b = c2_m2;
  c2_xi_y = c2_mh_a * c2_pi_b;
  c2_nh_a = -c2_mpower(chartInstance, c2_c2);
  c2_qi_b = c2_mpower(chartInstance, c2_l1);
  c2_yi_y = c2_nh_a * c2_qi_b;
  c2_oh_a = c2_yi_y;
  c2_ri_b = c2_mpower(chartInstance, c2_l2);
  c2_aj_y = c2_oh_a * c2_ri_b;
  c2_ph_a = c2_aj_y;
  c2_si_b = c2_mpower(chartInstance, c2_m2);
  c2_bj_y = c2_ph_a * c2_si_b;
  c2_qh_a = c2_mpower(chartInstance, c2_l1);
  c2_ti_b = c2_mpower(chartInstance, c2_l2);
  c2_cj_y = c2_qh_a * c2_ti_b;
  c2_rh_a = c2_cj_y;
  c2_ui_b = c2_mpower(chartInstance, c2_m2);
  c2_dj_y = c2_rh_a * c2_ui_b;
  c2_vi_b = c2_I2;
  c2_ej_y = 4.0 * c2_vi_b;
  c2_sh_a = c2_ej_y;
  c2_wi_b = c2_mpower(chartInstance, c2_l1);
  c2_fj_y = c2_sh_a * c2_wi_b;
  c2_th_a = c2_fj_y;
  c2_xi_b = c2_m2;
  c2_gj_y = c2_th_a * c2_xi_b;
  c2_uh_a = c2_I1;
  c2_yi_b = c2_mpower(chartInstance, c2_l2);
  c2_hj_y = c2_uh_a * c2_yi_b;
  c2_vh_a = c2_hj_y;
  c2_aj_b = c2_m2;
  c2_ij_y = c2_vh_a * c2_aj_b;
  c2_bj_b = c2_I1;
  c2_jj_y = 4.0 * c2_bj_b;
  c2_wh_a = c2_jj_y;
  c2_cj_b = c2_I2;
  c2_kj_y = c2_wh_a * c2_cj_b;
  c2_h_A = -(((((((((((((((((((((((((((c2_rd_y - c2_td_y) - c2_vd_y) + c2_xd_y)
    - c2_be_y) - c2_de_y) - c2_he_y) + c2_me_y) - c2_re_y) - c2_we_y) + c2_cf_y)
    + c2_hf_y) - c2_mf_y) + c2_rf_y) + c2_wf_y) + c2_cg_y) + c2_ig_y) + c2_og_y)
                      + c2_ug_y) - c2_bh_y) + c2_hh_y) + c2_mh_y) - c2_sh_y) +
                 c2_xh_y) + c2_di_y) + c2_ki_y) + c2_ri_y) - c2_xi_y);
  c2_b_B = (((c2_bj_y + c2_dj_y) + c2_gj_y) + c2_ij_y) + c2_kj_y;
  c2_w_x = c2_h_A;
  c2_lj_y = c2_b_B;
  c2_x_x = c2_w_x;
  c2_mj_y = c2_lj_y;
  c2_t2dd = c2_x_x / c2_mj_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -23);
  _SFD_SYMBOL_SCOPE_POP();
  *c2_b_t1dd = c2_t1dd;
  *c2_b_t2dd = c2_t2dd;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
}

static void initSimStructsc2_compassGaitSim_impacts1
  (SFc2_compassGaitSim_impacts1InstanceStruct *chartInstance)
{
}

static void registerMessagesc2_compassGaitSim_impacts1
  (SFc2_compassGaitSim_impacts1InstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber)
{
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_compassGaitSim_impacts1InstanceStruct *chartInstance;
  chartInstance = (SFc2_compassGaitSim_impacts1InstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_emlrt_marshallIn(SFc2_compassGaitSim_impacts1InstanceStruct
  *chartInstance, const mxArray *c2_t2dd, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_t2dd), &c2_thisId);
  sf_mex_destroy(&c2_t2dd);
  return c2_y;
}

static real_T c2_b_emlrt_marshallIn(SFc2_compassGaitSim_impacts1InstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d0, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_t2dd;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_compassGaitSim_impacts1InstanceStruct *chartInstance;
  chartInstance = (SFc2_compassGaitSim_impacts1InstanceStruct *)
    chartInstanceVoid;
  c2_t2dd = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_t2dd), &c2_thisId);
  sf_mex_destroy(&c2_t2dd);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

const mxArray *sf_c2_compassGaitSim_impacts1_get_eml_resolved_functions_info
  (void)
{
  const mxArray *c2_nameCaptureInfo;
  c2_ResolvedFunctionInfo c2_info[17];
  const mxArray *c2_m0 = NULL;
  int32_T c2_i0;
  c2_ResolvedFunctionInfo *c2_r0;
  c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  c2_info_helper(c2_info);
  sf_mex_assign(&c2_m0, sf_mex_createstruct("nameCaptureInfo", 1, 17), FALSE);
  for (c2_i0 = 0; c2_i0 < 17; c2_i0++) {
    c2_r0 = &c2_info[c2_i0];
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->context)), "context", "nameCaptureInfo",
                    c2_i0);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c2_r0->name)), "name", "nameCaptureInfo", c2_i0);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c2_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c2_i0);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->resolved)), "resolved", "nameCaptureInfo",
                    c2_i0);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c2_i0);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c2_i0);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c2_i0);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c2_i0);
  }

  sf_mex_assign(&c2_nameCaptureInfo, c2_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c2_nameCaptureInfo);
  return c2_nameCaptureInfo;
}

static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[17])
{
  c2_info[0].context = "";
  c2_info[0].name = "cos";
  c2_info[0].dominantType = "double";
  c2_info[0].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c2_info[0].fileTimeLo = 1343801572U;
  c2_info[0].fileTimeHi = 0U;
  c2_info[0].mFileTimeLo = 0U;
  c2_info[0].mFileTimeHi = 0U;
  c2_info[1].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c2_info[1].name = "eml_scalar_cos";
  c2_info[1].dominantType = "double";
  c2_info[1].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m";
  c2_info[1].fileTimeLo = 1286786322U;
  c2_info[1].fileTimeHi = 0U;
  c2_info[1].mFileTimeLo = 0U;
  c2_info[1].mFileTimeHi = 0U;
  c2_info[2].context = "";
  c2_info[2].name = "sin";
  c2_info[2].dominantType = "double";
  c2_info[2].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c2_info[2].fileTimeLo = 1343801586U;
  c2_info[2].fileTimeHi = 0U;
  c2_info[2].mFileTimeLo = 0U;
  c2_info[2].mFileTimeHi = 0U;
  c2_info[3].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  c2_info[3].name = "eml_scalar_sin";
  c2_info[3].dominantType = "double";
  c2_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m";
  c2_info[3].fileTimeLo = 1286786336U;
  c2_info[3].fileTimeHi = 0U;
  c2_info[3].mFileTimeLo = 0U;
  c2_info[3].mFileTimeHi = 0U;
  c2_info[4].context = "";
  c2_info[4].name = "mtimes";
  c2_info[4].dominantType = "double";
  c2_info[4].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[4].fileTimeLo = 1289483692U;
  c2_info[4].fileTimeHi = 0U;
  c2_info[4].mFileTimeLo = 0U;
  c2_info[4].mFileTimeHi = 0U;
  c2_info[5].context = "";
  c2_info[5].name = "mpower";
  c2_info[5].dominantType = "double";
  c2_info[5].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c2_info[5].fileTimeLo = 1286786442U;
  c2_info[5].fileTimeHi = 0U;
  c2_info[5].mFileTimeLo = 0U;
  c2_info[5].mFileTimeHi = 0U;
  c2_info[6].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c2_info[6].name = "power";
  c2_info[6].dominantType = "double";
  c2_info[6].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[6].fileTimeLo = 1348163130U;
  c2_info[6].fileTimeHi = 0U;
  c2_info[6].mFileTimeLo = 0U;
  c2_info[6].mFileTimeHi = 0U;
  c2_info[7].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c2_info[7].name = "eml_scalar_eg";
  c2_info[7].dominantType = "double";
  c2_info[7].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[7].fileTimeLo = 1286786396U;
  c2_info[7].fileTimeHi = 0U;
  c2_info[7].mFileTimeLo = 0U;
  c2_info[7].mFileTimeHi = 0U;
  c2_info[8].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c2_info[8].name = "eml_scalexp_alloc";
  c2_info[8].dominantType = "double";
  c2_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[8].fileTimeLo = 1352388860U;
  c2_info[8].fileTimeHi = 0U;
  c2_info[8].mFileTimeLo = 0U;
  c2_info[8].mFileTimeHi = 0U;
  c2_info[9].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c2_info[9].name = "floor";
  c2_info[9].dominantType = "double";
  c2_info[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c2_info[9].fileTimeLo = 1343801580U;
  c2_info[9].fileTimeHi = 0U;
  c2_info[9].mFileTimeLo = 0U;
  c2_info[9].mFileTimeHi = 0U;
  c2_info[10].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c2_info[10].name = "eml_scalar_floor";
  c2_info[10].dominantType = "double";
  c2_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c2_info[10].fileTimeLo = 1286786326U;
  c2_info[10].fileTimeHi = 0U;
  c2_info[10].mFileTimeLo = 0U;
  c2_info[10].mFileTimeHi = 0U;
  c2_info[11].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power";
  c2_info[11].name = "eml_scalar_eg";
  c2_info[11].dominantType = "double";
  c2_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[11].fileTimeLo = 1286786396U;
  c2_info[11].fileTimeHi = 0U;
  c2_info[11].mFileTimeLo = 0U;
  c2_info[11].mFileTimeHi = 0U;
  c2_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power";
  c2_info[12].name = "mtimes";
  c2_info[12].dominantType = "double";
  c2_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[12].fileTimeLo = 1289483692U;
  c2_info[12].fileTimeHi = 0U;
  c2_info[12].mFileTimeLo = 0U;
  c2_info[12].mFileTimeHi = 0U;
  c2_info[13].context = "";
  c2_info[13].name = "mrdivide";
  c2_info[13].dominantType = "double";
  c2_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c2_info[13].fileTimeLo = 1357915548U;
  c2_info[13].fileTimeHi = 0U;
  c2_info[13].mFileTimeLo = 1319697566U;
  c2_info[13].mFileTimeHi = 0U;
  c2_info[14].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c2_info[14].name = "rdivide";
  c2_info[14].dominantType = "double";
  c2_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[14].fileTimeLo = 1346481588U;
  c2_info[14].fileTimeHi = 0U;
  c2_info[14].mFileTimeLo = 0U;
  c2_info[14].mFileTimeHi = 0U;
  c2_info[15].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[15].name = "eml_scalexp_compatible";
  c2_info[15].dominantType = "double";
  c2_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m";
  c2_info[15].fileTimeLo = 1286786396U;
  c2_info[15].fileTimeHi = 0U;
  c2_info[15].mFileTimeLo = 0U;
  c2_info[15].mFileTimeHi = 0U;
  c2_info[16].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[16].name = "eml_div";
  c2_info[16].dominantType = "double";
  c2_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[16].fileTimeLo = 1313319010U;
  c2_info[16].fileTimeHi = 0U;
  c2_info[16].mFileTimeLo = 0U;
  c2_info[16].mFileTimeHi = 0U;
}

static real_T c2_mpower(SFc2_compassGaitSim_impacts1InstanceStruct
  *chartInstance, real_T c2_a)
{
  real_T c2_b_a;
  real_T c2_c_a;
  real_T c2_ak;
  real_T c2_d_a;
  real_T c2_e_a;
  real_T c2_b;
  c2_b_a = c2_a;
  c2_c_a = c2_b_a;
  c2_eml_scalar_eg(chartInstance);
  c2_ak = c2_c_a;
  c2_d_a = c2_ak;
  c2_eml_scalar_eg(chartInstance);
  c2_e_a = c2_d_a;
  c2_b = c2_d_a;
  return c2_e_a * c2_b;
}

static void c2_eml_scalar_eg(SFc2_compassGaitSim_impacts1InstanceStruct
  *chartInstance)
{
}

static real_T c2_b_mpower(SFc2_compassGaitSim_impacts1InstanceStruct
  *chartInstance, real_T c2_a)
{
  real_T c2_b_a;
  real_T c2_c_a;
  real_T c2_ak;
  real_T c2_d_a;
  real_T c2_ar;
  c2_b_a = c2_a;
  c2_c_a = c2_b_a;
  c2_eml_scalar_eg(chartInstance);
  c2_ak = c2_c_a;
  c2_d_a = c2_ak;
  c2_eml_scalar_eg(chartInstance);
  c2_ar = c2_d_a;
  return muDoubleScalarPower(c2_ar, 3.0);
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_compassGaitSim_impacts1InstanceStruct *chartInstance;
  chartInstance = (SFc2_compassGaitSim_impacts1InstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(int32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static int32_T c2_c_emlrt_marshallIn(SFc2_compassGaitSim_impacts1InstanceStruct *
  chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_y;
  int32_T c2_i1;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i1, 1, 6, 0U, 0, 0U, 0);
  c2_y = c2_i1;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y;
  SFc2_compassGaitSim_impacts1InstanceStruct *chartInstance;
  chartInstance = (SFc2_compassGaitSim_impacts1InstanceStruct *)
    chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static uint8_T c2_d_emlrt_marshallIn(SFc2_compassGaitSim_impacts1InstanceStruct *
  chartInstance, const mxArray *c2_b_is_active_c2_compassGaitSim_impacts1, const
  char_T *c2_identifier)
{
  uint8_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_compassGaitSim_impacts1), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_compassGaitSim_impacts1);
  return c2_y;
}

static uint8_T c2_e_emlrt_marshallIn(SFc2_compassGaitSim_impacts1InstanceStruct *
  chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_y;
  uint8_T c2_u0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u0, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void init_dsm_address_info(SFc2_compassGaitSim_impacts1InstanceStruct
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

void sf_c2_compassGaitSim_impacts1_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(524653827U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2019587421U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2056528596U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(294409760U);
}

mxArray *sf_c2_compassGaitSim_impacts1_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("5v4F2TbciW9wnov1jDvMKC");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,13,3,dataFields);

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
      pr[0] = (double)(1);
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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,7,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,7,"type",mxType);
    }

    mxSetField(mxData,7,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,8,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,8,"type",mxType);
    }

    mxSetField(mxData,8,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,9,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,9,"type",mxType);
    }

    mxSetField(mxData,9,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,10,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,10,"type",mxType);
    }

    mxSetField(mxData,10,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,11,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,11,"type",mxType);
    }

    mxSetField(mxData,11,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,12,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,12,"type",mxType);
    }

    mxSetField(mxData,12,"complexity",mxCreateDoubleScalar(0));
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

mxArray *sf_c2_compassGaitSim_impacts1_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c2_compassGaitSim_impacts1(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x3'type','srcId','name','auxInfo'{{M[1],M[5],T\"t1dd\",},{M[1],M[11],T\"t2dd\",},{M[8],M[0],T\"is_active_c2_compassGaitSim_impacts1\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 3, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_compassGaitSim_impacts1_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_compassGaitSim_impacts1InstanceStruct *chartInstance;
    chartInstance = (SFc2_compassGaitSim_impacts1InstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _compassGaitSim_impacts1MachineNumber_,
           2,
           1,
           1,
           15,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"T1");
          _SFD_SET_DATA_PROPS(1,2,0,1,"t1dd");
          _SFD_SET_DATA_PROPS(2,1,1,0,"T2");
          _SFD_SET_DATA_PROPS(3,1,1,0,"t1d");
          _SFD_SET_DATA_PROPS(4,1,1,0,"t2d");
          _SFD_SET_DATA_PROPS(5,1,1,0,"t1");
          _SFD_SET_DATA_PROPS(6,1,1,0,"t2");
          _SFD_SET_DATA_PROPS(7,2,0,1,"t2dd");
          _SFD_SET_DATA_PROPS(8,1,1,0,"I1");
          _SFD_SET_DATA_PROPS(9,1,1,0,"l1");
          _SFD_SET_DATA_PROPS(10,1,1,0,"m1");
          _SFD_SET_DATA_PROPS(11,1,1,0,"I2");
          _SFD_SET_DATA_PROPS(12,1,1,0,"l2");
          _SFD_SET_DATA_PROPS(13,1,1,0,"m2");
          _SFD_SET_DATA_PROPS(14,1,1,0,"g");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1633);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(10,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(11,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(12,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(13,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(14,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);

        {
          real_T *c2_T1;
          real_T *c2_t1dd;
          real_T *c2_T2;
          real_T *c2_t1d;
          real_T *c2_t2d;
          real_T *c2_t1;
          real_T *c2_t2;
          real_T *c2_t2dd;
          real_T *c2_I1;
          real_T *c2_l1;
          real_T *c2_m1;
          real_T *c2_I2;
          real_T *c2_l2;
          real_T *c2_m2;
          real_T *c2_g;
          c2_g = (real_T *)ssGetInputPortSignal(chartInstance->S, 12);
          c2_m2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 11);
          c2_l2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 10);
          c2_I2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 9);
          c2_m1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 8);
          c2_l1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 7);
          c2_I1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 6);
          c2_t2dd = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
          c2_t2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
          c2_t1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
          c2_t2d = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c2_t1d = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c2_T2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c2_t1dd = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          c2_T1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c2_T1);
          _SFD_SET_DATA_VALUE_PTR(1U, c2_t1dd);
          _SFD_SET_DATA_VALUE_PTR(2U, c2_T2);
          _SFD_SET_DATA_VALUE_PTR(3U, c2_t1d);
          _SFD_SET_DATA_VALUE_PTR(4U, c2_t2d);
          _SFD_SET_DATA_VALUE_PTR(5U, c2_t1);
          _SFD_SET_DATA_VALUE_PTR(6U, c2_t2);
          _SFD_SET_DATA_VALUE_PTR(7U, c2_t2dd);
          _SFD_SET_DATA_VALUE_PTR(8U, c2_I1);
          _SFD_SET_DATA_VALUE_PTR(9U, c2_l1);
          _SFD_SET_DATA_VALUE_PTR(10U, c2_m1);
          _SFD_SET_DATA_VALUE_PTR(11U, c2_I2);
          _SFD_SET_DATA_VALUE_PTR(12U, c2_l2);
          _SFD_SET_DATA_VALUE_PTR(13U, c2_m2);
          _SFD_SET_DATA_VALUE_PTR(14U, c2_g);
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
  return "0kfZq26qxebC010GyELmi";
}

static void sf_opaque_initialize_c2_compassGaitSim_impacts1(void
  *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_compassGaitSim_impacts1InstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c2_compassGaitSim_impacts1
    ((SFc2_compassGaitSim_impacts1InstanceStruct*) chartInstanceVar);
  initialize_c2_compassGaitSim_impacts1
    ((SFc2_compassGaitSim_impacts1InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c2_compassGaitSim_impacts1(void *chartInstanceVar)
{
  enable_c2_compassGaitSim_impacts1((SFc2_compassGaitSim_impacts1InstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c2_compassGaitSim_impacts1(void *chartInstanceVar)
{
  disable_c2_compassGaitSim_impacts1((SFc2_compassGaitSim_impacts1InstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c2_compassGaitSim_impacts1(void *chartInstanceVar)
{
  sf_c2_compassGaitSim_impacts1((SFc2_compassGaitSim_impacts1InstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c2_compassGaitSim_impacts1
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c2_compassGaitSim_impacts1
    ((SFc2_compassGaitSim_impacts1InstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_compassGaitSim_impacts1();/* state var info */
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

extern void sf_internal_set_sim_state_c2_compassGaitSim_impacts1(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_compassGaitSim_impacts1();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c2_compassGaitSim_impacts1
    ((SFc2_compassGaitSim_impacts1InstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c2_compassGaitSim_impacts1
  (SimStruct* S)
{
  return sf_internal_get_sim_state_c2_compassGaitSim_impacts1(S);
}

static void sf_opaque_set_sim_state_c2_compassGaitSim_impacts1(SimStruct* S,
  const mxArray *st)
{
  sf_internal_set_sim_state_c2_compassGaitSim_impacts1(S, st);
}

static void sf_opaque_terminate_c2_compassGaitSim_impacts1(void
  *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_compassGaitSim_impacts1InstanceStruct*)
                    chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_compassGaitSim_impacts1_optimization_info();
    }

    finalize_c2_compassGaitSim_impacts1
      ((SFc2_compassGaitSim_impacts1InstanceStruct*) chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_compassGaitSim_impacts1
    ((SFc2_compassGaitSim_impacts1InstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_compassGaitSim_impacts1(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_compassGaitSim_impacts1
      ((SFc2_compassGaitSim_impacts1InstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c2_compassGaitSim_impacts1(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_compassGaitSim_impacts1_optimization_info();
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
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 6, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 7, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 8, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 9, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 10, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 11, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 12, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,13);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,2);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=2; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 13; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(3540620300U));
  ssSetChecksum1(S,(945985858U));
  ssSetChecksum2(S,(2816940329U));
  ssSetChecksum3(S,(325410803U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c2_compassGaitSim_impacts1(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_compassGaitSim_impacts1(SimStruct *S)
{
  SFc2_compassGaitSim_impacts1InstanceStruct *chartInstance;
  chartInstance = (SFc2_compassGaitSim_impacts1InstanceStruct *)utMalloc(sizeof
    (SFc2_compassGaitSim_impacts1InstanceStruct));
  memset(chartInstance, 0, sizeof(SFc2_compassGaitSim_impacts1InstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c2_compassGaitSim_impacts1;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c2_compassGaitSim_impacts1;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c2_compassGaitSim_impacts1;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c2_compassGaitSim_impacts1;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c2_compassGaitSim_impacts1;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c2_compassGaitSim_impacts1;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c2_compassGaitSim_impacts1;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c2_compassGaitSim_impacts1;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_compassGaitSim_impacts1;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_compassGaitSim_impacts1;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c2_compassGaitSim_impacts1;
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

void c2_compassGaitSim_impacts1_method_dispatcher(SimStruct *S, int_T method,
  void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_compassGaitSim_impacts1(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_compassGaitSim_impacts1(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_compassGaitSim_impacts1(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_compassGaitSim_impacts1_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
