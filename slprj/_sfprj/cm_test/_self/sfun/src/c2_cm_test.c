/* Include files */

#include <stddef.h>
#include "blas.h"
#include "cm_test_sfun.h"
#include "c2_cm_test.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "cm_test_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c_with_debugger(S, sfGlobalDebugInstanceStruct);

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization);
static void chart_debug_initialize_data_addresses(SimStruct *S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c2_debug_family_names[8] = { "nargin", "nargout",
  "sensors_in", "ctr_law", "vehicle", "v", "gamma", "ctr_vars" };

static const char * c2_b_debug_family_names[6] = { "nargin", "nargout", "x",
  "lower_bnd", "upper_bnd", "x_clipped" };

static const char * c2_c_debug_family_names[28] = { "e_theta",
  "prev_pt_pos_r_axle", "e_x", "e_y", "d", "e_RT", "r_ct", "v_t", "K_vec", "K_d",
  "K_l", "K_o", "K_x", "K_RT", "K_theta", "l_b", "c_c", "v_b", "v_raw",
  "gamma_raw", "nargin", "nargout", "sensors_in", "ctr_law", "vehicle", "v",
  "gamma", "ctr_vars" };

/* Function Declarations */
static void initialize_c2_cm_test(SFc2_cm_testInstanceStruct *chartInstance);
static void initialize_params_c2_cm_test(SFc2_cm_testInstanceStruct
  *chartInstance);
static void enable_c2_cm_test(SFc2_cm_testInstanceStruct *chartInstance);
static void disable_c2_cm_test(SFc2_cm_testInstanceStruct *chartInstance);
static void c2_update_debugger_state_c2_cm_test(SFc2_cm_testInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c2_cm_test(SFc2_cm_testInstanceStruct
  *chartInstance);
static void set_sim_state_c2_cm_test(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_st);
static void finalize_c2_cm_test(SFc2_cm_testInstanceStruct *chartInstance);
static void sf_gateway_c2_cm_test(SFc2_cm_testInstanceStruct *chartInstance);
static void mdl_start_c2_cm_test(SFc2_cm_testInstanceStruct *chartInstance);
static void initSimStructsc2_cm_test(SFc2_cm_testInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber, uint32_T c2_instanceNumber);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static void c2_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance, const
  mxArray *c2_b_ctr_vars, const char_T *c2_identifier, real_T c2_y[4]);
static void c2_b_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[4]);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_c_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_b_gamma, const char_T *c2_identifier);
static real_T c2_d_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static c2_vhcl_bus c2_e_emlrt_marshallIn(SFc2_cm_testInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_f_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  c2_ctr_law_params_bus *c2_y);
static int8_T c2_g_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_h_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[6]);
static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_i_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, c2_SensorsOut_bus *
  c2_y);
static c2_imu_bus c2_j_emlrt_marshallIn(SFc2_cm_testInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_k_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[3]);
static c2_mag_bus c2_l_emlrt_marshallIn(SFc2_cm_testInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static c2_RPSensor_bus c2_m_emlrt_marshallIn(SFc2_cm_testInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static c2_SteerSensor_bus c2_n_emlrt_marshallIn(SFc2_cm_testInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static c2_OdoSensors_bus c2_o_emlrt_marshallIn(SFc2_cm_testInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_VilcaController(SFc2_cm_testInstanceStruct *chartInstance,
  c2_SensorsOut_bus *c2_b_sensors_in, c2_ctr_law_params_bus *c2_b_ctr_law,
  c2_vhcl_bus c2_b_vehicle, real_T *c2_b_v, real_T *c2_b_gamma, real_T
  c2_b_ctr_vars[4]);
static real_T c2_mpower(SFc2_cm_testInstanceStruct *chartInstance, real_T c2_a);
static void c2_eml_scalar_eg(SFc2_cm_testInstanceStruct *chartInstance);
static void c2_dimagree(SFc2_cm_testInstanceStruct *chartInstance);
static void c2_eml_error(SFc2_cm_testInstanceStruct *chartInstance);
static real_T c2_clip_signal(SFc2_cm_testInstanceStruct *chartInstance, real_T
  c2_x, real_T c2_lower_bnd, real_T c2_upper_bnd);
static const mxArray *c2_h_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_p_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_sensors_in_bus_io(void *chartInstanceVoid, void
  *c2_pData);
static const mxArray *c2_ctr_law_bus_io(void *chartInstanceVoid, void *c2_pData);
static const mxArray *c2_vehicle_bus_io(void *chartInstanceVoid, void *c2_pData);
static uint8_T c2_q_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_b_is_active_c2_cm_test, const char_T *c2_identifier);
static uint8_T c2_r_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void init_dsm_address_info(SFc2_cm_testInstanceStruct *chartInstance);
static void init_simulink_io_address(SFc2_cm_testInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c2_cm_test(SFc2_cm_testInstanceStruct *chartInstance)
{
  if (sf_is_first_init_cond(chartInstance->S)) {
    initSimStructsc2_cm_test(chartInstance);
    chart_debug_initialize_data_addresses(chartInstance->S);
  }

  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c2_is_active_c2_cm_test = 0U;
}

static void initialize_params_c2_cm_test(SFc2_cm_testInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void enable_c2_cm_test(SFc2_cm_testInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c2_cm_test(SFc2_cm_testInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c2_update_debugger_state_c2_cm_test(SFc2_cm_testInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c2_cm_test(SFc2_cm_testInstanceStruct
  *chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  const mxArray *c2_b_y = NULL;
  real_T c2_hoistedGlobal;
  real_T c2_u;
  const mxArray *c2_c_y = NULL;
  real_T c2_b_hoistedGlobal;
  real_T c2_b_u;
  const mxArray *c2_d_y = NULL;
  uint8_T c2_c_hoistedGlobal;
  uint8_T c2_c_u;
  const mxArray *c2_e_y = NULL;
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellmatrix(4, 1), false);
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", *chartInstance->c2_ctr_vars, 0, 0U,
    1U, 0U, 1, 4), false);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_hoistedGlobal = *chartInstance->c2_gamma;
  c2_u = c2_hoistedGlobal;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  c2_b_hoistedGlobal = *chartInstance->c2_v;
  c2_b_u = c2_b_hoistedGlobal;
  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 2, c2_d_y);
  c2_c_hoistedGlobal = chartInstance->c2_is_active_c2_cm_test;
  c2_c_u = c2_c_hoistedGlobal;
  c2_e_y = NULL;
  sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_c_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 3, c2_e_y);
  sf_mex_assign(&c2_st, c2_y, false);
  return c2_st;
}

static void set_sim_state_c2_cm_test(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_st)
{
  const mxArray *c2_u;
  real_T c2_dv0[4];
  int32_T c2_i0;
  chartInstance->c2_doneDoubleBufferReInit = true;
  c2_u = sf_mex_dup(c2_st);
  c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell("ctr_vars", c2_u,
    0)), "ctr_vars", c2_dv0);
  for (c2_i0 = 0; c2_i0 < 4; c2_i0++) {
    (*chartInstance->c2_ctr_vars)[c2_i0] = c2_dv0[c2_i0];
  }

  *chartInstance->c2_gamma = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("gamma", c2_u, 1)), "gamma");
  *chartInstance->c2_v = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("v", c2_u, 2)), "v");
  chartInstance->c2_is_active_c2_cm_test = c2_q_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell("is_active_c2_cm_test", c2_u, 3)),
    "is_active_c2_cm_test");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_cm_test(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_cm_test(SFc2_cm_testInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c2_cm_test(SFc2_cm_testInstanceStruct *chartInstance)
{
  int32_T c2_i1;
  c2_SensorsOut_bus c2_b_sensors_in;
  int32_T c2_i2;
  int32_T c2_i3;
  c2_ctr_law_params_bus c2_b_ctr_law;
  int32_T c2_i4;
  c2_vhcl_bus c2_b_vehicle;
  uint32_T c2_debug_family_var_map[8];
  real_T c2_nargin = 3.0;
  real_T c2_nargout = 3.0;
  real_T c2_b_v;
  real_T c2_b_gamma;
  real_T c2_b_ctr_vars[4];
  c2_SensorsOut_bus c2_c_sensors_in;
  c2_ctr_law_params_bus c2_c_ctr_law;
  real_T c2_c_ctr_vars[4];
  real_T c2_c_gamma;
  real_T c2_c_v;
  int32_T c2_i5;
  int32_T c2_i6;
  int32_T c2_i7;
  int32_T c2_i8;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  chartInstance->c2_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  for (c2_i1 = 0; c2_i1 < 3; c2_i1++) {
    c2_b_sensors_in.imu_out.Omega_0[c2_i1] = ((real_T *)&((char_T *)(c2_imu_bus *)
      &((char_T *)chartInstance->c2_sensors_in)[0])[0])[c2_i1];
  }

  for (c2_i2 = 0; c2_i2 < 3; c2_i2++) {
    c2_b_sensors_in.imu_out.Alpha_0[c2_i2] = ((real_T *)&((char_T *)(c2_imu_bus *)
      &((char_T *)chartInstance->c2_sensors_in)[0])[24])[c2_i2];
  }

  c2_b_sensors_in.mag_out.hdg = *(real_T *)&((char_T *)(c2_mag_bus *)&((char_T *)
    chartInstance->c2_sensors_in)[48])[0];
  c2_b_sensors_in.rpsensor_out.lat_pos_ctr_road = *(real_T *)&((char_T *)
    (c2_RPSensor_bus *)&((char_T *)chartInstance->c2_sensors_in)[56])[0];
  c2_b_sensors_in.rpsensor_out.lat_pos_ctr_lane = *(real_T *)&((char_T *)
    (c2_RPSensor_bus *)&((char_T *)chartInstance->c2_sensors_in)[56])[8];
  c2_b_sensors_in.rpsensor_out.hdg_err = *(real_T *)&((char_T *)(c2_RPSensor_bus
    *)&((char_T *)chartInstance->c2_sensors_in)[56])[16];
  c2_b_sensors_in.rpsensor_out.path_hdg = *(real_T *)&((char_T *)
    (c2_RPSensor_bus *)&((char_T *)chartInstance->c2_sensors_in)[56])[24];
  c2_b_sensors_in.rpsensor_out.curve_xy = *(real_T *)&((char_T *)
    (c2_RPSensor_bus *)&((char_T *)chartInstance->c2_sensors_in)[56])[32];
  for (c2_i3 = 0; c2_i3 < 3; c2_i3++) {
    c2_b_sensors_in.rpsensor_out.preview_pt_pos_Fr1[c2_i3] = ((real_T *)
      &((char_T *)(c2_RPSensor_bus *)&((char_T *)chartInstance->c2_sensors_in)
        [56])[40])[c2_i3];
  }

  c2_b_sensors_in.steersensor_out.steer_angle_1 = *(real_T *)&((char_T *)
    (c2_SteerSensor_bus *)&((char_T *)chartInstance->c2_sensors_in)[120])[0];
  c2_b_sensors_in.steersensor_out.steer_angle_2 = *(real_T *)&((char_T *)
    (c2_SteerSensor_bus *)&((char_T *)chartInstance->c2_sensors_in)[120])[8];
  c2_b_sensors_in.steersensor_out.mean_angle = *(real_T *)&((char_T *)
    (c2_SteerSensor_bus *)&((char_T *)chartInstance->c2_sensors_in)[120])[16];
  c2_b_sensors_in.odo_out.vel_from_motor = *(real_T *)&((char_T *)
    (c2_OdoSensors_bus *)&((char_T *)chartInstance->c2_sensors_in)[144])[0];
  c2_b_ctr_law.flag = *(int8_T *)&((char_T *)chartInstance->c2_ctr_law)[0];
  for (c2_i4 = 0; c2_i4 < 6; c2_i4++) {
    c2_b_ctr_law.K_vilca[c2_i4] = ((real_T *)&((char_T *)
      chartInstance->c2_ctr_law)[8])[c2_i4];
  }

  c2_b_ctr_law.target_speed = *(real_T *)&((char_T *)chartInstance->c2_ctr_law)
    [56];
  c2_b_ctr_law.d_x_target = *(real_T *)&((char_T *)chartInstance->c2_ctr_law)[64];
  c2_b_ctr_law.v_min = *(real_T *)&((char_T *)chartInstance->c2_ctr_law)[72];
  c2_b_ctr_law.v_max = *(real_T *)&((char_T *)chartInstance->c2_ctr_law)[80];
  c2_b_ctr_law.gamma_max = *(real_T *)&((char_T *)chartInstance->c2_ctr_law)[88];
  c2_b_ctr_law.Fs = *(real_T *)&((char_T *)chartInstance->c2_ctr_law)[96];
  c2_b_vehicle.wheelbase = *(real_T *)&((char_T *)chartInstance->c2_vehicle)[0];
  c2_b_vehicle.rear_axle_x = *(real_T *)&((char_T *)chartInstance->c2_vehicle)[8];
  c2_b_vehicle.wheel_radius = *(real_T *)&((char_T *)chartInstance->c2_vehicle)
    [16];
  c2_b_vehicle.mass = *(real_T *)&((char_T *)chartInstance->c2_vehicle)[24];
  c2_b_vehicle.gear_ratio = *(real_T *)&((char_T *)chartInstance->c2_vehicle)[32];
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 8U, 8U, c2_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 0U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 1U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_b_sensors_in, 2U, c2_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_b_ctr_law, 3U, c2_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_b_vehicle, 4U, c2_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_v, 5U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_gamma, 6U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_ctr_vars, 7U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 28);
  if (CV_EML_IF(0, 1, 0, CV_RELATIONAL_EVAL(4U, 0U, 0, (real_T)c2_b_ctr_law.flag,
        1.0, -1, 0U, (real_T)c2_b_ctr_law.flag == 1.0))) {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 29);
    c2_c_sensors_in = c2_b_sensors_in;
    c2_c_ctr_law = c2_b_ctr_law;
    c2_VilcaController(chartInstance, &c2_c_sensors_in, &c2_c_ctr_law,
                       c2_b_vehicle, &c2_c_v, &c2_c_gamma, c2_c_ctr_vars);
    c2_b_v = c2_c_v;
    c2_b_gamma = c2_c_gamma;
    for (c2_i5 = 0; c2_i5 < 4; c2_i5++) {
      c2_b_ctr_vars[c2_i5] = c2_c_ctr_vars[c2_i5];
    }
  } else {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 31);
    c2_b_v = 0.0;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 32);
    c2_b_gamma = 0.0;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 33);
    for (c2_i6 = 0; c2_i6 < 4; c2_i6++) {
      c2_b_ctr_vars[c2_i6] = 0.0;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -33);
  _SFD_SYMBOL_SCOPE_POP();
  *chartInstance->c2_v = c2_b_v;
  *chartInstance->c2_gamma = c2_b_gamma;
  for (c2_i7 = 0; c2_i7 < 4; c2_i7++) {
    (*chartInstance->c2_ctr_vars)[c2_i7] = c2_b_ctr_vars[c2_i7];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_cm_testMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_v, 3U, 1U, 0U,
                        chartInstance->c2_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_gamma, 4U, 1U, 0U,
                        chartInstance->c2_sfEvent, false);
  for (c2_i8 = 0; c2_i8 < 4; c2_i8++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c2_ctr_vars)[c2_i8], 5U, 1U, 0U,
                          chartInstance->c2_sfEvent, false);
  }
}

static void mdl_start_c2_cm_test(SFc2_cm_testInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void initSimStructsc2_cm_test(SFc2_cm_testInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber, uint32_T c2_instanceNumber)
{
  (void)c2_machineNumber;
  (void)c2_chartNumber;
  (void)c2_instanceNumber;
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i9;
  real_T c2_u[4];
  const mxArray *c2_y = NULL;
  SFc2_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_cm_testInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i9 = 0; c2_i9 < 4; c2_i9++) {
    c2_u[c2_i9] = (*(real_T (*)[4])c2_inData)[c2_i9];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 4), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance, const
  mxArray *c2_b_ctr_vars, const char_T *c2_identifier, real_T c2_y[4])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_ctr_vars), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_b_ctr_vars);
}

static void c2_b_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[4])
{
  real_T c2_dv1[4];
  int32_T c2_i10;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv1, 1, 0, 0U, 1, 0U, 1, 4);
  for (c2_i10 = 0; c2_i10 < 4; c2_i10++) {
    c2_y[c2_i10] = c2_dv1[c2_i10];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_ctr_vars;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[4];
  int32_T c2_i11;
  SFc2_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_cm_testInstanceStruct *)chartInstanceVoid;
  c2_b_ctr_vars = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_ctr_vars), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_b_ctr_vars);
  for (c2_i11 = 0; c2_i11 < 4; c2_i11++) {
    (*(real_T (*)[4])c2_outData)[c2_i11] = c2_y[c2_i11];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_cm_testInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static real_T c2_c_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_b_gamma, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_gamma), &c2_thisId);
  sf_mex_destroy(&c2_b_gamma);
  return c2_y;
}

static real_T c2_d_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d0;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d0, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_gamma;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_cm_testInstanceStruct *)chartInstanceVoid;
  c2_b_gamma = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_gamma), &c2_thisId);
  sf_mex_destroy(&c2_b_gamma);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData;
  c2_vhcl_bus c2_u;
  const mxArray *c2_y = NULL;
  real_T c2_b_u;
  const mxArray *c2_b_y = NULL;
  real_T c2_c_u;
  const mxArray *c2_c_y = NULL;
  real_T c2_d_u;
  const mxArray *c2_d_y = NULL;
  real_T c2_e_u;
  const mxArray *c2_e_y = NULL;
  real_T c2_f_u;
  const mxArray *c2_f_y = NULL;
  SFc2_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_cm_testInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  c2_u = *(c2_vhcl_bus *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_b_u = c2_u.wheelbase;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_y, c2_b_y, "wheelbase", "wheelbase", 0);
  c2_c_u = c2_u.rear_axle_x;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_y, c2_c_y, "rear_axle_x", "rear_axle_x", 0);
  c2_d_u = c2_u.wheel_radius;
  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_d_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_y, c2_d_y, "wheel_radius", "wheel_radius", 0);
  c2_e_u = c2_u.mass;
  c2_e_y = NULL;
  sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_e_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_y, c2_e_y, "mass", "mass", 0);
  c2_f_u = c2_u.gear_ratio;
  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_create("y", &c2_f_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_y, c2_f_y, "gear_ratio", "gear_ratio", 0);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData;
  c2_ctr_law_params_bus c2_u;
  const mxArray *c2_y = NULL;
  int8_T c2_b_u;
  const mxArray *c2_b_y = NULL;
  int32_T c2_i12;
  real_T c2_c_u[6];
  const mxArray *c2_c_y = NULL;
  real_T c2_d_u;
  const mxArray *c2_d_y = NULL;
  real_T c2_e_u;
  const mxArray *c2_e_y = NULL;
  real_T c2_f_u;
  const mxArray *c2_f_y = NULL;
  real_T c2_g_u;
  const mxArray *c2_g_y = NULL;
  real_T c2_h_u;
  const mxArray *c2_h_y = NULL;
  real_T c2_i_u;
  const mxArray *c2_i_y = NULL;
  SFc2_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_cm_testInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  c2_u = *(c2_ctr_law_params_bus *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_b_u = c2_u.flag;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 2, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_y, c2_b_y, "flag", "flag", 0);
  for (c2_i12 = 0; c2_i12 < 6; c2_i12++) {
    c2_c_u[c2_i12] = c2_u.K_vilca[c2_i12];
  }

  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", c2_c_u, 0, 0U, 1U, 0U, 1, 6), false);
  sf_mex_addfield(c2_y, c2_c_y, "K_vilca", "K_vilca", 0);
  c2_d_u = c2_u.target_speed;
  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_d_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_y, c2_d_y, "target_speed", "target_speed", 0);
  c2_e_u = c2_u.d_x_target;
  c2_e_y = NULL;
  sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_e_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_y, c2_e_y, "d_x_target", "d_x_target", 0);
  c2_f_u = c2_u.v_min;
  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_create("y", &c2_f_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_y, c2_f_y, "v_min", "v_min", 0);
  c2_g_u = c2_u.v_max;
  c2_g_y = NULL;
  sf_mex_assign(&c2_g_y, sf_mex_create("y", &c2_g_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_y, c2_g_y, "v_max", "v_max", 0);
  c2_h_u = c2_u.gamma_max;
  c2_h_y = NULL;
  sf_mex_assign(&c2_h_y, sf_mex_create("y", &c2_h_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_y, c2_h_y, "gamma_max", "gamma_max", 0);
  c2_i_u = c2_u.Fs;
  c2_i_y = NULL;
  sf_mex_assign(&c2_i_y, sf_mex_create("y", &c2_i_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_y, c2_i_y, "Fs", "Fs", 0);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData;
  c2_SensorsOut_bus c2_u;
  const mxArray *c2_y = NULL;
  c2_imu_bus c2_b_u;
  const mxArray *c2_b_y = NULL;
  int32_T c2_i13;
  real_T c2_c_u[3];
  const mxArray *c2_c_y = NULL;
  int32_T c2_i14;
  const mxArray *c2_d_y = NULL;
  c2_mag_bus c2_d_u;
  const mxArray *c2_e_y = NULL;
  real_T c2_e_u;
  const mxArray *c2_f_y = NULL;
  c2_RPSensor_bus c2_f_u;
  const mxArray *c2_g_y = NULL;
  real_T c2_g_u;
  const mxArray *c2_h_y = NULL;
  real_T c2_h_u;
  const mxArray *c2_i_y = NULL;
  real_T c2_i_u;
  const mxArray *c2_j_y = NULL;
  real_T c2_j_u;
  const mxArray *c2_k_y = NULL;
  real_T c2_k_u;
  const mxArray *c2_l_y = NULL;
  int32_T c2_i15;
  const mxArray *c2_m_y = NULL;
  c2_SteerSensor_bus c2_l_u;
  const mxArray *c2_n_y = NULL;
  real_T c2_m_u;
  const mxArray *c2_o_y = NULL;
  real_T c2_n_u;
  const mxArray *c2_p_y = NULL;
  real_T c2_o_u;
  const mxArray *c2_q_y = NULL;
  c2_OdoSensors_bus c2_p_u;
  const mxArray *c2_r_y = NULL;
  real_T c2_q_u;
  const mxArray *c2_s_y = NULL;
  SFc2_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_cm_testInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  c2_u = *(c2_SensorsOut_bus *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_b_u = c2_u.imu_out;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  for (c2_i13 = 0; c2_i13 < 3; c2_i13++) {
    c2_c_u[c2_i13] = c2_b_u.Omega_0[c2_i13];
  }

  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", c2_c_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_addfield(c2_b_y, c2_c_y, "Omega_0", "Omega_0", 0);
  for (c2_i14 = 0; c2_i14 < 3; c2_i14++) {
    c2_c_u[c2_i14] = c2_b_u.Alpha_0[c2_i14];
  }

  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", c2_c_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_addfield(c2_b_y, c2_d_y, "Alpha_0", "Alpha_0", 0);
  sf_mex_addfield(c2_y, c2_b_y, "imu_out", "imu_out", 0);
  c2_d_u = c2_u.mag_out;
  c2_e_y = NULL;
  sf_mex_assign(&c2_e_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_e_u = c2_d_u.hdg;
  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_create("y", &c2_e_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_e_y, c2_f_y, "hdg", "hdg", 0);
  sf_mex_addfield(c2_y, c2_e_y, "mag_out", "mag_out", 0);
  c2_f_u = c2_u.rpsensor_out;
  c2_g_y = NULL;
  sf_mex_assign(&c2_g_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_g_u = c2_f_u.lat_pos_ctr_road;
  c2_h_y = NULL;
  sf_mex_assign(&c2_h_y, sf_mex_create("y", &c2_g_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_g_y, c2_h_y, "lat_pos_ctr_road", "lat_pos_ctr_road", 0);
  c2_h_u = c2_f_u.lat_pos_ctr_lane;
  c2_i_y = NULL;
  sf_mex_assign(&c2_i_y, sf_mex_create("y", &c2_h_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_g_y, c2_i_y, "lat_pos_ctr_lane", "lat_pos_ctr_lane", 0);
  c2_i_u = c2_f_u.hdg_err;
  c2_j_y = NULL;
  sf_mex_assign(&c2_j_y, sf_mex_create("y", &c2_i_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_g_y, c2_j_y, "hdg_err", "hdg_err", 0);
  c2_j_u = c2_f_u.path_hdg;
  c2_k_y = NULL;
  sf_mex_assign(&c2_k_y, sf_mex_create("y", &c2_j_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_g_y, c2_k_y, "path_hdg", "path_hdg", 0);
  c2_k_u = c2_f_u.curve_xy;
  c2_l_y = NULL;
  sf_mex_assign(&c2_l_y, sf_mex_create("y", &c2_k_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_g_y, c2_l_y, "curve_xy", "curve_xy", 0);
  for (c2_i15 = 0; c2_i15 < 3; c2_i15++) {
    c2_c_u[c2_i15] = c2_f_u.preview_pt_pos_Fr1[c2_i15];
  }

  c2_m_y = NULL;
  sf_mex_assign(&c2_m_y, sf_mex_create("y", c2_c_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_addfield(c2_g_y, c2_m_y, "preview_pt_pos_Fr1", "preview_pt_pos_Fr1", 0);
  sf_mex_addfield(c2_y, c2_g_y, "rpsensor_out", "rpsensor_out", 0);
  c2_l_u = c2_u.steersensor_out;
  c2_n_y = NULL;
  sf_mex_assign(&c2_n_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_m_u = c2_l_u.steer_angle_1;
  c2_o_y = NULL;
  sf_mex_assign(&c2_o_y, sf_mex_create("y", &c2_m_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_n_y, c2_o_y, "steer_angle_1", "steer_angle_1", 0);
  c2_n_u = c2_l_u.steer_angle_2;
  c2_p_y = NULL;
  sf_mex_assign(&c2_p_y, sf_mex_create("y", &c2_n_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_n_y, c2_p_y, "steer_angle_2", "steer_angle_2", 0);
  c2_o_u = c2_l_u.mean_angle;
  c2_q_y = NULL;
  sf_mex_assign(&c2_q_y, sf_mex_create("y", &c2_o_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_n_y, c2_q_y, "mean_angle", "mean_angle", 0);
  sf_mex_addfield(c2_y, c2_n_y, "steersensor_out", "steersensor_out", 0);
  c2_p_u = c2_u.odo_out;
  c2_r_y = NULL;
  sf_mex_assign(&c2_r_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_q_u = c2_p_u.vel_from_motor;
  c2_s_y = NULL;
  sf_mex_assign(&c2_s_y, sf_mex_create("y", &c2_q_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_r_y, c2_s_y, "vel_from_motor", "vel_from_motor", 0);
  sf_mex_addfield(c2_y, c2_r_y, "odo_out", "odo_out", 0);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static c2_vhcl_bus c2_e_emlrt_marshallIn(SFc2_cm_testInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  c2_vhcl_bus c2_y;
  emlrtMsgIdentifier c2_thisId;
  static const char * c2_fieldNames[5] = { "wheelbase", "rear_axle_x",
    "wheel_radius", "mass", "gear_ratio" };

  c2_thisId.fParent = c2_parentId;
  sf_mex_check_struct(c2_parentId, c2_u, 5, c2_fieldNames, 0U, NULL);
  c2_thisId.fIdentifier = "wheelbase";
  c2_y.wheelbase = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "wheelbase", "wheelbase", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "rear_axle_x";
  c2_y.rear_axle_x = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "rear_axle_x", "rear_axle_x", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "wheel_radius";
  c2_y.wheel_radius = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "wheel_radius", "wheel_radius", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "mass";
  c2_y.mass = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c2_u, "mass", "mass", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "gear_ratio";
  c2_y.gear_ratio = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "gear_ratio", "gear_ratio", 0)), &c2_thisId);
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_vehicle;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  c2_vhcl_bus c2_y;
  SFc2_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_cm_testInstanceStruct *)chartInstanceVoid;
  c2_b_vehicle = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_vehicle),
    &c2_thisId);
  sf_mex_destroy(&c2_b_vehicle);
  *(c2_vhcl_bus *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static void c2_f_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  c2_ctr_law_params_bus *c2_y)
{
  emlrtMsgIdentifier c2_thisId;
  static const char * c2_fieldNames[8] = { "flag", "K_vilca", "target_speed",
    "d_x_target", "v_min", "v_max", "gamma_max", "Fs" };

  c2_thisId.fParent = c2_parentId;
  sf_mex_check_struct(c2_parentId, c2_u, 8, c2_fieldNames, 0U, NULL);
  c2_thisId.fIdentifier = "flag";
  c2_y->flag = c2_g_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c2_u, "flag", "flag", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "K_vilca";
  c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u,
    "K_vilca", "K_vilca", 0)), &c2_thisId, c2_y->K_vilca);
  c2_thisId.fIdentifier = "target_speed";
  c2_y->target_speed = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "target_speed", "target_speed", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "d_x_target";
  c2_y->d_x_target = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "d_x_target", "d_x_target", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "v_min";
  c2_y->v_min = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c2_u, "v_min", "v_min", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "v_max";
  c2_y->v_max = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c2_u, "v_max", "v_max", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "gamma_max";
  c2_y->gamma_max = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "gamma_max", "gamma_max", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "Fs";
  c2_y->Fs = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c2_u, "Fs", "Fs", 0)), &c2_thisId);
  sf_mex_destroy(&c2_u);
}

static int8_T c2_g_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int8_T c2_y;
  int8_T c2_i16;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i16, 1, 2, 0U, 0, 0U, 0);
  c2_y = c2_i16;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_h_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[6])
{
  real_T c2_dv2[6];
  int32_T c2_i17;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv2, 1, 0, 0U, 1, 0U, 1, 6);
  for (c2_i17 = 0; c2_i17 < 6; c2_i17++) {
    c2_y[c2_i17] = c2_dv2[c2_i17];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_ctr_law;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  c2_ctr_law_params_bus c2_y;
  SFc2_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_cm_testInstanceStruct *)chartInstanceVoid;
  c2_b_ctr_law = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_ctr_law), &c2_thisId,
                        &c2_y);
  sf_mex_destroy(&c2_b_ctr_law);
  *(c2_ctr_law_params_bus *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static void c2_i_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, c2_SensorsOut_bus *
  c2_y)
{
  emlrtMsgIdentifier c2_thisId;
  static const char * c2_fieldNames[5] = { "imu_out", "mag_out", "rpsensor_out",
    "steersensor_out", "odo_out" };

  c2_thisId.fParent = c2_parentId;
  sf_mex_check_struct(c2_parentId, c2_u, 5, c2_fieldNames, 0U, NULL);
  c2_thisId.fIdentifier = "imu_out";
  c2_y->imu_out = c2_j_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "imu_out", "imu_out", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "mag_out";
  c2_y->mag_out = c2_l_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "mag_out", "mag_out", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "rpsensor_out";
  c2_y->rpsensor_out = c2_m_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "rpsensor_out", "rpsensor_out", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "steersensor_out";
  c2_y->steersensor_out = c2_n_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "steersensor_out", "steersensor_out", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "odo_out";
  c2_y->odo_out = c2_o_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "odo_out", "odo_out", 0)), &c2_thisId);
  sf_mex_destroy(&c2_u);
}

static c2_imu_bus c2_j_emlrt_marshallIn(SFc2_cm_testInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  c2_imu_bus c2_y;
  emlrtMsgIdentifier c2_thisId;
  static const char * c2_fieldNames[2] = { "Omega_0", "Alpha_0" };

  c2_thisId.fParent = c2_parentId;
  sf_mex_check_struct(c2_parentId, c2_u, 2, c2_fieldNames, 0U, NULL);
  c2_thisId.fIdentifier = "Omega_0";
  c2_k_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u,
    "Omega_0", "Omega_0", 0)), &c2_thisId, c2_y.Omega_0);
  c2_thisId.fIdentifier = "Alpha_0";
  c2_k_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u,
    "Alpha_0", "Alpha_0", 0)), &c2_thisId, c2_y.Alpha_0);
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_k_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[3])
{
  real_T c2_dv3[3];
  int32_T c2_i18;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv3, 1, 0, 0U, 1, 0U, 1, 3);
  for (c2_i18 = 0; c2_i18 < 3; c2_i18++) {
    c2_y[c2_i18] = c2_dv3[c2_i18];
  }

  sf_mex_destroy(&c2_u);
}

static c2_mag_bus c2_l_emlrt_marshallIn(SFc2_cm_testInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  c2_mag_bus c2_y;
  emlrtMsgIdentifier c2_thisId;
  static const char * c2_fieldNames[1] = { "hdg" };

  c2_thisId.fParent = c2_parentId;
  sf_mex_check_struct(c2_parentId, c2_u, 1, c2_fieldNames, 0U, NULL);
  c2_thisId.fIdentifier = "hdg";
  c2_y.hdg = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c2_u, "hdg", "hdg", 0)), &c2_thisId);
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static c2_RPSensor_bus c2_m_emlrt_marshallIn(SFc2_cm_testInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  c2_RPSensor_bus c2_y;
  emlrtMsgIdentifier c2_thisId;
  static const char * c2_fieldNames[6] = { "lat_pos_ctr_road",
    "lat_pos_ctr_lane", "hdg_err", "path_hdg", "curve_xy", "preview_pt_pos_Fr1"
  };

  c2_thisId.fParent = c2_parentId;
  sf_mex_check_struct(c2_parentId, c2_u, 6, c2_fieldNames, 0U, NULL);
  c2_thisId.fIdentifier = "lat_pos_ctr_road";
  c2_y.lat_pos_ctr_road = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "lat_pos_ctr_road", "lat_pos_ctr_road", 0)),
    &c2_thisId);
  c2_thisId.fIdentifier = "lat_pos_ctr_lane";
  c2_y.lat_pos_ctr_lane = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "lat_pos_ctr_lane", "lat_pos_ctr_lane", 0)),
    &c2_thisId);
  c2_thisId.fIdentifier = "hdg_err";
  c2_y.hdg_err = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c2_u, "hdg_err", "hdg_err", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "path_hdg";
  c2_y.path_hdg = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "path_hdg", "path_hdg", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "curve_xy";
  c2_y.curve_xy = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "curve_xy", "curve_xy", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "preview_pt_pos_Fr1";
  c2_k_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u,
    "preview_pt_pos_Fr1", "preview_pt_pos_Fr1", 0)), &c2_thisId,
                        c2_y.preview_pt_pos_Fr1);
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static c2_SteerSensor_bus c2_n_emlrt_marshallIn(SFc2_cm_testInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  c2_SteerSensor_bus c2_y;
  emlrtMsgIdentifier c2_thisId;
  static const char * c2_fieldNames[3] = { "steer_angle_1", "steer_angle_2",
    "mean_angle" };

  c2_thisId.fParent = c2_parentId;
  sf_mex_check_struct(c2_parentId, c2_u, 3, c2_fieldNames, 0U, NULL);
  c2_thisId.fIdentifier = "steer_angle_1";
  c2_y.steer_angle_1 = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "steer_angle_1", "steer_angle_1", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "steer_angle_2";
  c2_y.steer_angle_2 = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "steer_angle_2", "steer_angle_2", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "mean_angle";
  c2_y.mean_angle = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "mean_angle", "mean_angle", 0)), &c2_thisId);
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static c2_OdoSensors_bus c2_o_emlrt_marshallIn(SFc2_cm_testInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  c2_OdoSensors_bus c2_y;
  emlrtMsgIdentifier c2_thisId;
  static const char * c2_fieldNames[1] = { "vel_from_motor" };

  c2_thisId.fParent = c2_parentId;
  sf_mex_check_struct(c2_parentId, c2_u, 1, c2_fieldNames, 0U, NULL);
  c2_thisId.fIdentifier = "vel_from_motor";
  c2_y.vel_from_motor = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "vel_from_motor", "vel_from_motor", 0)), &c2_thisId);
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sensors_in;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  c2_SensorsOut_bus c2_y;
  SFc2_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_cm_testInstanceStruct *)chartInstanceVoid;
  c2_b_sensors_in = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sensors_in), &c2_thisId,
                        &c2_y);
  sf_mex_destroy(&c2_b_sensors_in);
  *(c2_SensorsOut_bus *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i19;
  real_T c2_u[6];
  const mxArray *c2_y = NULL;
  SFc2_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_cm_testInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i19 = 0; c2_i19 < 6; c2_i19++) {
    c2_u[c2_i19] = (*(real_T (*)[6])c2_inData)[c2_i19];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 6), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_K_vec;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[6];
  int32_T c2_i20;
  SFc2_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_cm_testInstanceStruct *)chartInstanceVoid;
  c2_K_vec = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_K_vec), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_K_vec);
  for (c2_i20 = 0; c2_i20 < 6; c2_i20++) {
    (*(real_T (*)[6])c2_outData)[c2_i20] = c2_y[c2_i20];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i21;
  real_T c2_u[3];
  const mxArray *c2_y = NULL;
  SFc2_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_cm_testInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i21 = 0; c2_i21 < 3; c2_i21++) {
    c2_u[c2_i21] = (*(real_T (*)[3])c2_inData)[c2_i21];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_prev_pt_pos_r_axle;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[3];
  int32_T c2_i22;
  SFc2_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_cm_testInstanceStruct *)chartInstanceVoid;
  c2_prev_pt_pos_r_axle = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_k_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_prev_pt_pos_r_axle),
                        &c2_thisId, c2_y);
  sf_mex_destroy(&c2_prev_pt_pos_r_axle);
  for (c2_i22 = 0; c2_i22 < 3; c2_i22++) {
    (*(real_T (*)[3])c2_outData)[c2_i22] = c2_y[c2_i22];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

const mxArray *sf_c2_cm_test_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  sf_mex_assign(&c2_nameCaptureInfo, sf_mex_create("nameCaptureInfo", NULL, 0,
    0U, 1U, 0U, 2, 0, 1), false);
  return c2_nameCaptureInfo;
}

static void c2_VilcaController(SFc2_cm_testInstanceStruct *chartInstance,
  c2_SensorsOut_bus *c2_b_sensors_in, c2_ctr_law_params_bus *c2_b_ctr_law,
  c2_vhcl_bus c2_b_vehicle, real_T *c2_b_v, real_T *c2_b_gamma, real_T
  c2_b_ctr_vars[4])
{
  uint32_T c2_debug_family_var_map[28];
  real_T c2_e_theta;
  real_T c2_prev_pt_pos_r_axle[3];
  real_T c2_e_x;
  real_T c2_e_y;
  real_T c2_d;
  real_T c2_e_RT;
  real_T c2_r_ct;
  real_T c2_v_t;
  real_T c2_K_vec[6];
  real_T c2_K_d;
  real_T c2_K_l;
  real_T c2_K_o;
  real_T c2_K_x;
  real_T c2_K_RT;
  real_T c2_K_theta;
  real_T c2_l_b;
  real_T c2_c_c;
  real_T c2_v_b;
  real_T c2_v_raw;
  real_T c2_gamma_raw;
  real_T c2_nargin = 3.0;
  real_T c2_nargout = 3.0;
  real_T c2_c_vehicle[3];
  int32_T c2_i23;
  real_T c2_x;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_d_x;
  real_T c2_f_x;
  real_T c2_g_x;
  real_T c2_y;
  real_T c2_h_x;
  real_T c2_b_y;
  real_T c2_i_x;
  real_T c2_B;
  real_T c2_c_y;
  real_T c2_d_y;
  real_T c2_f_y;
  int32_T c2_i24;
  real_T c2_j_x;
  real_T c2_k_x;
  real_T c2_b_B;
  real_T c2_g_y;
  real_T c2_h_y;
  real_T c2_i_y;
  real_T c2_j_y;
  real_T c2_l_x;
  real_T c2_m_x;
  real_T c2_n_x;
  real_T c2_o_x;
  real_T c2_p_x;
  real_T c2_q_x;
  real_T c2_r_x;
  real_T c2_s_x;
  real_T c2_A;
  real_T c2_c_B;
  real_T c2_t_x;
  real_T c2_k_y;
  real_T c2_u_x;
  real_T c2_l_y;
  real_T c2_v_x;
  real_T c2_m_y;
  real_T c2_n_y;
  real_T c2_w_x;
  real_T c2_x_x;
  real_T c2_y_x;
  real_T c2_ab_x;
  real_T c2_bb_x;
  real_T c2_cb_x;
  real_T c2_b_A;
  real_T c2_d_B;
  real_T c2_db_x;
  real_T c2_o_y;
  real_T c2_eb_x;
  real_T c2_p_y;
  real_T c2_fb_x;
  real_T c2_q_y;
  real_T c2_r_y;
  real_T c2_gb_x;
  real_T c2_hb_x;
  real_T c2_ib_x;
  real_T c2_jb_x;
  real_T c2_kb_x;
  real_T c2_lb_x;
  real_T c2_mb_x;
  real_T c2_nb_x;
  real_T c2_ob_x;
  real_T c2_pb_x;
  real_T c2_qb_x;
  real_T c2_rb_x;
  real_T c2_d1;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 28U, 28U, c2_c_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_e_theta, 0U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_prev_pt_pos_r_axle, 1U,
    c2_g_sf_marshallOut, c2_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_e_x, 2U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_e_y, 3U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_d, 4U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_e_RT, 5U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_r_ct, 6U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_v_t, 7U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_K_vec, 8U, c2_f_sf_marshallOut,
    c2_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_K_d, 9U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_K_l, 10U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_K_o, 11U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_K_x, 12U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_K_RT, 13U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_K_theta, 14U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_l_b, 15U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_c_c, 16U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_v_b, 17U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_v_raw, 18U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_gamma_raw, 19U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 20U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 21U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_sensors_in, 22U, c2_e_sf_marshallOut,
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_ctr_law, 23U, c2_d_sf_marshallOut,
    c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_vehicle, 24U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_v, 25U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_gamma, 26U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_ctr_vars, 27U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_EML_FCN(0, 1);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 53);
  c2_e_theta = -c2_b_sensors_in->rpsensor_out.hdg_err;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 56);
  c2_c_vehicle[0] = c2_b_vehicle.rear_axle_x;
  c2_c_vehicle[1] = 0.0;
  c2_c_vehicle[2] = 0.0;
  for (c2_i23 = 0; c2_i23 < 3; c2_i23++) {
    c2_prev_pt_pos_r_axle[c2_i23] =
      c2_b_sensors_in->rpsensor_out.preview_pt_pos_Fr1[c2_i23] -
      c2_c_vehicle[c2_i23];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 59);
  c2_e_x = c2_prev_pt_pos_r_axle[0];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 61);
  c2_e_y = c2_prev_pt_pos_r_axle[1];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 63);
  c2_x = c2_mpower(chartInstance, c2_e_x) + c2_mpower(chartInstance, c2_e_y);
  c2_d = c2_x;
  if (c2_d < 0.0) {
    c2_eml_error(chartInstance);
  }

  c2_b_x = c2_d;
  c2_d = c2_b_x;
  c2_d = muDoubleScalarSqrt(c2_d);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 71);
  c2_c_x = c2_e_theta;
  c2_d_x = c2_c_x;
  c2_d_x = muDoubleScalarTan(c2_d_x);
  c2_f_x = c2_e_theta;
  c2_g_x = c2_f_x;
  c2_g_x = muDoubleScalarTan(c2_g_x);
  c2_y = c2_e_x * c2_d_x - c2_e_y;
  c2_h_x = c2_e_x + c2_g_x * c2_e_y;
  c2_eml_scalar_eg(chartInstance);
  c2_dimagree(chartInstance);
  c2_b_y = c2_y;
  c2_i_x = c2_h_x;
  c2_e_RT = muDoubleScalarAtan2(c2_b_y, c2_i_x);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 74);
  c2_B = c2_b_sensors_in->rpsensor_out.curve_xy;
  c2_c_y = c2_B;
  c2_d_y = c2_c_y;
  c2_f_y = c2_d_y;
  c2_r_ct = 1.0 / c2_f_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 80);
  c2_v_t = c2_b_ctr_law->target_speed;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 88);
  for (c2_i24 = 0; c2_i24 < 6; c2_i24++) {
    c2_K_vec[c2_i24] = c2_b_ctr_law->K_vilca[c2_i24];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 90);
  c2_K_d = c2_K_vec[0];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 91);
  c2_K_l = c2_K_vec[1];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 92);
  c2_K_o = c2_K_vec[2];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 93);
  c2_K_x = c2_K_vec[3];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 94);
  c2_K_RT = c2_K_vec[4];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 95);
  c2_K_theta = c2_K_vec[5];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 98);
  c2_l_b = c2_b_vehicle.wheelbase;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 102);
  c2_j_x = c2_e_theta;
  c2_k_x = c2_j_x;
  c2_k_x = muDoubleScalarCos(c2_k_x);
  c2_b_B = c2_r_ct * c2_k_x;
  c2_g_y = c2_b_B;
  c2_h_y = c2_g_y;
  c2_i_y = c2_h_y;
  c2_j_y = 1.0 / c2_i_y;
  c2_l_x = c2_e_theta;
  c2_m_x = c2_l_x;
  c2_m_x = muDoubleScalarTan(c2_m_x);
  c2_n_x = c2_e_RT;
  c2_o_x = c2_n_x;
  c2_o_x = muDoubleScalarSin(c2_o_x);
  c2_p_x = c2_e_theta;
  c2_q_x = c2_p_x;
  c2_q_x = muDoubleScalarCos(c2_q_x);
  c2_r_x = c2_e_theta;
  c2_s_x = c2_r_x;
  c2_s_x = muDoubleScalarCos(c2_s_x);
  c2_A = c2_K_d * c2_e_y - c2_K_l * c2_d * c2_o_x * c2_q_x;
  c2_c_B = c2_K_o * c2_s_x;
  c2_t_x = c2_A;
  c2_k_y = c2_c_B;
  c2_u_x = c2_t_x;
  c2_l_y = c2_k_y;
  c2_v_x = c2_u_x;
  c2_m_y = c2_l_y;
  c2_n_y = c2_v_x / c2_m_y;
  c2_w_x = c2_e_RT;
  c2_x_x = c2_w_x;
  c2_x_x = muDoubleScalarSin(c2_x_x);
  c2_y_x = c2_e_theta;
  c2_ab_x = c2_y_x;
  c2_ab_x = muDoubleScalarSin(c2_ab_x);
  c2_bb_x = c2_e_theta;
  c2_cb_x = c2_bb_x;
  c2_cb_x = muDoubleScalarCos(c2_cb_x);
  c2_b_A = c2_K_RT * c2_mpower(chartInstance, c2_x_x);
  c2_d_B = c2_ab_x * c2_cb_x;
  c2_db_x = c2_b_A;
  c2_o_y = c2_d_B;
  c2_eb_x = c2_db_x;
  c2_p_y = c2_o_y;
  c2_fb_x = c2_eb_x;
  c2_q_y = c2_p_y;
  c2_r_y = c2_fb_x / c2_q_y;
  c2_c_c = ((c2_j_y + c2_K_theta * c2_m_x) + c2_n_y) + c2_r_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 108);
  c2_gb_x = c2_e_RT;
  c2_hb_x = c2_gb_x;
  c2_hb_x = muDoubleScalarSin(c2_hb_x);
  c2_ib_x = c2_e_theta;
  c2_jb_x = c2_ib_x;
  c2_jb_x = muDoubleScalarSin(c2_jb_x);
  c2_kb_x = c2_e_theta;
  c2_lb_x = c2_kb_x;
  c2_lb_x = muDoubleScalarSin(c2_lb_x);
  c2_v_b = c2_K_x * ((c2_K_d * c2_e_x + c2_K_l * c2_d * c2_hb_x * c2_jb_x) +
                     c2_K_o * c2_lb_x * c2_c_c);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 114);
  c2_b_ctr_vars[0] = c2_e_x;
  c2_b_ctr_vars[1] = c2_e_y;
  c2_b_ctr_vars[2] = c2_e_theta;
  c2_b_ctr_vars[3] = c2_e_RT;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 123);
  c2_mb_x = c2_e_theta;
  c2_nb_x = c2_mb_x;
  c2_nb_x = muDoubleScalarCos(c2_nb_x);
  c2_v_raw = c2_v_t * c2_nb_x + c2_v_b;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 126);
  c2_ob_x = c2_l_b * c2_c_c;
  c2_gamma_raw = c2_ob_x;
  c2_pb_x = c2_gamma_raw;
  c2_gamma_raw = c2_pb_x;
  c2_gamma_raw = muDoubleScalarAtan(c2_gamma_raw);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 130U);
  *c2_b_v = c2_clip_signal(chartInstance, c2_v_raw, c2_b_ctr_law->v_min,
    c2_b_ctr_law->v_max);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 131U);
  *c2_b_gamma = c2_clip_signal(chartInstance, c2_gamma_raw,
    -c2_b_ctr_law->gamma_max, c2_b_ctr_law->gamma_max);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 135U);
  guard1 = false;
  guard2 = false;
  if (CV_EML_COND(0, 1, 0, CV_RELATIONAL_EVAL(4U, 0U, 1, *c2_b_v, c2_v_raw, -1,
        1U, *c2_b_v != c2_v_raw))) {
    guard2 = true;
  } else if (CV_EML_COND(0, 1, 1, CV_RELATIONAL_EVAL(4U, 0U, 2, *c2_b_gamma,
               c2_gamma_raw, -1, 1U, *c2_b_gamma != c2_gamma_raw))) {
    guard2 = true;
  } else {
    c2_qb_x = c2_e_y;
    c2_rb_x = c2_qb_x;
    c2_d1 = muDoubleScalarAbs(c2_rb_x);
    if (CV_EML_COND(0, 1, 2, CV_RELATIONAL_EVAL(4U, 0U, 3, c2_d1, 0.1, -1, 4U,
          c2_d1 > 0.1))) {
      guard1 = true;
    } else {
      CV_EML_MCDC(0, 1, 0, false);
      CV_EML_IF(0, 1, 1, false);
    }
  }

  if (guard2 == true) {
    guard1 = true;
  }

  if (guard1 == true) {
    CV_EML_MCDC(0, 1, 0, true);
    CV_EML_IF(0, 1, 1, true);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 136U);
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -136);
  _SFD_SYMBOL_SCOPE_POP();
}

static real_T c2_mpower(SFc2_cm_testInstanceStruct *chartInstance, real_T c2_a)
{
  real_T c2_b_a;
  real_T c2_c_a;
  real_T c2_ak;
  real_T c2_d_a;
  c2_b_a = c2_a;
  c2_c_a = c2_b_a;
  c2_eml_scalar_eg(chartInstance);
  c2_dimagree(chartInstance);
  c2_ak = c2_c_a;
  c2_d_a = c2_ak;
  c2_eml_scalar_eg(chartInstance);
  return c2_d_a * c2_d_a;
}

static void c2_eml_scalar_eg(SFc2_cm_testInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_dimagree(SFc2_cm_testInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_eml_error(SFc2_cm_testInstanceStruct *chartInstance)
{
  const mxArray *c2_y = NULL;
  static char_T c2_u[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  const mxArray *c2_b_y = NULL;
  static char_T c2_b_u[4] = { 's', 'q', 'r', 't' };

  (void)chartInstance;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 30), false);
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c2_y, 14, c2_b_y));
}

static real_T c2_clip_signal(SFc2_cm_testInstanceStruct *chartInstance, real_T
  c2_x, real_T c2_lower_bnd, real_T c2_upper_bnd)
{
  real_T c2_x_clipped;
  uint32_T c2_debug_family_var_map[6];
  real_T c2_nargin = 3.0;
  real_T c2_nargout = 1.0;
  real_T c2_varargin_1;
  real_T c2_varargin_2;
  real_T c2_b_varargin_2;
  real_T c2_varargin_3;
  real_T c2_b_x;
  real_T c2_y;
  real_T c2_c_x;
  real_T c2_b_y;
  real_T c2_xk;
  real_T c2_yk;
  real_T c2_d_x;
  real_T c2_c_y;
  real_T c2_minval;
  real_T c2_b_varargin_1;
  real_T c2_c_varargin_2;
  real_T c2_d_varargin_2;
  real_T c2_b_varargin_3;
  real_T c2_e_x;
  real_T c2_d_y;
  real_T c2_f_x;
  real_T c2_e_y;
  real_T c2_b_xk;
  real_T c2_b_yk;
  real_T c2_g_x;
  real_T c2_f_y;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 6U, 6U, c2_b_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 0U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 1U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_x, 2U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_lower_bnd, 3U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_upper_bnd, 4U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_x_clipped, 5U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  CV_EML_FCN(0, 2);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 146U);
  c2_varargin_1 = c2_upper_bnd;
  c2_varargin_2 = c2_x;
  c2_b_varargin_2 = c2_varargin_1;
  c2_varargin_3 = c2_varargin_2;
  c2_b_x = c2_b_varargin_2;
  c2_y = c2_varargin_3;
  c2_c_x = c2_b_x;
  c2_b_y = c2_y;
  c2_eml_scalar_eg(chartInstance);
  c2_dimagree(chartInstance);
  c2_xk = c2_c_x;
  c2_yk = c2_b_y;
  c2_d_x = c2_xk;
  c2_c_y = c2_yk;
  c2_eml_scalar_eg(chartInstance);
  c2_minval = muDoubleScalarMin(c2_d_x, c2_c_y);
  c2_b_varargin_1 = c2_lower_bnd;
  c2_c_varargin_2 = c2_minval;
  c2_d_varargin_2 = c2_b_varargin_1;
  c2_b_varargin_3 = c2_c_varargin_2;
  c2_e_x = c2_d_varargin_2;
  c2_d_y = c2_b_varargin_3;
  c2_f_x = c2_e_x;
  c2_e_y = c2_d_y;
  c2_eml_scalar_eg(chartInstance);
  c2_dimagree(chartInstance);
  c2_b_xk = c2_f_x;
  c2_b_yk = c2_e_y;
  c2_g_x = c2_b_xk;
  c2_f_y = c2_b_yk;
  c2_eml_scalar_eg(chartInstance);
  c2_x_clipped = muDoubleScalarMax(c2_g_x, c2_f_y);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -146);
  _SFD_SYMBOL_SCOPE_POP();
  return c2_x_clipped;
}

static const mxArray *c2_h_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_cm_testInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(int32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static int32_T c2_p_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_y;
  int32_T c2_i25;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i25, 1, 6, 0U, 0, 0U, 0);
  c2_y = c2_i25;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y;
  SFc2_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_cm_testInstanceStruct *)chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_p_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_sensors_in_bus_io(void *chartInstanceVoid, void
  *c2_pData)
{
  const mxArray *c2_mxVal = NULL;
  int32_T c2_i26;
  c2_SensorsOut_bus c2_tmp;
  int32_T c2_i27;
  int32_T c2_i28;
  SFc2_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_cm_testInstanceStruct *)chartInstanceVoid;
  c2_mxVal = NULL;
  for (c2_i26 = 0; c2_i26 < 3; c2_i26++) {
    c2_tmp.imu_out.Omega_0[c2_i26] = ((real_T *)&((char_T *)(c2_imu_bus *)
      &((char_T *)(c2_SensorsOut_bus *)c2_pData)[0])[0])[c2_i26];
  }

  for (c2_i27 = 0; c2_i27 < 3; c2_i27++) {
    c2_tmp.imu_out.Alpha_0[c2_i27] = ((real_T *)&((char_T *)(c2_imu_bus *)
      &((char_T *)(c2_SensorsOut_bus *)c2_pData)[0])[24])[c2_i27];
  }

  c2_tmp.mag_out.hdg = *(real_T *)&((char_T *)(c2_mag_bus *)&((char_T *)
    (c2_SensorsOut_bus *)c2_pData)[48])[0];
  c2_tmp.rpsensor_out.lat_pos_ctr_road = *(real_T *)&((char_T *)(c2_RPSensor_bus
    *)&((char_T *)(c2_SensorsOut_bus *)c2_pData)[56])[0];
  c2_tmp.rpsensor_out.lat_pos_ctr_lane = *(real_T *)&((char_T *)(c2_RPSensor_bus
    *)&((char_T *)(c2_SensorsOut_bus *)c2_pData)[56])[8];
  c2_tmp.rpsensor_out.hdg_err = *(real_T *)&((char_T *)(c2_RPSensor_bus *)
    &((char_T *)(c2_SensorsOut_bus *)c2_pData)[56])[16];
  c2_tmp.rpsensor_out.path_hdg = *(real_T *)&((char_T *)(c2_RPSensor_bus *)
    &((char_T *)(c2_SensorsOut_bus *)c2_pData)[56])[24];
  c2_tmp.rpsensor_out.curve_xy = *(real_T *)&((char_T *)(c2_RPSensor_bus *)
    &((char_T *)(c2_SensorsOut_bus *)c2_pData)[56])[32];
  for (c2_i28 = 0; c2_i28 < 3; c2_i28++) {
    c2_tmp.rpsensor_out.preview_pt_pos_Fr1[c2_i28] = ((real_T *)&((char_T *)
      (c2_RPSensor_bus *)&((char_T *)(c2_SensorsOut_bus *)c2_pData)[56])[40])
      [c2_i28];
  }

  c2_tmp.steersensor_out.steer_angle_1 = *(real_T *)&((char_T *)
    (c2_SteerSensor_bus *)&((char_T *)(c2_SensorsOut_bus *)c2_pData)[120])[0];
  c2_tmp.steersensor_out.steer_angle_2 = *(real_T *)&((char_T *)
    (c2_SteerSensor_bus *)&((char_T *)(c2_SensorsOut_bus *)c2_pData)[120])[8];
  c2_tmp.steersensor_out.mean_angle = *(real_T *)&((char_T *)(c2_SteerSensor_bus
    *)&((char_T *)(c2_SensorsOut_bus *)c2_pData)[120])[16];
  c2_tmp.odo_out.vel_from_motor = *(real_T *)&((char_T *)(c2_OdoSensors_bus *)
    &((char_T *)(c2_SensorsOut_bus *)c2_pData)[144])[0];
  sf_mex_assign(&c2_mxVal, c2_e_sf_marshallOut(chartInstance, &c2_tmp), false);
  return c2_mxVal;
}

static const mxArray *c2_ctr_law_bus_io(void *chartInstanceVoid, void *c2_pData)
{
  const mxArray *c2_mxVal = NULL;
  c2_ctr_law_params_bus c2_tmp;
  int32_T c2_i29;
  SFc2_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_cm_testInstanceStruct *)chartInstanceVoid;
  c2_mxVal = NULL;
  c2_tmp.flag = *(int8_T *)&((char_T *)(c2_ctr_law_params_bus *)c2_pData)[0];
  for (c2_i29 = 0; c2_i29 < 6; c2_i29++) {
    c2_tmp.K_vilca[c2_i29] = ((real_T *)&((char_T *)(c2_ctr_law_params_bus *)
      c2_pData)[8])[c2_i29];
  }

  c2_tmp.target_speed = *(real_T *)&((char_T *)(c2_ctr_law_params_bus *)c2_pData)
    [56];
  c2_tmp.d_x_target = *(real_T *)&((char_T *)(c2_ctr_law_params_bus *)c2_pData)
    [64];
  c2_tmp.v_min = *(real_T *)&((char_T *)(c2_ctr_law_params_bus *)c2_pData)[72];
  c2_tmp.v_max = *(real_T *)&((char_T *)(c2_ctr_law_params_bus *)c2_pData)[80];
  c2_tmp.gamma_max = *(real_T *)&((char_T *)(c2_ctr_law_params_bus *)c2_pData)
    [88];
  c2_tmp.Fs = *(real_T *)&((char_T *)(c2_ctr_law_params_bus *)c2_pData)[96];
  sf_mex_assign(&c2_mxVal, c2_d_sf_marshallOut(chartInstance, &c2_tmp), false);
  return c2_mxVal;
}

static const mxArray *c2_vehicle_bus_io(void *chartInstanceVoid, void *c2_pData)
{
  const mxArray *c2_mxVal = NULL;
  c2_vhcl_bus c2_tmp;
  SFc2_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_cm_testInstanceStruct *)chartInstanceVoid;
  c2_mxVal = NULL;
  c2_tmp.wheelbase = *(real_T *)&((char_T *)(c2_vhcl_bus *)c2_pData)[0];
  c2_tmp.rear_axle_x = *(real_T *)&((char_T *)(c2_vhcl_bus *)c2_pData)[8];
  c2_tmp.wheel_radius = *(real_T *)&((char_T *)(c2_vhcl_bus *)c2_pData)[16];
  c2_tmp.mass = *(real_T *)&((char_T *)(c2_vhcl_bus *)c2_pData)[24];
  c2_tmp.gear_ratio = *(real_T *)&((char_T *)(c2_vhcl_bus *)c2_pData)[32];
  sf_mex_assign(&c2_mxVal, c2_c_sf_marshallOut(chartInstance, &c2_tmp), false);
  return c2_mxVal;
}

static uint8_T c2_q_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_b_is_active_c2_cm_test, const char_T *c2_identifier)
{
  uint8_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_r_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_cm_test), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_cm_test);
  return c2_y;
}

static uint8_T c2_r_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_y;
  uint8_T c2_u0;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u0, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void init_dsm_address_info(SFc2_cm_testInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc2_cm_testInstanceStruct *chartInstance)
{
  chartInstance->c2_sensors_in = (c2_SensorsOut_bus *)
    ssGetInputPortSignal_wrapper(chartInstance->S, 0);
  chartInstance->c2_v = (real_T *)ssGetOutputPortSignal_wrapper(chartInstance->S,
    1);
  chartInstance->c2_ctr_law = (c2_ctr_law_params_bus *)
    ssGetInputPortSignal_wrapper(chartInstance->S, 1);
  chartInstance->c2_vehicle = (c2_vhcl_bus *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c2_gamma = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c2_ctr_vars = (real_T (*)[4])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 3);
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

void sf_c2_cm_test_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3438848891U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1574620362U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1330603114U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(890015759U);
}

mxArray* sf_c2_cm_test_get_post_codegen_info(void);
mxArray *sf_c2_cm_test_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("PwgvIY11J4nSTh1IUgWweD");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,3,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(13));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
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
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(13));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
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
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(13));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,3,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
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
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo = sf_c2_cm_test_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c2_cm_test_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c2_cm_test_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "hiddenFallbackType", "hiddenFallbackReason", "incompatibleSymbol" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 5, infoFields);
  mxArray *fallbackType = mxCreateString("pre");
  mxArray *fallbackReason = mxCreateString("hasBreakpoints");
  mxArray *hiddenFallbackType = mxCreateString("none");
  mxArray *hiddenFallbackReason = mxCreateString("");
  mxArray *incompatibleSymbol = mxCreateString("");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], hiddenFallbackType);
  mxSetField(mxInfo, 0, infoFields[3], hiddenFallbackReason);
  mxSetField(mxInfo, 0, infoFields[4], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c2_cm_test_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c2_cm_test_get_post_codegen_info(void)
{
  const char* fieldNames[] = { "exportedFunctionsUsedByThisChart",
    "exportedFunctionsChecksum" };

  mwSize dims[2] = { 1, 1 };

  mxArray* mxPostCodegenInfo = mxCreateStructArray(2, dims, sizeof(fieldNames)/
    sizeof(fieldNames[0]), fieldNames);

  {
    mxArray* mxExportedFunctionsChecksum = mxCreateString("");
    mwSize exp_dims[2] = { 0, 1 };

    mxArray* mxExportedFunctionsUsedByThisChart = mxCreateCellArray(2, exp_dims);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsUsedByThisChart",
               mxExportedFunctionsUsedByThisChart);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsChecksum",
               mxExportedFunctionsChecksum);
  }

  return mxPostCodegenInfo;
}

static const mxArray *sf_get_sim_state_info_c2_cm_test(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x4'type','srcId','name','auxInfo'{{M[1],M[9],T\"ctr_vars\",},{M[1],M[8],T\"gamma\",},{M[1],M[5],T\"v\",},{M[8],M[0],T\"is_active_c2_cm_test\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 4, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_cm_test_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_cm_testInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc2_cm_testInstanceStruct *) chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _cm_testMachineNumber_,
           2,
           1,
           1,
           0,
           6,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize its own list of scripts */
        init_script_number_translation(_cm_testMachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_cm_testMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _cm_testMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"sensors_in");
          _SFD_SET_DATA_PROPS(1,1,1,0,"ctr_law");
          _SFD_SET_DATA_PROPS(2,1,1,0,"vehicle");
          _SFD_SET_DATA_PROPS(3,2,0,1,"v");
          _SFD_SET_DATA_PROPS(4,2,0,1,"gamma");
          _SFD_SET_DATA_PROPS(5,2,0,1,"ctr_vars");
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
        _SFD_CV_INIT_EML(0,1,3,0,2,0,0,0,0,0,3,1);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,723);
        _SFD_CV_INIT_EML_FCN(0,1,"VilcaController",726,-1,3460);
        _SFD_CV_INIT_EML_FCN(0,2,"clip_signal",3463,-1,3578);
        _SFD_CV_INIT_EML_IF(0,1,0,559,579,654,715);
        _SFD_CV_INIT_EML_IF(0,1,1,3384,3444,-1,3455);

        {
          static int condStart[] = { 3388, 3404, 3428 };

          static int condEnd[] = { 3398, 3422, 3443 };

          static int pfixExpr[] = { 0, 1, -2, 2, -2 };

          _SFD_CV_INIT_EML_MCDC(0,1,0,3387,3444,3,0,&(condStart[0]),&(condEnd[0]),
                                5,&(pfixExpr[0]));
        }

        _SFD_CV_INIT_EML_RELATIONAL(0,1,0,562,579,-1,0);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,1,3388,3398,-1,1);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,2,3404,3422,-1,1);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,3,3428,3443,-1,4);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sensors_in_bus_io,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_ctr_law_bus_io,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_vehicle_bus_io,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)c2_b_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)c2_b_sf_marshallIn);

        {
          unsigned int dimVector[1];
          dimVector[0]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)
            c2_sf_marshallIn);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _cm_testMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static void chart_debug_initialize_data_addresses(SimStruct *S)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_cm_testInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc2_cm_testInstanceStruct *) chartInfo->chartInstance;
    if (ssIsFirstInitCond(S)) {
      /* do this only if simulation is starting and after we know the addresses of all data */
      {
        _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c2_sensors_in);
        _SFD_SET_DATA_VALUE_PTR(3U, chartInstance->c2_v);
        _SFD_SET_DATA_VALUE_PTR(1U, chartInstance->c2_ctr_law);
        _SFD_SET_DATA_VALUE_PTR(2U, chartInstance->c2_vehicle);
        _SFD_SET_DATA_VALUE_PTR(4U, chartInstance->c2_gamma);
        _SFD_SET_DATA_VALUE_PTR(5U, *chartInstance->c2_ctr_vars);
      }
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "s9rIFacB4mPGwSzqRE6TgHH";
}

static void sf_opaque_initialize_c2_cm_test(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_cm_testInstanceStruct*) chartInstanceVar)->S,
    0);
  initialize_params_c2_cm_test((SFc2_cm_testInstanceStruct*) chartInstanceVar);
  initialize_c2_cm_test((SFc2_cm_testInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c2_cm_test(void *chartInstanceVar)
{
  enable_c2_cm_test((SFc2_cm_testInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c2_cm_test(void *chartInstanceVar)
{
  disable_c2_cm_test((SFc2_cm_testInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c2_cm_test(void *chartInstanceVar)
{
  sf_gateway_c2_cm_test((SFc2_cm_testInstanceStruct*) chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c2_cm_test(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  return get_sim_state_c2_cm_test((SFc2_cm_testInstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c2_cm_test(SimStruct* S, const mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  set_sim_state_c2_cm_test((SFc2_cm_testInstanceStruct*)chartInfo->chartInstance,
    st);
}

static void sf_opaque_terminate_c2_cm_test(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_cm_testInstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_cm_test_optimization_info();
    }

    finalize_c2_cm_test((SFc2_cm_testInstanceStruct*) chartInstanceVar);
    utFree(chartInstanceVar);
    if (crtInfo != NULL) {
      utFree(crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_cm_test((SFc2_cm_testInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_cm_test(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    initialize_params_c2_cm_test((SFc2_cm_testInstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c2_cm_test(SimStruct *S)
{
  ssMdlUpdateIsEmpty(S, 1);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_cm_test_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,1);
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,2,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,2);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,3);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,3);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=3; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 3; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(4283410382U));
  ssSetChecksum1(S,(3888588838U));
  ssSetChecksum2(S,(4080135374U));
  ssSetChecksum3(S,(2395056739U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c2_cm_test(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_cm_test(SimStruct *S)
{
  SFc2_cm_testInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc2_cm_testInstanceStruct *)utMalloc(sizeof
    (SFc2_cm_testInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc2_cm_testInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c2_cm_test;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c2_cm_test;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c2_cm_test;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c2_cm_test;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c2_cm_test;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c2_cm_test;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c2_cm_test;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c2_cm_test;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_cm_test;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_cm_test;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c2_cm_test;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.callAtomicSubchartUserFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartAutoFcn = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  crtInfo->isEnhancedMooreMachine = 0;
  crtInfo->checksum = SF_RUNTIME_INFO_CHECKSUM;
  crtInfo->fCheckOverflow = sf_runtime_overflow_check_is_on(S);
  crtInfo->instanceInfo = (&(chartInstance->chartInfo));
  crtInfo->isJITEnabled = false;
  crtInfo->compiledInfo = NULL;
  ssSetUserData(S,(void *)(crtInfo));  /* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  chart_debug_initialization(S,1);
}

void c2_cm_test_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_cm_test(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_cm_test(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_cm_test(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_cm_test_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
