/* Include files */

#include <stddef.h>
#include "blas.h"
#include "cm_test_sfun.h"
#include "c1_cm_test.h"
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
static const char * c1_debug_family_names[29] = { "v_ref", "gamma_ref", "Fs",
  "v", "R_wheel", "mass", "torque_to_gas", "torque_to_brake", "Ts", "a_ref_raw",
  "a_ref", "torque_ref", "torque_mot_ref", "gas_torque", "gas_out",
  "gas_out_clipped", "brake_torque", "brake_out", "brake_out_clipped", "gas",
  "brake", "nargin", "nargout", "sensors_in", "refs", "steer_flag", "low_ctr",
  "vhcl", "y" };

static const char * c1_b_debug_family_names[6] = { "nargin", "nargout", "x",
  "lower_bnd", "upper_bnd", "x_clipped" };

/* Function Declarations */
static void initialize_c1_cm_test(SFc1_cm_testInstanceStruct *chartInstance);
static void initialize_params_c1_cm_test(SFc1_cm_testInstanceStruct
  *chartInstance);
static void enable_c1_cm_test(SFc1_cm_testInstanceStruct *chartInstance);
static void disable_c1_cm_test(SFc1_cm_testInstanceStruct *chartInstance);
static void c1_update_debugger_state_c1_cm_test(SFc1_cm_testInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c1_cm_test(SFc1_cm_testInstanceStruct
  *chartInstance);
static void set_sim_state_c1_cm_test(SFc1_cm_testInstanceStruct *chartInstance,
  const mxArray *c1_st);
static void finalize_c1_cm_test(SFc1_cm_testInstanceStruct *chartInstance);
static void sf_gateway_c1_cm_test(SFc1_cm_testInstanceStruct *chartInstance);
static void mdl_start_c1_cm_test(SFc1_cm_testInstanceStruct *chartInstance);
static void c1_chartstep_c1_cm_test(SFc1_cm_testInstanceStruct *chartInstance);
static void initSimStructsc1_cm_test(SFc1_cm_testInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber, uint32_T c1_instanceNumber);
static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData);
static void c1_emlrt_marshallIn(SFc1_cm_testInstanceStruct *chartInstance, const
  mxArray *c1_b_y, const char_T *c1_identifier, real_T c1_c_y[2]);
static void c1_b_emlrt_marshallIn(SFc1_cm_testInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId, real_T c1_b_y[2]);
static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static const mxArray *c1_e_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real_T c1_c_emlrt_marshallIn(SFc1_cm_testInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static real_T c1_rdivide(SFc1_cm_testInstanceStruct *chartInstance, real_T c1_x,
  real_T c1_b_y);
static real_T c1_clip_signal(SFc1_cm_testInstanceStruct *chartInstance, real_T
  c1_x, real_T c1_lower_bnd, real_T c1_upper_bnd);
static void c1_eml_scalar_eg(SFc1_cm_testInstanceStruct *chartInstance);
static void c1_dimagree(SFc1_cm_testInstanceStruct *chartInstance);
static const mxArray *c1_f_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static int32_T c1_d_emlrt_marshallIn(SFc1_cm_testInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_sensors_in_bus_io(void *chartInstanceVoid, void
  *c1_pData);
static const mxArray *c1_low_ctr_bus_io(void *chartInstanceVoid, void *c1_pData);
static const mxArray *c1_vhcl_bus_io(void *chartInstanceVoid, void *c1_pData);
static uint8_T c1_e_emlrt_marshallIn(SFc1_cm_testInstanceStruct *chartInstance,
  const mxArray *c1_b_is_active_c1_cm_test, const char_T *c1_identifier);
static uint8_T c1_f_emlrt_marshallIn(SFc1_cm_testInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void init_dsm_address_info(SFc1_cm_testInstanceStruct *chartInstance);
static void init_simulink_io_address(SFc1_cm_testInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c1_cm_test(SFc1_cm_testInstanceStruct *chartInstance)
{
  if (sf_is_first_init_cond(chartInstance->S)) {
    initSimStructsc1_cm_test(chartInstance);
    chart_debug_initialize_data_addresses(chartInstance->S);
  }

  chartInstance->c1_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c1_is_active_c1_cm_test = 0U;
}

static void initialize_params_c1_cm_test(SFc1_cm_testInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void enable_c1_cm_test(SFc1_cm_testInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c1_cm_test(SFc1_cm_testInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c1_update_debugger_state_c1_cm_test(SFc1_cm_testInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c1_cm_test(SFc1_cm_testInstanceStruct
  *chartInstance)
{
  const mxArray *c1_st;
  const mxArray *c1_b_y = NULL;
  const mxArray *c1_c_y = NULL;
  uint8_T c1_hoistedGlobal;
  uint8_T c1_u;
  const mxArray *c1_d_y = NULL;
  c1_st = NULL;
  c1_st = NULL;
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_createcellmatrix(2, 1), false);
  c1_c_y = NULL;
  sf_mex_assign(&c1_c_y, sf_mex_create("y", *chartInstance->c1_y, 0, 0U, 1U, 0U,
    1, 2), false);
  sf_mex_setcell(c1_b_y, 0, c1_c_y);
  c1_hoistedGlobal = chartInstance->c1_is_active_c1_cm_test;
  c1_u = c1_hoistedGlobal;
  c1_d_y = NULL;
  sf_mex_assign(&c1_d_y, sf_mex_create("y", &c1_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_b_y, 1, c1_d_y);
  sf_mex_assign(&c1_st, c1_b_y, false);
  return c1_st;
}

static void set_sim_state_c1_cm_test(SFc1_cm_testInstanceStruct *chartInstance,
  const mxArray *c1_st)
{
  const mxArray *c1_u;
  real_T c1_dv0[2];
  int32_T c1_i0;
  chartInstance->c1_doneDoubleBufferReInit = true;
  c1_u = sf_mex_dup(c1_st);
  c1_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell("y", c1_u, 0)),
                      "y", c1_dv0);
  for (c1_i0 = 0; c1_i0 < 2; c1_i0++) {
    (*chartInstance->c1_y)[c1_i0] = c1_dv0[c1_i0];
  }

  chartInstance->c1_is_active_c1_cm_test = c1_e_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell("is_active_c1_cm_test", c1_u, 1)),
    "is_active_c1_cm_test");
  sf_mex_destroy(&c1_u);
  c1_update_debugger_state_c1_cm_test(chartInstance);
  sf_mex_destroy(&c1_st);
}

static void finalize_c1_cm_test(SFc1_cm_testInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c1_cm_test(SFc1_cm_testInstanceStruct *chartInstance)
{
  int32_T c1_i1;
  int32_T c1_i2;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_steer_flag, 2U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  for (c1_i1 = 0; c1_i1 < 2; c1_i1++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c1_refs)[c1_i1], 1U, 1U, 0U,
                          chartInstance->c1_sfEvent, false);
  }

  chartInstance->c1_sfEvent = CALL_EVENT;
  c1_chartstep_c1_cm_test(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_cm_testMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c1_i2 = 0; c1_i2 < 2; c1_i2++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c1_y)[c1_i2], 5U, 1U, 0U,
                          chartInstance->c1_sfEvent, false);
  }
}

static void mdl_start_c1_cm_test(SFc1_cm_testInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c1_chartstep_c1_cm_test(SFc1_cm_testInstanceStruct *chartInstance)
{
  real_T c1_hoistedGlobal;
  int32_T c1_i3;
  c1_SensorsOut_bus c1_b_sensors_in;
  int32_T c1_i4;
  int32_T c1_i5;
  int32_T c1_i6;
  real_T c1_b_refs[2];
  real_T c1_b_steer_flag;
  c1_low_ctr_params_bus c1_b_low_ctr;
  int32_T c1_i7;
  c1_vhcl_bus c1_b_vhcl;
  uint32_T c1_debug_family_var_map[29];
  real_T c1_v_ref;
  real_T c1_gamma_ref;
  real_T c1_Fs;
  real_T c1_v;
  real_T c1_R_wheel;
  real_T c1_mass;
  real_T c1_torque_to_gas;
  real_T c1_torque_to_brake[2];
  real_T c1_Ts;
  real_T c1_a_ref_raw;
  real_T c1_a_ref;
  real_T c1_torque_ref;
  real_T c1_torque_mot_ref;
  real_T c1_gas_torque;
  real_T c1_gas_out;
  real_T c1_gas_out_clipped;
  real_T c1_brake_torque;
  real_T c1_brake_out;
  real_T c1_brake_out_clipped;
  real_T c1_gas;
  real_T c1_brake;
  real_T c1_nargin = 5.0;
  real_T c1_nargout = 1.0;
  real_T c1_b_y[2];
  int32_T c1_i8;
  real_T c1_b[2];
  int32_T c1_i9;
  int32_T c1_i10;
  real_T c1_B;
  real_T c1_A;
  real_T c1_b_B;
  real_T c1_b_A;
  real_T c1_c_B;
  int32_T c1_i11;
  real_T c1_c_y;
  int32_T c1_i12;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  c1_hoistedGlobal = *chartInstance->c1_steer_flag;
  for (c1_i3 = 0; c1_i3 < 3; c1_i3++) {
    c1_b_sensors_in.imu_out.Omega_0[c1_i3] = ((real_T *)&((char_T *)(c1_imu_bus *)
      &((char_T *)chartInstance->c1_sensors_in)[0])[0])[c1_i3];
  }

  for (c1_i4 = 0; c1_i4 < 3; c1_i4++) {
    c1_b_sensors_in.imu_out.Alpha_0[c1_i4] = ((real_T *)&((char_T *)(c1_imu_bus *)
      &((char_T *)chartInstance->c1_sensors_in)[0])[24])[c1_i4];
  }

  c1_b_sensors_in.mag_out.hdg = *(real_T *)&((char_T *)(c1_mag_bus *)&((char_T *)
    chartInstance->c1_sensors_in)[48])[0];
  c1_b_sensors_in.rpsensor_out.lat_pos_ctr_road = *(real_T *)&((char_T *)
    (c1_RPSensor_bus *)&((char_T *)chartInstance->c1_sensors_in)[56])[0];
  c1_b_sensors_in.rpsensor_out.lat_pos_ctr_lane = *(real_T *)&((char_T *)
    (c1_RPSensor_bus *)&((char_T *)chartInstance->c1_sensors_in)[56])[8];
  c1_b_sensors_in.rpsensor_out.hdg_err = *(real_T *)&((char_T *)(c1_RPSensor_bus
    *)&((char_T *)chartInstance->c1_sensors_in)[56])[16];
  c1_b_sensors_in.rpsensor_out.path_hdg = *(real_T *)&((char_T *)
    (c1_RPSensor_bus *)&((char_T *)chartInstance->c1_sensors_in)[56])[24];
  c1_b_sensors_in.rpsensor_out.curve_xy = *(real_T *)&((char_T *)
    (c1_RPSensor_bus *)&((char_T *)chartInstance->c1_sensors_in)[56])[32];
  for (c1_i5 = 0; c1_i5 < 3; c1_i5++) {
    c1_b_sensors_in.rpsensor_out.preview_pt_pos_Fr1[c1_i5] = ((real_T *)
      &((char_T *)(c1_RPSensor_bus *)&((char_T *)chartInstance->c1_sensors_in)
        [56])[40])[c1_i5];
  }

  c1_b_sensors_in.steersensor_out.steer_angle_1 = *(real_T *)&((char_T *)
    (c1_SteerSensor_bus *)&((char_T *)chartInstance->c1_sensors_in)[120])[0];
  c1_b_sensors_in.steersensor_out.steer_angle_2 = *(real_T *)&((char_T *)
    (c1_SteerSensor_bus *)&((char_T *)chartInstance->c1_sensors_in)[120])[8];
  c1_b_sensors_in.steersensor_out.mean_angle = *(real_T *)&((char_T *)
    (c1_SteerSensor_bus *)&((char_T *)chartInstance->c1_sensors_in)[120])[16];
  c1_b_sensors_in.odo_out.vel_from_motor = *(real_T *)&((char_T *)
    (c1_OdoSensors_bus *)&((char_T *)chartInstance->c1_sensors_in)[144])[0];
  for (c1_i6 = 0; c1_i6 < 2; c1_i6++) {
    c1_b_refs[c1_i6] = (*chartInstance->c1_refs)[c1_i6];
  }

  c1_b_steer_flag = c1_hoistedGlobal;
  c1_b_low_ctr.Fs = *(real_T *)&((char_T *)chartInstance->c1_low_ctr)[0];
  c1_b_low_ctr.K_accel = *(real_T *)&((char_T *)chartInstance->c1_low_ctr)[8];
  c1_b_low_ctr.max_accel = *(real_T *)&((char_T *)chartInstance->c1_low_ctr)[16];
  c1_b_low_ctr.trq_full = *(real_T *)&((char_T *)chartInstance->c1_low_ctr)[24];
  c1_b_low_ctr.trq_zero = *(real_T *)&((char_T *)chartInstance->c1_low_ctr)[32];
  for (c1_i7 = 0; c1_i7 < 2; c1_i7++) {
    c1_b_low_ctr.brake_to_Trq[c1_i7] = ((real_T *)&((char_T *)
      chartInstance->c1_low_ctr)[40])[c1_i7];
  }

  c1_b_vhcl.wheelbase = *(real_T *)&((char_T *)chartInstance->c1_vhcl)[0];
  c1_b_vhcl.rear_axle_x = *(real_T *)&((char_T *)chartInstance->c1_vhcl)[8];
  c1_b_vhcl.wheel_radius = *(real_T *)&((char_T *)chartInstance->c1_vhcl)[16];
  c1_b_vhcl.mass = *(real_T *)&((char_T *)chartInstance->c1_vhcl)[24];
  c1_b_vhcl.gear_ratio = *(real_T *)&((char_T *)chartInstance->c1_vhcl)[32];
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 29U, 29U, c1_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_v_ref, 0U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_gamma_ref, 1U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_Fs, 2U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_v, 3U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_R_wheel, 4U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_mass, 5U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_torque_to_gas, 6U,
    c1_d_sf_marshallOut, c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_torque_to_brake, 7U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_Ts, 8U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_a_ref_raw, 9U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_a_ref, 10U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_torque_ref, 11U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_torque_mot_ref, 12U,
    c1_d_sf_marshallOut, c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_gas_torque, 13U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_gas_out, 14U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_gas_out_clipped, 15U,
    c1_d_sf_marshallOut, c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_brake_torque, 16U,
    c1_d_sf_marshallOut, c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_brake_out, 17U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_brake_out_clipped, 18U,
    c1_d_sf_marshallOut, c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_gas, 19U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_brake, 20U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 21U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 22U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_sensors_in, 23U, c1_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c1_b_refs, 24U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_steer_flag, 25U, c1_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_low_ctr, 26U, c1_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_vhcl, 27U, c1_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_b_y, 28U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 23);
  c1_v_ref = c1_b_refs[0];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 24);
  c1_gamma_ref = c1_b_refs[1];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 26);
  c1_Fs = c1_b_low_ctr.Fs;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 32);
  c1_v = c1_b_sensors_in.odo_out.vel_from_motor;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 35);
  c1_R_wheel = c1_b_vhcl.wheel_radius;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 38);
  c1_mass = c1_b_vhcl.mass;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 41);
  c1_torque_to_gas = c1_rdivide(chartInstance, 1.0, c1_b_low_ctr.trq_full -
    c1_b_low_ctr.trq_zero);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 47);
  for (c1_i8 = 0; c1_i8 < 2; c1_i8++) {
    c1_b[c1_i8] = c1_b_low_ctr.brake_to_Trq[c1_i8];
  }

  for (c1_i9 = 0; c1_i9 < 2; c1_i9++) {
    c1_b[c1_i9] *= 2.0;
  }

  for (c1_i10 = 0; c1_i10 < 2; c1_i10++) {
    c1_torque_to_brake[c1_i10] = 1.0 / c1_b[c1_i10];
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 55);
  c1_B = c1_Fs;
  c1_Ts = c1_rdivide(chartInstance, 1.0, c1_B);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 65);
  c1_A = c1_b_low_ctr.K_accel * (c1_v_ref - c1_v);
  c1_b_B = c1_Ts;
  c1_a_ref_raw = c1_rdivide(chartInstance, c1_A, c1_b_B);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 68);
  c1_a_ref = c1_clip_signal(chartInstance, c1_a_ref_raw, -c1_b_low_ctr.max_accel,
    c1_b_low_ctr.max_accel);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 88);
  c1_torque_ref = c1_mass * c1_R_wheel * c1_a_ref;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 92);
  c1_b_A = c1_torque_ref;
  c1_c_B = c1_b_vhcl.gear_ratio;
  c1_torque_mot_ref = c1_rdivide(chartInstance, c1_b_A, c1_c_B);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 94);
  c1_gas_torque = c1_clip_signal(chartInstance, c1_torque_mot_ref, 0.0, rtInf);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 96);
  c1_gas_out = c1_gas_torque * c1_torque_to_gas;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 98);
  c1_gas_out_clipped = c1_clip_signal(chartInstance, c1_gas_out, 0.0, 1.0);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 103);
  c1_brake_torque = c1_clip_signal(chartInstance, -c1_torque_ref, 0.0, rtInf);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 106);
  for (c1_i11 = 0; c1_i11 < 2; c1_i11++) {
    c1_b[c1_i11] = c1_torque_to_brake[c1_i11];
  }

  c1_c_y = c1_b[0];
  c1_c_y += c1_b[1];
  c1_brake_out = c1_brake_torque * c1_c_y;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 108);
  c1_brake_out_clipped = c1_clip_signal(chartInstance, c1_brake_out, 0.0, 1.0);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 115);
  c1_gas = c1_gas_out_clipped;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 116);
  c1_brake = c1_brake_out_clipped;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 119);
  c1_b_y[0] = c1_gas;
  c1_b_y[1] = c1_brake;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, -119);
  _SFD_SYMBOL_SCOPE_POP();
  for (c1_i12 = 0; c1_i12 < 2; c1_i12++) {
    (*chartInstance->c1_y)[c1_i12] = c1_b_y[c1_i12];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
}

static void initSimStructsc1_cm_test(SFc1_cm_testInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber, uint32_T c1_instanceNumber)
{
  (void)c1_machineNumber;
  (void)c1_chartNumber;
  (void)c1_instanceNumber;
}

static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i13;
  real_T c1_u[2];
  const mxArray *c1_b_y = NULL;
  SFc1_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc1_cm_testInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  for (c1_i13 = 0; c1_i13 < 2; c1_i13++) {
    c1_u[c1_i13] = (*(real_T (*)[2])c1_inData)[c1_i13];
  }

  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 1, 2), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_b_y, false);
  return c1_mxArrayOutData;
}

static void c1_emlrt_marshallIn(SFc1_cm_testInstanceStruct *chartInstance, const
  mxArray *c1_b_y, const char_T *c1_identifier, real_T c1_c_y[2])
{
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_y), &c1_thisId, c1_c_y);
  sf_mex_destroy(&c1_b_y);
}

static void c1_b_emlrt_marshallIn(SFc1_cm_testInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId, real_T c1_b_y[2])
{
  real_T c1_dv1[2];
  int32_T c1_i14;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_dv1, 1, 0, 0U, 1, 0U, 1, 2);
  for (c1_i14 = 0; c1_i14 < 2; c1_i14++) {
    c1_b_y[c1_i14] = c1_dv1[c1_i14];
  }

  sf_mex_destroy(&c1_u);
}

static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_y;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_c_y[2];
  int32_T c1_i15;
  SFc1_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc1_cm_testInstanceStruct *)chartInstanceVoid;
  c1_b_y = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_y), &c1_thisId, c1_c_y);
  sf_mex_destroy(&c1_b_y);
  for (c1_i15 = 0; c1_i15 < 2; c1_i15++) {
    (*(real_T (*)[2])c1_outData)[c1_i15] = c1_c_y[c1_i15];
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData;
  c1_vhcl_bus c1_u;
  const mxArray *c1_b_y = NULL;
  real_T c1_b_u;
  const mxArray *c1_c_y = NULL;
  real_T c1_c_u;
  const mxArray *c1_d_y = NULL;
  real_T c1_d_u;
  const mxArray *c1_e_y = NULL;
  real_T c1_e_u;
  const mxArray *c1_f_y = NULL;
  real_T c1_f_u;
  const mxArray *c1_g_y = NULL;
  SFc1_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc1_cm_testInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_mxArrayOutData = NULL;
  c1_u = *(c1_vhcl_bus *)c1_inData;
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c1_b_u = c1_u.wheelbase;
  c1_c_y = NULL;
  sf_mex_assign(&c1_c_y, sf_mex_create("y", &c1_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c1_b_y, c1_c_y, "wheelbase", "wheelbase", 0);
  c1_c_u = c1_u.rear_axle_x;
  c1_d_y = NULL;
  sf_mex_assign(&c1_d_y, sf_mex_create("y", &c1_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c1_b_y, c1_d_y, "rear_axle_x", "rear_axle_x", 0);
  c1_d_u = c1_u.wheel_radius;
  c1_e_y = NULL;
  sf_mex_assign(&c1_e_y, sf_mex_create("y", &c1_d_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c1_b_y, c1_e_y, "wheel_radius", "wheel_radius", 0);
  c1_e_u = c1_u.mass;
  c1_f_y = NULL;
  sf_mex_assign(&c1_f_y, sf_mex_create("y", &c1_e_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c1_b_y, c1_f_y, "mass", "mass", 0);
  c1_f_u = c1_u.gear_ratio;
  c1_g_y = NULL;
  sf_mex_assign(&c1_g_y, sf_mex_create("y", &c1_f_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c1_b_y, c1_g_y, "gear_ratio", "gear_ratio", 0);
  sf_mex_assign(&c1_mxArrayOutData, c1_b_y, false);
  return c1_mxArrayOutData;
}

static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData;
  c1_low_ctr_params_bus c1_u;
  const mxArray *c1_b_y = NULL;
  real_T c1_b_u;
  const mxArray *c1_c_y = NULL;
  real_T c1_c_u;
  const mxArray *c1_d_y = NULL;
  real_T c1_d_u;
  const mxArray *c1_e_y = NULL;
  real_T c1_e_u;
  const mxArray *c1_f_y = NULL;
  real_T c1_f_u;
  const mxArray *c1_g_y = NULL;
  int32_T c1_i16;
  real_T c1_g_u[2];
  const mxArray *c1_h_y = NULL;
  SFc1_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc1_cm_testInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_mxArrayOutData = NULL;
  c1_u = *(c1_low_ctr_params_bus *)c1_inData;
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c1_b_u = c1_u.Fs;
  c1_c_y = NULL;
  sf_mex_assign(&c1_c_y, sf_mex_create("y", &c1_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c1_b_y, c1_c_y, "Fs", "Fs", 0);
  c1_c_u = c1_u.K_accel;
  c1_d_y = NULL;
  sf_mex_assign(&c1_d_y, sf_mex_create("y", &c1_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c1_b_y, c1_d_y, "K_accel", "K_accel", 0);
  c1_d_u = c1_u.max_accel;
  c1_e_y = NULL;
  sf_mex_assign(&c1_e_y, sf_mex_create("y", &c1_d_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c1_b_y, c1_e_y, "max_accel", "max_accel", 0);
  c1_e_u = c1_u.trq_full;
  c1_f_y = NULL;
  sf_mex_assign(&c1_f_y, sf_mex_create("y", &c1_e_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c1_b_y, c1_f_y, "trq_full", "trq_full", 0);
  c1_f_u = c1_u.trq_zero;
  c1_g_y = NULL;
  sf_mex_assign(&c1_g_y, sf_mex_create("y", &c1_f_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c1_b_y, c1_g_y, "trq_zero", "trq_zero", 0);
  for (c1_i16 = 0; c1_i16 < 2; c1_i16++) {
    c1_g_u[c1_i16] = c1_u.brake_to_Trq[c1_i16];
  }

  c1_h_y = NULL;
  sf_mex_assign(&c1_h_y, sf_mex_create("y", c1_g_u, 0, 0U, 1U, 0U, 1, 2), false);
  sf_mex_addfield(c1_b_y, c1_h_y, "brake_to_Trq", "brake_to_Trq", 0);
  sf_mex_assign(&c1_mxArrayOutData, c1_b_y, false);
  return c1_mxArrayOutData;
}

static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real_T c1_u;
  const mxArray *c1_b_y = NULL;
  SFc1_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc1_cm_testInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_b_y, false);
  return c1_mxArrayOutData;
}

static const mxArray *c1_e_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData;
  c1_SensorsOut_bus c1_u;
  const mxArray *c1_b_y = NULL;
  c1_imu_bus c1_b_u;
  const mxArray *c1_c_y = NULL;
  int32_T c1_i17;
  real_T c1_c_u[3];
  const mxArray *c1_d_y = NULL;
  int32_T c1_i18;
  const mxArray *c1_e_y = NULL;
  c1_mag_bus c1_d_u;
  const mxArray *c1_f_y = NULL;
  real_T c1_e_u;
  const mxArray *c1_g_y = NULL;
  c1_RPSensor_bus c1_f_u;
  const mxArray *c1_h_y = NULL;
  real_T c1_g_u;
  const mxArray *c1_i_y = NULL;
  real_T c1_h_u;
  const mxArray *c1_j_y = NULL;
  real_T c1_i_u;
  const mxArray *c1_k_y = NULL;
  real_T c1_j_u;
  const mxArray *c1_l_y = NULL;
  real_T c1_k_u;
  const mxArray *c1_m_y = NULL;
  int32_T c1_i19;
  const mxArray *c1_n_y = NULL;
  c1_SteerSensor_bus c1_l_u;
  const mxArray *c1_o_y = NULL;
  real_T c1_m_u;
  const mxArray *c1_p_y = NULL;
  real_T c1_n_u;
  const mxArray *c1_q_y = NULL;
  real_T c1_o_u;
  const mxArray *c1_r_y = NULL;
  c1_OdoSensors_bus c1_p_u;
  const mxArray *c1_s_y = NULL;
  real_T c1_q_u;
  const mxArray *c1_t_y = NULL;
  SFc1_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc1_cm_testInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_mxArrayOutData = NULL;
  c1_u = *(c1_SensorsOut_bus *)c1_inData;
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c1_b_u = c1_u.imu_out;
  c1_c_y = NULL;
  sf_mex_assign(&c1_c_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  for (c1_i17 = 0; c1_i17 < 3; c1_i17++) {
    c1_c_u[c1_i17] = c1_b_u.Omega_0[c1_i17];
  }

  c1_d_y = NULL;
  sf_mex_assign(&c1_d_y, sf_mex_create("y", c1_c_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_addfield(c1_c_y, c1_d_y, "Omega_0", "Omega_0", 0);
  for (c1_i18 = 0; c1_i18 < 3; c1_i18++) {
    c1_c_u[c1_i18] = c1_b_u.Alpha_0[c1_i18];
  }

  c1_e_y = NULL;
  sf_mex_assign(&c1_e_y, sf_mex_create("y", c1_c_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_addfield(c1_c_y, c1_e_y, "Alpha_0", "Alpha_0", 0);
  sf_mex_addfield(c1_b_y, c1_c_y, "imu_out", "imu_out", 0);
  c1_d_u = c1_u.mag_out;
  c1_f_y = NULL;
  sf_mex_assign(&c1_f_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c1_e_u = c1_d_u.hdg;
  c1_g_y = NULL;
  sf_mex_assign(&c1_g_y, sf_mex_create("y", &c1_e_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c1_f_y, c1_g_y, "hdg", "hdg", 0);
  sf_mex_addfield(c1_b_y, c1_f_y, "mag_out", "mag_out", 0);
  c1_f_u = c1_u.rpsensor_out;
  c1_h_y = NULL;
  sf_mex_assign(&c1_h_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c1_g_u = c1_f_u.lat_pos_ctr_road;
  c1_i_y = NULL;
  sf_mex_assign(&c1_i_y, sf_mex_create("y", &c1_g_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c1_h_y, c1_i_y, "lat_pos_ctr_road", "lat_pos_ctr_road", 0);
  c1_h_u = c1_f_u.lat_pos_ctr_lane;
  c1_j_y = NULL;
  sf_mex_assign(&c1_j_y, sf_mex_create("y", &c1_h_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c1_h_y, c1_j_y, "lat_pos_ctr_lane", "lat_pos_ctr_lane", 0);
  c1_i_u = c1_f_u.hdg_err;
  c1_k_y = NULL;
  sf_mex_assign(&c1_k_y, sf_mex_create("y", &c1_i_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c1_h_y, c1_k_y, "hdg_err", "hdg_err", 0);
  c1_j_u = c1_f_u.path_hdg;
  c1_l_y = NULL;
  sf_mex_assign(&c1_l_y, sf_mex_create("y", &c1_j_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c1_h_y, c1_l_y, "path_hdg", "path_hdg", 0);
  c1_k_u = c1_f_u.curve_xy;
  c1_m_y = NULL;
  sf_mex_assign(&c1_m_y, sf_mex_create("y", &c1_k_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c1_h_y, c1_m_y, "curve_xy", "curve_xy", 0);
  for (c1_i19 = 0; c1_i19 < 3; c1_i19++) {
    c1_c_u[c1_i19] = c1_f_u.preview_pt_pos_Fr1[c1_i19];
  }

  c1_n_y = NULL;
  sf_mex_assign(&c1_n_y, sf_mex_create("y", c1_c_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_addfield(c1_h_y, c1_n_y, "preview_pt_pos_Fr1", "preview_pt_pos_Fr1", 0);
  sf_mex_addfield(c1_b_y, c1_h_y, "rpsensor_out", "rpsensor_out", 0);
  c1_l_u = c1_u.steersensor_out;
  c1_o_y = NULL;
  sf_mex_assign(&c1_o_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c1_m_u = c1_l_u.steer_angle_1;
  c1_p_y = NULL;
  sf_mex_assign(&c1_p_y, sf_mex_create("y", &c1_m_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c1_o_y, c1_p_y, "steer_angle_1", "steer_angle_1", 0);
  c1_n_u = c1_l_u.steer_angle_2;
  c1_q_y = NULL;
  sf_mex_assign(&c1_q_y, sf_mex_create("y", &c1_n_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c1_o_y, c1_q_y, "steer_angle_2", "steer_angle_2", 0);
  c1_o_u = c1_l_u.mean_angle;
  c1_r_y = NULL;
  sf_mex_assign(&c1_r_y, sf_mex_create("y", &c1_o_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c1_o_y, c1_r_y, "mean_angle", "mean_angle", 0);
  sf_mex_addfield(c1_b_y, c1_o_y, "steersensor_out", "steersensor_out", 0);
  c1_p_u = c1_u.odo_out;
  c1_s_y = NULL;
  sf_mex_assign(&c1_s_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c1_q_u = c1_p_u.vel_from_motor;
  c1_t_y = NULL;
  sf_mex_assign(&c1_t_y, sf_mex_create("y", &c1_q_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c1_s_y, c1_t_y, "vel_from_motor", "vel_from_motor", 0);
  sf_mex_addfield(c1_b_y, c1_s_y, "odo_out", "odo_out", 0);
  sf_mex_assign(&c1_mxArrayOutData, c1_b_y, false);
  return c1_mxArrayOutData;
}

static real_T c1_c_emlrt_marshallIn(SFc1_cm_testInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_b_y;
  real_T c1_d0;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d0, 1, 0, 0U, 0, 0U, 0);
  c1_b_y = c1_d0;
  sf_mex_destroy(&c1_u);
  return c1_b_y;
}

static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_nargout;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_b_y;
  SFc1_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc1_cm_testInstanceStruct *)chartInstanceVoid;
  c1_nargout = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_b_y = c1_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_nargout),
    &c1_thisId);
  sf_mex_destroy(&c1_nargout);
  *(real_T *)c1_outData = c1_b_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

const mxArray *sf_c1_cm_test_get_eml_resolved_functions_info(void)
{
  const mxArray *c1_nameCaptureInfo = NULL;
  c1_nameCaptureInfo = NULL;
  sf_mex_assign(&c1_nameCaptureInfo, sf_mex_create("nameCaptureInfo", NULL, 0,
    0U, 1U, 0U, 2, 0, 1), false);
  return c1_nameCaptureInfo;
}

static real_T c1_rdivide(SFc1_cm_testInstanceStruct *chartInstance, real_T c1_x,
  real_T c1_b_y)
{
  real_T c1_b_x;
  real_T c1_c_y;
  real_T c1_c_x;
  real_T c1_d_y;
  (void)chartInstance;
  c1_b_x = c1_x;
  c1_c_y = c1_b_y;
  c1_c_x = c1_b_x;
  c1_d_y = c1_c_y;
  return c1_c_x / c1_d_y;
}

static real_T c1_clip_signal(SFc1_cm_testInstanceStruct *chartInstance, real_T
  c1_x, real_T c1_lower_bnd, real_T c1_upper_bnd)
{
  real_T c1_x_clipped;
  uint32_T c1_debug_family_var_map[6];
  real_T c1_nargin = 3.0;
  real_T c1_nargout = 1.0;
  real_T c1_varargin_1;
  real_T c1_varargin_2;
  real_T c1_b_varargin_2;
  real_T c1_varargin_3;
  real_T c1_b_x;
  real_T c1_b_y;
  real_T c1_c_x;
  real_T c1_c_y;
  real_T c1_xk;
  real_T c1_yk;
  real_T c1_d_x;
  real_T c1_d_y;
  real_T c1_minval;
  real_T c1_b_varargin_1;
  real_T c1_c_varargin_2;
  real_T c1_d_varargin_2;
  real_T c1_b_varargin_3;
  real_T c1_e_x;
  real_T c1_e_y;
  real_T c1_f_x;
  real_T c1_f_y;
  real_T c1_b_xk;
  real_T c1_b_yk;
  real_T c1_g_x;
  real_T c1_g_y;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 6U, 6U, c1_b_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 0U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 1U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_x, 2U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_lower_bnd, 3U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_upper_bnd, 4U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_x_clipped, 5U, c1_d_sf_marshallOut,
    c1_b_sf_marshallIn);
  CV_EML_FCN(0, 1);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 133U);
  c1_varargin_1 = c1_upper_bnd;
  c1_varargin_2 = c1_x;
  c1_b_varargin_2 = c1_varargin_1;
  c1_varargin_3 = c1_varargin_2;
  c1_b_x = c1_b_varargin_2;
  c1_b_y = c1_varargin_3;
  c1_c_x = c1_b_x;
  c1_c_y = c1_b_y;
  c1_eml_scalar_eg(chartInstance);
  c1_dimagree(chartInstance);
  c1_xk = c1_c_x;
  c1_yk = c1_c_y;
  c1_d_x = c1_xk;
  c1_d_y = c1_yk;
  c1_eml_scalar_eg(chartInstance);
  c1_minval = muDoubleScalarMin(c1_d_x, c1_d_y);
  c1_b_varargin_1 = c1_lower_bnd;
  c1_c_varargin_2 = c1_minval;
  c1_d_varargin_2 = c1_b_varargin_1;
  c1_b_varargin_3 = c1_c_varargin_2;
  c1_e_x = c1_d_varargin_2;
  c1_e_y = c1_b_varargin_3;
  c1_f_x = c1_e_x;
  c1_f_y = c1_e_y;
  c1_eml_scalar_eg(chartInstance);
  c1_dimagree(chartInstance);
  c1_b_xk = c1_f_x;
  c1_b_yk = c1_f_y;
  c1_g_x = c1_b_xk;
  c1_g_y = c1_b_yk;
  c1_eml_scalar_eg(chartInstance);
  c1_x_clipped = muDoubleScalarMax(c1_g_x, c1_g_y);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, -133);
  _SFD_SYMBOL_SCOPE_POP();
  return c1_x_clipped;
}

static void c1_eml_scalar_eg(SFc1_cm_testInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c1_dimagree(SFc1_cm_testInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *c1_f_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_u;
  const mxArray *c1_b_y = NULL;
  SFc1_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc1_cm_testInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(int32_T *)c1_inData;
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", &c1_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_b_y, false);
  return c1_mxArrayOutData;
}

static int32_T c1_d_emlrt_marshallIn(SFc1_cm_testInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  int32_T c1_b_y;
  int32_T c1_i20;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_i20, 1, 6, 0U, 0, 0U, 0);
  c1_b_y = c1_i20;
  sf_mex_destroy(&c1_u);
  return c1_b_y;
}

static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_sfEvent;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  int32_T c1_b_y;
  SFc1_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc1_cm_testInstanceStruct *)chartInstanceVoid;
  c1_b_sfEvent = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_b_y = c1_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_sfEvent),
    &c1_thisId);
  sf_mex_destroy(&c1_b_sfEvent);
  *(int32_T *)c1_outData = c1_b_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_sensors_in_bus_io(void *chartInstanceVoid, void
  *c1_pData)
{
  const mxArray *c1_mxVal = NULL;
  int32_T c1_i21;
  c1_SensorsOut_bus c1_tmp;
  int32_T c1_i22;
  int32_T c1_i23;
  SFc1_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc1_cm_testInstanceStruct *)chartInstanceVoid;
  c1_mxVal = NULL;
  for (c1_i21 = 0; c1_i21 < 3; c1_i21++) {
    c1_tmp.imu_out.Omega_0[c1_i21] = ((real_T *)&((char_T *)(c1_imu_bus *)
      &((char_T *)(c1_SensorsOut_bus *)c1_pData)[0])[0])[c1_i21];
  }

  for (c1_i22 = 0; c1_i22 < 3; c1_i22++) {
    c1_tmp.imu_out.Alpha_0[c1_i22] = ((real_T *)&((char_T *)(c1_imu_bus *)
      &((char_T *)(c1_SensorsOut_bus *)c1_pData)[0])[24])[c1_i22];
  }

  c1_tmp.mag_out.hdg = *(real_T *)&((char_T *)(c1_mag_bus *)&((char_T *)
    (c1_SensorsOut_bus *)c1_pData)[48])[0];
  c1_tmp.rpsensor_out.lat_pos_ctr_road = *(real_T *)&((char_T *)(c1_RPSensor_bus
    *)&((char_T *)(c1_SensorsOut_bus *)c1_pData)[56])[0];
  c1_tmp.rpsensor_out.lat_pos_ctr_lane = *(real_T *)&((char_T *)(c1_RPSensor_bus
    *)&((char_T *)(c1_SensorsOut_bus *)c1_pData)[56])[8];
  c1_tmp.rpsensor_out.hdg_err = *(real_T *)&((char_T *)(c1_RPSensor_bus *)
    &((char_T *)(c1_SensorsOut_bus *)c1_pData)[56])[16];
  c1_tmp.rpsensor_out.path_hdg = *(real_T *)&((char_T *)(c1_RPSensor_bus *)
    &((char_T *)(c1_SensorsOut_bus *)c1_pData)[56])[24];
  c1_tmp.rpsensor_out.curve_xy = *(real_T *)&((char_T *)(c1_RPSensor_bus *)
    &((char_T *)(c1_SensorsOut_bus *)c1_pData)[56])[32];
  for (c1_i23 = 0; c1_i23 < 3; c1_i23++) {
    c1_tmp.rpsensor_out.preview_pt_pos_Fr1[c1_i23] = ((real_T *)&((char_T *)
      (c1_RPSensor_bus *)&((char_T *)(c1_SensorsOut_bus *)c1_pData)[56])[40])
      [c1_i23];
  }

  c1_tmp.steersensor_out.steer_angle_1 = *(real_T *)&((char_T *)
    (c1_SteerSensor_bus *)&((char_T *)(c1_SensorsOut_bus *)c1_pData)[120])[0];
  c1_tmp.steersensor_out.steer_angle_2 = *(real_T *)&((char_T *)
    (c1_SteerSensor_bus *)&((char_T *)(c1_SensorsOut_bus *)c1_pData)[120])[8];
  c1_tmp.steersensor_out.mean_angle = *(real_T *)&((char_T *)(c1_SteerSensor_bus
    *)&((char_T *)(c1_SensorsOut_bus *)c1_pData)[120])[16];
  c1_tmp.odo_out.vel_from_motor = *(real_T *)&((char_T *)(c1_OdoSensors_bus *)
    &((char_T *)(c1_SensorsOut_bus *)c1_pData)[144])[0];
  sf_mex_assign(&c1_mxVal, c1_e_sf_marshallOut(chartInstance, &c1_tmp), false);
  return c1_mxVal;
}

static const mxArray *c1_low_ctr_bus_io(void *chartInstanceVoid, void *c1_pData)
{
  const mxArray *c1_mxVal = NULL;
  c1_low_ctr_params_bus c1_tmp;
  int32_T c1_i24;
  SFc1_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc1_cm_testInstanceStruct *)chartInstanceVoid;
  c1_mxVal = NULL;
  c1_tmp.Fs = *(real_T *)&((char_T *)(c1_low_ctr_params_bus *)c1_pData)[0];
  c1_tmp.K_accel = *(real_T *)&((char_T *)(c1_low_ctr_params_bus *)c1_pData)[8];
  c1_tmp.max_accel = *(real_T *)&((char_T *)(c1_low_ctr_params_bus *)c1_pData)
    [16];
  c1_tmp.trq_full = *(real_T *)&((char_T *)(c1_low_ctr_params_bus *)c1_pData)[24];
  c1_tmp.trq_zero = *(real_T *)&((char_T *)(c1_low_ctr_params_bus *)c1_pData)[32];
  for (c1_i24 = 0; c1_i24 < 2; c1_i24++) {
    c1_tmp.brake_to_Trq[c1_i24] = ((real_T *)&((char_T *)(c1_low_ctr_params_bus *)
      c1_pData)[40])[c1_i24];
  }

  sf_mex_assign(&c1_mxVal, c1_c_sf_marshallOut(chartInstance, &c1_tmp), false);
  return c1_mxVal;
}

static const mxArray *c1_vhcl_bus_io(void *chartInstanceVoid, void *c1_pData)
{
  const mxArray *c1_mxVal = NULL;
  c1_vhcl_bus c1_tmp;
  SFc1_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc1_cm_testInstanceStruct *)chartInstanceVoid;
  c1_mxVal = NULL;
  c1_tmp.wheelbase = *(real_T *)&((char_T *)(c1_vhcl_bus *)c1_pData)[0];
  c1_tmp.rear_axle_x = *(real_T *)&((char_T *)(c1_vhcl_bus *)c1_pData)[8];
  c1_tmp.wheel_radius = *(real_T *)&((char_T *)(c1_vhcl_bus *)c1_pData)[16];
  c1_tmp.mass = *(real_T *)&((char_T *)(c1_vhcl_bus *)c1_pData)[24];
  c1_tmp.gear_ratio = *(real_T *)&((char_T *)(c1_vhcl_bus *)c1_pData)[32];
  sf_mex_assign(&c1_mxVal, c1_b_sf_marshallOut(chartInstance, &c1_tmp), false);
  return c1_mxVal;
}

static uint8_T c1_e_emlrt_marshallIn(SFc1_cm_testInstanceStruct *chartInstance,
  const mxArray *c1_b_is_active_c1_cm_test, const char_T *c1_identifier)
{
  uint8_T c1_b_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_b_y = c1_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c1_b_is_active_c1_cm_test), &c1_thisId);
  sf_mex_destroy(&c1_b_is_active_c1_cm_test);
  return c1_b_y;
}

static uint8_T c1_f_emlrt_marshallIn(SFc1_cm_testInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  uint8_T c1_b_y;
  uint8_T c1_u0;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_u0, 1, 3, 0U, 0, 0U, 0);
  c1_b_y = c1_u0;
  sf_mex_destroy(&c1_u);
  return c1_b_y;
}

static void init_dsm_address_info(SFc1_cm_testInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc1_cm_testInstanceStruct *chartInstance)
{
  chartInstance->c1_sensors_in = (c1_SensorsOut_bus *)
    ssGetInputPortSignal_wrapper(chartInstance->S, 0);
  chartInstance->c1_refs = (real_T (*)[2])ssGetInputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c1_y = (real_T (*)[2])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c1_steer_flag = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c1_low_ctr = (c1_low_ctr_params_bus *)
    ssGetInputPortSignal_wrapper(chartInstance->S, 3);
  chartInstance->c1_vhcl = (c1_vhcl_bus *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 4);
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

void sf_c1_cm_test_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2585213699U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(90211706U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2819998924U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3928438576U);
}

mxArray* sf_c1_cm_test_get_post_codegen_info(void);
mxArray *sf_c1_cm_test_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("CT5Xptuj90fsjj6wtETseG");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
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
      pr[0] = (double)(2);
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
      pr[0] = (double)(1);
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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(13));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
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
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(13));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));
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
      pr[0] = (double)(2);
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo = sf_c1_cm_test_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c1_cm_test_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c1_cm_test_jit_fallback_info(void)
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

mxArray *sf_c1_cm_test_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c1_cm_test_get_post_codegen_info(void)
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

static const mxArray *sf_get_sim_state_info_c1_cm_test(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"y\",},{M[8],M[0],T\"is_active_c1_cm_test\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c1_cm_test_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_cm_testInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc1_cm_testInstanceStruct *) chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _cm_testMachineNumber_,
           1,
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
          _SFD_SET_DATA_PROPS(1,1,1,0,"refs");
          _SFD_SET_DATA_PROPS(2,1,1,0,"steer_flag");
          _SFD_SET_DATA_PROPS(3,1,1,0,"low_ctr");
          _SFD_SET_DATA_PROPS(4,1,1,0,"vhcl");
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
        _SFD_CV_INIT_EML(0,1,2,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,3208);
        _SFD_CV_INIT_EML_FCN(0,1,"clip_signal",3212,-1,3327);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sensors_in_bus_io,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_d_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_low_ctr_bus_io,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_vhcl_bus_io,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)
            c1_sf_marshallIn);
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
    SFc1_cm_testInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc1_cm_testInstanceStruct *) chartInfo->chartInstance;
    if (ssIsFirstInitCond(S)) {
      /* do this only if simulation is starting and after we know the addresses of all data */
      {
        _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c1_sensors_in);
        _SFD_SET_DATA_VALUE_PTR(1U, *chartInstance->c1_refs);
        _SFD_SET_DATA_VALUE_PTR(5U, *chartInstance->c1_y);
        _SFD_SET_DATA_VALUE_PTR(2U, chartInstance->c1_steer_flag);
        _SFD_SET_DATA_VALUE_PTR(3U, chartInstance->c1_low_ctr);
        _SFD_SET_DATA_VALUE_PTR(4U, chartInstance->c1_vhcl);
      }
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "sB6CEOsg7UCrksmQaLFIr6";
}

static void sf_opaque_initialize_c1_cm_test(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc1_cm_testInstanceStruct*) chartInstanceVar)->S,
    0);
  initialize_params_c1_cm_test((SFc1_cm_testInstanceStruct*) chartInstanceVar);
  initialize_c1_cm_test((SFc1_cm_testInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c1_cm_test(void *chartInstanceVar)
{
  enable_c1_cm_test((SFc1_cm_testInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c1_cm_test(void *chartInstanceVar)
{
  disable_c1_cm_test((SFc1_cm_testInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c1_cm_test(void *chartInstanceVar)
{
  sf_gateway_c1_cm_test((SFc1_cm_testInstanceStruct*) chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c1_cm_test(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  return get_sim_state_c1_cm_test((SFc1_cm_testInstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c1_cm_test(SimStruct* S, const mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  set_sim_state_c1_cm_test((SFc1_cm_testInstanceStruct*)chartInfo->chartInstance,
    st);
}

static void sf_opaque_terminate_c1_cm_test(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc1_cm_testInstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_cm_test_optimization_info();
    }

    finalize_c1_cm_test((SFc1_cm_testInstanceStruct*) chartInstanceVar);
    utFree(chartInstanceVar);
    if (crtInfo != NULL) {
      utFree(crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc1_cm_test((SFc1_cm_testInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c1_cm_test(SimStruct *S)
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
    initialize_params_c1_cm_test((SFc1_cm_testInstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c1_cm_test(SimStruct *S)
{
  ssMdlUpdateIsEmpty(S, 1);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_cm_test_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,1);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,1);
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,1,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,1);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,1,5);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,1,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 5; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,1);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1761827192U));
  ssSetChecksum1(S,(1824167908U));
  ssSetChecksum2(S,(2046430722U));
  ssSetChecksum3(S,(1378118735U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c1_cm_test(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c1_cm_test(SimStruct *S)
{
  SFc1_cm_testInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc1_cm_testInstanceStruct *)utMalloc(sizeof
    (SFc1_cm_testInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc1_cm_testInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c1_cm_test;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c1_cm_test;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c1_cm_test;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c1_cm_test;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c1_cm_test;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c1_cm_test;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c1_cm_test;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c1_cm_test;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c1_cm_test;
  chartInstance->chartInfo.mdlStart = mdlStart_c1_cm_test;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c1_cm_test;
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

void c1_cm_test_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c1_cm_test(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c1_cm_test(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c1_cm_test(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c1_cm_test_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
