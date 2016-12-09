/* Include files */

#include <stddef.h>
#include "blas.h"
#include "cm_test_sfun.h"
#include "c2_cm_test.h"
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
static const char * c2_debug_family_names[5] = { "nargin", "nargout",
  "sensors_in", "ctr_law", "y" };

static const char * c2_b_debug_family_names[8] = { "a", "e_theta", "e_y", "e_RT",
  "b", "nargin", "nargout", "sensors_in" };

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
static real_T c2_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_b_y, const char_T *c2_identifier);
static real_T c2_b_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_c_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, c2_SensorsOut_bus *
  c2_b_y);
static c2_imu_bus c2_d_emlrt_marshallIn(SFc2_cm_testInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_e_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_b_y[3]);
static c2_RPSensor_bus c2_f_emlrt_marshallIn(SFc2_cm_testInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static c2_SteerSensor_bus c2_g_emlrt_marshallIn(SFc2_cm_testInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_h_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_sensors_in_bus_io(void *chartInstanceVoid, void
  *c2_pData);
static const mxArray *c2_ctr_law_bus_io(void *chartInstanceVoid, void *c2_pData);
static uint8_T c2_i_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_b_is_active_c2_cm_test, const char_T *c2_identifier);
static uint8_T c2_j_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
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
  const mxArray *c2_b_y = NULL;
  real_T c2_hoistedGlobal;
  real_T c2_u;
  const mxArray *c2_c_y = NULL;
  uint8_T c2_b_hoistedGlobal;
  uint8_T c2_b_u;
  const mxArray *c2_d_y = NULL;
  c2_st = NULL;
  c2_st = NULL;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_createcellmatrix(2, 1), false);
  c2_hoistedGlobal = *chartInstance->c2_y;
  c2_u = c2_hoistedGlobal;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_b_y, 0, c2_c_y);
  c2_b_hoistedGlobal = chartInstance->c2_is_active_c2_cm_test;
  c2_b_u = c2_b_hoistedGlobal;
  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_b_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_b_y, 1, c2_d_y);
  sf_mex_assign(&c2_st, c2_b_y, false);
  return c2_st;
}

static void set_sim_state_c2_cm_test(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_st)
{
  const mxArray *c2_u;
  chartInstance->c2_doneDoubleBufferReInit = true;
  c2_u = sf_mex_dup(c2_st);
  *chartInstance->c2_y = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("y", c2_u, 0)), "y");
  chartInstance->c2_is_active_c2_cm_test = c2_i_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell("is_active_c2_cm_test", c2_u, 1)),
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
  int32_T c2_i0;
  c2_SensorsOut_bus c2_b_sensors_in;
  int32_T c2_i1;
  c2_ctr_law_params_bus c2_b_ctr_law;
  int32_T c2_i2;
  uint32_T c2_debug_family_var_map[5];
  real_T c2_nargin = 2.0;
  real_T c2_nargout = 1.0;
  real_T c2_b_y;
  c2_SensorsOut_bus c2_c_sensors_in;
  uint32_T c2_b_debug_family_var_map[8];
  real_T c2_a;
  real_T c2_e_theta;
  real_T c2_e_y;
  real_T c2_e_RT;
  real_T c2_b;
  real_T c2_b_nargin = 2.0;
  real_T c2_b_nargout = 1.0;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  chartInstance->c2_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  for (c2_i0 = 0; c2_i0 < 3; c2_i0++) {
    c2_b_sensors_in.imu_out.Omega_0[c2_i0] = ((real_T *)&((char_T *)(c2_imu_bus *)
      &((char_T *)chartInstance->c2_sensors_in)[0])[0])[c2_i0];
  }

  for (c2_i1 = 0; c2_i1 < 3; c2_i1++) {
    c2_b_sensors_in.imu_out.Alpha_0[c2_i1] = ((real_T *)&((char_T *)(c2_imu_bus *)
      &((char_T *)chartInstance->c2_sensors_in)[0])[24])[c2_i1];
  }

  c2_b_sensors_in.rpsensor_out.lat_pos_ctr_road = *(real_T *)&((char_T *)
    (c2_RPSensor_bus *)&((char_T *)chartInstance->c2_sensors_in)[48])[0];
  c2_b_sensors_in.rpsensor_out.lat_pos_ctr_lane = *(real_T *)&((char_T *)
    (c2_RPSensor_bus *)&((char_T *)chartInstance->c2_sensors_in)[48])[8];
  c2_b_sensors_in.rpsensor_out.hdg_err = *(real_T *)&((char_T *)(c2_RPSensor_bus
    *)&((char_T *)chartInstance->c2_sensors_in)[48])[16];
  c2_b_sensors_in.rpsensor_out.path_hdg = *(real_T *)&((char_T *)
    (c2_RPSensor_bus *)&((char_T *)chartInstance->c2_sensors_in)[48])[24];
  c2_b_sensors_in.rpsensor_out.curve_xy = *(real_T *)&((char_T *)
    (c2_RPSensor_bus *)&((char_T *)chartInstance->c2_sensors_in)[48])[32];
  c2_b_sensors_in.steersensor_out.steer_angle_1 = *(real_T *)&((char_T *)
    (c2_SteerSensor_bus *)&((char_T *)chartInstance->c2_sensors_in)[88])[0];
  c2_b_sensors_in.steersensor_out.steer_angle_2 = *(real_T *)&((char_T *)
    (c2_SteerSensor_bus *)&((char_T *)chartInstance->c2_sensors_in)[88])[8];
  c2_b_sensors_in.steersensor_out.mean_angle = *(real_T *)&((char_T *)
    (c2_SteerSensor_bus *)&((char_T *)chartInstance->c2_sensors_in)[88])[16];
  c2_b_ctr_law.flag = *(int8_T *)&((char_T *)chartInstance->c2_ctr_law)[0];
  for (c2_i2 = 0; c2_i2 < 6; c2_i2++) {
    c2_b_ctr_law.K_vilca[c2_i2] = ((real_T *)&((char_T *)
      chartInstance->c2_ctr_law)[8])[c2_i2];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 5U, 5U, c2_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 0U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 1U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_b_sensors_in, 2U, c2_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_b_ctr_law, 3U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_y, 4U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 28);
  if (CV_EML_IF(0, 1, 0, CV_RELATIONAL_EVAL(4U, 0U, 0, (real_T)c2_b_ctr_law.flag,
        1.0, -1, 0U, (real_T)c2_b_ctr_law.flag == 1.0))) {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 29);
    c2_c_sensors_in = c2_b_sensors_in;
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 8U, 8U, c2_b_debug_family_names,
      c2_b_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_a, 0U, c2_sf_marshallOut,
      c2_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_e_theta, 1U, c2_sf_marshallOut,
      c2_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_e_y, 2U, c2_sf_marshallOut,
      c2_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_e_RT, 3U, c2_sf_marshallOut,
      c2_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b, 4U, c2_sf_marshallOut,
      c2_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_nargin, 5U, c2_sf_marshallOut,
      c2_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_nargout, 6U, c2_sf_marshallOut,
      c2_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_c_sensors_in, 7U,
      c2_c_sf_marshallOut, c2_b_sf_marshallIn);
    CV_EML_FCN(0, 1);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 46);
    c2_e_theta = c2_c_sensors_in.rpsensor_out.hdg_err;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 48);
    c2_e_y = c2_c_sensors_in.rpsensor_out.lat_pos_ctr_lane;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 51);
    c2_e_RT = c2_c_sensors_in.rpsensor_out.path_hdg;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 54);
    c2_a = 0.0;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 55);
    c2_b = 0.0;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -55);
    _SFD_SYMBOL_SCOPE_POP();
    c2_b_y = 0.0;
  } else {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 31);
    c2_b_y = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -31);
  _SFD_SYMBOL_SCOPE_POP();
  *chartInstance->c2_y = c2_b_y;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_cm_testMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_y, 2U, 1U, 0U,
                        chartInstance->c2_sfEvent, false);
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
  real_T c2_u;
  const mxArray *c2_b_y = NULL;
  SFc2_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_cm_testInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_b_y, false);
  return c2_mxArrayOutData;
}

static real_T c2_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_b_y, const char_T *c2_identifier)
{
  real_T c2_c_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_c_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_y), &c2_thisId);
  sf_mex_destroy(&c2_b_y);
  return c2_c_y;
}

static real_T c2_b_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_b_y;
  real_T c2_d0;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d0, 1, 0, 0U, 0, 0U, 0);
  c2_b_y = c2_d0;
  sf_mex_destroy(&c2_u);
  return c2_b_y;
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_y;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_c_y;
  SFc2_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_cm_testInstanceStruct *)chartInstanceVoid;
  c2_b_y = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_c_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_y), &c2_thisId);
  sf_mex_destroy(&c2_b_y);
  *(real_T *)c2_outData = c2_c_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData;
  c2_ctr_law_params_bus c2_u;
  const mxArray *c2_b_y = NULL;
  int8_T c2_b_u;
  const mxArray *c2_c_y = NULL;
  int32_T c2_i3;
  real_T c2_c_u[6];
  const mxArray *c2_d_y = NULL;
  SFc2_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_cm_testInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  c2_u = *(c2_ctr_law_params_bus *)c2_inData;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_b_u = c2_u.flag;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_b_u, 2, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_b_y, c2_c_y, "flag", "flag", 0);
  for (c2_i3 = 0; c2_i3 < 6; c2_i3++) {
    c2_c_u[c2_i3] = c2_u.K_vilca[c2_i3];
  }

  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", c2_c_u, 0, 0U, 1U, 0U, 1, 6), false);
  sf_mex_addfield(c2_b_y, c2_d_y, "K_vilca", "K_vilca", 0);
  sf_mex_assign(&c2_mxArrayOutData, c2_b_y, false);
  return c2_mxArrayOutData;
}

static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData;
  c2_SensorsOut_bus c2_u;
  const mxArray *c2_b_y = NULL;
  c2_imu_bus c2_b_u;
  const mxArray *c2_c_y = NULL;
  int32_T c2_i4;
  real_T c2_c_u[3];
  const mxArray *c2_d_y = NULL;
  int32_T c2_i5;
  const mxArray *c2_e_y = NULL;
  c2_RPSensor_bus c2_d_u;
  const mxArray *c2_f_y = NULL;
  real_T c2_e_u;
  const mxArray *c2_g_y = NULL;
  real_T c2_f_u;
  const mxArray *c2_h_y = NULL;
  real_T c2_g_u;
  const mxArray *c2_i_y = NULL;
  real_T c2_h_u;
  const mxArray *c2_j_y = NULL;
  real_T c2_i_u;
  const mxArray *c2_k_y = NULL;
  c2_SteerSensor_bus c2_j_u;
  const mxArray *c2_l_y = NULL;
  real_T c2_k_u;
  const mxArray *c2_m_y = NULL;
  real_T c2_l_u;
  const mxArray *c2_n_y = NULL;
  real_T c2_m_u;
  const mxArray *c2_o_y = NULL;
  SFc2_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_cm_testInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  c2_u = *(c2_SensorsOut_bus *)c2_inData;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_b_u = c2_u.imu_out;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  for (c2_i4 = 0; c2_i4 < 3; c2_i4++) {
    c2_c_u[c2_i4] = c2_b_u.Omega_0[c2_i4];
  }

  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", c2_c_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_addfield(c2_c_y, c2_d_y, "Omega_0", "Omega_0", 0);
  for (c2_i5 = 0; c2_i5 < 3; c2_i5++) {
    c2_c_u[c2_i5] = c2_b_u.Alpha_0[c2_i5];
  }

  c2_e_y = NULL;
  sf_mex_assign(&c2_e_y, sf_mex_create("y", c2_c_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_addfield(c2_c_y, c2_e_y, "Alpha_0", "Alpha_0", 0);
  sf_mex_addfield(c2_b_y, c2_c_y, "imu_out", "imu_out", 0);
  c2_d_u = c2_u.rpsensor_out;
  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_e_u = c2_d_u.lat_pos_ctr_road;
  c2_g_y = NULL;
  sf_mex_assign(&c2_g_y, sf_mex_create("y", &c2_e_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_f_y, c2_g_y, "lat_pos_ctr_road", "lat_pos_ctr_road", 0);
  c2_f_u = c2_d_u.lat_pos_ctr_lane;
  c2_h_y = NULL;
  sf_mex_assign(&c2_h_y, sf_mex_create("y", &c2_f_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_f_y, c2_h_y, "lat_pos_ctr_lane", "lat_pos_ctr_lane", 0);
  c2_g_u = c2_d_u.hdg_err;
  c2_i_y = NULL;
  sf_mex_assign(&c2_i_y, sf_mex_create("y", &c2_g_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_f_y, c2_i_y, "hdg_err", "hdg_err", 0);
  c2_h_u = c2_d_u.path_hdg;
  c2_j_y = NULL;
  sf_mex_assign(&c2_j_y, sf_mex_create("y", &c2_h_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_f_y, c2_j_y, "path_hdg", "path_hdg", 0);
  c2_i_u = c2_d_u.curve_xy;
  c2_k_y = NULL;
  sf_mex_assign(&c2_k_y, sf_mex_create("y", &c2_i_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_f_y, c2_k_y, "curve_xy", "curve_xy", 0);
  sf_mex_addfield(c2_b_y, c2_f_y, "rpsensor_out", "rpsensor_out", 0);
  c2_j_u = c2_u.steersensor_out;
  c2_l_y = NULL;
  sf_mex_assign(&c2_l_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c2_k_u = c2_j_u.steer_angle_1;
  c2_m_y = NULL;
  sf_mex_assign(&c2_m_y, sf_mex_create("y", &c2_k_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_l_y, c2_m_y, "steer_angle_1", "steer_angle_1", 0);
  c2_l_u = c2_j_u.steer_angle_2;
  c2_n_y = NULL;
  sf_mex_assign(&c2_n_y, sf_mex_create("y", &c2_l_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_l_y, c2_n_y, "steer_angle_2", "steer_angle_2", 0);
  c2_m_u = c2_j_u.mean_angle;
  c2_o_y = NULL;
  sf_mex_assign(&c2_o_y, sf_mex_create("y", &c2_m_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c2_l_y, c2_o_y, "mean_angle", "mean_angle", 0);
  sf_mex_addfield(c2_b_y, c2_l_y, "steersensor_out", "steersensor_out", 0);
  sf_mex_assign(&c2_mxArrayOutData, c2_b_y, false);
  return c2_mxArrayOutData;
}

static void c2_c_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, c2_SensorsOut_bus *
  c2_b_y)
{
  emlrtMsgIdentifier c2_thisId;
  static const char * c2_fieldNames[3] = { "imu_out", "rpsensor_out",
    "steersensor_out" };

  c2_thisId.fParent = c2_parentId;
  sf_mex_check_struct(c2_parentId, c2_u, 3, c2_fieldNames, 0U, NULL);
  c2_thisId.fIdentifier = "imu_out";
  c2_b_y->imu_out = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "imu_out", "imu_out", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "rpsensor_out";
  c2_b_y->rpsensor_out = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "rpsensor_out", "rpsensor_out", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "steersensor_out";
  c2_b_y->steersensor_out = c2_g_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "steersensor_out", "steersensor_out", 0)), &c2_thisId);
  sf_mex_destroy(&c2_u);
}

static c2_imu_bus c2_d_emlrt_marshallIn(SFc2_cm_testInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  c2_imu_bus c2_b_y;
  emlrtMsgIdentifier c2_thisId;
  static const char * c2_fieldNames[2] = { "Omega_0", "Alpha_0" };

  c2_thisId.fParent = c2_parentId;
  sf_mex_check_struct(c2_parentId, c2_u, 2, c2_fieldNames, 0U, NULL);
  c2_thisId.fIdentifier = "Omega_0";
  c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u,
    "Omega_0", "Omega_0", 0)), &c2_thisId, c2_b_y.Omega_0);
  c2_thisId.fIdentifier = "Alpha_0";
  c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c2_u,
    "Alpha_0", "Alpha_0", 0)), &c2_thisId, c2_b_y.Alpha_0);
  sf_mex_destroy(&c2_u);
  return c2_b_y;
}

static void c2_e_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_b_y[3])
{
  real_T c2_dv0[3];
  int32_T c2_i6;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv0, 1, 0, 0U, 1, 0U, 1, 3);
  for (c2_i6 = 0; c2_i6 < 3; c2_i6++) {
    c2_b_y[c2_i6] = c2_dv0[c2_i6];
  }

  sf_mex_destroy(&c2_u);
}

static c2_RPSensor_bus c2_f_emlrt_marshallIn(SFc2_cm_testInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  c2_RPSensor_bus c2_b_y;
  emlrtMsgIdentifier c2_thisId;
  static const char * c2_fieldNames[5] = { "lat_pos_ctr_road",
    "lat_pos_ctr_lane", "hdg_err", "path_hdg", "curve_xy" };

  c2_thisId.fParent = c2_parentId;
  sf_mex_check_struct(c2_parentId, c2_u, 5, c2_fieldNames, 0U, NULL);
  c2_thisId.fIdentifier = "lat_pos_ctr_road";
  c2_b_y.lat_pos_ctr_road = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "lat_pos_ctr_road", "lat_pos_ctr_road", 0)),
    &c2_thisId);
  c2_thisId.fIdentifier = "lat_pos_ctr_lane";
  c2_b_y.lat_pos_ctr_lane = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "lat_pos_ctr_lane", "lat_pos_ctr_lane", 0)),
    &c2_thisId);
  c2_thisId.fIdentifier = "hdg_err";
  c2_b_y.hdg_err = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "hdg_err", "hdg_err", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "path_hdg";
  c2_b_y.path_hdg = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "path_hdg", "path_hdg", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "curve_xy";
  c2_b_y.curve_xy = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "curve_xy", "curve_xy", 0)), &c2_thisId);
  sf_mex_destroy(&c2_u);
  return c2_b_y;
}

static c2_SteerSensor_bus c2_g_emlrt_marshallIn(SFc2_cm_testInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  c2_SteerSensor_bus c2_b_y;
  emlrtMsgIdentifier c2_thisId;
  static const char * c2_fieldNames[3] = { "steer_angle_1", "steer_angle_2",
    "mean_angle" };

  c2_thisId.fParent = c2_parentId;
  sf_mex_check_struct(c2_parentId, c2_u, 3, c2_fieldNames, 0U, NULL);
  c2_thisId.fIdentifier = "steer_angle_1";
  c2_b_y.steer_angle_1 = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "steer_angle_1", "steer_angle_1", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "steer_angle_2";
  c2_b_y.steer_angle_2 = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "steer_angle_2", "steer_angle_2", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "mean_angle";
  c2_b_y.mean_angle = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c2_u, "mean_angle", "mean_angle", 0)), &c2_thisId);
  sf_mex_destroy(&c2_u);
  return c2_b_y;
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sensors_in;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  c2_SensorsOut_bus c2_b_y;
  SFc2_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_cm_testInstanceStruct *)chartInstanceVoid;
  c2_b_sensors_in = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sensors_in), &c2_thisId,
                        &c2_b_y);
  sf_mex_destroy(&c2_b_sensors_in);
  *(c2_SensorsOut_bus *)c2_outData = c2_b_y;
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

static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_u;
  const mxArray *c2_b_y = NULL;
  SFc2_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_cm_testInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(int32_T *)c2_inData;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_b_y, false);
  return c2_mxArrayOutData;
}

static int32_T c2_h_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_b_y;
  int32_T c2_i7;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i7, 1, 6, 0U, 0, 0U, 0);
  c2_b_y = c2_i7;
  sf_mex_destroy(&c2_u);
  return c2_b_y;
}

static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_b_y;
  SFc2_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_cm_testInstanceStruct *)chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_b_y = c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_b_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_sensors_in_bus_io(void *chartInstanceVoid, void
  *c2_pData)
{
  const mxArray *c2_mxVal = NULL;
  int32_T c2_i8;
  c2_SensorsOut_bus c2_tmp;
  int32_T c2_i9;
  SFc2_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_cm_testInstanceStruct *)chartInstanceVoid;
  c2_mxVal = NULL;
  for (c2_i8 = 0; c2_i8 < 3; c2_i8++) {
    c2_tmp.imu_out.Omega_0[c2_i8] = ((real_T *)&((char_T *)(c2_imu_bus *)
      &((char_T *)(c2_SensorsOut_bus *)c2_pData)[0])[0])[c2_i8];
  }

  for (c2_i9 = 0; c2_i9 < 3; c2_i9++) {
    c2_tmp.imu_out.Alpha_0[c2_i9] = ((real_T *)&((char_T *)(c2_imu_bus *)
      &((char_T *)(c2_SensorsOut_bus *)c2_pData)[0])[24])[c2_i9];
  }

  c2_tmp.rpsensor_out.lat_pos_ctr_road = *(real_T *)&((char_T *)(c2_RPSensor_bus
    *)&((char_T *)(c2_SensorsOut_bus *)c2_pData)[48])[0];
  c2_tmp.rpsensor_out.lat_pos_ctr_lane = *(real_T *)&((char_T *)(c2_RPSensor_bus
    *)&((char_T *)(c2_SensorsOut_bus *)c2_pData)[48])[8];
  c2_tmp.rpsensor_out.hdg_err = *(real_T *)&((char_T *)(c2_RPSensor_bus *)
    &((char_T *)(c2_SensorsOut_bus *)c2_pData)[48])[16];
  c2_tmp.rpsensor_out.path_hdg = *(real_T *)&((char_T *)(c2_RPSensor_bus *)
    &((char_T *)(c2_SensorsOut_bus *)c2_pData)[48])[24];
  c2_tmp.rpsensor_out.curve_xy = *(real_T *)&((char_T *)(c2_RPSensor_bus *)
    &((char_T *)(c2_SensorsOut_bus *)c2_pData)[48])[32];
  c2_tmp.steersensor_out.steer_angle_1 = *(real_T *)&((char_T *)
    (c2_SteerSensor_bus *)&((char_T *)(c2_SensorsOut_bus *)c2_pData)[88])[0];
  c2_tmp.steersensor_out.steer_angle_2 = *(real_T *)&((char_T *)
    (c2_SteerSensor_bus *)&((char_T *)(c2_SensorsOut_bus *)c2_pData)[88])[8];
  c2_tmp.steersensor_out.mean_angle = *(real_T *)&((char_T *)(c2_SteerSensor_bus
    *)&((char_T *)(c2_SensorsOut_bus *)c2_pData)[88])[16];
  sf_mex_assign(&c2_mxVal, c2_c_sf_marshallOut(chartInstance, &c2_tmp), false);
  return c2_mxVal;
}

static const mxArray *c2_ctr_law_bus_io(void *chartInstanceVoid, void *c2_pData)
{
  const mxArray *c2_mxVal = NULL;
  c2_ctr_law_params_bus c2_tmp;
  int32_T c2_i10;
  SFc2_cm_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_cm_testInstanceStruct *)chartInstanceVoid;
  c2_mxVal = NULL;
  c2_tmp.flag = *(int8_T *)&((char_T *)(c2_ctr_law_params_bus *)c2_pData)[0];
  for (c2_i10 = 0; c2_i10 < 6; c2_i10++) {
    c2_tmp.K_vilca[c2_i10] = ((real_T *)&((char_T *)(c2_ctr_law_params_bus *)
      c2_pData)[8])[c2_i10];
  }

  sf_mex_assign(&c2_mxVal, c2_b_sf_marshallOut(chartInstance, &c2_tmp), false);
  return c2_mxVal;
}

static uint8_T c2_i_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_b_is_active_c2_cm_test, const char_T *c2_identifier)
{
  uint8_T c2_b_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_b_y = c2_j_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_cm_test), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_cm_test);
  return c2_b_y;
}

static uint8_T c2_j_emlrt_marshallIn(SFc2_cm_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_b_y;
  uint8_T c2_u0;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u0, 1, 3, 0U, 0, 0U, 0);
  c2_b_y = c2_u0;
  sf_mex_destroy(&c2_u);
  return c2_b_y;
}

static void init_dsm_address_info(SFc2_cm_testInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc2_cm_testInstanceStruct *chartInstance)
{
  chartInstance->c2_sensors_in = (c2_SensorsOut_bus *)
    ssGetInputPortSignal_wrapper(chartInstance->S, 0);
  chartInstance->c2_y = (real_T *)ssGetOutputPortSignal_wrapper(chartInstance->S,
    1);
  chartInstance->c2_ctr_law = (c2_ctr_law_params_bus *)
    ssGetInputPortSignal_wrapper(chartInstance->S, 1);
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
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3638430615U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3188845491U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(4038855345U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1762848103U);
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
    mxArray *mxChecksum = mxCreateString("cHrb49ebfF5J1U6IrdE50G");
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
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"y\",},{M[8],M[0],T\"is_active_c2_cm_test\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
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
           3,
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
          _SFD_SET_DATA_PROPS(2,2,0,1,"y");
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
        _SFD_CV_INIT_EML(0,1,2,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,633);
        _SFD_CV_INIT_EML_FCN(0,1,"VilcaController",636,-1,1051);
        _SFD_CV_INIT_EML_IF(0,1,0,531,551,606,625);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,0,534,551,-1,0);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sensors_in_bus_io,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_ctr_law_bus_io,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
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
        _SFD_SET_DATA_VALUE_PTR(2U, chartInstance->c2_y);
        _SFD_SET_DATA_VALUE_PTR(1U, chartInstance->c2_ctr_law);
      }
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "sOZ8H5w8fOg5HQE6MNbsvrE";
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
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,2);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,1);
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

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1644045835U));
  ssSetChecksum1(S,(566855984U));
  ssSetChecksum2(S,(275991095U));
  ssSetChecksum3(S,(3387643596U));
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
