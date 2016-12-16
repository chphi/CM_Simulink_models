#ifndef __c1_cm_test_h__
#define __c1_cm_test_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef struct_imu_bus_tag
#define struct_imu_bus_tag

struct imu_bus_tag
{
  real_T Omega_0[3];
  real_T Alpha_0[3];
};

#endif                                 /*struct_imu_bus_tag*/

#ifndef typedef_c1_imu_bus
#define typedef_c1_imu_bus

typedef struct imu_bus_tag c1_imu_bus;

#endif                                 /*typedef_c1_imu_bus*/

#ifndef struct_mag_bus_tag
#define struct_mag_bus_tag

struct mag_bus_tag
{
  real_T hdg;
};

#endif                                 /*struct_mag_bus_tag*/

#ifndef typedef_c1_mag_bus
#define typedef_c1_mag_bus

typedef struct mag_bus_tag c1_mag_bus;

#endif                                 /*typedef_c1_mag_bus*/

#ifndef struct_RPSensor_bus_tag
#define struct_RPSensor_bus_tag

struct RPSensor_bus_tag
{
  real_T lat_pos_ctr_road;
  real_T lat_pos_ctr_lane;
  real_T hdg_err;
  real_T path_hdg;
  real_T curve_xy;
  real_T preview_pt_pos_Fr1[3];
};

#endif                                 /*struct_RPSensor_bus_tag*/

#ifndef typedef_c1_RPSensor_bus
#define typedef_c1_RPSensor_bus

typedef struct RPSensor_bus_tag c1_RPSensor_bus;

#endif                                 /*typedef_c1_RPSensor_bus*/

#ifndef struct_SteerSensor_bus_tag
#define struct_SteerSensor_bus_tag

struct SteerSensor_bus_tag
{
  real_T steer_angle_1;
  real_T steer_angle_2;
  real_T mean_angle;
};

#endif                                 /*struct_SteerSensor_bus_tag*/

#ifndef typedef_c1_SteerSensor_bus
#define typedef_c1_SteerSensor_bus

typedef struct SteerSensor_bus_tag c1_SteerSensor_bus;

#endif                                 /*typedef_c1_SteerSensor_bus*/

#ifndef struct_OdoSensors_bus_tag
#define struct_OdoSensors_bus_tag

struct OdoSensors_bus_tag
{
  real_T vel_from_motor;
};

#endif                                 /*struct_OdoSensors_bus_tag*/

#ifndef typedef_c1_OdoSensors_bus
#define typedef_c1_OdoSensors_bus

typedef struct OdoSensors_bus_tag c1_OdoSensors_bus;

#endif                                 /*typedef_c1_OdoSensors_bus*/

#ifndef struct_SensorsOut_bus_tag
#define struct_SensorsOut_bus_tag

struct SensorsOut_bus_tag
{
  c1_imu_bus imu_out;
  c1_mag_bus mag_out;
  c1_RPSensor_bus rpsensor_out;
  c1_SteerSensor_bus steersensor_out;
  c1_OdoSensors_bus odo_out;
};

#endif                                 /*struct_SensorsOut_bus_tag*/

#ifndef typedef_c1_SensorsOut_bus
#define typedef_c1_SensorsOut_bus

typedef struct SensorsOut_bus_tag c1_SensorsOut_bus;

#endif                                 /*typedef_c1_SensorsOut_bus*/

#ifndef struct_low_ctr_params_bus_tag
#define struct_low_ctr_params_bus_tag

struct low_ctr_params_bus_tag
{
  real_T Fs;
  real_T K_accel;
  real_T max_accel;
  real_T trq_full;
  real_T trq_zero;
  real_T brake_to_Trq[2];
};

#endif                                 /*struct_low_ctr_params_bus_tag*/

#ifndef typedef_c1_low_ctr_params_bus
#define typedef_c1_low_ctr_params_bus

typedef struct low_ctr_params_bus_tag c1_low_ctr_params_bus;

#endif                                 /*typedef_c1_low_ctr_params_bus*/

#ifndef struct_vhcl_bus_tag
#define struct_vhcl_bus_tag

struct vhcl_bus_tag
{
  real_T wheelbase;
  real_T rear_axle_x;
  real_T wheel_radius;
  real_T mass;
  real_T gear_ratio;
};

#endif                                 /*struct_vhcl_bus_tag*/

#ifndef typedef_c1_vhcl_bus
#define typedef_c1_vhcl_bus

typedef struct vhcl_bus_tag c1_vhcl_bus;

#endif                                 /*typedef_c1_vhcl_bus*/

#ifndef typedef_SFc1_cm_testInstanceStruct
#define typedef_SFc1_cm_testInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c1_sfEvent;
  boolean_T c1_isStable;
  boolean_T c1_doneDoubleBufferReInit;
  uint8_T c1_is_active_c1_cm_test;
  c1_SensorsOut_bus *c1_sensors_in;
  real_T (*c1_refs)[2];
  real_T (*c1_y)[2];
  real_T *c1_steer_flag;
  c1_low_ctr_params_bus *c1_low_ctr;
  c1_vhcl_bus *c1_vhcl;
} SFc1_cm_testInstanceStruct;

#endif                                 /*typedef_SFc1_cm_testInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c1_cm_test_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c1_cm_test_get_check_sum(mxArray *plhs[]);
extern void c1_cm_test_method_dispatcher(SimStruct *S, int_T method, void *data);

#endif
