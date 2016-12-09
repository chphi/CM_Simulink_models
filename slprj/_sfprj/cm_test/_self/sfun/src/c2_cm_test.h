#ifndef __c2_cm_test_h__
#define __c2_cm_test_h__

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

#ifndef typedef_c2_imu_bus
#define typedef_c2_imu_bus

typedef struct imu_bus_tag c2_imu_bus;

#endif                                 /*typedef_c2_imu_bus*/

#ifndef struct_RPSensor_bus_tag
#define struct_RPSensor_bus_tag

struct RPSensor_bus_tag
{
  real_T lat_pos_ctr_road;
  real_T lat_pos_ctr_lane;
  real_T hdg_err;
  real_T path_hdg;
  real_T curve_xy;
};

#endif                                 /*struct_RPSensor_bus_tag*/

#ifndef typedef_c2_RPSensor_bus
#define typedef_c2_RPSensor_bus

typedef struct RPSensor_bus_tag c2_RPSensor_bus;

#endif                                 /*typedef_c2_RPSensor_bus*/

#ifndef struct_SteerSensor_bus_tag
#define struct_SteerSensor_bus_tag

struct SteerSensor_bus_tag
{
  real_T steer_angle_1;
  real_T steer_angle_2;
  real_T mean_angle;
};

#endif                                 /*struct_SteerSensor_bus_tag*/

#ifndef typedef_c2_SteerSensor_bus
#define typedef_c2_SteerSensor_bus

typedef struct SteerSensor_bus_tag c2_SteerSensor_bus;

#endif                                 /*typedef_c2_SteerSensor_bus*/

#ifndef struct_SensorsOut_bus_tag
#define struct_SensorsOut_bus_tag

struct SensorsOut_bus_tag
{
  c2_imu_bus imu_out;
  c2_RPSensor_bus rpsensor_out;
  c2_SteerSensor_bus steersensor_out;
};

#endif                                 /*struct_SensorsOut_bus_tag*/

#ifndef typedef_c2_SensorsOut_bus
#define typedef_c2_SensorsOut_bus

typedef struct SensorsOut_bus_tag c2_SensorsOut_bus;

#endif                                 /*typedef_c2_SensorsOut_bus*/

#ifndef struct_ctr_law_params_bus_tag
#define struct_ctr_law_params_bus_tag

struct ctr_law_params_bus_tag
{
  int8_T flag;
  real_T K_vilca[6];
};

#endif                                 /*struct_ctr_law_params_bus_tag*/

#ifndef typedef_c2_ctr_law_params_bus
#define typedef_c2_ctr_law_params_bus

typedef struct ctr_law_params_bus_tag c2_ctr_law_params_bus;

#endif                                 /*typedef_c2_ctr_law_params_bus*/

#ifndef typedef_SFc2_cm_testInstanceStruct
#define typedef_SFc2_cm_testInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c2_sfEvent;
  boolean_T c2_isStable;
  boolean_T c2_doneDoubleBufferReInit;
  uint8_T c2_is_active_c2_cm_test;
  c2_SensorsOut_bus *c2_sensors_in;
  real_T *c2_y;
  c2_ctr_law_params_bus *c2_ctr_law;
} SFc2_cm_testInstanceStruct;

#endif                                 /*typedef_SFc2_cm_testInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c2_cm_test_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c2_cm_test_get_check_sum(mxArray *plhs[]);
extern void c2_cm_test_method_dispatcher(SimStruct *S, int_T method, void *data);

#endif
