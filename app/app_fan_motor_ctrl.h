/*! \file fan_motor_control.h
	\brief This header file contains structure definition, enum definitions for related to fan and motor control.
 */

#ifndef APP_FAN_MOTOR_CONTROL_H_
#define APP_FAN_MOTOR_CONTROL_H_

#include "configuration.h"

#ifdef APP_FAN_MOTOR_CONTROL

#include <stdint.h>

#if ( CONFIG_DICM_SUPPORT_INTEGRATED_BSEC_LIB_1_X == 1 )
#include "bsec_integration.h"
#endif

#if ( CONFIG_DICM_SUPPORT_INTEGRATED_BSEC_LIB_2_X == 1 )
#include "bsec_integration_2_x.h"
#endif

#include "hal_nvs.h"
#include <math.h>

#define INV_ALGO_DEBUG 1

#define OFF_MODE_RPM_MIN_PERCENT      ((uint8_t)      0u)
#define OFF_MODE_RPM_MAX_PERCENT      ((uint8_t)      0u)
  
#define SLEEP_MODE_RPM_MIN_PERCENT    ((uint8_t)     40u)
#define SLEEP_MODE_RPM_MAX_PERCENT    ((uint8_t)     40u)
  
#define AUTO_MODE_RPM_MIN_PERCENT     ((uint8_t)     40u)
#define AUTO_MODE_RPM_MAX_PERCENT     ((uint8_t)    100u)

#define TURBO_MODE_RPM_MIN_PERCENT    ((uint8_t)    100u)
#define TURBO_MODE_RPM_MAX_PERCENT    ((uint8_t)    100u)

#define STORAGE_MODE_RPM_MIN_PERCENT  ((uint8_t)    100u)
#define STORAGE_MODE_RPM_MAX_PERCENT  ((uint8_t)    100u)

#define FAN_MOTOR_DUTY_CYCLE_MIN      ((uint32_t)     0u)
#define FAN_MOTOR_DUTY_CYCLE_MAX      ((uint32_t)   100u)

#define IV0MODE_STORAGE  ( IV0MODE_SLEEP + 1 )

#define DEV_RPM_DB_TABLE_SIZE MAX_NUM_DEVICE

#define INVENT_IAQ_INDEX_MIN ((uint16_t)     0u)
#define INVENT_IAQ_INDEX_MAX ((uint16_t)   500u)

#define INVENT_RELATIVE_HUMIDITY_MAX_VALUE ((uint16_t)   100u)

#define DEF_RPM_STEP_LEVEL_1  ((int32_t)   100)
#define DEF_RPM_STEP_LEVEL_2  ((int32_t)   150)
#define DEF_RPM_STEP_LEVEL_3  ((int32_t)   200)
#define DEF_RPM_STEP_LEVEL_4  ((int32_t)   350)
#define DEF_RPM_STEP_LEVEL_5  ((int32_t)   500)

#define DDMP_UNAVAILABLE       ((uint8_t) 0xFFu)
#define CAP_COUNTER_U32_MAX    (4294967295uL)

#define CERAMIC_DISC_MOTOR_DEFAULT_RPM            ((uint32_t) 864)
#define NUM_OPERATING_MODES                       ((uint8_t)   4u + 1u)

#define IV_CONTROL_PROCESSING_INTERVAL_MIN        ((uint8_t)    1u)
#define RV_IDLE_COND_WAIT_TIME_MIN                ((uint8_t)   30u)   
#define RV_PRESS_COMP_WAIT_TIME_MIN               ((uint8_t)    2u)
#define RV_PRESS_COMP_EXCEED_COND_RUN_TIME_MIN    ((uint8_t)   10u) 

#define IAQ_DEF_GOOD_MIN    ((uint16_t)   0u)
#define IAQ_DEF_GOOD_MAX    ((uint16_t) 100u)
#define IAQ_DEF_BAD_MIN     ((uint16_t) 101u)
#define IAQ_DEF_BAD_MAX     ((uint16_t) 200u)
#define IAQ_DEF_WORSE_MIN   ((uint16_t) 201u)
#define IAQ_DEF_WORSE_MAX   ((uint16_t) 500u)
#define IAQ_RANGE_LEVELS    ((uint8_t)    3u)

#define MOTOR_FAN_MAX_RPM  ((uint32_t) 10000u)
#define MOTOR_FAN_MIN_RPM  ((uint32_t)     0u)

#define IAQ_CONFIG_MIN     ((uint32_t)     0u)
#define IAQ_CONFIG_MAX     ((uint32_t)   500u)

#define FAN_MOTOR_MAX_DUTY_CYCLE                 ((uint32_t)   100u)
#define NVS_DB_SIZE                              ((uint32_t)    12u)
#define PRESS_COMP_EXCEEDS_LIMIT_RPM             ((uint32_t)     0u)

#define RELATIVE_HUMIDITY_ACCEPTABLE_MIN_VALUE   ((int32_t)      20)
#define RELATIVE_HUMIDITY_ACCEPTABLE_MAX_VALUE   ((int32_t)      60)

/*

// This is actual storage time configuartion as per requiremnt 
#define STORAGE_MODE_SLEEP_TIME_24_HR             ((TickType_t)  1440u) // 24 hours --> 24 * 60 = 1440 minutes
#define STORAGE_MODE_RUN_TIME_03_HR               ((TickType_t)   180u) // 3 hours  -->  3 * 60 = 180  minutes

*/

#define STORAGE_MODE_SLEEP_TIME_21_HR             ((TickType_t)  30u) // 2
#define STORAGE_MODE_RUN_TIME_03_HR               ((TickType_t)   3u) // 1

/*
   DP values shall support max of thress decimal point resolution
   ex:
   2.345

   And we can select the decimal point resolution from 0 to 3

   DP_NO_DEC_POINT_RESOL
   ---------------------
 
   2.345 ->  2     ( The numbers after the decimal point will be eliminated )

   DP_SIN_DEC_POINT_RESOL
   ----------------------

   2.345 ->  2.3   ( One digit after the decimal point will be consider others will be eliminated )

   DP_TWO_DEC_POINT_RESOL
   ----------------------

   2.345 ->  2.34  ( Two digits after the decimal point will be consider others will be eliminated )

   DP_THREE_DEC_POINT_RESOL
   ------------------------

   2.345 -> 2.345  ( Three digits after the decimal point will be consider )

   This resolution can be selected using the macro given below
*/
#define DP_NO_DEC_POINT_RESOL                     ((int32_t) 0)
#define DP_SIN_DEC_POINT_RESOL                    ((int32_t) 1)
#define DP_TWO_DEC_POINT_RESOL                    ((int32_t) 2)
#define DP_THREE_DEC_POINT_RESOL                  ((int32_t) 3)
#define DP_RESOLUTION_SELECTION                   DP_THREE_DEC_POINT_RESOL

#define DIFF_PRESSURE_MAX_LIMIT                   ((float)  25.0f) 
#define DIFF_PRESSURE_MIN_LIMIT                   ((float) -25.0f) 
#define DIFF_PRESSURE_ACCEPTABLE_NEG_LIMIT        ((float)  -0.5f) 
#define DIFF_PRESSURE_ACCEPTABLE_POS_LIMIT        ((float)   0.5f) 
#define DP_POS_PRESS_COMP_VALIDATION_THRESHOLD    ((float)   0.5f) 
#define DP_NEG_PRESS_COMP_VALIDATION_THRESHOLD    ((float)  -0.5f)
#define DIFF_PRESS_ROC_MAX                        ((float)  50.0f)

#define INVENT_CONTROL_PERIODIC_TMR_TICKS         pdMS_TO_TICKS(MIN_TO_MSEC(IV_CONTROL_PROCESSING_INTERVAL_MIN))
#define RV_IDLE_COND_SETTLE_WAIT_TIME_TICKS       pdMS_TO_TICKS(MIN_TO_MSEC(RV_IDLE_COND_WAIT_TIME_MIN))
#define RV_PRESS_COMP_WAIT_TIME_TICKS             pdMS_TO_TICKS(MIN_TO_MSEC(RV_PRESS_COMP_WAIT_TIME_MIN))

typedef enum _invent_device_id
{
	DEV_FAN1_AIR_OUT = 0,              // Exhaust FAN        ( Air Out )
	DEV_FAN2_AIR_IN  = 1,              // Supply FAN         ( Air In  )
    DEV_MOTOR        = 2,              // Ceramic Disc Motor ( Heat Exchange )
	MAX_NUM_DEVICE   = 3
}invent_device_id_t;

typedef enum __dev_comp_config
{
    IDLE_COMP_DEV  = 0,
	IAQ_COMP_DEV   = 1,         
	PRESS_COMP_DEV = 2,  
}DEV_COMP_CONFIG;

typedef enum __error_code
{
	ERR_NONE = 0, 
	ERR_INVALID_INPUT_PARAM = 1,
    ERR_OUT_OF_RANGE = 2
}error_code;

typedef enum __rpm_step_level
{
    RPM_STEP_LEVEL_1    = 0,
    RPM_STEP_LEVEL_2    = 1,
    RPM_STEP_LEVEL_3    = 2,
    RPM_STEP_LEVEL_4    = 3,
    RPM_STEP_LEVEL_5    = 4,
	RPM_STEP_MAX_LEVEL  = 5
}RPM_STEP_LEVEL;

typedef enum __data_type
{
    IAQ_DATA               = 0,
    DP_DATA                = 1,
    IV_MODE                = 2,
    IV_PWR_STATE           = 3,
    IV_PER_TMR_EXP         = 4,
    IV_STATE_CHANGE        = 5,
    IV_RPM_STEP_LVL1       = 6,
    IV_RPM_STEP_LVL2       = 7,
    IV_RPM_STEP_LVL3       = 8,
    IV_RPM_STEP_LVL4       = 9,
    IV_RPM_STEP_LVL5       = 10,
    IV_FAN1_RPM_MIN        = 11,
    IV_FAN2_RPM_MIN        = 12,
    IV_MTR_RPM_MIN         = 13,
    IV_FAN1_RPM_MAX        = 14,
    IV_FAN2_RPM_MAX        = 15,
    IV_MTR_RPM_MAX         = 16,
    IV_FAN1_SET_RPM        = 17,
    IV_FAN2_SET_RPM        = 18,
    IV_MTR_SET_RPM         = 19,
    IV_VOC_SENSOR_ACC      = 20,
    IV_IAQ_GOOD_MIN        = 21,
    IV_IAQ_GOOD_MAX        = 22,
    IV_IAQ_BAD_MIN         = 23,
    IV_IAQ_BAD_MAX         = 24,
    IV_IAQ_WORSE_MIN       = 25,
    IV_IAQ_WORSE_MAX       = 26,
    IV_ONE_SHOT_TMR_EXP    = 27,
    DP_SENSOR_STATUS       = 28,
    IV_STORAGE_TMR_EXP     = 29,
    IV_HUMIDITY_DATA       = 30,
    INVALID_DATA_RECEIVED  = 31
}DATA_ID;

typedef enum __dp_sens_status
{
    DP_SENSOR_NOT_AVAILABLE = 0,
    DP_SENSOR_AVAILABLE     = 1
}DP_SENS_STATUS;

typedef enum __invent_control_state
{
	INVENTILATE_STATE_STANDBY                = 0,
    INVENTILATE_STATE_PROCESS_PARAM          = 1,
    INVENTILATE_STATE_PRESS_CTRL             = 2,
    INVENTILATE_STATE_AQ_CTRL                = 3,
    INVENTILATE_STATE_PRESS_CTRL_EX_LIMIT    = 4,
    INVENTILATE_STATE_WAIT_FOR_IDLE_SETTLE   = 5,
	INVENTILATE_STATE_IDLE                   = 6,
    INVENTILATE_STATE_PROCESS_STANDBY        = 7,
    INVENTILATE_STATE_STORAGE                = 8
}INVENT_CONTROL_STATE;

typedef struct _rpm_range
{
    uint32_t min_rpm;
    uint32_t max_rpm;
}rpm_range;

typedef struct _fan_motor_control_info_table
{
    uint8_t         dev_id;
    uint8_t         dev_pwm_mode;
    rpm_range       whole_rpm_range;
    rpm_range       rpm_range[NUM_OPERATING_MODES];
}fan_motor_control_info_db_t;

typedef struct _percent_range
{
    uint8_t min_percent;
    uint8_t max_percent;
}percent_range_t;

typedef struct __iv_data
{
    int32_t     data;
    DATA_ID  data_id;
}IV_DATA;

typedef struct _iaq_index_range
{
    uint32_t min;
    uint32_t max;
}iaq_index_range;

typedef struct __iaq_range
{
    iaq_index_range   iaq_range;
    IV0AQST_ENUM      iaq_status;
}IAQ_RANGE;

typedef enum __storage_timer_config
{
    STORAGE_TIMER_21H        = 0,
    STORAGE_TIMER_03H        = 1,
    STORAGE_TIMER_MAX_CONFIG = 2
}STORAGE_TIMER_CONFIG;

typedef struct __inventilate_control_algo
{
    bool                  per_tmr_exp;
    bool                  storage_tmr_exp;
    bool                  wait_tmr_exp;
    bool                  change_dev;
    bool                  data_received;
    int32_t               ionizer_status;
    int32_t               dp_comp_thr_val;
    int32_t               dp_resol_factor;
    int32_t               dp_resolution;
    int32_t               dp_neg_acceptable_lim;
    int32_t               dp_pos_acceptable_lim;
    int32_t               dp_range_min;
    int32_t               dp_range_max;
    int32_t               dp_max_roc;
    int32_t               prev_avg_iaq_value;
    int32_t               prev_avg_dp_value;
    int32_t               prev_avg_hum_value;
    int32_t               curr_avg_iaq_value;
    int32_t               curr_avg_dp_value;
    int32_t               curr_avg_hum_value;
    int32_t               roc_max_val;
    int32_t*              ptr_curr_data;
    int32_t*              ptr_prev_data;
    int32_t               rpm_step_table_iaq[RPM_STEP_MAX_LEVEL];
    int32_t               rpm_step_table_dp[RPM_STEP_MAX_LEVEL];
    uint8_t               step_table_index_iaq;
    uint8_t               step_table_index_dp;
    uint16_t              set_rpm[MAX_NUM_DEVICE];
    uint16_t              prev_set_rpm[MAX_NUM_DEVICE];
    int32_t               iaq_data_count;
    int32_t               dp_data_count;
    int32_t               humidity_data_count;
    uint32_t              constant_rpm_motor;
    invent_device_id_t    prim_dev_id;
    invent_device_id_t    sec_dev_id;
    DEV_COMP_CONFIG       dev_comp_config[MAX_NUM_DEVICE];
    BME6X_ACCURACY        sens_acc;
    IV0PRST_ENUM          curr_pr_stat;
    IV0AQST_ENUM          curr_iaq_stat;
    IV0PRST_ENUM          prev_pr_stat;
    IV0AQST_ENUM          prev_iaq_stat;
    IV0MODE_ENUM          cur_sel_mode;
    IV0MODE_ENUM          prev_sel_mode;
    IV0MODE_ENUM          storage_mode_ctrl;
    IVPMGR0STATE_ENUM     iv_pwr_state;
	INVENT_CONTROL_STATE  curr_state;
    INVENT_CONTROL_STATE  prev_state;
    IV_DATA               iv_data;
    DP_SENS_STATUS        dp_senor_status;
    STORAGE_TIMER_CONFIG  storage_timer_config;
    TickType_t            storage_tmr_val_ticks[STORAGE_TIMER_MAX_CONFIG];
}INVENTILATE_CONTROL_ALGO;

typedef struct __nvs_config_conn_fan_mtr
{
   DATA_ID              data_id;
   HAL_NVS_DATA_TYPE    data_type;
   uint32_t             data_size;
   uint32_t             min_val;
   uint32_t             max_val;
   uint32_t             default_val;
   uint32_t             ddm;
   void*                data_ptr;
   char*                nvs_key;
}nvs_config_conn_fan_mtr;

extern INVENTILATE_CONTROL_ALGO  iv_ctrl_algo;
extern INVENTILATE_CONTROL_ALGO* ptr_ctrl_algo;

void init_iv_control_algo(INVENTILATE_CONTROL_ALGO* iv_algo);

void parse_received_data(void);

void initialize_fan_motor(void);

error_code fan2_control_logic(int32_t recv_diff_press_val);

void read_data_from_nvs(void);

uint32_t calc_duty_cycle(uint32_t min_rpm, uint32_t max_rpm, uint32_t duty_min, uint32_t duty_max, uint32_t calc_rpm);

void set_fan_motor_rpm(invent_device_id_t dev_id, uint32_t rpm);

void set_iv_rpm_step_level(RPM_STEP_LEVEL step_level, uint32_t rpm);

void update_and_send_val_to_broker(uint32_t ddm_parameter, int32_t value);

void update_data_in_nvm(DATA_ID data_id, uint32_t data);

void process_standby_request(INVENTILATE_CONTROL_ALGO* ptr_iv);

void calc_avg_for_iaq_dp(INVENTILATE_CONTROL_ALGO* ptr_iv);

void update_dev_rpm(INVENTILATE_CONTROL_ALGO* ptr_iv, IV0MODE_ENUM mode);

void reset_accumulated_data(INVENTILATE_CONTROL_ALGO* ptr_iv);

void update_dp_comp_threshold_val(INVENTILATE_CONTROL_ALGO* ptr_iv);

void reset_dev_config(void);

INVENT_CONTROL_STATE aq_control_routine(INVENTILATE_CONTROL_ALGO* ptr_iv);

INVENT_CONTROL_STATE press_control_routine(INVENTILATE_CONTROL_ALGO* ptr_iv);

IV0PRST_ENUM find_press_comp_state(INVENTILATE_CONTROL_ALGO* ptr_iv);

IV0AQST_ENUM find_air_quality_status(INVENTILATE_CONTROL_ALGO* ptr_iv);

extern void start_wait_tmr(uint32_t period_ms);

extern void stop_wait_tmr(void);

extern void start_periodic_timer(void);

extern void stop_periodic_timer(void);

extern void change_state(INVENT_CONTROL_STATE iv_ctrl_state);

extern void update_iaq_status_to_broker(IV0AQST_ENUM iaq_status);

extern void update_dp_status_to_broker(IV0PRST_ENUM press_status);

extern void update_set_rpm_to_broker(invent_device_id_t dev_id, uint32_t rpm);

extern void push_data_in_queue(int32_t data, DATA_ID data_id);

void calc_mode_min_max_rpm(void);;

#endif /* APP_FAN_MOTOR_CONTROL */

#endif /* APP_FAN_MOTOR_CONTROL_H_ */


