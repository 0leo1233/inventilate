/*! \file app_fan_motor_control.c
	\brief This file contains implementation related to fan and moto control algorithm's and API's
 */

#include "configuration.h"

#ifdef APP_FAN_MOTOR_CONTROL

#include <stdlib.h>
#include "hal_pwm.h"
#include "osal.h"
#include "app_fan_motor_ctrl.h"

int32_t bme68x_humid_value;
int32_t bm_humid_prev;
int32_t bm_humid_curr;
static uint32_t calc_percentage(uint32_t value, uint32_t percent);
static int8_t calc_rate_of_change(float curr_val, float prev_val, float max_val);
static uint8_t get_step_level(int32_t roc);
void calc_mode_min_max_rpm(void);

EXT_RAM_ATTR fan_motor_control_info_db_t fan_motor_control_db[DEV_RPM_DB_TABLE_SIZE];

/* IAQ Range */
static IAQ_RANGE iaq_range_level[IAQ_RANGE_LEVELS] =
{ 
    //          min                  max                Air Quality Status
    {   {  IAQ_DEF_GOOD_MIN,    IAQ_DEF_GOOD_MAX },   IV0AQST_AIR_QUALITY_GOOD  },
    {   {   IAQ_DEF_BAD_MIN,     IAQ_DEF_BAD_MAX },   IV0AQST_AIR_QUALITY_BAD   },
    {   { IAQ_DEF_WORSE_MIN,   IAQ_DEF_WORSE_MAX },   IV0AQST_AIR_QUALITY_WORSE }
};

EXT_RAM_ATTR IV0_SETTINGS   ivsett_config;
EXT_RAM_ATTR FILTER_INFO filter_data;
/* Table for RPM Min and Max Percentage for modes */
static percent_range_t percent_range[NUM_OPERATING_MODES] =
{
	{ .min_percent =  OFF_MODE_RPM_MIN_PERCENT     ,   .max_percent = OFF_MODE_RPM_MAX_PERCENT     }, /* OFF Mode     */
	{ .min_percent =  AUTO_MODE_RPM_MIN_PERCENT    ,   .max_percent = AUTO_MODE_RPM_MAX_PERCENT    }, /* Auto Mode    */
	{ .min_percent =  TURBO_MODE_RPM_MIN_PERCENT   ,   .max_percent = TURBO_MODE_RPM_MAX_PERCENT   }, /* Turbo Mode   */
	{ .min_percent =  SLEEP_MODE_RPM_MIN_PERCENT   ,   .max_percent = SLEEP_MODE_RPM_MAX_PERCENT   }, /* Sleep Mode   */
    { .min_percent =  STORAGE_MODE_RPM_MIN_PERCENT ,   .max_percent = STORAGE_MODE_RPM_MAX_PERCENT }  /* Storage Mode */
};

/* Structure instance for inventilate control algorithm */
INVENTILATE_CONTROL_ALGO iv_ctrl_algo;

/* NVS Table for connecor fan motor */
static nvs_config_conn_fan_mtr nvs_db[NVS_DB_SIZE] = 
{
    //   data_id               data_type               data_size             min_val                max_val            default_val           ddmp                           data_ptr                                                          nvs_key
    {IV_FAN1_RPM_MIN  ,  HAL_NVS_DATA_TYPE_UINT32, sizeof(uint32_t),    MOTOR_FAN_MIN_RPM,     MOTOR_FAN_MAX_RPM,    DEV_FAN1_MIN_RPM,  MTR0MINSPD|DDM2_PARAMETER_INSTANCE(0),  (void*)&fan_motor_control_db[DEV_FAN1_AIR_IN].whole_rpm_range.min_rpm      ,  "mn_rpm_f1"},
    {IV_FAN2_RPM_MIN  ,  HAL_NVS_DATA_TYPE_UINT32, sizeof(uint32_t),    MOTOR_FAN_MIN_RPM,     MOTOR_FAN_MAX_RPM,    DEV_FAN2_MIN_RPM,  MTR0MINSPD|DDM2_PARAMETER_INSTANCE(1),  (void*)&fan_motor_control_db[DEV_FAN2_AIR_OUT].whole_rpm_range.min_rpm     ,  "mn_rpm_f2"},
    {IV_MTR_RPM_MIN   ,  HAL_NVS_DATA_TYPE_UINT32, sizeof(uint32_t),    MOTOR_FAN_MIN_RPM,     MOTOR_FAN_MAX_RPM,   DEV_MOTOR_MIN_RPM,  MTR0MINSPD|DDM2_PARAMETER_INSTANCE(2),  (void*)&fan_motor_control_db[DEV_MOTOR].whole_rpm_range.min_rpm            ,  "mn_rpm_mt"},
    {IV_FAN1_RPM_MAX  ,  HAL_NVS_DATA_TYPE_UINT32, sizeof(uint32_t),    MOTOR_FAN_MIN_RPM,     MOTOR_FAN_MAX_RPM,    DEV_FAN1_MAX_RPM,  MTR0MAXSPD|DDM2_PARAMETER_INSTANCE(0),  (void*)&fan_motor_control_db[DEV_FAN1_AIR_IN].whole_rpm_range.max_rpm      ,  "mx_rpm_f1"},
    {IV_FAN2_RPM_MAX  ,  HAL_NVS_DATA_TYPE_UINT32, sizeof(uint32_t),    MOTOR_FAN_MIN_RPM,     MOTOR_FAN_MAX_RPM,    DEV_FAN2_MAX_RPM,  MTR0MAXSPD|DDM2_PARAMETER_INSTANCE(1),  (void*)&fan_motor_control_db[DEV_FAN2_AIR_OUT].whole_rpm_range.max_rpm     ,  "mx_rpm_f2"},
    {IV_MTR_RPM_MAX   ,  HAL_NVS_DATA_TYPE_UINT32, sizeof(uint32_t),    MOTOR_FAN_MIN_RPM,     MOTOR_FAN_MAX_RPM,   DEV_MOTOR_MAX_RPM,  MTR0MAXSPD|DDM2_PARAMETER_INSTANCE(2),  (void*)&fan_motor_control_db[DEV_MOTOR].whole_rpm_range.max_rpm            ,  "mx_rpm_mt"},
    {IV_IAQ_GOOD_MIN  ,  HAL_NVS_DATA_TYPE_UINT32, sizeof(uint32_t),       IAQ_CONFIG_MIN,        IAQ_CONFIG_MAX,    IAQ_DEF_GOOD_MIN,  IVAQR0MIN|DDM2_PARAMETER_INSTANCE(0),  (void*)&iaq_range_level[IV0AQST_AIR_QUALITY_GOOD].iaq_range.min             ,  "mn_gd_aq" },
    {IV_IAQ_BAD_MIN   ,  HAL_NVS_DATA_TYPE_UINT32, sizeof(uint32_t),       IAQ_CONFIG_MIN,        IAQ_CONFIG_MAX,     IAQ_DEF_BAD_MIN,  IVAQR0MIN|DDM2_PARAMETER_INSTANCE(1),  (void*)&iaq_range_level[IV0AQST_AIR_QUALITY_BAD].iaq_range.min              ,  "mn_bd_aq" },
    {IV_IAQ_WORSE_MIN ,  HAL_NVS_DATA_TYPE_UINT32, sizeof(uint32_t),       IAQ_CONFIG_MIN,        IAQ_CONFIG_MAX,   IAQ_DEF_WORSE_MIN,  IVAQR0MIN|DDM2_PARAMETER_INSTANCE(2),  (void*)&iaq_range_level[IV0AQST_AIR_QUALITY_WORSE].iaq_range.min            ,  "mn_wr_aq" },
    {IV_IAQ_GOOD_MAX  ,  HAL_NVS_DATA_TYPE_UINT32, sizeof(uint32_t),       IAQ_CONFIG_MIN,        IAQ_CONFIG_MAX,    IAQ_DEF_GOOD_MAX,  IVAQR0MAX|DDM2_PARAMETER_INSTANCE(0),  (void*)&iaq_range_level[IV0AQST_AIR_QUALITY_GOOD].iaq_range.max             ,  "mx_gd_aq" },
    {IV_IAQ_BAD_MAX   ,  HAL_NVS_DATA_TYPE_UINT32, sizeof(uint32_t),       IAQ_CONFIG_MIN,        IAQ_CONFIG_MAX,     IAQ_DEF_BAD_MAX,  IVAQR0MAX|DDM2_PARAMETER_INSTANCE(1),  (void*)&iaq_range_level[IV0AQST_AIR_QUALITY_BAD].iaq_range.max              ,  "mx_bd_aq" },
    {IV_IAQ_WORSE_MAX ,  HAL_NVS_DATA_TYPE_UINT32, sizeof(uint32_t),       IAQ_CONFIG_MIN,        IAQ_CONFIG_MAX,   IAQ_DEF_WORSE_MAX,  IVAQR0MAX|DDM2_PARAMETER_INSTANCE(2),  (void*)&iaq_range_level[IV0AQST_AIR_QUALITY_WORSE].iaq_range.max            ,  "mx_wr_aq" },
    {IV_IVSETT        ,  HAL_NVS_DATA_TYPE_UINT32, sizeof(uint32_t),       IV_IVSETT_MIN,         IV_IVSETT_MAX,   IV_IVSETT_DEFAULT,   IV0SETT|DDM2_PARAMETER_INSTANCE(0),    (void*)&ivsett_config.byte                                                  ,  "iv_ivsett" },
    {IV_FILTER_TIMER  ,  HAL_NVS_DATA_TYPE_UINT32, sizeof(uint32_t),       IV_FILTER_MIN_MIN,     IV_FILTER_MIN_MAX,IV_FILTER_MIN_MIN,   IV0FILST,                             (void*)&filter_data.filter_min                                              ,  "iv_filter_min" },
    {IV_FILTER_STATUS  ,  HAL_NVS_DATA_TYPE_UINT32, sizeof(uint32_t),       IV_FILTER_MIN_MIN,     IV_FILTER_MIN_MAX,IV_FILTER_MIN_MIN,   IV0FILST,                             (void*)&filter_data.filter_status                                          ,  "iv_filter_sts" },
};

/**
  * @brief  Calculate the rpm to derive the motor and fan using linear interpolation.
  * @param  min_iaq_press.
  * @param  max_iaq_press.
  * @param  min_rpm.
  * @param  max_rpm.
  * @param  x Coordinate.
  * @retval y Coordinate
  */
uint32_t calc_fan_motor_rpm(uint32_t min_iaq_press, uint32_t max_iaq_press, uint32_t min_rpm, uint32_t max_rpm, uint32_t iaq_press)
{

	/*
   		Auto mode min rpm -700 max rpm - 2800
   		IAQ range min = 0  max = 500
     	x1 = 0
	 	x2 = 500
	 	y1 = 700
	 	y2 = 2800
	 	x = 220

      	y = ((( x - x1) * (y2 -y1)) / (x2 -x1)) + y1;
	  	y = ((220 - 0) * (2800 - 700) / (500 - 0) ) + 700;
	    y = (220 * 2100 / 500) + 700;

   */
    uint32_t numerator = 0;
    uint32_t denominator = 0;
    uint32_t calc_rpm = 0;

    numerator = ((iaq_press - min_iaq_press) * (max_rpm - min_rpm));
    
    denominator = max_iaq_press - min_iaq_press;

    if ( denominator > 0 )
    { 
        calc_rpm = (numerator / denominator) + min_rpm;
    }

	return calc_rpm;
}

/**
  * @brief  Calculate the duty cycle using linear interpolation
  * @param  duty_min.
  * @param  duty_max.
  * @param  min_rpm.
  * @param  max_rpm.
  * @param  x coordinate.
  * @retval y cordinate
  */
uint32_t calc_duty_cycle(uint32_t min_rpm, uint32_t max_rpm, uint32_t duty_min, uint32_t duty_max, uint32_t calc_rpm)
{
    uint32_t numerator = 0u;
    uint32_t denominator = 0u;
    uint32_t duty_cycle = 0u;

    numerator = ((calc_rpm - min_rpm) * (duty_max - duty_min));
    denominator = max_rpm - min_rpm;

    if ( denominator > 0u )
    {
        duty_cycle = ( numerator / denominator) + duty_min;
    }

    return duty_cycle;
}

/**
  * @brief  Function to get the index of the nvs database table using the data ID
  * @param  data_id
  * @retval Table index
  */
uint8_t get_nvsdb_index(DATA_ID data_id)
{
    uint8_t index;
    bool found = false;

    for ( index = 0u; ( ( index < NVS_DB_SIZE ) && ( found == false ) );  )
    {
        if ( data_id == nvs_db[index].data_id )
        {
            found = true;
        }
        else
        {
            index++;
        }
    }

    return index;
}

/**
  * @brief  Function used to update the max rpm range of fan and motor
  * @param  dev_id
  * @param  rpm
  * @retval void
  */
void update_data_in_nvm(DATA_ID data_id, uint32_t data)
{
    uint8_t db_idx = get_nvsdb_index(data_id);
    hal_nvs_error nvs_err;

    if ( ( NVS_DB_SIZE != db_idx ) )
    {
#if INV_ALGO_DEBUG         
        LOG(I, "NVS Write data_id = %d data = %d db_idx = %d", data_id, data, db_idx);
#endif        
        /* Update the data in the pointer */
        *((uint32_t*)(nvs_db[db_idx].data_ptr)) = data;
        /* write the data in the NVS */
        nvs_err = hal_nvs_write(nvs_db[db_idx].nvs_key, nvs_db[db_idx].data_type, nvs_db[db_idx].data_ptr, nvs_db[db_idx].data_size);
        /* Validate the error code */
#if INV_ALGO_DEBUG         
        if ( HAL_NVS_OK != nvs_err )
        {
            LOG(E, "NVS Write failed = %d", nvs_err);
        }
#endif        
    }
    else
    {
        LOG(E, "Invalid data_id = %d", data_id);
    }
}

/**
  * @brief  Function used set the rpm for the Fan and Motor
  * @param  dev_id
  * @param  rpm
  * @retval void
  */
void set_fan_motor_rpm(invent_device_id_t dev_id, uint32_t rpm)
{
	uint32_t duty_cycle = 0;
    uint32_t whole_range_min_rpm = fan_motor_control_db[dev_id].whole_rpm_range.min_rpm;
    uint32_t whole_range_max_rpm = fan_motor_control_db[dev_id].whole_rpm_range.max_rpm;
    uint8_t  dev_pwm_mode        = fan_motor_control_db[dev_id].dev_pwm_mode;
    uint32_t calc_duty;

    if ( rpm > whole_range_max_rpm )
    {
        rpm = whole_range_max_rpm;
    }

    if ( DEV_MOTOR == dev_id )
    {
        iv_ctrl_algo.constant_rpm_motor = rpm;
    }
        
    /* Calculate duty cycle from rpm */
    calc_duty = calc_duty_cycle(whole_range_min_rpm, whole_range_max_rpm, FAN_MOTOR_DUTY_CYCLE_MIN, FAN_MOTOR_DUTY_CYCLE_MAX, rpm);

	if ( dev_pwm_mode == INVERTED_PWM )
    {
        /* Inverted PWM Duty Cycle calculation */
        duty_cycle = 100 - calc_duty;
    }
    else
    {
        /* Dutycycle for Non-Inverted PWM */
        duty_cycle = calc_duty;
    }

#if INV_ALGO_DEBUG 
    LOG(I, "dev_id=%d rpm = %d calc_duty = %d duty_cycle = %d", dev_id, rpm, calc_duty, duty_cycle);
#endif

	/* Set the duty cycle */
    hal_pwm_set_duty_cycle(CONNECTOR_PWM_FAN_MOTOR_CAPTURE_UNIT, dev_id, duty_cycle);
}

/**
  * @brief  Function used set the rpm level 
  * @param  RPM_STEP_LEVEL step level index
  * @param  Value of RPM step
  * @retval void
  */
void set_iv_rpm_step_level(RPM_STEP_LEVEL step_level, uint32_t rpm_step)
{
    if ( step_level < RPM_STEP_MAX_LEVEL )
    {
#if INV_ALGO_DEBUG 
        LOG(I, "step_level = %d rpm_step = %d", step_level, rpm_step);
#endif
        iv_ctrl_algo.rpm_step_table_dp[step_level]  = rpm_step;
        iv_ctrl_algo.rpm_step_table_iaq[step_level] = rpm_step;
    }
}

/**
  * @brief  Function to initialize the inventilate control algo structure instance
  * @param  pointer to struct instance type INVENTILATE_CONTROL_ALGO.
  * @retval none.
  */
void init_iv_control_algo(INVENTILATE_CONTROL_ALGO* ptr_iv)
{
    /* Initialize all variables */
    ptr_iv->prev_avg_dp_value     = 0;
    ptr_iv->prev_avg_iaq_value    = 0;
    ptr_iv->curr_avg_iaq_value    = 0;
    ptr_iv->curr_avg_dp_value     = 0;
    ptr_iv->iaq_data_count        = 0u;
    ptr_iv->dp_data_count         = 0u;
    ptr_iv->step_table_index_iaq  = RPM_STEP_LEVEL_1;
    ptr_iv->step_table_index_dp   = RPM_STEP_LEVEL_1;
	ptr_iv->curr_state            = INVENTILATE_STATE_STANDBY;
    ptr_iv->prev_state            = INVENTILATE_STATE_STANDBY;
    /* At startup before user turn ON the inventilate system the mode will be OFF */
    ptr_iv->cur_sel_mode          = IV0MODE_OFF;
    /* At startup the inventilate will be in standby state */
    ptr_iv->iv_pwr_state          = IVPMGR0STATE_STANDBY;
    /* set the RPM of motor */
    ptr_iv->constant_rpm_motor    = CERAMIC_DISC_MOTOR_DEFAULT_RPM;
    ptr_iv->curr_pr_stat          = IV0PRST_PRESS_STATUS_UNKNOWN;
    ptr_iv->curr_iaq_stat         = IV0AQST_AIR_QUALITY_UNKNOWN;
    ptr_iv->prev_pr_stat          = IV0PRST_PRESS_STATUS_UNKNOWN;
    ptr_iv->prev_iaq_stat         = IV0AQST_AIR_QUALITY_UNKNOWN;

    /* Differential pressure resolution calculation based the selection of decimal point accuracy */
    ptr_iv->dp_resolution         = pow(10, DP_RESOLUTION_SELECTION);
    ptr_iv->dp_resol_factor       = 1000 / ptr_iv->dp_resolution;
    ptr_iv->dp_range_min          = ptr_iv->dp_resolution * DIFF_PRESSURE_MIN_LIMIT;
    ptr_iv->dp_range_max          = ptr_iv->dp_resolution * DIFF_PRESSURE_MAX_LIMIT;
    ptr_iv->dp_neg_acceptable_lim = ptr_iv->dp_resolution * DIFF_PRESSURE_ACCEPTABLE_NEG_LIMIT;
    ptr_iv->dp_pos_acceptable_lim = ptr_iv->dp_resolution * DIFF_PRESSURE_ACCEPTABLE_POS_LIMIT;
    ptr_iv->dp_max_roc            = ptr_iv->dp_resolution * DIFF_PRESS_ROC_MAX;

    ptr_iv->ptr_curr_data         = &ptr_iv->curr_avg_iaq_value;
    ptr_iv->ptr_prev_data         = &ptr_iv->prev_avg_iaq_value;

    ptr_iv->roc_max_val           = INVENT_IAQ_INDEX_MAX;

    LOG(W, "dp_resolution = %d", ptr_iv->dp_resolution);
    LOG(W, "dp_resol_factor = %d", ptr_iv->dp_resol_factor);
    LOG(W, "dp_range_min = %d", ptr_iv->dp_range_min);
    LOG(W, "dp_range_max = %d", ptr_iv->dp_range_max);
    LOG(W, "dp_neg_acceptable_lim = %d", ptr_iv->dp_neg_acceptable_lim);
    LOG(W, "dp_pos_acceptable_lim = %d", ptr_iv->dp_pos_acceptable_lim);
    LOG(W, "dp_max_roc = %d", ptr_iv->dp_max_roc);

    /*
       Based on the rate of change of IAQ.
       This step table shall be used to increase RPM of FAN1 and FAN2 to maintain the IAQ in acceptable range 0 to 50
    */
    ptr_iv->rpm_step_table_iaq[RPM_STEP_LEVEL_1] = DEF_RPM_STEP_LEVEL_1;     //TBD
    ptr_iv->rpm_step_table_iaq[RPM_STEP_LEVEL_2] = DEF_RPM_STEP_LEVEL_2;     //TBD
    ptr_iv->rpm_step_table_iaq[RPM_STEP_LEVEL_3] = DEF_RPM_STEP_LEVEL_3;     //TBD
    ptr_iv->rpm_step_table_iaq[RPM_STEP_LEVEL_4] = DEF_RPM_STEP_LEVEL_4;     //TBD
    ptr_iv->rpm_step_table_iaq[RPM_STEP_LEVEL_5] = DEF_RPM_STEP_LEVEL_5;     //TBD

    /*
       Based on the rate of change of Differential pressure, This step table shall be used to increase
       RPM of FAN2 to maintain the differential pressure in the acceptable range -5 Pascal to +5 Pascal
    */
    ptr_iv->rpm_step_table_dp[RPM_STEP_LEVEL_1] = DEF_RPM_STEP_LEVEL_1;      //TBD
    ptr_iv->rpm_step_table_dp[RPM_STEP_LEVEL_2] = DEF_RPM_STEP_LEVEL_2;      //TBD
    ptr_iv->rpm_step_table_dp[RPM_STEP_LEVEL_3] = DEF_RPM_STEP_LEVEL_3;      //TBD
    ptr_iv->rpm_step_table_dp[RPM_STEP_LEVEL_4] = DEF_RPM_STEP_LEVEL_4;      //TBD
    ptr_iv->rpm_step_table_dp[RPM_STEP_LEVEL_5] = DEF_RPM_STEP_LEVEL_5;      //TBD

    /* Storage timer ticks configuration */
    ptr_iv->storage_tmr_val_ticks[STORAGE_TIMER_21H] = pdMS_TO_TICKS(MIN_TO_MSEC(STORAGE_MODE_SLEEP_TIME_21_HR));
    ptr_iv->storage_tmr_val_ticks[STORAGE_TIMER_03H] = pdMS_TO_TICKS(MIN_TO_MSEC(STORAGE_MODE_RUN_TIME_03_HR));

    ptr_iv->rated_speed_percent[DEV_FAN1_AIR_IN] = DEV_FAN1_RATED_SPEED_PERCENT;
    ptr_iv->rated_speed_percent[DEV_FAN2_AIR_OUT]  = DEV_FAN2_RATED_SPEED_PERCENT;
    ptr_iv->rated_speed_percent[DEV_MOTOR]        = DEV_MOTOR_RATED_SPEED_PERCENT;

    /* Calculate and update min, max RPM according the modes */
    calc_mode_min_max_rpm();
}

/**
  * @brief  Function to reset the accumulated data
  * @param  pointer to struct instance type INVENTILATE_CONTROL_ALGO.
  * @retval none.
  */
void reset_accumulated_data(INVENTILATE_CONTROL_ALGO* ptr_iv)
{
    ptr_iv->prev_avg_dp_value  = 0;
    ptr_iv->prev_avg_iaq_value = 0;
    ptr_iv->curr_avg_dp_value  = 0;
    ptr_iv->curr_avg_iaq_value = 0;
    ptr_iv->dp_data_count      = 0;
    ptr_iv->iaq_data_count     = 0;
}

/**
  * @brief  Function to calculate the average od accumulated IAQ and DP data
  * @param  pointer to struct instance type INVENTILATE_CONTROL_ALGO.
  * @retval void .
  */
void calc_avg_for_iaq_dp(INVENTILATE_CONTROL_ALGO* ptr_iv)
{
    if ( ptr_iv->iaq_data_count > 0u )
    {
        /* Calculate the average value of IAQ */
        ptr_iv->curr_avg_iaq_value = ptr_iv->curr_avg_iaq_value / ptr_iv->iaq_data_count;
#if INV_ALGO_DEBUG 
        LOG(I, "avg_iaq=%d cnt=%d", ptr_iv->curr_avg_iaq_value, ptr_iv->iaq_data_count);
#endif
         /* Average value will be consider as 1 sample */
        ptr_iv->iaq_data_count = 1u;                                                     
    }

    if ( ptr_iv->dp_data_count > 0u )
    {
        /* Calculate the average value of DP */
        ptr_iv->curr_avg_dp_value = ptr_iv->curr_avg_dp_value / ptr_iv->dp_data_count;
#if INV_ALGO_DEBUG 
        LOG(I, "avg_dp=%d cnt=%d", ptr_iv->curr_avg_dp_value, ptr_iv->dp_data_count);
#endif
        /* Average value will be consider as 1 sample */
        ptr_iv->dp_data_count = 1u;                                                     
    }

    if ( ptr_iv->humidity_data_count > 0u )
    {
        /* Calculate the average value of relative humidity */
        ptr_iv->curr_avg_hum_value = ptr_iv->curr_avg_hum_value / ptr_iv->humidity_data_count;

         /* Average value will be consider as 1 sample */
        ptr_iv->humidity_data_count = 1u;
    }
}

/**
  * @brief  Function to find the pressure compensation state based on the current average dp value
  * @param  pointer to struct instance type INVENTILATE_CONTROL_ALGO.
  * @retval Presure compensation state refer enum PRESS_COMP_STATES.
  */
IV0PRST_ENUM find_press_comp_state(INVENTILATE_CONTROL_ALGO* ptr_iv)
{
    IV0PRST_ENUM pressure_stat = IV0PRST_PRESS_STATUS_UNKNOWN;

    if ( ptr_iv->dp_data_count > DP_ZERO_COUNT )
    {
        /* Find the pressure compensation status */
        if ( ptr_iv->curr_avg_dp_value < ptr_iv->dp_neg_acceptable_lim )
        {
            /* Under pressure */
            ptr_iv->dp_exceed_count ++;
            if(ptr_iv->dp_exceed_count >DP_EXCEED_LIMIT)
            {
                pressure_stat = IV0PRST_UNDER_PRESS;
            }
        }
        else if ( ptr_iv->curr_avg_dp_value > ptr_iv->dp_pos_acceptable_lim )
        {
            ptr_iv->dp_exceed_count ++;
            if(ptr_iv->dp_exceed_count >DP_EXCEED_LIMIT)
            {
                pressure_stat = IV0PRST_OVER_PRESS;
            }
        }
        else
        {
            /* Differential pressure is within the acceptable range -5 to +5
               So pressure compensation not needed */
            pressure_stat = IV0PRST_VALID_PRESS_LEVEL;
            ptr_iv->dp_exceed_count = 0;
        }
    }
    else
    {
        pressure_stat = IV0PRST_PRESS_STATUS_UNKNOWN;
    }
    
    return pressure_stat;
}

/**
  * @brief  Function to find the air quality status from the current IAQ value
  * @param  pointer to struct instance type INVENTILATE_CONTROL_ALGO.
  * @retval iaq_status - Air quality status Refer enum type INVENT_AIR_QUALITY_STATUS_T.
  */
IV0AQST_ENUM find_air_quality_status(INVENTILATE_CONTROL_ALGO* ptr_iv)
{
    uint8_t      index;
    IV0AQST_ENUM iaq_status = IV0AQST_AIR_QUALITY_UNKNOWN;

    /* Update the data pointer by IAQ parameter */
    ptr_iv->ptr_curr_data  = &ptr_iv->curr_avg_iaq_value;
    ptr_iv->ptr_prev_data  = &ptr_iv->prev_avg_iaq_value;
    ptr_iv->roc_max_val    = INVENT_IAQ_INDEX_MAX;

    if ( ( ptr_iv->iaq_data_count > 0u ) && ( ptr_iv->sens_acc == BME6X_HIGH_ACCURACY ) )
    {
        /* Find the Air Quality status from IAQ */
        for ( index = 0; index < IAQ_RANGE_LEVELS; index++ )
        {
            if ( ( ptr_iv->curr_avg_iaq_value >= iaq_range_level[index].iaq_range.min ) && 
                 ( ptr_iv->curr_avg_iaq_value <= iaq_range_level[index].iaq_range.max ) )
            {
                iaq_status = iaq_range_level[index].iaq_status;       /* Get the IAQ status */
                index      = IAQ_RANGE_LEVELS;                        /* Exit from the loop */
            }
        }

        /* Validate the humidity when the air quality is good inside the RV */
        if ( ( IV0AQST_AIR_QUALITY_GOOD == iaq_status ) && ( ptr_iv->humidity_data_count > 0u ) )
        {
            /* Even when IAQ is GOOD, but the relative humdity is not within the acceptable range,
               then air quality will be considered as BAD */
            if ( ( ptr_iv->curr_avg_hum_value < RELATIVE_HUMIDITY_ACCEPTABLE_MIN_VALUE ) ||
                 ( ptr_iv->curr_avg_hum_value > RELATIVE_HUMIDITY_ACCEPTABLE_MAX_VALUE ) )
            {
                /* Update the data pointer by humdity parameter */
                ptr_iv->ptr_curr_data  = &ptr_iv->curr_avg_hum_value;
                ptr_iv->ptr_prev_data  = &ptr_iv->prev_avg_hum_value;
                ptr_iv->roc_max_val    = INVENT_RELATIVE_HUMIDITY_MAX_VALUE;
                iaq_status             = IV0AQST_AIR_QUALITY_BAD;
            }
        }
    }
    else if ( ptr_iv->iaq_data_count == 0u )
    {
        iaq_status = IV0AQST_AIR_QUALITY_UNKNOWN;
    }
    else
    {
        iaq_status = IV0AQST_CALIBRATION_ONGOING;
    }

    return iaq_status;
}

/**
  * @brief  Function to control the over/under pressure condition by adjusting the RPM of FAN1 and FAN2
  * @param  ptr_iv            - Pointer to the structure INVENTILATE_CONTROL_ALGO
  * @retval invent_ctrl_state - Refer enum INVENT_CONTROL_STATE
  */
INVENT_CONTROL_STATE press_control_routine(INVENTILATE_CONTROL_ALGO* ptr_iv)
{
    INVENT_CONTROL_STATE  iv_ctrl_state       = ptr_iv->curr_state;
    int32_t               curr_roc_dp_percent = 0;
    bool                  press_comp_working  = false;
    invent_device_id_t    dev_id;

    if ( IV0PRST_VALID_PRESS_LEVEL != ptr_iv->curr_pr_stat )
    {
        /* Check the wait timer expired or not */
        if ( true == ptr_iv->wait_tmr_exp )
        {
            
            
            /* "pr comp tmr exp" Reset the timer exp flag */
            ptr_iv->wait_tmr_exp = false;

            /* Validate the compensation */
            if ( IV0PRST_OVER_PRESS == ptr_iv->curr_pr_stat )
            {
                //         12                             14
                if ( ptr_iv->curr_avg_dp_value < ptr_iv->dp_comp_thr_val )
                {
                    press_comp_working = true;
                }
            }
            else if ( IV0PRST_UNDER_PRESS == ptr_iv->curr_pr_stat )
            {
                //    -55                            -72
                if ( ptr_iv->curr_avg_dp_value > ptr_iv->dp_comp_thr_val )
                {
                    press_comp_working = true;
                }
            }
            else
            {
        #if INV_ALGO_DEBUG 
                LOG(E, "err pr_st=%d", ptr_iv->curr_pr_stat);
        #endif
            }

            if ( false == press_comp_working )
            {
                /* Check the primary device already changed */
                if ( false == ptr_iv->change_dev )
                {
                    LOG(I, "pr_comp not working with pr dev=%d", ptr_iv->prim_dev_id);
                    /* Compensation not worked with primary fan..Try with the secondary fan */
                    ptr_iv->change_dev = true;
                    /* Update the DP threshold value as per the current DP value */
                    update_dp_comp_threshold_val(ptr_iv);
                }
                else
                {
                    LOG(I, "pr_comp ex limit");
                    // Pressure is not able to control by both primary and secondary fan due to unknown environmental condition
                    // Turn OFF both the FANS for 10 minutes and then start the pressure compensation action
                    
                    /* Reset the flag */
                    ptr_iv->change_dev = false;

                    /* Set constant RPM and run for 10 minutes */
                    for ( dev_id = DEV_FAN1_AIR_IN; dev_id <= DEV_FAN2_AIR_OUT; dev_id++ )
                    {
                        ptr_iv->set_rpm[dev_id]         = PRESS_COMP_EXCEEDS_LIMIT_RPM;
                        ptr_iv->dev_comp_config[dev_id] = IDLE_COMP_DEV;
                    }

                    /* Reset the timer flag before starting the timer */
                    ptr_iv->wait_tmr_exp = false;
                    /* Start timer to wait for 10 min */
                    start_wait_tmr(MIN_TO_MSEC(RV_PRESS_COMP_EXCEED_COND_RUN_TIME_MIN));
                    /* Change the state */
                    iv_ctrl_state = INVENTILATE_STATE_PRESS_CTRL_EX_LIMIT;
                }
            }
            else
            {
        #if INV_ALGO_DEBUG 
                LOG(I, "pr_comp working");
        #endif
            }

            if ( INVENTILATE_STATE_PRESS_CTRL_EX_LIMIT != iv_ctrl_state  )
            {
                /* Start the DP comp timer */
                start_wait_tmr(MIN_TO_MSEC(RV_PRESS_COMP_WAIT_TIME_MIN));
            }
        }

        if ( INVENTILATE_STATE_PRESS_CTRL_EX_LIMIT != iv_ctrl_state )
        {
            /* Find the rate of change (ROC) of Differential pressure */
            curr_roc_dp_percent = calc_rate_of_change(ptr_iv->curr_avg_dp_value, ptr_iv->prev_avg_dp_value, ptr_iv->dp_max_roc);

            /* Get the RPM step level from the ROC of DP */
            ptr_iv->step_table_index_dp = get_step_level(abs(curr_roc_dp_percent));

    #if INV_ALGO_DEBUG 
            LOG(I,"roc_dp=%d step=%d", curr_roc_dp_percent, ptr_iv->step_table_index_dp);
    #endif
            /* Find the device to be used for compensating the pressure and direction of rpm of control ( Increase or Decrease ) */

            if ( IV0PRST_OVER_PRESS == ptr_iv->curr_pr_stat )
            {
                // 1. To compensate the positive/over pressure inside the RV, The FAN1 (Exhaust FAN) RPM has to be increased
                // 2. If the FAN1 speed reaches the MAX rpm then FAN2 shall be used to control the Over pressure
                // 3. FAN2 (Supply FAN / Air In) speed shall be reduce to compensate the pressure
                // 4. Even after this, If still the pressure is not compensated then set same RPM on both the FANS
                // 5. Run for configuarable time
                // 6. Reset all variables related to pressure compensation
                /* increase Fan out speed to push air out side*/
                //Over pressure
                ptr_iv->prim_dev_id = DEV_FAN2_AIR_OUT;
                ptr_iv->sec_dev_id  = DEV_FAN1_AIR_IN;


            }
            else
            {
                /* To compensate the negative/under pressure FAN2 (Supply FAN) RPM has to be increased */
                // 1. The FAN2 (Air In) RPM has to be increase.
                // 2. If the FAN2 speed reaches the MAX rpm then FAN1 shall be used to control the Under pressure
                // 3. FAN1 (Exhaust FAN / Air Out) speed shall be reduce to compensate the pressure
                // 4. Even after this, If the pressure is not compensated then set same RPM on both the FANS
                // 5. Run both the FANS with same RPM for 10 min.
                //Under pressure condition
                /*Increase FAN in speed to supply air inside the cabin*/
                ptr_iv->prim_dev_id = DEV_FAN1_AIR_IN;
                ptr_iv->sec_dev_id  = DEV_FAN2_AIR_OUT;
            
            }

            
            if ( ( ptr_iv->set_rpm[ptr_iv->prim_dev_id] < fan_motor_control_db[ptr_iv->prim_dev_id].whole_rpm_range.max_rpm ) &&
                 ( ptr_iv->change_dev == false ) )
            {
                /* Rate of change in positive direction */
                ptr_iv->set_rpm[ptr_iv->prim_dev_id] += ptr_iv->rpm_step_table_dp[ptr_iv->step_table_index_dp];

                /* set the device compenastion configuration */
                ptr_iv->dev_comp_config[ptr_iv->prim_dev_id] = PRESS_COMP_DEV;
            }
            else if ( ptr_iv->set_rpm[ptr_iv->sec_dev_id] > fan_motor_control_db[ptr_iv->sec_dev_id].whole_rpm_range.min_rpm )
            {
                if ( ptr_iv->set_rpm[ptr_iv->sec_dev_id] > ptr_iv->rpm_step_table_dp[ptr_iv->step_table_index_dp] )
                {
                    /* Decrease the RPM by step level */
                    ptr_iv->set_rpm[ptr_iv->sec_dev_id] -=  ptr_iv->rpm_step_table_dp[ptr_iv->step_table_index_dp];
                }
                else //  current RPM < step value
                {
                    /* Set the RPM to zero */
                    ptr_iv->set_rpm[ptr_iv->sec_dev_id] = 0u;
                }

                /* set the device compenastion configuration */
                ptr_iv->dev_comp_config[ptr_iv->sec_dev_id] = PRESS_COMP_DEV;
            }
            else
            {
                /* Stop the timer */
                stop_wait_tmr();

                /* Set constant RPM and run for 10 minutes */
                for ( dev_id = DEV_FAN1_AIR_IN; dev_id <= DEV_FAN2_AIR_OUT; dev_id++ )
                {
                    ptr_iv->set_rpm[dev_id]         = PRESS_COMP_EXCEEDS_LIMIT_RPM;
                    ptr_iv->dev_comp_config[dev_id] = IDLE_COMP_DEV;
                }

                /* Reset the timer flag before starting teh timer */
                ptr_iv->wait_tmr_exp = false;
                /* Start timer to wait for 10 min */
                start_wait_tmr(MIN_TO_MSEC(RV_PRESS_COMP_EXCEED_COND_RUN_TIME_MIN));
                /* Change the state */
                iv_ctrl_state = INVENTILATE_STATE_PRESS_CTRL_EX_LIMIT;
            }
        }
    }
    else
    {
        /* stop the timer */
        stop_wait_tmr();
        /* reset the flag */
        ptr_iv->wait_tmr_exp = false;
        /* change the state */
        iv_ctrl_state = INVENTILATE_STATE_AQ_CTRL;
    }
    
    return iv_ctrl_state;
}

/**
  * @brief  Function to control the air quality by adjusting the RPM of FAN1 and FAN2 based on the IAQ data 
  * @param  ptr_iv - Pointer to the structure INVENTILATE_CONTROL_ALGO
  * @retval void
  */
INVENT_CONTROL_STATE aq_control_routine(INVENTILATE_CONTROL_ALGO* ptr_iv)
{
    INVENT_CONTROL_STATE iv_state = ptr_iv->curr_state;
    invent_device_id_t   dev_id;
    int32_t              roc_iaq_percent = 0;
    
    if ( IV0AQST_AIR_QUALITY_GOOD != ptr_iv->curr_iaq_stat )
    {
        /* Find the rate of change (ROC) of IAQ */
        roc_iaq_percent = calc_rate_of_change(*ptr_iv->ptr_curr_data, *ptr_iv->ptr_prev_data, ptr_iv->roc_max_val);

         // When the ROC is zero no need to change the RPM 
        if ( roc_iaq_percent != 0 ) 
        {
            /* Get the RPM step level from the ROC of IAQ */
            ptr_iv->step_table_index_iaq = get_step_level(abs(roc_iaq_percent));

#if INV_ALGO_DEBUG 
            LOG(I, "roc_iaq=%d curr_data=%d prev_data=%d roc_max_val=%d", \
            roc_iaq_percent, *ptr_iv->ptr_curr_data, *ptr_iv->ptr_prev_data, ptr_iv->roc_max_val);
#endif
            
            if ( roc_iaq_percent >= 0 )
            {
                /* Rate of change increased, So increase the RPM to reduce the IAQ level */
                for ( dev_id = DEV_FAN1_AIR_IN; dev_id <= DEV_FAN2_AIR_OUT; dev_id++ )
                {
                    ptr_iv->set_rpm[dev_id] += ptr_iv->rpm_step_table_iaq[ptr_iv->step_table_index_iaq];
                    /* set the device compenastion configuration */
                    ptr_iv->dev_comp_config[dev_id] = IAQ_COMP_DEV;
                }
            }
            else
            {
                /* Rate of change decreased, So reduce the RPM */
                for ( dev_id = DEV_FAN1_AIR_IN; dev_id <= DEV_FAN2_AIR_OUT; dev_id++ )
                {
                    if ( ptr_iv->set_rpm[dev_id] > ptr_iv->rpm_step_table_iaq[ptr_iv->step_table_index_iaq] )
                    {
                        /* Decrease the RPM by step level */
                        ptr_iv->set_rpm[dev_id] -=  ptr_iv->rpm_step_table_iaq[ptr_iv->step_table_index_iaq];
                    }
                    else
                    {
                        /* Set the RPM to zero */
                        ptr_iv->set_rpm[dev_id] = 0;
                    }
                    
                    /* set the device compenastion configuration */
                    ptr_iv->dev_comp_config[dev_id] = IAQ_COMP_DEV;
                }
            }
        }
    }
    else
    {
        /* IAQ good */
        /* Reset the timer flag, Before Start */
        ptr_ctrl_algo->wait_tmr_exp = false;
        /* Set the compensation configuration */
        ptr_iv->dev_comp_config[DEV_FAN1_AIR_IN] = IAQ_COMP_DEV;
        ptr_iv->dev_comp_config[DEV_FAN2_AIR_OUT]  = IAQ_COMP_DEV;
        /* Start the timer */
        start_wait_tmr(MIN_TO_MSEC(RV_IDLE_COND_WAIT_TIME_MIN));
        /* IAQ value is good ..Change the state to IDLE */
        iv_state = INVENTILATE_STATE_WAIT_FOR_IDLE_SETTLE;
    }

	return iv_state;
}

/**
  * @brief  Function to control device RPM based on the mode and data 
  * @param  ptr_iv - Pointer to the structure INVENTILATE_CONTROL_ALGO
  * @param  mode   - User selected mode
  * @retval void
  */
void update_dev_rpm(INVENTILATE_CONTROL_ALGO* ptr_iv, IV0MODE_ENUM mode)
{
	uint32_t duty_cycle   = 0u;
	uint32_t mode_min_rpm = 0u;
	uint32_t mode_max_rpm = 0u;
    invent_device_id_t  dev_id;

	for ( dev_id = DEV_FAN1_AIR_IN; dev_id < MAX_NUM_DEVICE; dev_id++ )
    {
        // Add comments
        if ( ( PRESS_COMP_DEV != ptr_iv->dev_comp_config[dev_id] ) || ( IV0MODE_OFF == mode ) || ( IV0MODE_STORAGE == mode ) )
        {
            /* RPM based on selected modes */ 
            mode_min_rpm = fan_motor_control_db[dev_id].rpm_range[mode].min_rpm;
            mode_max_rpm = fan_motor_control_db[dev_id].rpm_range[mode].max_rpm;
        }
        else
        {
            /* For pressure compensation the RPM range will not be restrict based on user selected mode */
            mode_min_rpm = fan_motor_control_db[dev_id].whole_rpm_range.min_rpm;
            mode_max_rpm = fan_motor_control_db[dev_id].whole_rpm_range.max_rpm;
        }

        if ( DEV_MOTOR != dev_id )
        {
            /* Zero RPM can be can be set in some scenarios */
            if ( ptr_iv->set_rpm[dev_id] != 0 )
            {
                /* Normalize the rpm */
                if ( ptr_iv->set_rpm[dev_id] < mode_min_rpm )
                {
                    ptr_iv->set_rpm[dev_id] = mode_min_rpm;
                }
                else if ( ptr_iv->set_rpm[dev_id] > mode_max_rpm )
                {
                    ptr_iv->set_rpm[dev_id] = mode_max_rpm;
                }
                else
                {
                    /* Do Nothing */
                }
            }
        }
        else
        {
            if ( ( ptr_iv->set_rpm[DEV_FAN1_AIR_IN] > 0u ) || ( ptr_iv->set_rpm[DEV_FAN2_AIR_OUT] > 0u ) )
            {
                /* Motor rpm shall not be controlled based IAQ or DP.
                   It is decided to run the ceramic disc motor in constant speed, when either the FAN1 or FAN2 is running */
                ptr_iv->set_rpm[DEV_MOTOR] = ptr_iv->constant_rpm_motor;
            }
            else
            {
                ptr_iv->set_rpm[DEV_MOTOR] = 0u;
            }
        }

        if ( ptr_iv->prev_set_rpm[dev_id] != ptr_iv->set_rpm[dev_id] )
        {
#if INV_ALGO_DEBUG 
        LOG(I, "mode = %d comp_cfg = %d, min_rpm = %d, max_rpm = %d, calc_rpm = %d", mode, \
            ptr_iv->dev_comp_config[dev_id], mode_min_rpm, mode_max_rpm, ptr_iv->set_rpm[dev_id]);
#endif
            /* Calculate duty cycle from rpm */
            duty_cycle = calc_duty_cycle(fan_motor_control_db[dev_id].whole_rpm_range.min_rpm, fan_motor_control_db[dev_id].whole_rpm_range.max_rpm, \
                                         FAN_MOTOR_DUTY_CYCLE_MIN, FAN_MOTOR_DUTY_CYCLE_MAX, ptr_iv->set_rpm[dev_id]);
            
            if ( INVERTED_PWM == fan_motor_control_db[dev_id].dev_pwm_mode )
            {
                duty_cycle = FAN_MOTOR_MAX_DUTY_CYCLE - duty_cycle;
            }
            
#if INV_ALGO_DEBUG 
            LOG(I, "dev_id=%d set_rpm=%d duty=%d", dev_id, ptr_iv->set_rpm[dev_id], duty_cycle);
#endif

            /* Set the duty cycle */
            hal_pwm_set_duty_cycle(CONNECTOR_PWM_FAN_MOTOR_CAPTURE_UNIT, dev_id, duty_cycle);

            /* store the current set rpm */
            ptr_iv->prev_set_rpm[dev_id] = ptr_iv->set_rpm[dev_id];

            /* Send the current set RPM to the broker */
            update_and_send_val_to_broker(MTR0SETSPD|DDM2_PARAMETER_INSTANCE(dev_id), ptr_iv->set_rpm[dev_id]);
        }
    }
}

/**
  * @brief  Function to set the parameters used to monitor the pressure compensation
  * @param  ptr_iv - Pointer to the structure INVENTILATE_CONTROL_ALGO
  * @retval None
  */
void update_dp_comp_threshold_val(INVENTILATE_CONTROL_ALGO* ptr_iv)
{
    /* Set the DP threshold */
    if ( IV0PRST_OVER_PRESS == ptr_iv->curr_pr_stat )
    {
        ptr_iv->dp_comp_thr_val = ptr_iv->curr_avg_dp_value + ( ptr_iv->dp_resolution * DP_POS_PRESS_COMP_VALIDATION_THRESHOLD );
    }
    else
    {
        ptr_iv->dp_comp_thr_val = ptr_iv->curr_avg_dp_value + ( ptr_iv->dp_resolution * DP_NEG_PRESS_COMP_VALIDATION_THRESHOLD );
    }

    /* Normalize the threshold value */
    if ( ptr_iv->dp_comp_thr_val > ptr_iv->dp_range_max )
    {
        ptr_iv->dp_comp_thr_val = ptr_iv->dp_range_max;
    }
    else if ( ptr_iv->dp_comp_thr_val < ptr_iv->dp_range_min )
    {
        ptr_iv->dp_comp_thr_val = ptr_iv->dp_range_min;
    }
    else
    {

    }
}

/**
  * @brief  Function to find the step level corresponding to the rate of change
  * @param  roc_percent - Absolute value of Rate of Change(ROC).
  * @retval step_level  - RPM step increment value.
  */
static uint8_t get_step_level(int32_t roc_percent)
{
    uint8_t rpm_step = 0;
	
	if ( ( roc_percent > ROC_STEP_LEVEL_1_MIN_PERCENT ) && ( roc_percent <= ROC_STEP_LEVEL_1_MAX_PERCENT ) )
    {
        rpm_step = RPM_STEP_LEVEL_1;
    }
    else if ( ( roc_percent >= ROC_STEP_LEVEL_2_MIN_PERCENT ) && ( roc_percent <= ROC_STEP_LEVEL_2_MAX_PERCENT ) )
    {
        rpm_step = RPM_STEP_LEVEL_2;
    }
    else if ( ( roc_percent >= ROC_STEP_LEVEL_3_MIN_PERCENT ) && ( roc_percent <= ROC_STEP_LEVEL_3_MAX_PERCENT ) )
    {
        rpm_step = RPM_STEP_LEVEL_3;
    }
	else if ( ( roc_percent >= ROC_STEP_LEVEL_4_MIN_PERCENT ) && ( roc_percent <= ROC_STEP_LEVEL_4_MAX_PERCENT ) )
    {
        rpm_step = RPM_STEP_LEVEL_4;
    }
	else 
	{
		rpm_step = RPM_STEP_LEVEL_5;
	}
	
	return rpm_step;
}

/**
  * @brief  Function to calculate the rate of change
  * @param  curr_val.
  * @param  prev_val.
  * @param  max_val.
  * @retval Rate of change.
  */
static int8_t calc_rate_of_change(float curr_val, float prev_val, float max_val)
{
	int32_t roc;
	
	roc = (int32_t) ( ( ( curr_val - prev_val ) / max_val ) * (float) 100 );
	
	return (int8_t)roc;
}

/**
  * @brief  Function to initialize RPM configuartion of FAN1 FAN2 and Motor
  * @param  None
  * @retval None
  */
void read_data_from_nvs(void)
{
    uint8_t index;
	hal_nvs_error nvs_err;

    // Plausibility check to be done
    // If success.....no change required
    // If failed default value should be configured

    // Read from NVS
    for ( index = 0u; index < NVS_DB_SIZE; index++ )
	{
        nvs_err = hal_nvs_read(nvs_db[index].nvs_key, nvs_db[index].data_type, nvs_db[index].data_ptr, nvs_db[index].data_size);
        
        if ( HAL_NVS_OK != nvs_err )
        {
#if INV_ALGO_DEBUG 
            LOG(E, "hal_nvs_read nvs_err = %d", nvs_err);	
#endif 
            // NVS error found ..Store defaut value
            *((uint32_t*)(nvs_db[index].data_ptr)) = nvs_db[index].default_val;

            // No data stored..May be new hardware
            if ( HAL_NVS_ERROR_NO_DATA_STORED == nvs_err )
            {
                // write the default value
                nvs_err = hal_nvs_write(nvs_db[index].nvs_key, nvs_db[index].data_type, nvs_db[index].data_ptr, nvs_db[index].data_size);
            }
        }
        else 
        {
            uint32_t data = *((uint32_t*)(nvs_db[index].data_ptr));
#if INV_ALGO_DEBUG 
            LOG(I, "hal_nvs_read data = %d", data);
#endif 
            // Plausibility check 
            if ( ( data < nvs_db[index].min_val ) || ( data > nvs_db[index].max_val ) )
            {
#ifdef INV_ALGO_DEBUG 
                LOG(E, "Plausible error data read = %d", data);	
#endif 
                // Plausibilty failed ..Store default value
                *((uint32_t*)(nvs_db[index].data_ptr)) = nvs_db[index].default_val;
            }
        }

        /* Update the value in database and send to broker */
        update_and_send_val_to_broker(nvs_db[index].ddm, *((uint32_t*)(nvs_db[index].data_ptr)));
    }
}

/**
  * @brief  Function to calculate min and max rpm corresponding to user mode
  * @param  void
  * @retval void
  */
void calc_mode_min_max_rpm(void)
{
    invent_device_id_t dev_id;
    uint8_t mode_idx;

    fan_motor_control_db[DEV_FAN1_AIR_IN].dev_pwm_mode      = DEV_FAN1_AIR_IN_PWM_MODE;
    fan_motor_control_db[DEV_FAN2_AIR_OUT].dev_pwm_mode     = DEV_FAN2_AIR_OUT_PWM_MODE;
    fan_motor_control_db[DEV_MOTOR].dev_pwm_mode            = DEV_MOTOR_PWM_MODE;
    for ( dev_id = 0u; dev_id < MAX_NUM_DEVICE; dev_id++ )
	{
		for ( mode_idx = 0u; mode_idx < NUM_OPERATING_MODES; mode_idx++ )
		{
			fan_motor_control_db[dev_id].rpm_range[mode_idx].min_rpm = calc_percentage(fan_motor_control_db[dev_id].whole_rpm_range.max_rpm, percent_range[mode_idx].min_percent);
			fan_motor_control_db[dev_id].rpm_range[mode_idx].max_rpm = calc_percentage(fan_motor_control_db[dev_id].whole_rpm_range.max_rpm, percent_range[mode_idx].max_percent);
#if	INV_ALGO_DEBUG 	
			LOG(I, "dev_id=%d mode_idx %d min_rpm = %d max_rpm = %d", dev_id, mode_idx, fan_motor_control_db[dev_id].rpm_range[mode_idx].min_rpm, fan_motor_control_db[dev_id].rpm_range[mode_idx].max_rpm);
#endif 
		}
	}

#if	INV_ALGO_DEBUG 
    for ( mode_idx = 0u; mode_idx < IAQ_RANGE_LEVELS; mode_idx++ )
	{
        LOG(I, "IAQ level %d min = %d max %d", mode_idx, iaq_range_level[mode_idx].iaq_range.min, iaq_range_level[mode_idx].iaq_range.max);
    }
#endif
}

/**
  * @brief  Function to calculate the percentage of RPM range
  * @param  Value
  * @param  Percentage of value
  * @retval Calculated percentage of value
  */
static uint32_t calc_percentage(uint32_t value, uint32_t percent) 
{
	uint32_t calc_percent = 0u;

	if ( ( value > 0u ) && ( percent > 0u ) )
	{
		calc_percent = (value * percent) / 100;
	}
	
	return calc_percent;
}

/**
  * @brief  Function to reset the device configuartion
  * @param  none.
  * @retval none.
  */
void reset_dev_config(void)
{
    /* Reset the dev compensation config status */
    ptr_ctrl_algo->dev_comp_config[DEV_FAN1_AIR_IN]     = IDLE_COMP_DEV;
    ptr_ctrl_algo->dev_comp_config[DEV_FAN2_AIR_OUT]    = IDLE_COMP_DEV;
    ptr_ctrl_algo->dev_comp_config[DEV_MOTOR]           = IDLE_COMP_DEV;       
    ptr_ctrl_algo->set_rpm[DEV_FAN1_AIR_IN]             = fan_motor_control_db[DEV_FAN1_AIR_IN].rpm_range[ptr_ctrl_algo->cur_sel_mode].min_rpm;
    ptr_ctrl_algo->set_rpm[DEV_FAN2_AIR_OUT]            = fan_motor_control_db[DEV_FAN2_AIR_OUT].rpm_range[ptr_ctrl_algo->cur_sel_mode].min_rpm;
}

#endif /* APP_FAN_MOTOR_CONTROL */
