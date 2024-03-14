/*! \file app_power_ctrl_service.h
	\brief This header file contains structure definition, enum definitions for Application Power Control Service.
 */

#ifndef APP_POWER_CTRL_SERVICE_H_
#define APP_POWER_CTRL_SERVICE_H_

#include "dicm_framework_config.h"

#ifdef APP_POWER_CONTROL_SERVICE

typedef enum __vehicle_run_status
{
    VEHICLE_STATUS_UNKNOWN = 0,
	VEHICLE_STATUS_HALTED  = 1,
	VEHICLE_STATUS_RUNNING = 2
}VEHICLE_RUN_STATUS;

typedef enum __filter_reset_status
{
    FILTER_IDLE              = 0,
    FILTER_STARTED           = 1,
    FILTER_TIME_EXPIRED      = 2
}FILTER_RESET_STATUS;

typedef enum __pwr_data_id
{
    INVENT_POWER_STATUS         = 0,
    INVENT_VEHICLE_RUN_STATUS   = 1,
    INV_FIL_TMR_EXP             = 2,
    INVENT_FILTER_RESET_REQ     = 3,
    INVENT_STORAGE_MODE_SEL     = 4,
    INVENT_PWR_CTRL_STATE       = 5,
    INVENT_POWER_SOURCE_CHANGED = 6,
    INVENT_SET_CHARGING_CURRENT = 7,
    INVALID_DATA                = 8
}PWR_CTRL_DATA_ID;

typedef struct __pwr_ctrl_data
{
    int32_t           data;
    PWR_CTRL_DATA_ID  data_id;
}PWR_CTRL_DATA;

typedef struct __pwr_ctrl_sm
{
    IVPMGR0STATE_ENUM      prev_set_state;
    IVPMGR0STATE_ENUM      inv_pwr_ctrl_state;
	VEHICLE_RUN_STATUS     veh_running_status;
    IV0PWRON_ENUM          invent_pwr_mode;
    IV0STORAGE_ENUM        storage_mode_sel;
    uint32_t               filter_min_counter;
    uint32_t               filter_sec_counter;
    FILTER_RESET_STATUS    filter_cur_status;
    FILTER_RESET_STATUS    filter_prev_status;
    IV0PWRSRC_ENUM         active_power_src;
}PWR_CTRL_SM;

#endif /* APP_POWER_CONTROL_SERVICE */

#endif /* APP_POWER_CTRL_SERVICE_H_ */
