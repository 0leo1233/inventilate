/*! \file connector_dp_sens_service.h
	\brief Header file for connector dp sensor service
	\Author Sundaramoorthy-LTTS
 */

#ifndef CONNECTOR_DP_SENS_SERVICE_H_
#define CONNECTOR_DP_SENS_SERVICE_H_

/** Includes ******************************************************************/
#include "configuration.h"

#include "connector.h"

typedef enum __dpsens_err
{
    DPSENS_NO_ERROR                 = 0,
    DPSENS_BOARD_DISCONNECTED       = 1,
    DPSENS_BOARD_CONN_RETRY         = 2,
    DPSENS_NO_DATA_ERROR            = 3,
    DPSENS_DATA_PLAUSIBLE_ERROR     = 4,
    DPSENS_BOARD_BATTERY_LOW        = 5
} DPSENS_ERROR;


typedef void (*conn_diff_param_changed_t )( int32_t i32Value);

typedef struct
{
    uint32_t ddm_parameter;
    DDM2_TYPE_ENUM type;
    uint8_t pub;
    uint8_t sub;
    int32_t i32Value;
    conn_diff_param_changed_t cb_func;
} conn_diff_press_sensor_param_t;

extern CONNECTOR connector_diffpress_sensor;

#define CONN_DIFF_PRESS_READ_TASK_DELAY_MSEC                 ((uint32_t) 1000u)

#endif /* CONNECTOR_DP_SENS_SERVICE_H_ */

