/*! \file app_voc_sensor.h
	\brief This header file contains structure definition, enum definitions for related to VOC sensor application.
 */

#ifndef APP_VOC_SENSOR_H_
#define APP_VOC_SENSOR_H_

#include "dicm_framework_config.h"

#ifdef APP_VOC_SENSOR

#include "osal.h"

#if ( CONFIG_DICM_SUPPORT_INTEGRATED_BSEC_LIB_1_X == 1 )
#include "bsec_integration.h"
#endif

#if ( CONFIG_DICM_SUPPORT_INTEGRATED_BSEC_LIB_2_X == 1 )
#include "bsec_integration_2_x.h"
#endif

#define BME680_MAX_ERR_COUNT  ((uint8_t) 2u)

#define PLAUSIBLE_ERROR                   ((uint8_t) 1u)
#define PLAUSIBLE_OK                      ((uint8_t) 0u)

#define IAQ_CALC_ERROR                    ((uint8_t) 1u)
#define IAQ_CALC_SUCCESS                  ((uint8_t) 0u)

#define BME680_SENSOR_READ_DELAY_TIME_MS  ((uint32_t) 10000u)

#define BSEC_NEXT_CALL_TIME_OFFSET        ((uint32_t) 100u)

#define BME68X_TEMPERATURE_OFFSET         ((float)  0.0f)

typedef enum _voc_rd_sm_state
{
  VOC_TASK_STATE_IDLE = 0,
  VOC_TASK_STATE_GET_SENSOR_CONFIG = 1,
  VOC_TASK_STATE_RD_VOC_SENSOR = 2,
  VOC_TASK_STATE_PLAUSIBILITY_CHECK = 3, 
  VOC_TASK_STATE_CALC_AND_PUBLISH_IAQ = 4,
  VOC_TASK_STATE_STATE_ERROR = 5
}voc_rd_sm_state;

typedef enum _voc_rd_error_code
{
  VOC_SENSOR_NO_ERROR = 0,
  VOC_SENSOR_PLAUSIBLE_ERROR = 1,
  VOC_SENSOR_COMM_ERROR = 2,
  VOC_SENSOR_TIME_OUT_ERROR = 3
}voc_rd_error_code;

typedef struct _read_voc_data_service
{
  voc_rd_sm_state       sm_state;
  voc_rd_error_code     error_code;
  uint8_t               rd_seq_cnt;
  uint8_t               rd_err_cnt;
  uint32_t              iaq_index;
  uint32_t              aqrc_lvl;
  uint32_t              read_sens_delay;
  volatile TickType_t   last_tick;
  volatile uint32_t     wait_time_ms;
  uint32_t              bsec_ctrl_time_stamp;
}read_voc_data_service;

extern uint32_t bsec_critical_error;
#endif /*APP_VOC_SENSOR*/

#endif /*APP_VOC_SENSOR_H_*/
