/*! \file connector_voc_sensor.h
	\brief Header file for connector VOC sensor
	\Author Sundaramoorthy-LTTS
 */

#ifndef CONNECTOR_VOC_SENSOR_H_
#define CONNECTOR_VOC_SENSOR_H_

/** Includes ******************************************************************/
#include "configuration.h"
#ifdef CONNECTOR_VOC_SENSOR
#include "connector_voc_sensor_api.h"

#include "connector.h"

typedef void (*conn_voc_param_changed_t )(uint8_t table_index, int32_t i32Value);

typedef struct
{
    uint32_t ddm_parameter;
    DDM2_TYPE_ENUM type;
    uint8_t pub;
    uint8_t sub;
    int32_t i32Value;
    conn_voc_param_changed_t cb_func;
} conn_voc_sensor_param_t;

extern CONNECTOR connector_voc_sensor;

#define VOC_MEAS_TYPE_TEMP BME680_MEAS_TYPE_TEMP

#define CONN_VOC_READ_TASK_DELAY_MSEC                 ((uint8_t) 10u)

#endif /* CONNECTOR_VOC_SENSOR */
#endif /* CONNECTOR_VOC_SENSOR_H_ */

