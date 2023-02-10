/*! \file connector_pwr_ctrl_service.h
	\brief Header file for Connector for Power Control Service
	\Author Sundaramoorthy-LTTS
 */
#ifndef CONNECTOR_PWR_CTRL_SERVICE_H_
#define CONNECTOR_PWR_CTRL_SERVICE_H_

/** Includes ******************************************************************/
#include "configuration.h"

#ifdef CONNECTOR_POWER_CONTROL_SERVICE

#include "connector.h"

/* Function pointer declaration */
typedef void (*conn_pwr_ctrl_serv_param_ch_t)(uint32_t ddm_param, int32_t i32Value);

/* Struct Type Definitions */
typedef struct
{
    uint32_t ddm_parameter;
    DDM2_TYPE_ENUM type;
    uint8_t pub;
    uint8_t sub;
    int32_t i32Value;
    conn_pwr_ctrl_serv_param_ch_t cb_func;
} conn_pwr_ctrl_parameter_t;

/* Extern Declarations */
extern CONNECTOR connector_pwr_ctrl_service;

#endif //CONNECTOR_POWER_CONTROL_SERVICE

#endif //CONNECTOR_PWR_CTRL_SERVICE_H_

