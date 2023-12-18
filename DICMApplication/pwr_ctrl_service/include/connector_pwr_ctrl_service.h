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

typedef struct 
{
    uint8_t charging_current_exceed:1;

}inv_bat_status_flags;

typedef struct __batt_reg
{
    uint16_t bat_volt;                 //charging voltage           VBAT
    uint16_t bat_curr;                 //charging current           IBAT
    uint16_t veh_batt_volt;            //vehicle batt voltage       VAC1
    uint16_t solar_volt;               //Solar voltage              VAC2
    uint8_t chrg_stat;                 //Charging status
    uint16_t batt_ip_curr;             //Batttery input current     IBUS
} batt_reg;

/* Extern Declarations */
extern CONNECTOR connector_pwr_ctrl_service;
batt_reg get_battval(void);
int16_t power_consumption(void);
#endif //CONNECTOR_POWER_CONTROL_SERVICE

#endif //CONNECTOR_PWR_CTRL_SERVICE_H_

