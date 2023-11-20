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

typedef enum 
{
    PWR_BACKUP_BAT_NORMAL            = 0,
    PWR_BACKUP_BAT_LOW               = 1,
    PWR_CAR_BAT_INPUT_NOT_FOUND      = 2,
    PWR_CAR_BAT_INPUT_FOUND          = 3,
    PWR_SOLAR_INPUT_NOT_FOUND        = 4,
    PWR_SOLAR_INPUT_FOUND            = 5,
    PWR_COMM_ERROR_WITH_BAT_IC       = 6,
    PWR_BAT_OVER_HEATING             = 7,
    PWR_BAT_COOL                     = 8,   
    PWR_BAT_EXPIRED                  = 9,
}PWR_CTRL_ERR_CODES;




/*
typedef union
{
	uint8_t byte;
    
    struct 
	{
        uint8_t RESERVED           : 1;  // BIT 0
        uint8_t PRECHG_TMR_STAT    : 1;  // BIT 1
        uint8_t TRICHG_TMR_STAT    : 1;  // BIT 2
        uint8_t CHG_TMR_STAT       : 1;  // BIT 3
        uint8_t VSYS_STAT          : 1;  // BIT 4
        uint8_t ADC_DONE_STAT      : 1;  // BIT 5
        uint8_t ACRB1_STAT         : 1;  // BIT 6
        uint8_t ACRB2_STAT         : 1;  // BIT 7
    } __attribute__((packed));
}REG1E_CHARGER_STATUS_3_REG;
*/


/* Extern Declarations */
extern CONNECTOR connector_pwr_ctrl_service;

#endif //CONNECTOR_POWER_CONTROL_SERVICE

#endif //CONNECTOR_PWR_CTRL_SERVICE_H_

