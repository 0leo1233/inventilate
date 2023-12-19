/*! \file connector_eol_service.h
	\brief Header file for connector eol service
	
 */
#ifndef CONNECTOR_EOL_SERVICE_H_
#define CONNECTOR_EOL_SERVICE_H_

/** Includes ******************************************************************/
#include "configuration.h"

#ifdef CONNECTOR_EOL_SERVICE

#include "connector.h"

/* Macro Definitions */
#define BQ25672_PART_NUMBER     1u

#define CONN_EOL_DEBUG_LOG      1u
#define CONN_EOL_SUB_DEPTH      50
#define DDMP_UNAVAILABLE       ((uint8_t) 0xFFu)
#define LINTX_FRAME_SIZE_MAX		20

#define CONVERT_DATAID         ((uint8_t) 0x0F)
#define START_TESTID           ((uint8_t) 0xD0)
#define END_TESTID             ((uint8_t) 0xEF)
#define NO_OF_BYTES            ((uint8_t) 4)
#define START_RESP_ID          ((uint8_t) 0xB0) 
#ifdef INVENT_PCBA
#define END_TESTID             ((uint8_t) 0xDF)
#endif 

/* Function pointer declaration */
typedef void (*conn_eol_param_changed_t)(uint32_t dev_id, int32_t i32Value);

/* Struct Type Definitions */
typedef struct __conn_eol_parameter_t
{
    uint32_t ddm_parameter;
    DDM2_TYPE_ENUM type;
    uint8_t pub;
    uint8_t sub;
    int32_t i32Value;
    conn_eol_param_changed_t cb_func;
} conn_eol_parameter_t;

typedef struct
{
	uint8_t bus_instance;
	uint8_t data_len;
	uint8_t data[LINTX_FRAME_SIZE_MAX];
} lin_send_data_t;

/* Enum Type Definitions */
typedef enum _PWR_CONSUMPTION
{
    NOLOAD  = 0,
    GET_CURRENT = 1,
    SET_FAN1_RPM = 2,
    SET_FAN2_RPM = 3,
    SET_MOTOR_RPM = 4,
    SET_LED_BRIGNTNESS = 5,
}PWR_CONSUMPTION;

typedef union
{
	uint8_t byte;

    struct 
	{
		uint8_t PART_NUMBER     : 3;  // BIT 0 to BIT 2
        uint8_t DEV_REV         : 3;  // BIT 3 to BIT 5
        uint8_t RESERVED        : 2;  // BIT 6 and 7
	} __attribute__((packed));
}REG48_PART_INFO_REG_BQ25672;

extern CONNECTOR connector_eol;
extern int16_t* power_consumption_eol(PWR_CONSUMPTION pwrused_dev);

#endif // CONNECTOR_EOL_SERVICE

#endif // CONNECTOR_EOL_SERVICE_H_
