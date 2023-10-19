/*! \file connector_eol_service.h
	\brief Header file for connector eol service
	
 */
#ifndef CONNECTOR_EOL_SERVICE_H_
#define CONNECTOR_EOL_SERVICE_H_

/** Includes ******************************************************************/
#include "configuration.h"

#ifdef CONNECTOR_EOL_SERVICE

#include "connector.h"
#include "eolpcba_invent.h"

/* Macro Definitions */
#define CONN_EOL_DEBUG_LOG      1u
#define CONN_EOL_SUB_DEPTH      50
#define DDMP_UNAVAILABLE       ((uint8_t) 0xFFu)
#define LINTX_FRAME_SIZE_MAX		20

#define CONVERT_DATAID         ((uint8_t) 0x0F)
#define START_TESTID           ((uint8_t) 0xD0)
#define END_TESTID             ((uint8_t) 0xDF) 

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

extern CONNECTOR connector_eol;

#endif // CONNECTOR_EOL_SERVICE

#endif // CONNECTOR_EOL_SERVICE_H_