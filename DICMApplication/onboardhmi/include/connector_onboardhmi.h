/*! \file connector_onboardhmi.h
	\brief Header file for connector Onboard HMI
	\Author Sundaramoorthy-LTTS
 */

#ifndef CONNECTOR_ONBOARD_HMI_H_
#define CONNECTOR_ONBOARD_HMI_H_

/** Includes ******************************************************************/
#include "dicm_framework_config.h"
#include "ddm2_parameter_list.h"

#ifdef CONNECTOR_ONBOARD_HMI

/** Includes ******************************************************************/
#include "connector.h"

/* Macro Definitions */


/* Function pointer declaration */
typedef void (*conn_onbhmi_param_changed_t)(uint32_t ddm_param, int32_t i32value);

/* Struct Type Definitions */
typedef struct
{
    uint32_t ddm_parameter;
    DDM2_TYPE_ENUM type;
    uint8_t pub;
    uint8_t sub;
    int32_t i32Value;
    conn_onbhmi_param_changed_t cb_func;
} conn_onboardhmi_param_t;

/* Extern Declarations */
extern CONNECTOR connector_onboard_hmi;

#endif /* CONNECTOR_ONBOARD_HMI */
#endif /* CONNECTOR_ONBOARD_HMI_H_ */
