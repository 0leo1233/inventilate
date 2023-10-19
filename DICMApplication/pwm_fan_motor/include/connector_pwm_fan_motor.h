/*! \file connector_pwm_fan_motor.h
	\brief Header file for connector pwm fan and motor
	\Author Sundaramoorthy-LTTS
 */
#ifndef CONNECTOR_PWM_FAN_MOTOR_H_
#define CONNECTOR_PWM_FAN_MOTOR_H_

/** Includes ******************************************************************/
#include "configuration.h"

#ifdef CONNECTOR_PWM_FAN_MOTOR

#include "connector.h"

/* Function pointer declaration */
typedef void (*conn_mtr_param_changed_t)(uint32_t dev_id, int32_t i32Value);

/* Struct Type Definitions */
typedef struct
{
    uint32_t ddm_parameter;
    DDM2_TYPE_ENUM type;
    uint8_t pub;
    uint8_t sub;
    int32_t i32Value;
    conn_mtr_param_changed_t cb_func;
} conn_fan_motor_parameter_t;

/* Extern Declarations */
extern CONNECTOR connector_pwm_fan_motor;

/* Macro Definitions */
#define CONN_PWM_DEBUG_LOG             0u
#define DP_ERR_DEBUG                   0u
#define CONN_PWM_FAN_MTR_SUB_DEPTH	   50
#define DP_BAT_LIMIT                   2700u

#endif //CONNECTOR_FAN_MOTOR

#endif //CONNECTOR_PWM_FAN_MOTOR_H_

