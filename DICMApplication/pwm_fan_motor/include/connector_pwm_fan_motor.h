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


/* Extern Declarations */
extern CONNECTOR connector_pwm_fan_motor;

/* Macro Definitions */
#define CONN_PWM_DEBUG_LOG             0u
#define DP_ERR_DEBUG                   0u

#endif //CONNECTOR_FAN_MOTOR

#endif //CONNECTOR_PWM_FAN_MOTOR_H_

