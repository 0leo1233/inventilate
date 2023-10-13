/*! \file app_light_ctrl.h
	\brief This header file contains structure definition, enum definitions for related to app light control.
 */

#ifndef APP_LIGHT_CTRL_H_
#define APP_LIGHT_CTRL_H_

#include "configuration.h"

#ifdef APP_LIGHT_CONTROL


//LED strip Brightness configuration
typedef enum __dim_level_duty_cycle
{
    DIM_LVL_DUTY_CYCLE_0    =   0,
    DIM_LVL_DUTY_CYCLE_5   =    5,
    DIM_LVL_DUTY_CYCLE_40   =  40,
    DIM_LVL_DUTY_CYCLE_100  = 100,
}DIM_LEVEL_DUTY_CYCLE;


#endif // APP_LIGHT_CONTROL

#endif // APP_LIGHT_CTRL_H_ 