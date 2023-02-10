/*! \file app_light_ctrl.h
	\brief This header file contains structure definition, enum definitions for related to app light control.
 */

#ifndef APP_LIGHT_CTRL_H_
#define APP_LIGHT_CTRL_H_

#include "dicm_framework_config.h"

#ifdef APP_LIGHT_CONTROL

typedef enum __dim_level_cmd
{
    DIM_LEVEL_0_OFF = 0x0000,
    DIM_LEVEL_1     = 0x0010,
    DIM_LEVEL_2     = 0x0100,
    DIM_LEVEL_3     = 0x0101,
    DIM_LEVEL_4     = 0x0110,
    DIM_LEVEL_5     = 0x1000,
    DIM_LEVEL_6     = 0x1010,
    DIM_NUM_LEVELS  = 7
}DIM_LEVEL_COMMAND;

typedef enum __dim_level_duty_cycle
{
    DIM_LVL_DUTY_CYCLE_0    =   0,
    DIM_LVL_DUTY_CYCLE_20   =  20,
    DIM_LVL_DUTY_CYCLE_40   =  40,
    DIM_LVL_DUTY_CYCLE_50   =  50,
    DIM_LVL_DUTY_CYCLE_60   =  60,
    DIM_LVL_DUTY_CYCLE_80   =  80,
    DIM_LVL_DUTY_CYCLE_100  = 100,
}DIM_LEVEL_DUTY_CYCLE;

typedef struct _dim_level
{
  DIM_LEVEL_COMMAND        dim_level;
  DIM_LEVEL_DUTY_CYCLE    duty_cycle;
}DIM_LEVEL;

#endif // APP_LIGHT_CONTROL

#endif // APP_LIGHT_CTRL_H_ 
