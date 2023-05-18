/*! \file app_error_code.h
	\brief This header file contains structure definition, enum definitions for inventilate error status
 */

#ifndef APP_ERROR_CODE_H_
#define APP_ERROR_CODE_H_

typedef enum __invent_err_code_bit_pos
{
    VOC_SENSOR_COMMUNICATION_ERROR   =  0,
    VOC_SENSOR_DATA_PLAUSIBLE_ERROR  =  1,
    DP_SENSOR_BOARD_DISCONNECTED     =  2,
    DP_SENSOR_NO_DATA_ERROR          =  3,
    DP_SENSOR_DATA_PLAUSIBLE_ERROR   =  4,
    DP_SENSOR_BOARD_BATTERY_LOW      =  5,
    DP_SENSOR_BOARD_CONN_RETRY       =  6,
    //ERROR_CODE:   FAN1
    FAN1_RPM_MISMATCH                =  7,
    FAN1_TACHO_READ_DEVICE_INACTIVE  =  8,
    FAN1_NO_TACHO_DEVICE_ACTIVE      =  9,
    //ERROR_CODE:   FAN2
    FAN2_RPM_MISMATCH                = 10,
    FAN2_TACHO_READ_DEVICE_INACTIVE  = 11,
    FAN2_NO_TACHO_DEVICE_ACTIVE      = 12,
    //ERROR_CODE:   Motor
    MOTOR_RPM_MISMATCH               = 13,
    MOTOR_TACHO_READ_DEVICE_INACTIVE = 14,
    MOTOR_NO_TACHO_DEVICE_ACTIVE     = 15,
    //Error_code:   BMS
    BACKUP_BATTERY_LOW               = 16,
    CAR_BATTERY_INPUT_NOT_FOUND      = 17,
    SOLAR_INPUT_NOT_FOUND            = 18,
    COMM_ERROR_WITH_BATTERY_IC       = 19,
    BATTERY_OVER_HEATING             = 20,
    BATTERY_EXPIRED                  = 21,
    //Error_code:   LCD
    COMM_ERROR_WITH_LCD_DRIVER_IC    = 22,
    TOUCH_BUTTON_EVENT_PROCESS_ERROR = 23,
    CAN_COMMUNICATION_ERROR          = 24,
    LIN_COMMUNICATION_ERROR          = 25,
    MAX_NUM_ERROR_CODES              = 26
}INVENT_ERR_CODE_BIT_POS;

#endif /* APP_ERROR_CODE_H_ */


