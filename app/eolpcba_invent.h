/*! \file eol_invent.h
	\brief EOL invent header

 */
#ifndef EOL_INVENT_H
#define EOL_INVENT_H

/** Includes ******************************************************************/
#include <stdio.h>
#include <stdlib.h>

/** Defines ******************************************************************/

//#define STATUS_FILE         (uint8_t)1

#define INVENTEOL_TEST
//#define INVENTPCBA_TEST


#define EOL_FAN_MIN_RPM 	((uint32_t)500u)
#define EOL_FAN_MID_RPM		((uint32_t)1000u)
#define EOL_FAN_MAX_RPM		((uint32_t)2000u)

#define MOTOR_SPEED			((uint32_t)0u)
#define MOTOR_SPEED_2		((uint32_t)10u)
#define MOTOR_SPEED_3		((uint32_t)4u)

#define BACKLIGHT_DUTY_CYCLE_MIN    ((uint8_t)0u)
#define BACKLIGHT_DUTY_CYCLE_MAX    ((uint8_t)100u)
#define BACKLIGHT_INCR_LVL          ((uint8_t)25)
#define DISP_WAIT_TIME              ((uint8_t)1000)

#define EOL_DATA_BYTE               ((uint8_t) 0x00)


#ifdef INVENTPCBA_TEST
typedef enum __eol_test_id
{
    TEST_START       = 0,
    TEST_SWVER       = 1,
	TEST_HWVER       = 2,
    TEST_DISP    	 = 3,
    TEST_FANMTRRPM   = 4,
    TEST_TACHO       = 5,
    TEST_LEDSTRIP    = 6,
    TEST_VOC         = 7,
    TEST_BLE         = 8,
    TEST_WIFI        = 9,
    TEST_CAN         = 10,
    TEST_LIN         = 11,
    //TEST_BATTERY     = 15,
    TEST_END = 12,
    TEST_STOP = 13,
}EOL_TEST_ID;
#endif

#ifdef INVENTEOL_TEST
typedef enum __eol_test_id
{
    TEST_START  = 0,    //      0xD0    
    TEST_SWVER,         //1     0xD1
    TEST_HWVER,         //2     0xD2
    TEST_DISP_SEG,      //3     0xD3
    TEST_DISP_BRGHT,    //4     0xD4
    TEST_FAN1,          //5     0xD5
    TEST_FAN2,          //6     0xD6
    TEST_MOTOR,         //7     0xD7
    TEST_LEDSTRIP,      //8     0xD8
    TEST_VOC,           //9     0xD9
    TEST_DIFFSEN,       //10    0xDA
    TEST_IONIZ,         //11    0xDB
    TEST_BLE,           //12    0xDC
    TEST_WIFI,          //13    0xDD
    TEST_CAN,           //14    0xDE
    TEST_LIN,           //15    0xDF
    //TEST_BATTERY,
    TEST_END,           //16    0xE0
    TEST_STOP,           //17    0xE1
    TEST_MAX
}EOL_TEST_ID;
#define TEST_RESULT     TEST_MAX

#endif

typedef enum _error_frame
{
    PCBA_EOL_NO_ERR             = 0xA0,
    PCBA_EOL_ERR_TIMEOUT        = 0xA1,
    PCBA_EOL_ERR_NOBYTES        = 0xA2,
    PCBA_EOL_ERR_TESTID         = 0xA3,
    PCBA_EOL_ERR_DATARX         = 0xA4,
    PCBA_EOL_ERR_COMM           = 0xA5,
    PCBA_EOL_ERR_DATAMISMATCH   = 0xA6,
}EOL_ERROR_FRAME;


typedef struct __eol_data_frame
{
    uint16_t        data;
    EOL_TEST_ID     data_id;
    EOL_ERROR_FRAME eol_ack;    
}EOL_DATA_FRAME;

typedef struct 
{
    uint8_t pcba_tstid;
    uint16_t pcba_data;
    EOL_ERROR_FRAME pcba_ack;
}PCBA_DATA_FRAME;

/** Variables ******************************************************************/
typedef int32_t error_type;

/** Function declarations *******************************************************/
//void initialize_eol_invent(EOL_TST_ID invent_id);
extern int8_t check_can_status(EOL_DATA_FRAME eol_data_frame);
EOL_ERROR_FRAME displaytest(EOL_DATA_FRAME* eol_data_frame);
extern void set_tacho(uint8_t dev_id, int32_t data);
extern void set_lightstrip(EOL_DATA_FRAME eol_data_frame);
EOL_ERROR_FRAME validate_fan_mtr_rpm(EOL_DATA_FRAME* ptr_eol_frame, uint16_t* fan_rpm);
//extern uint8_t check_wifi(void);

#if 0
/**
 * @brief Get the wifistatus object
 * 
 * @param[out] rssi_value wifi signal strength 
 * @return uint8_t 
 */
uint8_t get_wifistatus(int8_t *rssi_value);
#endif

#endif