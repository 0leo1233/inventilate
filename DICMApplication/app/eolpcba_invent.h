/*! \file eol_invent.h
	\brief EOL invent header

 */
#ifndef EOL_INVENT_H
#define EOL_INVENT_H

/** Includes ******************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

/** Defines ******************************************************************/

//#define STATUS_FILE         (uint8_t)1

#define INVENTEOL_TEST
//#define INVENTPCBA_TEST

#define EOL_FAN_MIN_RPM 	((uint32_t)500u)
#define EOL_FAN_MID_RPM		((uint32_t)1000u)
#define EOL_FAN_MAX_RPM		((uint32_t)2800u)

#define MOTOR_SPEED			((uint32_t)0u)
#define MOTOR_SPEED_2		((uint32_t)10u)
#define MOTOR_SPEED_3		((uint32_t)4u)

#define BACKLIGHT_DUTY_CYCLE_MIN    ((uint8_t)0u)
#define BACKLIGHT_DUTY_CYCLE_MAX    ((uint8_t)100u)
#define BACKLIGHT_INCR_LVL          ((uint8_t)25)
#define DISP_WAIT_TIME              ((uint8_t)1000)

#define EOL_DATA_BYTE               ((uint8_t) 0x00)
#define SEG_ALL_ON                  ((uint16_t) 0xFFFF)
#define SEG_ALL_OFF                 ((uint16_t) 0xEEEE)

//#define POWER_CONSUMPTION         

#ifdef INVENTPCBA_TEST
typedef enum __eol_test_id
{
    TEST_START       = 0,   //D0
    TEST_SWVER       = 1,   //D1
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
    TEST_BATTERY     = 12,
    TEST_MACID      = 13,
    TEST_END = 14,
    TEST_STOP = 15,
    TEST_MAX
}EOL_TEST_ID;
#endif

#ifdef INVENTEOL_TEST
//!< Make sure this is in sync with enum "TEXTBOX_INFO"
typedef enum __eol_test_id
{
    TEST_START  = 0,    //0     0xD0    
    TEST_SWVER,         //1     0xD1
    TEST_HWVER,         //2     0xD2
    TEST_DISP_SEG,      //3     0xD3
    TEST_DISP_BRGHT,    //4     0xD4
    TEST_FAN1,          //5     0xD5
    TEST_FAN2,          //6     0xD6
    TEST_MOTOR,         //7     0xD7
    TEST_TACHO,         //8     0xD8
    TEST_LEDSTRIP,      //9     0xD9
    TEST_IONIZ,         //10    0xDA
    TEST_DIFFSEN,       //11    0xDB
    TEST_VOC,           //12    0xDC
    TEST_BLE,           //13    0xDD
    TEST_WIFI,          //14    0xDE
    TEST_CAN,           //15    0xDF
    TEST_LIN,           //16    0xE0
    TEST_CHGVOLT,       //17    0xE1
    TEST_CHGCURR,       //18    0xE2
    TEST_CHGMODE,       //19    0xE3
    TEST_12VSRCVOLT,    //20    0xE4
    TEST_SOLARSRCVOLT,  //21    0xE5
    TEST_BATTERY,       //22    0xE6
    TEST_MACID,         //23    0xE7
    TEST_PWRCONSUMPTION,//XX    0xEX
    TEST_END,           //24    0xE8
    TEST_STOP,          //25    0xE9
    TEST_MAX            //26    0xEA
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
    PCBA_EOL_OUT_OF_RANGE       = 0xA7,
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
    uint32_t pcba_data;
    EOL_ERROR_FRAME pcba_ack;
}PCBA_DATA_FRAME;

typedef struct __disp_segment
{
    uint8_t seg_num;
    uint8_t seg_status;
}DISP_CTRL;

typedef struct __pwrctrl
{
    uint8_t pwr_stat;
}PWR_CTRL;

/** Variables ******************************************************************/
typedef int32_t error_type;

/** Function declarations *******************************************************/
//void initialize_eol_invent(EOL_TST_ID invent_id);
//extern int8_t check_can_status(EOL_DATA_FRAME eol_data_frame);
//extern void set_tacho(uint8_t dev_id, int32_t data);
//EOL_ERROR_FRAME validate_fan_mtr_rpm(EOL_DATA_FRAME* ptr_eol_frame, uint16_t* fan_rpm);
#if 0
extern EOL_ERROR_FRAME display_seg_test(EOL_DATA_FRAME* eol_data_frame);
#endif
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
