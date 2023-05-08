/*! \file app_onboard_hmi_ctrl.h
	\brief 
 */
#ifndef APP_ONBOARD_HMI_CONTROL_H_
#define APP_ONBOARD_HMI_CONTROL_H_

#include "configuration.h"

#ifdef APP_ONBOARD_HMI_CONTROL

#include "osal.h"

#define HMI_EVT_PRO_SUCCESS                   ((uint8_t)  0u)
#define HMI_EVT_PRO_FAIL                      ((uint8_t)  1u)

#define ONBOARD_HMI_EVENT_INVALID             ((uint8_t)0xFFu)
#define MODE_MENU_SEGEMENTS                   ((uint8_t)  4u)
#define ONBOARD_HMI_EVT_TABLE_SIZE            ((uint8_t)  3u)

#define OBHMI_NO_HMI_ACTION_WAIT_TIME_MSEC    30000u
#define OBHMI_WAIT_TIME_MSEC                  3000u
#define OBHMI_WAIT_TIME_TICKS                 pdMS_TO_TICKS(OBHMI_WAIT_TIME_MSEC)

#define OBHMI_STARTUP_SEQ_WAIT_TIME_MSEC      2000u
#define OBHMI_STARTUP_SEQ_TICKS               pdMS_TO_TICKS(OBHMI_WAIT_TIME_MSEC)

#define OBHMI_IDLE_TIME_MSEC                  5000
#define OBHMI_IDLE_TIME_TICKS                 pdMS_TO_TICKS(OBHMI_IDLE_TIME_MSEC)

#define LONG_PRESS_TIMER_MSEC                 3000u
#define LONG_PRESS_TIMER_TICKS                pdMS_TO_TICKS(LONG_PRESS_TIMER_MSEC)

#define LONG_PRESS_REL_TIMER_MSEC             1000u
#define LONG_PRESS_REL_TIMER_TICKS            pdMS_TO_TICKS(LONG_PRESS_REL_TIMER_MSEC)
  
#define SHORT_PRESS_TIMER_MSEC                100u //250u
#define SHORT_PRESS_TIMER_TICKS               pdMS_TO_TICKS(SHORT_PRESS_TIMER_MSEC)

#define BLINK_TIMER_MSEC                      500u
#define BLINK_TIMER_TICKS                     pdMS_TO_TICKS(BLINK_TIMER_MSEC)

#define HMI_BACKLIGHT_MIN_DUTY_CYCLE          0
#define HMI_BACKLIGHT_MAX_DUTY_CYCLE          100

typedef enum __blink_action
{
    BLINK_ACTIVATE   = 0,
    BLINK_DEACTIVATE = 1
}BLINK_ACTION;

typedef enum __segment_state
{
    SEGMENT_STATE_OFF       = 0,
    SEGMENT_STATE_ON        = 1,
    SEGMENT_STATE_NO_CHANGE = 2
}SEGMENT_STATE;

typedef struct __blink_seg_info
{
    BLINK_ACTION  blink_action;
    SEGMENT_STATE blink_end_state;
    uint16_t      blink_run_time_ms;
    uint16_t      timer_counter;
    uint16_t      run_time_counter;
    uint8_t       segment_flag;
}BLINK_SEG_INFO;

typedef enum __obhmi_ctrl_data_id
{
    INV_STATE                     =  0,
    HMI_TIMER_EXPIRED             =  1,
    ONBOARD_HMI_CTRL_STATE_CHANGE =  2,
    BTN_PRESSED_EVENT             =  3,
    UPDATE_SEG_AQ                 =  4,
    UPDATE_SEG_FILT_STAT          =  5,
    UPDATE_SEG_WARN               =  6,
    UPDATE_SEG_MODE               =  7,
    UPDATE_SEG_VEHMOD             =  8,
    UPDATE_SEG_PWRSRC             =  9,
    UPDATE_SEG_WIFI               = 10,
    UPDATE_SEG_BLE                = 11,
    UPDATE_SEG_IONIZER            = 12,
    UPDATE_SEG_DP_SENS_STAT       = 13,
    BLINK_TIMER_EXPIRED           = 14,
    BLINK_SEG_FILT_STAT           = 15,
    BLINK_SEG_BLE_STAT            = 16,
    HMI_BACK_LIGHT_TEST           = 17,
    INVALID_DATA_ID               = 18
}OBHMI_CTRL_DATA_ID;

typedef enum __obhmi_blink_ctrl_data_id
{
    BLINK_BLE_ICON              = 1,
    INVALID_BLINK_CTRL_DATA_ID  = 2            
}OBHMI_BLINK_CTRL_DATA_ID;

typedef enum __onbrdhmi_blink_ctrl_state
{
    STATE_IDLE          = 0,
	STATE_OFF           = 1,
    STATE_ACTIVE        = 2
}ONBRDHMI_BLINK_CTRL_STATE;

typedef enum __onbrdhmi_ctrl_state
{
    ONBOARD_HMI_STATE_IDLE          = 0,
	ONBOARD_HMI_STATE_OFF           = 1,
    ONBOARD_HMI_STATE_STARTUP       = 2,
    ONBOARD_HMI_STATE_ACTIVE        = 3
}ONBRDHMI_CTRL_STATE;

typedef enum __hmi_startup_seq_state
{
    STARTUP_SEQ_STATE_IDLE     = 0,
    STARTUP_SEQ_STATE_ONGOING  = 1,
    STARTUP_SEQ_STATE_COMPLETE = 2
}HMI_STARTUP_SEQ_STATE;

#if ( INVENT_HARWARE_VERSION == HW_VERSION_4_1 )

#ifdef LCD_DRIVER_IC_VERSION_OLD

typedef enum _button_pressed_evt
{
    BTN_NO_EVENTS      = 0x00,
    BTN_LIGHT          = 0x0B,  // LCD Driver IC UC1510C will send the data as three for Light button
    BTN_MODE           = 0x0C,  // LCD Driver IC UC1510C will send the data as four for Mode button
    BTN_POWER          = 0x0D   // LCD Driver IC UC1510C will send the data as five for Power button
}BTN_PRESSED_EVT;

#else

typedef enum _button_pressed_evt
{
    BTN_NO_EVENTS      = 0x00,
    BTN_LIGHT          = 0x0F,  // LCD Driver IC UC1510C will send the data as three for Light button
    BTN_MODE           = 0x10,  // LCD Driver IC UC1510C will send the data as four for Mode button
    BTN_POWER          = 0x11   // LCD Driver IC UC1510C will send the data as five for Power button
}BTN_PRESSED_EVT;

#endif

#else

typedef enum _button_pressed_evt
{
    BTN_NO_EVENTS      = 0x00,
    BTN_LIGHT          = 0x03,  // LCD Driver IC UC1510C will send the data as three for Light button
    BTN_MODE           = 0x04,  // LCD Driver IC UC1510C will send the data as four for Mode button
    BTN_POWER          = 0x05   // LCD Driver IC UC1510C will send the data as five for Power button
}BTN_PRESSED_EVT;

#endif

typedef enum _button_state
{
    BTN_STATE_UNKNOWN           = 0x00,
    BTN_PRESSED                 = 0x01,
    BTN_RELEASED                = 0x02,
    BTN_LONG_PRESS_ONGOING      = 0x03,
    BTN_LONG_PRESS_RELEASED     = 0x04
}button_state;

typedef enum __INV_PWR_STATE
{
    PWR_STATE_OFF = 0,
    PWR_STATE_ON  = 1
}INV_PWR_STATE;

typedef enum _power_btn_event
{
    SHORT_PRESS_ACTION_PWR_BTN_OFF        = 0,
    SHORT_PRESS_ACTION_PWR_BTN_ON         = 1,
    LONG_PRESS_ACT_PWR_BTN_RESET_FILT_TMR = 2
}POWER_BTN_EVENT;

typedef enum _mode_btn_event
{
    SHORT_PRESS_ACTION_MODE_AUTO       = 1,
    SHORT_PRESS_ACTION_MODE_TURBO      = 2,
    SHORT_PRESS_ACTION_MODE_SLEEP      = 3,
    LONG_PRESS_ACTION_STORAGE_MODE     = 4,
    LONG_PRESS_ACTION_ACTIVE_MODE      = 5
}MODE_BTN_EVENT;

typedef enum _light_btn_event
{
    SHORT_PRESS_ACT_LIGHT_OFF            =  0, //!< 0%  Duty cycle - light OFF
    SHORT_PRESS_ACT_LIGHT_DIM_LEVEL_1    =  1, //!< 20% Duty cycle
    SHORT_PRESS_ACT_LIGHT_DIM_LEVEL_2    =  2, //!< 40% Duty cycle
    SHORT_PRESS_ACT_LIGHT_DIM_LEVEL_3    =  3, //!< 50% Duty cycle
    SHORT_PRESS_ACT_LIGHT_DIM_LEVEL_4    =  4, //!< 60% Duty cycle
    SHORT_PRESS_ACT_LIGHT_DIM_LEVEL_5    =  5, //!< 80% Duty cycle
    SHORT_PRESS_ACT_LIGHT_DIM_LEVEL_MAX  =  6, //!< 100% Duty cycle - Max light intensity
    LONG_PRESS_ACT_LIGHT_BTN_BT_SCAN     =  7,
    LONG_PRESS_ACT_LIGHT_BTN_BT_PAIR     =  8
}LIGHT_BTN_EVENT;

#ifdef LCD_DRIVER_IC_VERSION_OLD

//!< Onboard HMI segement address
typedef enum __invent_onboard_hmi_segement
{
    SEG_AIR_QUALITY_LEVEL_1_LOW  =  0,
    SEG_FILTER_STATUS            =  1,
    SEG_WARNING_STATUS           =  2,
    SEG_MODE_AUTO                =  3,
    SEG_MODE_TURBO               =  4,
    SEG_MODE_SLEEP               =  5,
    SEG_MODE_MENU_LINE           =  6,
    SEG_STORAGE_MODE             =  7,
    SEG_SOLAR_BATTERY_STATUS     =  8,
    SEG_WIFI_STATUS              =  9,
    SEG_LIGHT_BUTTON             = 10,
    SEG_MODE_BUTTON              = 11,
    SEG_POWER_BUTTON             = 12,
    ONBOARD_HMI_MAX_SEGEMENT     = 13,
}ONBOARDHMI_SEGMENT;

#else

typedef enum __invent_onboard_hmi_segement
{
    SEG_AIR_QUALITY_LEVEL_1_LOW     =  0,
    SEG_AIR_QUALITY_LEVEL_2_MID     =  1,
    SEG_AIR_QUALITY_LEVEL_3_HIGH    =  2,
    SEG_IONIZER_STATUS              =  3,
    SEG_FILTER_STATUS               =  4,
    SEG_WARNING_STATUS              =  5,
    SEG_MODE_AUTO                   =  6,
    SEG_MODE_TURBO                  =  7,
    SEG_MODE_SLEEP                  =  8,
    SEG_MODE_MENU_LINE              =  9,
    SEG_STORAGE_MODE                = 10,
    SEG_SOLAR_BATTERY_STATUS        = 11,
    SEG_BLE_STATUS                  = 12,
    SEG_WIFI_STATUS                 = 13,
    SEG_LIGHT_BUTTON                = 14,
    SEG_MODE_BUTTON                 = 15,
    SEG_POWER_BUTTON                = 16,
    ONBOARD_HMI_MAX_SEGEMENT        = 17
}ONBOARDHMI_SEGMENT;

#endif

typedef enum __onboard_hmi_seg_ctrl
{
	ENABLE_ALL_SEGEMENT           = 0,
	DISABLE_ALL_SEGEMENT          = 1,
    POWERED_OFF_STATE_SEG         = 2,
    POWERED_ON_STATE_SEG          = 3
}ONBOARD_HMI_SEG_CTRL;

/* Enums for LCD segn=ment status */
typedef enum lcdseg_stat
{
    SEG_OFF = 0,                //!< Turns OFF the LED segment
    SEG_ON  = 1                 //!< Turns ON the LED segment
}LCDSEG_STATUS;

typedef enum __btn_press_event_type
{
    BUTTON_EVT_SHORT_PRESS = 0,
    BUTTON_EVT_LONG_PRESS  = 1
}BTN_PRESS_EVENT_TYPE;

typedef struct _hmi_evt_process_res
{
   uint32_t           ddm_parameter;
   uint8_t            data;
}hmi_evt_process_res;

typedef struct _hmi_domain_to_ddm_system
{
	int32_t   hmi_domain_value;
    uint32_t  ddm_parameter;
	int32_t   ddm_system_value;
}hmi_domain_to_ddm_system_t;

typedef struct _data_conversion
{
    const hmi_domain_to_ddm_system_t* const        hmi_domain_to_ddm_system;     // HMI domain to DDM system conversion 
    const uint8_t                                                      size;     // Size of elements 
}data_conversion_t;

typedef struct __obhmi_ctrl_data_frame
{
    int32_t             data;
    OBHMI_CTRL_DATA_ID  data_id;
}OBHMI_CTRL_DATA_FRAME;


typedef struct __obhmi_ctrl_sm
{
    bool                        blink_tmr_status;
    HMI_STARTUP_SEQ_STATE       hmi_starup_state;
    uint8_t                     last_btn_evt_cnt;
	uint32_t                    cur_btn_evt_cnt;
    IVPMGR0STATE_ENUM           inv_state;
    ONBOARD_HMI_SEG_CTRL        seg_state;
    OBHMI_CTRL_DATA_FRAME       data_frame;
	ONBRDHMI_CTRL_STATE         onbrd_hmi_ctrl_state;
    ONBRDHMI_BLINK_CTRL_STATE   blink_ctrl_state;
}ONBRD_HMI_CTRL_SM;

typedef struct _onboard_hmi_button_event
{
   const BTN_PRESSED_EVT                        				hmi_btn_evt;     // hmi btn event 
   uint8_t                                				short_press_evt_cnt;     // Event counter
   uint8_t                                				 long_press_evt_cnt;     // Long press event
   const uint8_t                          				   min_evt_sh_press;     // Min event short press 
   const uint8_t                          				   max_evt_sh_press;     // Max event short press
   const uint8_t                          				  min_evt_lng_press;     // Min event short press 
   const uint8_t                          				  max_evt_lng_press;     // Max event long press
   data_conversion_t                                        data_conversion;     // Contains the output data
}ONBOARD_HMI_BUTTON_EVENT;

extern LCDSEG_STATUS seg_stat[ONBOARD_HMI_MAX_SEGEMENT];

extern ONBOARD_HMI_BUTTON_EVENT onboard_hmi_btn_evt[];

uint8_t process_onboardhmi_btn_status(BTN_PRESSED_EVT btn_press_evt, BTN_PRESS_EVENT_TYPE evt_type, hmi_evt_process_res* evt_process_res);

void obhmi_set_segment(OBHMI_CTRL_DATA_ID data_id, int32_t i32value);

void onboard_hmi_update_segments(ONBOARD_HMI_SEG_CTRL hmi_ctrl_cmd);

void onboard_hmi_ctrl_sm(ONBRD_HMI_CTRL_SM* obhmi_ctrl_sm);

void parse_obhmi_ctrl_frame(ONBRD_HMI_CTRL_SM* obhmi_ctrl_sm, OBHMI_CTRL_DATA_FRAME* obhmi_ctrl_data_frame);

void reset_hmi_btn_ctrl_variables(void);

extern void start_hmi_wait_timer(uint32_t period_ms);

extern void reset_hmi_wait_timer(void);

extern void stop_hmi_wait_timer(void);

extern void change_onhmi_ctrl_state(ONBRDHMI_CTRL_STATE onbrd_hmi_ctrl_state);

extern void update_and_send_value_to_broker(uint32_t ddm_parameter, int32_t value);

uint8_t handle_onboard_hmi_button_event(uint16_t event_data, IVPMGR0STATE_ENUM inv_state);

extern void obhmi_update_var(OBHMI_CTRL_DATA_ID data_id, int32_t i32value);

extern void update_blink_info(const uint32_t error_code);

extern void error_check_ack(void);

#endif /* APP_ONBOARD_HMI_CONTROL */

#endif /* APP_ONBOARD_HMI_CONTROL_H_ */



