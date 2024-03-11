/*! \file app_onboard_hmi_ctrl.c
	\brief
 */

#include "dicm_framework_config.h"

#ifdef APP_ONBOARD_HMI_CONTROL
#include "iGeneralDefinitions.h"
#include "ddm2_parameter_list.h"
#include "app_onboard_hmi_ctrl.h"
#include "app_fan_motor_ctrl.h"
#include "app_light_ctrl.h"
#include "app_error_code.h"

#ifdef DEVICE_UC1510C
#include "drv_uc1510c.h"
#endif
#include "hal_pwm.h"
#include "hal_ledc.h"
#include "ddm2.h"
#include <string.h>

//extern int ble_peri_status; // Tobe defined in BLE_peripheral.c
int ble_peri_status = 0;
extern void error_check_ack(void);
extern void update_blink_info(const uint32_t error_code, bool update);

void set_err_ackstate(uint8_t errack_l);
static const hmi_domain_to_ddm_system_t hmi_btn_power_to_ddmp[] =
{
    //          hmi_domain_value               ddm_parameter        ddm_system_value
    { SHORT_PRESS_ACTION_PWR_BTN_OFF,           IV0PWRON,           IV0PWRON_OFF },
    { SHORT_PRESS_ACTION_PWR_BTN_ON,            IV0PWRON,           IV0PWRON_ON },
    { LONG_PRESS_ACT_PWR_BTN_RESET_FILT_TMR,    IV0FILST,           IV0FILST_FILTER_RESET }
};

static const hmi_domain_to_ddm_system_t hmi_btn_mode_to_ddmp[] =
{
    //   hmi_domain_value                  ddm_parameter        ddm_system_value
    { SHORT_PRESS_ACTION_MODE_AUTO,         IV0MODE,            IV0MODE_AUTO },
    { SHORT_PRESS_ACTION_MODE_TURBO,        IV0MODE,            IV0MODE_TURBO },
    { SHORT_PRESS_ACTION_MODE_SLEEP,        IV0MODE,            IV0MODE_SLEEP },
    { LONG_PRESS_ACTION_STORAGE_MODE,       IV0STORAGE,         IV0STORAGE_ACTIVATE },
    { LONG_PRESS_ACTION_ACTIVE_MODE,        IV0STORAGE,         IV0STORAGE_DEACTIVATE }
};

static const hmi_domain_to_ddm_system_t hmi_btn_light_to_ddmp[] =
{
    //       hmi_domain_value               ddm_parameter       ddm_system_value
    { SHORT_PRESS_ACT_LIGHT_OFF,            DIM0LVL,            DIM_LVL_DUTY_CYCLE_0  },
    { SHORT_PRESS_ACT_LIGHT_DIM_LEVEL_1,    DIM0LVL,            DIM_LVL_DUTY_CYCLE_5 },
    { SHORT_PRESS_ACT_LIGHT_DIM_LEVEL_3,    DIM0LVL,            DIM_LVL_DUTY_CYCLE_40 },
    { SHORT_PRESS_ACT_LIGHT_DIM_LEVEL_MAX,  DIM0LVL,            DIM_LVL_DUTY_CYCLE_100 },
    { LONG_PRESS_ACT_LIGHT_BTN_BT_SCAN,     IV0BLREQ,           IV0BLREQ_SCAN },
    { LONG_PRESS_ACT_LIGHT_BTN_BT_PAIR,     IV0BLREQ,           IV0BLREQ_PAIR }
};

ONBOARD_HMI_BUTTON_EVENT onboard_hmi_btn_evt[ONBOARD_HMI_EVT_TABLE_SIZE] =
{
    /* hmi_btn_evt  short_press_evt_cnt             long_press_evt                          min_evt_sh_press                max_evt_sh_press                    min_evt_lng_press                       max_evt_lng_press                       hmi_domain_to_ddm_system_t      size                      */
    { BTN_POWER,    SHORT_PRESS_ACTION_PWR_BTN_OFF, LONG_PRESS_ACT_PWR_BTN_RESET_FILT_TMR,  SHORT_PRESS_ACTION_PWR_BTN_OFF, SHORT_PRESS_ACTION_PWR_BTN_ON,      LONG_PRESS_ACT_PWR_BTN_RESET_FILT_TMR,  LONG_PRESS_ACT_PWR_BTN_RESET_FILT_TMR,  { &hmi_btn_power_to_ddmp[0u],   ELEMENTS(hmi_btn_power_to_ddmp) }},
    { BTN_MODE,     SHORT_PRESS_ACTION_MODE_AUTO,   LONG_PRESS_ACTION_STORAGE_MODE,         SHORT_PRESS_ACTION_MODE_AUTO,   SHORT_PRESS_ACTION_MODE_SLEEP,      LONG_PRESS_ACTION_STORAGE_MODE,         LONG_PRESS_ACTION_ACTIVE_MODE,          { &hmi_btn_mode_to_ddmp[0u],    ELEMENTS(hmi_btn_mode_to_ddmp) }},
    { BTN_LIGHT,    SHORT_PRESS_ACT_LIGHT_OFF,      LONG_PRESS_ACT_LIGHT_BTN_BT_SCAN,       SHORT_PRESS_ACT_LIGHT_OFF,      SHORT_PRESS_ACT_LIGHT_DIM_LEVEL_MAX,LONG_PRESS_ACT_LIGHT_BTN_BT_SCAN,       LONG_PRESS_ACT_LIGHT_BTN_BT_PAIR,       { &hmi_btn_light_to_ddmp[0u],   ELEMENTS(hmi_btn_light_to_ddmp) }}
};

static const LCDSEG_STATUS hmi_mode_seg_stat[NUM_OPERATING_MODES][MODE_MENU_SEGEMENTS] =
{
    // AUTO     TURBO    SLEEP    LINE       // LCD SEGEMENT
    {SEG_OFF,   SEG_OFF, SEG_OFF, SEG_OFF }, // IV0MODE_OFF
    {SEG_ON ,   SEG_OFF, SEG_OFF, SEG_ON },  // IV0MODE_AUTO
    {SEG_OFF,   SEG_ON,  SEG_OFF, SEG_ON },  // IV0MODE_TURBO
    {SEG_OFF,   SEG_OFF, SEG_ON,  SEG_ON },  // IV0MODE_SLEEP
};

#ifdef LCD_DRIVER_IC_VERSION_OLD

LCDSEG_STATUS seg_stat[ONBOARD_HMI_MAX_SEGEMENT] =
{
    SEG_OFF,  // SEG_AIR_QUALITY
    SEG_OFF,  // SEG_FILTER_STATUS
    SEG_OFF,  // SEG_WARNING_STATUS
    SEG_OFF,  // SEG_MODE_AUTO
    SEG_OFF,  // SEG_MODE_TURBO
    SEG_OFF,  // SEG_MODE_SLEEP
    SEG_OFF,  // SEG_MODE_MENU_LINE
    SEG_OFF,  // SEG_STORAGE_MODE
    SEG_OFF,  // SEG_SOLAR_BATTERY_STATUS
    SEG_OFF,  // SEG_WIFI_STATUS
    SEG_OFF,  // SEG_LIGHT_BUTTON
    SEG_OFF,  // SEG_MODE_BUTTON
    SEG_OFF,  // SEG_POWER_BUTTON
};

#else

LCDSEG_STATUS seg_stat[ONBOARD_HMI_MAX_SEGEMENT] =
{
    SEG_OFF,  // SEG_AIR_QUALITY_LEVEL_1_LOW
    SEG_OFF,  // SEG_FILTER_STATUS
    SEG_OFF,  // SEG_WARNING_STATUS
    SEG_OFF,  // SEG_MODE_AUTO
    SEG_OFF,  // SEG_MODE_TURBO
    SEG_OFF,  // SEG_MODE_SLEEP
    SEG_OFF,  // SEG_MODE_MENU_LINE
    SEG_OFF,  // SEG_STORAGE_MODE
    SEG_OFF,  // SEG_SOLAR_BATTERY_STATUS
    SEG_OFF,  // SEG_WIFI_STATUS
    SEG_OFF,  // SEG_LIGHT_BUTTON
    SEG_OFF,  // SEG_MODE_BUTTON
    SEG_OFF,  // SEG_POWER_BUTTON
    SEG_OFF,  // S13_BLE_STATUS
    SEG_OFF,  // S14_IONIZER_STATUS,
    SEG_OFF,  // S15_AIR_QUALITY_LEVEL_3_HIGH,
    SEG_OFF   // S16_AIR_QUALITY_LEVEL_2_MID,
};

LCDSEG_STATUS prev_seg_stat[ONBOARD_HMI_MAX_SEGEMENT] =
{
    SEG_OFF,  // SEG_AIR_QUALITY_LEVEL_1_LOW
    SEG_OFF,  // SEG_FILTER_STATUS
    SEG_OFF,  // SEG_WARNING_STATUS
    SEG_OFF,  // SEG_MODE_AUTO
    SEG_OFF,  // SEG_MODE_TURBO
    SEG_OFF,  // SEG_MODE_SLEEP
    SEG_OFF,  // SEG_MODE_MENU_LINE
    SEG_OFF,  // SEG_STORAGE_MODE
    SEG_OFF,  // SEG_SOLAR_BATTERY_STATUS
    SEG_OFF,  // SEG_WIFI_STATUS
    SEG_OFF,  // SEG_LIGHT_BUTTON
    SEG_OFF,  // SEG_MODE_BUTTON
    SEG_OFF,  // SEG_POWER_BUTTON
    SEG_OFF,  // S13_BLE_STATUS
    SEG_OFF,  // S14_IONIZER_STATUS,
    SEG_OFF,  // S15_AIR_QUALITY_LEVEL_3_HIGH,
    SEG_OFF   // S16_AIR_QUALITY_LEVEL_2_MID,
};

#endif

static uint8_t errack = 0;


static void onboard_hmi_mode_seg_update(LCDSEG_STATUS* ptr_mode_seg);

/**
  * @brief  Function to reset the button control variables to default value
  * @param  void.
  * @retval void.
  */
void reset_hmi_btn_ctrl_variables(void)
{
    onboard_hmi_btn_evt[0].short_press_evt_cnt = SHORT_PRESS_ACTION_PWR_BTN_OFF;
    onboard_hmi_btn_evt[1].short_press_evt_cnt = SHORT_PRESS_ACTION_MODE_AUTO;
    onboard_hmi_btn_evt[0].long_press_evt_cnt = LONG_PRESS_ACT_PWR_BTN_RESET_FILT_TMR;
    onboard_hmi_btn_evt[1].long_press_evt_cnt = LONG_PRESS_ACTION_STORAGE_MODE;
    onboard_hmi_btn_evt[2].long_press_evt_cnt = LONG_PRESS_ACT_LIGHT_BTN_BT_SCAN;
}

/**
  * @brief  Function to handle onboard HMI button events
  * @param  Button pressed value.
  * @retval Error Type.
  */
uint8_t handle_onboard_hmi_button_event(uint16_t event_data, IVPMGR0STATE_ENUM inv_state, uint32_t error_code)
{
	uint8_t result = HMI_EVT_PRO_FAIL;
    uint8_t index = 0u;
    uint8_t onbrd_hmi_evt = 0xFF;
    ONBOARD_HMI_BUTTON_EVENT *ptr_event = NULL;
    BTN_PRESSED_EVT hmi_btn_evt = (BTN_PRESSED_EVT)(event_data & 0xFF);
    BTN_PRESS_EVENT_TYPE event_type = (BTN_PRESS_EVENT_TYPE)((event_data >> 8) & BUTTON_PRESS_MASK_3);
    uint8_t seg = 0;
    uint32_t inv_err_codes = 0;

    inv_err_codes = error_code;

    if (IVPMGR0STATE_STANDBY == inv_state)
    {
        //Check for battery low condition and turn on inventilate
        if ((inv_err_codes & (1 << BACKUP_BATTERY_LOW)) > 0)
        {
            /* Check battery low*/
#ifdef EN_APP_ONBHMI_LOG
            LOG(W, "low battery: Skip mode/LP btn event");
#endif
            index = ONBOARD_HMI_EVT_TABLE_SIZE;
        }
        else if ((inv_err_codes & (1 << COMM_ERROR_WITH_BATTERY_IC)) > 0)
        {
        #ifdef EN_APP_ONBHMI_LOG
            LOG(W, "Communication error of I2C");
        #endif
            index = ONBOARD_HMI_EVT_TABLE_SIZE;
        }
        else if (hmi_standby_mode == 1)
        {
            /* When the inventilate is in STANDBY state, skip the processing of this MODE button events and long press events */
            if ((BTN_MODE == hmi_btn_evt) || (BUTTON_EVT_LONG_PRESS == event_type) || (BUTTON_EVT_LONG_PRESS_2 == event_type))
            {
#ifdef EN_APP_ONBHMI_LOG
                LOG(W, "Skip mode/LP btn event");
#endif
                index = ONBOARD_HMI_EVT_TABLE_SIZE;
            }
        }
    }
    else if (IVPMGR0STATE_STORAGE == inv_state)
    {
        if (((BTN_MODE == hmi_btn_evt) && (BUTTON_EVT_SHORT_PRESS == event_type)) || ((BTN_POWER == hmi_btn_evt) && (BUTTON_EVT_SHORT_PRESS == event_type)) || ((BTN_LIGHT == hmi_btn_evt) && (BUTTON_EVT_SHORT_PRESS == event_type)))
        {
            //("Storage skip mode btn event");
            /* When the inventilate is in STORAGE state, skip the processing of this MODE button events */
            index = ONBOARD_HMI_EVT_TABLE_SIZE;
        }
    }
    else
    {
#ifdef EN_APP_ONBHMI_LOG
        LOG(I, "process hmi_btn_evt = %d event_type = %d err_code %d", hmi_btn_evt, event_type,inv_err_codes);
#endif
    }

    /*  When any error occurs, MODE button, warning and respective icons will blink.
    First short press of MODE button is the action to acknowledge the error. */
    if ((BTN_MODE == hmi_btn_evt) && (BUTTON_EVT_SHORT_PRESS == event_type) && (errack != 0))
    {
        error_check_ack();
        update_blink_info(0, false);
        for (seg = 0; seg < ONBOARD_HMI_MAX_SEGEMENT; seg++)
        {
            uc1510c_set_segment(seg, seg_stat[seg]);
        }
        index = ONBOARD_HMI_EVT_TABLE_SIZE;
        errack = 0;
    }

    for ( ; index < ONBOARD_HMI_EVT_TABLE_SIZE; index++)
    {
        if (hmi_btn_evt == onboard_hmi_btn_evt[index].hmi_btn_evt)
        {
            /* Call function to handle the button events */
            ptr_event = &onboard_hmi_btn_evt[index];

            switch (event_type)
            {
                case BUTTON_EVT_SHORT_PRESS:
                    if (ptr_event->short_press_evt_cnt < ptr_event->max_evt_sh_press)
                    {
                        ptr_event->short_press_evt_cnt++;
                    }
                    else
                    {
                        ptr_event->short_press_evt_cnt = ptr_event->min_evt_sh_press;
                    }

                    onbrd_hmi_evt = ptr_event->short_press_evt_cnt;
                    break;

                case BUTTON_EVT_LONG_PRESS:

                    onbrd_hmi_evt = ptr_event->long_press_evt_cnt;

                    ptr_event->long_press_evt_cnt++;

                    if (ptr_event->long_press_evt_cnt > ptr_event->max_evt_lng_press)
                    {
                        ptr_event->long_press_evt_cnt = ptr_event->min_evt_lng_press;
                    }
                    break;

                case BUTTON_EVT_LONG_PRESS_2:
                    onbrd_hmi_evt = ptr_event->long_press_evt_cnt;
                    ptr_event->long_press_evt_cnt = ptr_event->max_evt_lng_press;
                    break;

                default:
#ifdef EN_APP_ONBHMI_LOG
                    LOG(W, "Button event: invalid = %d", event_type);
#endif
                    break;
            }

            for (index = 0; index < ptr_event->data_conversion.size; index++)
            {
                if (ptr_event->data_conversion.hmi_domain_to_ddm_system[index].hmi_domain_value == onbrd_hmi_evt)
                {
#ifdef EN_APP_ONBHMI_LOG
                    LOG(W, "send the req to broker ddm_parameter = 0x%x ddm_system_value = %d",
                         ptr_event->data_conversion.hmi_domain_to_ddm_system[index].ddm_parameter,
                         ptr_event->data_conversion.hmi_domain_to_ddm_system[index].ddm_system_value);
#endif
                    /* Update the new event value in db and send to the broker */
                    update_and_send_value_to_broker(ptr_event->data_conversion.hmi_domain_to_ddm_system[index].ddm_parameter,
                                                    ptr_event->data_conversion.hmi_domain_to_ddm_system[index].ddm_system_value);
                    break;
                }
            }
            /* Exit from the loop */
            index = ONBOARD_HMI_EVT_TABLE_SIZE;
        }
    }
    return result;
}

/**
  * @brief  Function to update(Enable / Disable) the Onboard HMI segments based on the received DDMP values
  * @param  data_id     -> Data ID refer the enum OBHMI_CTRL_DATA_ID
  * @param  i32value    -> Value of selected DDM parameter
  * @retval none
  */
void obhmi_set_segment(OBHMI_CTRL_DATA_ID data_id, int32_t i32value)
{
	switch (data_id)
    {
		case UPDATE_SEG_AQ:
            {
#ifndef LCD_DRIVER_IC_VERSION_OLD
                ONBOARDHMI_SEGMENT seg;
                if (seg_stat[SEG_STORAGE_MODE] == SEG_ON)
                {
                    seg_stat[SEG_AIR_QUALITY_LEVEL_1_LOW] = SEG_OFF;
                    seg_stat[SEG_AIR_QUALITY_LEVEL_2_MID] = SEG_OFF;
                    seg_stat[SEG_AIR_QUALITY_LEVEL_3_HIGH] = SEG_OFF;
                }
                else
                {
                    if (inv_acqrc_level > BME6X_LOW_ACCURACY)
                    {
                        if (IV0AQST_AIR_QUALITY_GOOD == i32value)
                        {
                            seg_stat[SEG_AIR_QUALITY_LEVEL_1_LOW] = SEG_ON;
                            seg_stat[SEG_AIR_QUALITY_LEVEL_2_MID] = SEG_ON;
                            seg_stat[SEG_AIR_QUALITY_LEVEL_3_HIGH] = SEG_ON;
                        }
                        else if (IV0AQST_AIR_QUALITY_FAIR == i32value)
                        {
                            seg_stat[SEG_AIR_QUALITY_LEVEL_1_LOW] = SEG_ON;
                            seg_stat[SEG_AIR_QUALITY_LEVEL_2_MID] = SEG_ON;
                            seg_stat[SEG_AIR_QUALITY_LEVEL_3_HIGH] = SEG_OFF;
                        }
                        else
                        {
                            seg_stat[SEG_AIR_QUALITY_LEVEL_1_LOW] = SEG_ON;
                            seg_stat[SEG_AIR_QUALITY_LEVEL_2_MID] = SEG_OFF;
                            seg_stat[SEG_AIR_QUALITY_LEVEL_3_HIGH] = SEG_OFF;
                        }
                    }
                    else
                    {
                        seg_stat[SEG_AIR_QUALITY_LEVEL_1_LOW] = SEG_OFF;
                        seg_stat[SEG_AIR_QUALITY_LEVEL_2_MID] = SEG_OFF;
                        seg_stat[SEG_AIR_QUALITY_LEVEL_3_HIGH] = SEG_OFF;
                    }
                }

                for (seg = SEG_AIR_QUALITY_LEVEL_1_LOW; seg <= SEG_AIR_QUALITY_LEVEL_3_HIGH; seg++)
                {
                    uc1510c_set_segment(seg, seg_stat[seg]);
                }
#else
                /* IAQ value greater than threshold means air quality bad --> Turn OFF segement */
                bool curr_aq_stat = SEG_OFF;

                if (seg_stat[SEG_STORAGE_MODE] == SEG_ON)
                {
                    seg_stat[SEG_AIR_QUALITY_LEVEL_1_LOW] = SEG_OFF;
                    uc1510c_set_segment(SEG_AIR_QUALITY_LEVEL_1_LOW, SEG_OFF);
                }
                else
                {
                    curr_aq_stat = (i32value != IV0AQST_AIR_QUALITY_GOOD) ? SEG_OFF : SEG_ON;

                    if (curr_aq_stat != seg_stat[SEG_AIR_QUALITY_LEVEL_1_LOW])
                    {
                        LOG(I, "SEG_AIR_QUALITY segemnt stat = %d", curr_aq_stat);
                        /* store current air quality status */
                        seg_stat[SEG_AIR_QUALITY_LEVEL_1_LOW] = curr_aq_stat;
                        /* set the air quality segment */
                        uc1510c_set_segment(SEG_AIR_QUALITY_LEVEL_1_LOW, curr_aq_stat);
                    }
                }
#endif
            }
            break;
		case UPDATE_SEG_AQ_OFF:
            uc1510c_set_segment(SEG_AIR_QUALITY_LEVEL_1_LOW, SEG_OFF);
            uc1510c_set_segment(SEG_AIR_QUALITY_LEVEL_2_MID, SEG_OFF);
            uc1510c_set_segment(SEG_AIR_QUALITY_LEVEL_3_HIGH, SEG_OFF);
            break;
        case UPDATE_SEG_FILT_STAT:
			{
				seg_stat[SEG_FILTER_STATUS] = (i32value != IV0FILST_FILTER_CHANGE_REQ) ? SEG_OFF : SEG_ON;
#ifdef EN_APP_ONBHMI_LOG
				LOG(I, "SEG_FILTER_STATUS  = %d", seg_stat[SEG_FILTER_STATUS]);
#endif
				uc1510c_set_segment(SEG_FILTER_STATUS, seg_stat[SEG_FILTER_STATUS]);      /* Filter change needed SEG_ON */
			}
            break;

		case UPDATE_SEG_WARN:
            {
                seg_stat[SEG_WARNING_STATUS] = ((IV0WARN_ENUM) i32value != IV0WARN_NO_WARNING) ? SEG_ON : SEG_OFF;
                uc1510c_set_segment(SEG_WARNING_STATUS, seg_stat[SEG_WARNING_STATUS]);
            }
            break;

		case UPDATE_SEG_MODE:
            {
                if (NUM_OPERATING_MODES > (uint8_t)i32value)
                {
                    onboard_hmi_mode_seg_update((LCDSEG_STATUS*)&hmi_mode_seg_stat[(IV0MODE_ENUM)i32value][0u]);
                }
            }
            break;

        case UPDATE_SEG_VEHMOD: //For Storage mode active or inactive
			{
				seg_stat[SEG_STORAGE_MODE] = (IV0STORAGE_ACTIVATE == (IV0STORAGE_ENUM) i32value) ? SEG_ON : SEG_OFF;
                uc1510c_set_segment(SEG_STORAGE_MODE, seg_stat[SEG_STORAGE_MODE]);
                seg_stat[SEG_WIFI_STATUS] = SEG_OFF;
                uc1510c_set_segment(SEG_WIFI_STATUS, seg_stat[SEG_WIFI_STATUS]);
                seg_stat[SEG_BLE_STATUS] = SEG_OFF;
                uc1510c_set_segment(SEG_BLE_STATUS, seg_stat[SEG_BLE_STATUS]);
                seg_stat[SEG_LIGHT_BUTTON] = (IV0STORAGE_ACTIVATE == (IV0STORAGE_ENUM) i32value) ? SEG_OFF : SEG_ON;
                uc1510c_set_segment(SEG_LIGHT_BUTTON, seg_stat[SEG_LIGHT_BUTTON]);
                seg_stat[SEG_POWER_BUTTON] = (IV0STORAGE_ACTIVATE == (IV0STORAGE_ENUM) i32value) ? SEG_OFF : SEG_ON;
                uc1510c_set_segment(SEG_POWER_BUTTON, seg_stat[SEG_POWER_BUTTON]);

                seg_stat[SEG_AIR_QUALITY_LEVEL_1_LOW] = (IV0STORAGE_ACTIVATE == (IV0STORAGE_ENUM) i32value) ? SEG_OFF : SEG_ON;
                uc1510c_set_segment(SEG_AIR_QUALITY_LEVEL_1_LOW, seg_stat[SEG_AIR_QUALITY_LEVEL_1_LOW]);

                seg_stat[SEG_AIR_QUALITY_LEVEL_2_MID] = (IV0STORAGE_ACTIVATE == (IV0STORAGE_ENUM) i32value) ? SEG_OFF : SEG_ON;
                uc1510c_set_segment(SEG_AIR_QUALITY_LEVEL_2_MID, seg_stat[SEG_AIR_QUALITY_LEVEL_2_MID]);

                seg_stat[SEG_AIR_QUALITY_LEVEL_3_HIGH] = (IV0STORAGE_ACTIVATE == (IV0STORAGE_ENUM) i32value) ? SEG_OFF : SEG_ON;
                uc1510c_set_segment(SEG_AIR_QUALITY_LEVEL_3_HIGH, seg_stat[SEG_AIR_QUALITY_LEVEL_3_HIGH]);
                //Ionizer, Warning, Filter, Solar
			}
            break;

        case UPDATE_SEG_PWRSRC:
            {
                seg_stat[SEG_SOLAR_BATTERY_STATUS] = (IV0PWRSRC_SOLAR_POWER_INPUT == (IV0PWRSRC_ENUM) i32value) ? SEG_ON : SEG_OFF;
                uc1510c_set_segment(SEG_SOLAR_BATTERY_STATUS, seg_stat[SEG_SOLAR_BATTERY_STATUS]);
            }
            break;

        case UPDATE_SEG_WIFI:
            {
                if (seg_stat[SEG_STORAGE_MODE] == SEG_ON)
                {
                    seg_stat[SEG_WIFI_STATUS] = SEG_OFF;
                    uc1510c_set_segment(SEG_WIFI_STATUS, seg_stat[SEG_WIFI_STATUS]);
                }
                else
                {
                    seg_stat[SEG_WIFI_STATUS] = (WIFI0STS_CONNECTED == (WIFI0STS_ENUM)i32value) ? SEG_ON : SEG_OFF;
                    uc1510c_set_segment(SEG_WIFI_STATUS, seg_stat[SEG_WIFI_STATUS]);
                }
            }
            break;

        case UPDATE_SEG_BLE:
            {
                if (seg_stat[SEG_STORAGE_MODE] == SEG_ON)
                {
                    seg_stat[SEG_BLE_STATUS] = SEG_OFF;
                    uc1510c_set_segment(SEG_BLE_STATUS, seg_stat[SEG_BLE_STATUS]);
                }
                else
                {
                    if (i32value != BT0PAIR_OUT_PAIRING_MODE_INACTIVE)
                    {
                        seg_stat[SEG_BLE_STATUS] = (BT0PAIR_OUT_PAIRING_MODE_ACTIVE == (BT0PAIR_OUT_ENUM)i32value) ? SEG_ON : SEG_OFF;
#ifdef EN_APP_ONBHMI_LOG
                        LOG(I, "SEG_BLE_STATUS = %d app_conn_sts = %d i32v = %d", seg_stat[SEG_BLE_STATUS], ble_peri_status, i32value);
#endif
                        if ((seg_stat[SEG_BLE_STATUS] == 0) && (ble_peri_status == 1))
                        {
                                seg_stat[SEG_BLE_STATUS] = 1;
#ifdef EN_APP_ONBHMI_LOG
                                LOG(I, "App connected Show BLE_B");
#endif
                        }
                        if (ble_dp_con_sts == 1)
                        {
                            seg_stat[SEG_BLE_STATUS] = 1;
                            LOG(I, "DP connected Show BLE_A");
                        }

                        uc1510c_set_segment(SEG_BLE_STATUS, seg_stat[SEG_BLE_STATUS]);
                    }
                }
            }
            break;

        case UPDATE_SEG_DP_SENS_STAT:
            {
                if (seg_stat[SEG_STORAGE_MODE] == SEG_ON)
                {
                    seg_stat[SEG_BLE_STATUS] = SEG_OFF;
                    uc1510c_set_segment(SEG_BLE_STATUS, seg_stat[SEG_BLE_STATUS]);
                }
                else
                {
                    seg_stat[SEG_BLE_STATUS] = ( DP_SENSOR_AVAILABLE == (DP_SENS_STATUS)i32value ) ? SEG_ON : SEG_OFF;
                    if ((seg_stat[SEG_BLE_STATUS] == 0) && (ble_peri_status == 1))
                    {
                        seg_stat[SEG_BLE_STATUS] = 1;
                    }
#ifdef EN_APP_ONBHMI_LOG
                    LOG(I, "UPDATE_SEG_BLE = %d App_con_sts %d", seg_stat[SEG_BLE_STATUS], ble_peri_status);
#endif
                    //Check for App connection before hiding BLE ICON for DP sensor
                    uc1510c_set_segment(SEG_BLE_STATUS, seg_stat[SEG_BLE_STATUS]);
                }
            }
            break;

        case UPDATE_SEG_IONIZER:
            {
                seg_stat[SEG_IONIZER_STATUS] = (IV0IONST_ON == (IV0IONST_ENUM)i32value) ? SEG_ON : SEG_OFF;
#ifdef EN_APP_ONBHMI_LOG
                LOG(I, "SEG_IONIZER_STATUS = %d", seg_stat[SEG_IONIZER_STATUS]);
#endif
                uc1510c_set_segment(SEG_IONIZER_STATUS, seg_stat[SEG_IONIZER_STATUS]);
            }
            break;

        default:
            /* Do nothing */
            break;
    }
}

/**
  * @brief  Function to update(Enable / Disable) the variable
  * @param  data_id     -> Data ID refer the enum OBHMI_CTRL_DATA_ID
  * @param  i32value    -> Value of selected DDM parameter
  * @retval none
  */
void obhmi_update_var(OBHMI_CTRL_DATA_ID data_id, int32_t i32value)
{
    switch (data_id)
    {
		case UPDATE_SEG_AQ:
            {
#ifndef LCD_DRIVER_IC_VERSION_OLD
                if (IV0AQST_AIR_QUALITY_GOOD == i32value)
                {
                    seg_stat[SEG_AIR_QUALITY_LEVEL_1_LOW] = SEG_ON;
                    seg_stat[SEG_AIR_QUALITY_LEVEL_2_MID] = SEG_ON;
                    seg_stat[SEG_AIR_QUALITY_LEVEL_3_HIGH] = SEG_ON;
                }
                else if (IV0AQST_AIR_QUALITY_FAIR == i32value)
                {
                    seg_stat[SEG_AIR_QUALITY_LEVEL_1_LOW] = SEG_ON;
                    seg_stat[SEG_AIR_QUALITY_LEVEL_2_MID] = SEG_ON;
                    seg_stat[SEG_AIR_QUALITY_LEVEL_3_HIGH] = SEG_OFF;
                }
                else // Bad Air Quality
                {
                    seg_stat[SEG_AIR_QUALITY_LEVEL_1_LOW] = SEG_ON;
                    seg_stat[SEG_AIR_QUALITY_LEVEL_2_MID] = SEG_OFF;
                    seg_stat[SEG_AIR_QUALITY_LEVEL_3_HIGH] = SEG_OFF;
                }
#else
                if (seg_stat[SEG_STORAGE_MODE] == SEG_ON)
                {
                    seg_stat[SEG_AIR_QUALITY_LEVEL_1_LOW] = SEG_OFF;
                }
                else
                {
                    /* Air quality not good --> Turn OFF segement */
                    bool curr_aq_stat = (i32value != IV0AQST_AIR_QUALITY_GOOD) ? SEG_OFF : SEG_ON;
                    /* store current air quality status */
                    seg_stat[SEG_AIR_QUALITY_LEVEL_1_LOW] = curr_aq_stat;
                }
#endif
            }
            break;

        case UPDATE_SEG_FILT_STAT:
			seg_stat[SEG_FILTER_STATUS] = (i32value != IV0FILST_FILTER_CHANGE_REQ) ? SEG_OFF : SEG_ON;
            break;

		case UPDATE_SEG_WARN:
            seg_stat[SEG_WARNING_STATUS] = ((IV0WARN_ENUM)i32value != IV0WARN_NO_WARNING) ? SEG_ON : SEG_OFF;
            break;

        case UPDATE_SEG_VEHMOD:
            seg_stat[SEG_STORAGE_MODE] = (IV0STORAGE_ACTIVATE == (IV0STORAGE_ENUM)i32value) ? SEG_ON : SEG_OFF;
            break;

        case UPDATE_SEG_PWRSRC:
            seg_stat[SEG_SOLAR_BATTERY_STATUS] = (IV0PWRSRC_SOLAR_POWER_INPUT == (IV0PWRSRC_ENUM)i32value) ? SEG_ON : SEG_OFF;
            break;

        case UPDATE_SEG_WIFI:
            seg_stat[SEG_WIFI_STATUS] = (WIFI0STS_CONNECTED == (WIFI0STS_ENUM)i32value) ? SEG_ON : SEG_OFF;
            break;

        case UPDATE_SEG_BLE:
            if (seg_stat[SEG_STORAGE_MODE] == SEG_ON)
            {
                seg_stat[SEG_BLE_STATUS] = SEG_OFF;
            }
            else
            {
                seg_stat[SEG_BLE_STATUS] = (BT0PAIR_OUT_PAIRING_MODE_ACTIVE == (BT0PAIR_OUT_ENUM)i32value) ? SEG_ON : SEG_OFF;

                if ((seg_stat[SEG_BLE_STATUS] == 0) && (ble_peri_status == 1))
                {
                    seg_stat[SEG_BLE_STATUS] = 1;
                }

                if (ble_dp_con_sts == 1)
                {
                    seg_stat[SEG_BLE_STATUS] = 1;
                }
            }
            break;

        case UPDATE_SEG_DP_SENS_STAT:
            seg_stat[SEG_BLE_STATUS] = (DP_SENSOR_AVAILABLE == (DP_SENS_STATUS)i32value) ? SEG_ON : SEG_OFF;
            if ((seg_stat[SEG_BLE_STATUS] == 0) && (ble_peri_status == 1))
            {
                seg_stat[SEG_BLE_STATUS] = 1;
            }
            break;

        case UPDATE_SEG_IONIZER:
            seg_stat[SEG_IONIZER_STATUS] = (IV0IONST_ON == (IV0IONST_ENUM)i32value) ? SEG_ON : SEG_OFF;
            break;

        default:
            /* Do nothing */
            break;
    }
}

/**
  * @brief  Function to update the onboard HMI mode update
  * @param  Pointer to the structure type LCDSEG_STATUS
  * @retval none.
  */
static void onboard_hmi_mode_seg_update(LCDSEG_STATUS *ptr_mode_seg)
{
    uint8_t index;

    for (index = 0; index < NUM_OPERATING_MODES - 1;  index++)
    {
        uc1510c_set_segment(index + SEG_MODE_AUTO, ptr_mode_seg[index]);
        //update segment ON/OFF seg_stat[]
        seg_stat[index + SEG_MODE_AUTO] = ptr_mode_seg[index];
    }
}

/**
 * @brief Function to update the onboard HMI segments based on the status ONBOARD_HMI_SEG_CTRL
 * @param hmi_ctrl_cmd refer enum type ONBOARD_HMI_SEG_CTRL
 */
void onboard_hmi_update_segments(ONBOARD_HMI_SEG_CTRL hmi_ctrl_cmd)
{
    ONBOARDHMI_SEGMENT seg;

    static uint8_t i2c_comm_err_flag = false;

	switch (hmi_ctrl_cmd)
	{
		case ENABLE_ALL_SEGEMENT:
			for (seg = SEG_AIR_QUALITY_LEVEL_1_LOW; seg < ONBOARD_HMI_MAX_SEGEMENT; seg++)
			{
                uc1510c_set_segment(seg, SEG_ON);
			}
			break;

		case DISABLE_ALL_SEGEMENT:
            for (seg = SEG_AIR_QUALITY_LEVEL_1_LOW; seg < ONBOARD_HMI_MAX_SEGEMENT; seg++)
			{
                uc1510c_set_segment(seg, SEG_OFF);
			}
            break;

        case POWERED_OFF_STATE_SEG:
            {
                if (i2c_comm_err_flag == false)
                {
                    for (seg = SEG_AIR_QUALITY_LEVEL_1_LOW; seg < ONBOARD_HMI_MAX_SEGEMENT; seg++)
                    {
                        if ((seg != SEG_LIGHT_BUTTON) && (seg != SEG_POWER_BUTTON))
                        {
                            uc1510c_set_segment(seg, SEG_OFF);
                        }
                        else
                        {
                            if (seg_stat[seg] != SEG_ON)
                            {
                                uc1510c_set_segment(seg, SEG_ON);
                                seg_stat[seg] = SEG_ON;
                            }
                        }
                    }

                    seg_stat[SEG_MODE_AUTO] = SEG_OFF;
                    seg_stat[SEG_MODE_TURBO] = SEG_OFF;
                    seg_stat[SEG_MODE_SLEEP] = SEG_OFF;
                }
            }
			break;

        case POWERED_ON_STATE_SEG:
            {
                /* Set the default segment */
                seg_stat[SEG_LIGHT_BUTTON] = SEG_ON;
                seg_stat[SEG_MODE_BUTTON] = SEG_ON;
                seg_stat[SEG_POWER_BUTTON] = SEG_ON;
                seg_stat[SEG_MODE_AUTO] = SEG_ON;
                seg_stat[SEG_MODE_MENU_LINE] = SEG_ON;
                seg_stat[SEG_STORAGE_MODE] = SEG_OFF;

                for (seg = SEG_AIR_QUALITY_LEVEL_1_LOW; seg < ONBOARD_HMI_MAX_SEGEMENT; seg++)
                {
                    uc1510c_set_segment(seg, seg_stat[seg]);
                }
            }
            break;

        case ENTER_I2C_COMM_ERR:
            {
                i2c_comm_err_flag = true;

                seg_stat[SEG_LIGHT_BUTTON] = SEG_OFF;
                seg_stat[SEG_WARNING_STATUS] = SEG_ON;
                seg_stat[SEG_POWER_BUTTON] = SEG_ON;
                for (seg = SEG_AIR_QUALITY_LEVEL_1_LOW; seg < ONBOARD_HMI_MAX_SEGEMENT; seg++)
                {
                    if ((seg != SEG_WARNING_STATUS) && (seg != SEG_POWER_BUTTON))
                    {
                        uc1510c_set_segment(seg, SEG_OFF);
                    }
                    else
                    {
                        uc1510c_set_segment(seg, seg_stat[seg]);
                    }
                }
            }
            break;

        case EXIT_I2C_COMM_ERR:
            {
                i2c_comm_err_flag = false;

                seg_stat[SEG_LIGHT_BUTTON] = SEG_ON;
                seg_stat[SEG_WARNING_STATUS] = SEG_OFF;
                seg_stat[SEG_POWER_BUTTON] = SEG_ON;
                for (seg = SEG_AIR_QUALITY_LEVEL_1_LOW; seg < ONBOARD_HMI_MAX_SEGEMENT; seg++)
                {
                    if ((seg != SEG_LIGHT_BUTTON) && (seg != SEG_POWER_BUTTON))
                    {
                        uc1510c_set_segment(seg, SEG_OFF);
                    }
                    else
                    {
                        uc1510c_set_segment(seg, seg_stat[seg]);
                    }
                }
            }

        default:
            break;
	}
}

/**
 * @brief Set the err ackstate object
 * Set error acknowledgement value
 *
 * @param errack_l  Set ot Reset value
 */
void set_err_ackstate(uint8_t errack_l)
{
    errack = errack_l;
}

#endif /* APP_ONBOARD_HMI_CONTROL */
