/*! \file connector_onboardhmi.c
	\brief Connector for Onboard HMI
	\Author Sundaramoorthy-LTTS
 */

/** Includes ******************************************************************/
#include "configuration.h"

#ifdef CONNECTOR_ONBOARD_HMI
#include "iGeneralDefinitions.h"

#include "connector_onboardhmi.h"
#include "inventory_handler.h"
#include "broker.h"
#include "app_api.h"

#ifdef DEVICE_UC1510C
#include "drv_uc1510c.h"
#endif

#include "sorted_list.h"
#include "linear_interpolation.h"
#include "hal_ledc.h"
#include "hal_timer.h"
#include "osal.h"


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

/******************************** Defines ************************************/
#define CONN_ONBHMI_DEBUG_LOG               1
#define CONN_ONBHMI_CRITICAL_LOG            0

#define CONN_OBHMI_SUB_DEPTH		        ((uint8_t) 20u)
#define DDMP_UNAVAILABLE                    ((uint8_t) 0xFFu)
#define ONBOARD_HMI_EVENT_QUEUE_LEN         15
#define ONBOARD_HMI_EVENT_QUEUE_SIZE        sizeof(int)
#define ONBOARD_HMI_BTN_TASK_DELAY          ((uint32_t) 500u)
#define ONBOARD_HMI_PWR_CTRL_TASK_DELAY     ((uint32_t) 500u)
#define OBHMI_PWR_CTRL_TASK_QUE_LEN	        ((osal_base_type_t) 30)
#define OBHMI_PWR_CTRL_TASK_QUE_ITEM_SIZE   ((osal_base_type_t) sizeof(OBHMI_CTRL_DATA_FRAME))
#define NUM_AVL_ELEMENTS                    ((uint8_t) 2u)

/* Blink functionality macro definitions */
#define FAST_BLINK_TIME_INTERVAL             500u
#define SLOW_BLINK_TIME_INTERVAL            1000u
#define OBHMI_BLINK_CTRL_TASK_DELAY_MSEC     500u
#define BLINK_RUN_TIME_INFINITE              ((uint16_t)  0xFFFF)
#define LCD_BK_LIT_BLINK_DELAY                      500u

#ifndef INVENT_EOL_TESTING
#define FILTER_RESET_STATUS_RUN_TIME_MSEC    ((uint16_t)  30000u)
#define BT_PAIR_STATUS_RUN_TIME_MSEC         ((uint16_t)  60000u)
#define STORAGE_MODE_STATUS_RUN_TIME_MSEC    ((uint16_t)  10000u)
#else
#define FILTER_RESET_STATUS_RUN_TIME_MSEC    ((uint16_t)  10000u)
#define BT_PAIR_STATUS_RUN_TIME_MSEC         ((uint16_t)  10000u)
#define STORAGE_MODE_STATUS_RUN_TIME_MSEC    ((uint16_t)  10000u)
#endif

#define ESP_HW_TIMER

#ifdef ESP_HW_TIMER

#ifdef LCD_DRIVER_ENABLE
//----------for_time_base/128----------------------//
/*
Time_base/Divider = freq in Hz
time_base = 80MHz (ESP32 Clock)
selected divider = 4096
interrupt generated at an interval of 4ms
*/
#define HW_TIME_INTERVAL      4       //in_ms
#define LONG_PRESS_THRESHOLD    (3000/HW_TIME_INTERVAL)
#define SHORT_PRESS_THRESHOLD   (2200/HW_TIME_INTERVAL)
#define LONG_PRESS_COUNT_VALUE  (2000/HW_TIME_INTERVAL)
#define BUTTON_EXPIRED_COUNT    (800/HW_TIME_INTERVAL)
#define KEY_DEBOUNCE_COUNT      (500/HW_TIME_INTERVAL)
#define BTN_REL_WAIT_TIME       (1000/HW_TIME_INTERVAL)
#define BTN_SCAN_TIME           (200/HW_TIME_INTERVAL)
#define TOUCH_MONITER_TIME      (800/HW_TIME_INTERVAL)

#define TIME_60MS               (60/HW_TIME_INTERVAL)
#define TIME_68MS               (68/HW_TIME_INTERVAL)
#define TIME_100MS              (100/HW_TIME_INTERVAL)

#define TIME_200MS              (200/HW_TIME_INTERVAL)
#define TIME_208MS              (208/HW_TIME_INTERVAL)

#define TIME_500MS              (500/HW_TIME_INTERVAL)
#define TIME_508MS              (508/HW_TIME_INTERVAL)

#define TIME_750MS              (750/HW_TIME_INTERVAL)
#define TIME_1000MS              (1000/HW_TIME_INTERVAL)
#define TIME_1008MS              (1008/HW_TIME_INTERVAL)
#define TIME_1500MS             (1500/HW_TIME_INTERVAL)
#define TIME_3000MS             (3000/HW_TIME_INTERVAL)
#define TIME_6000MS             (6000/HW_TIME_INTERVAL)
#define TIME_60SEC             (60000/HW_TIME_INTERVAL)

#endif

#define LONG_PRESS_OCCURED      2
#define PRESS_ZERO_VALUE        0
#define PRESS_START_VALUE       1

#endif

#define  USER_ERRACK_MAX        ((uint8_t)  5u)

typedef enum _sel_blink_ack
{
    BLINK_LED_BK_LIGHT  =   0,
    BLINK_POWER         =   1
}SEL_TOUCH_ACK;

static xQueueHandle timer_queue;

static uint8_t user_errack = 0;
static int32_t  inv_wifi_status = 0;
static int32_t  inv_ble_status = 0;
static int32_t  inv_iaq_status = 0;
int32_t  inv_acqrc_level = 0;
uint8_t ble_dp_con_sts = 0;

/** Static Function declarations ********************************************/
static void inventory_handler_cb(void *arg, uint32_t device_class_instance, bool is_available);
static int initialize_connector_onboardhmi(void);
static void install_parameters(void);
static void initialize_dev_onboardhmi(void);
static void conn_onboardhmi_process_task(void *pvParameter);
static void conn_onboardhmi_ctrl_task(void *pvParameter);
static void conn_obdhmi_btn_task(void *pvParameter);
static void blink_led_bk_light(uint8_t sel_blink);
static void process_subscribe_request(uint32_t ddm_param);
static uint8_t get_ddm_index_from_db(uint32_t ddm_param);
static void process_set_and_publish_request(uint32_t ddm_param, int32_t i32value, DDMP2_CONTROL_ENUM req_type);
static void handle_hmi_ctrl_sub_data(uint32_t ddm_param, int32_t data);
static void onboard_hmi_timer_cb_func(TimerHandle_t xTimer);
static void inv_timer_cb(BaseType_t *data_in);                                 /*Timer call back function for button event called by timer isr*/
static void start_publish(void);
static void start_subscribe(void);
void change_onhmi_ctrl_state(ONBRDHMI_CTRL_STATE onbrd_hmi_ctrl_state);
static void push_data_to_the_queue(int32_t data, OBHMI_CTRL_DATA_ID data_id);
#ifndef ESP_HW_TIMER
static void long_press_tmr_cb( TimerHandle_t xTimer );
static void short_press_tmr_cb( TimerHandle_t xTimer );
static void long_press_rel_tmr_cb( TimerHandle_t xTimer );
#endif
void update_and_send_value_to_broker(uint32_t ddm_parameter, int32_t value);
static void update_hmi_btn_ctrl_variables(uint32_t ddmp, int32_t data);
static void obhmi_blink_tmr_cb(TimerHandle_t xTimer);
static void start_obhmi_blink_timer(void);
static void stop_obhmi_blink_timer(void);
void update_blink_info(const uint32_t error_code);
void error_check_ack(void);

/* Handles for the tasks create by main(). */
static osal_queue_handle_t obhmi_pwr_ctrl_que_hdl;

TimerHandle_t obHmiOneShotTimer;
TimerHandle_t long_press_tmr_hdle;
TimerHandle_t long_press_rel_tmr_hdle;
TimerHandle_t short_press_tmr_hdle;
TimerHandle_t blink_ctrl_timer_hdle;

ONBRD_HMI_CTRL_SM onbrd_hmi_ctrl_sm;
static uint32_t configured_time_ms = 0;
static uint16_t lcd_btn_press_evt  = BTN_NO_EVENTS;
static uint16_t prev_btn_press_evt = BTN_NO_EVENTS;
static button_state lcd_btn_state  = BTN_RELEASED;
static uint8_t button_press_expired         = 0;
static uint8_t  button_press_flag           = 0;
static uint8_t button_first_press           = 0;
static uint8_t pbutton_first_press           = 0;
static uint8_t button_released              = 0;
static uint8_t  btn_dbnc_delay              = 0;
static uint16_t btn_press_wait              = 0;
static uint16_t btn_dbnc_count              = 0;

static uint16_t tch_timer_cnt               = 0;
static uint8_t tch_btn_event                = 0;
static uint8_t blink_event                  = 0;
static uint8_t button_event                 = 0;
static uint8_t int_touch                    = 0 ;

static INV_PWR_STATE inv_pwr_state = PWR_STATE_OFF;
static uint32_t blink_timer_counter         = 0;
static uint8_t  blink_flag                  = 0;
static uint32_t g_blink_segment_status      = 0;
static uint32_t g_error_ack                 = 0;
static uint32_t g_error_code                = 0;
static ONBRD_HMI_CTRL_SM* ptr_hmi_ctrl_sm = &onbrd_hmi_ctrl_sm;
uint8_t hmi_standby_mode = 0;

extern LCDSEG_STATUS seg_stat[ONBOARD_HMI_MAX_SEGEMENT];
extern LCDSEG_STATUS prev_seg_stat[ONBOARD_HMI_MAX_SEGEMENT];

/* Structure for Connector OnboardHMI */
CONNECTOR connector_onboard_hmi =
{
	.name = "OnboardHMI Connector",
	.initialize = initialize_connector_onboardhmi
};

// Inventory handler structures
static EXT_RAM_ATTR inventory_handler_t l_ih;
DECLARE_SORTED_LIST_EXTRAM(inv_storage, 3);


/* DDM Parameter table for connector OnboardHMI */
static conn_onboardhmi_param_t conn_onboardhmi_param_db[] =
{
    //.ddm_parameter        .type            .pub      .sub             .i32Value   			             .cb_func
    {IV0AQST     ,       DDM2_TYPE_INT32_T,    1,        0,       IV0AQST_AIR_QUALITY_UNKNOWN,   handle_hmi_ctrl_sub_data},     //Seg S0
    {IV0FILST    ,       DDM2_TYPE_INT32_T,    1,        0,    IV0FILST_FILTER_CHANGE_NOT_REQ,   handle_hmi_ctrl_sub_data},     //Seg S1
    {IV0WARN     ,       DDM2_TYPE_INT32_T,    1,        0,                IV0WARN_NO_WARNING,   handle_hmi_ctrl_sub_data},     //Seg S2
	{IV0MODE     ,       DDM2_TYPE_INT32_T,    1,        0,                      IV0MODE_AUTO,   handle_hmi_ctrl_sub_data},     //Seg S3, S4, S5, S6
	{IV0STORAGE  ,       DDM2_TYPE_INT32_T,    1,        0,             IV0STORAGE_DEACTIVATE,   handle_hmi_ctrl_sub_data},     //Seg S7
    {IV0PWRSRC   ,       DDM2_TYPE_INT32_T,    1,        0,   IV0PWRSRC_12V_CAR_BATTERY_INPUT,   handle_hmi_ctrl_sub_data},     //Seg S8
    {WIFI0STS    ,       DDM2_TYPE_INT32_T,    0,        1,                  WIFI0STS_UNKNOWN,   handle_hmi_ctrl_sub_data},     //Seg S9
	{IV0PWRON    ,       DDM2_TYPE_INT32_T,    1,        0,                      IV0PWRON_OFF,   handle_hmi_ctrl_sub_data},     //Seg S12
    {IV0IONST    ,       DDM2_TYPE_INT32_T,    1,        0,                      IV0IONST_ON,   handle_hmi_ctrl_sub_data},
    {IV0ERRST    ,       DDM2_TYPE_INT32_T,    1,        0,                 IV0ERRST_NO_ERROR,   handle_hmi_ctrl_sub_data},
    {IV0BLREQ    ,       DDM2_TYPE_INT32_T,    1,        0,                     IV0BLREQ_IDLE,   handle_hmi_ctrl_sub_data},
    {IV0HMITST   ,       DDM2_TYPE_INT32_T,    1,        0,                                 0,   handle_hmi_ctrl_sub_data},
    {IVPMGR0STATE,       DDM2_TYPE_INT32_T,    0,        1,              IVPMGR0STATE_STANDBY,   handle_hmi_ctrl_sub_data},
    {DIM0LVL     ,       DDM2_TYPE_INT32_T,    0,        1,              DIM_LVL_DUTY_CYCLE_0, 	 handle_hmi_ctrl_sub_data},
    {BT0PAIR     ,       DDM2_TYPE_INT32_T,    0,        1, BT0PAIR_OUT_PAIRING_MODE_INACTIVE, 	 handle_hmi_ctrl_sub_data},
//    {SDP0AVL     ,       DDM2_TYPE_INT32_T,    0,        1,           DP_SENSOR_NOT_AVAILABLE,   handle_hmi_ctrl_sub_data},
    {SDP0DP      ,       DDM2_TYPE_INT32_T,    0, 	     1, 	                            0,   handle_hmi_ctrl_sub_data},
    {IV0PRST     ,       DDM2_TYPE_INT32_T,    1,        0,      IV0PRST_PRESS_STATUS_UNKNOWN,                       NULL},
    {IV0SETCHRGCRNT,     DDM2_TYPE_INT32_T,    1,        0,                                 0,   handle_hmi_ctrl_sub_data},
    {IV0SETT       ,     DDM2_TYPE_INT32_T,    1,        0,                                 0,   handle_hmi_ctrl_sub_data},
    {IV0STGT        ,   DDM2_TYPE_INT32_T,    1,        0,                                 0,   handle_hmi_ctrl_sub_data},     //
    {SBMEB0AQR      ,   DDM2_TYPE_INT32_T,    0,        1,                                 0,   handle_hmi_ctrl_sub_data},    //

};

static const uint32_t conn_onbhmi_db_elements = ELEMENTS(conn_onboardhmi_param_db);

static void inventory_handler_cb(void *arg, uint32_t device_class_instance, bool is_available)
{
    if (SDP0 == DDMP2_INVENTORY_CLASS(device_class_instance))
    {
        //data_id = UPDATE_SEG_DP_SENS_STAT;
        ble_dp_con_sts = (uint8_t)(is_available ? 1 : 0);
        push_data_to_the_queue(ble_dp_con_sts, UPDATE_SEG_DP_SENS_STAT);
    }
    if (is_available)
    {
        conn_onboardhmi_param_t *ptr_param_db;

        for (uint8_t db_idx = 0; db_idx < conn_onbhmi_db_elements; db_idx++)
        {
            ptr_param_db = &conn_onboardhmi_param_db[db_idx];

            /* Check the DDM parameter need subscription */
            if (ptr_param_db->sub)
            {
                if (DDM2_PARAMETER_CLASS_INSTANCE(ptr_param_db->ddm_parameter) == DDM2_PARAMETER_CLASS_INSTANCE(device_class_instance))
                {
                    TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SUBSCRIBE, ptr_param_db->ddm_parameter, NULL, 0, connector_onboard_hmi.connector_id, portMAX_DELAY));
                }
            }
        }
    }
}

/**
  * @brief  Initialize the connector for Onboard HMI
  * @param  none.
  * @retval none.
  */
static int initialize_connector_onboardhmi(void)
{
    hal_timer_config_t timer_config_parameter;

    /* Initialize the onboard hmi state machine structure variables */
    onbrd_hmi_ctrl_sm.onbrd_hmi_ctrl_state = ONBOARD_HMI_STATE_IDLE;
    onbrd_hmi_ctrl_sm.seg_state            = ENABLE_ALL_SEGEMENT;

    /* Create queue for invent control task */
    obhmi_pwr_ctrl_que_hdl = osal_queue_create(OBHMI_PWR_CTRL_TASK_QUE_LEN, OBHMI_PWR_CTRL_TASK_QUE_ITEM_SIZE);

    if (NULL == obhmi_pwr_ctrl_que_hdl)
    {
#if CONN_ONBHMI_DEBUG_LOG
        LOG(E, "Queue creation failed for inventilate onboard hmi control task");
#endif
    }

    /* Create the one shot timer software timer for onboard HMI events handling */
    obHmiOneShotTimer = xTimerCreate("obHmiOneShotTimer", OBHMI_WAIT_TIME_TICKS, pdFALSE, 0, onboard_hmi_timer_cb_func);

#ifndef ESP_HW_TIMER
    /* Create the one shot timer software timer for onboard HMI events handling */
    long_press_tmr_hdle = xTimerCreate("long_pr_tmr", LONG_PRESS_TIMER_TICKS, pdFALSE, 0, long_press_tmr_cb);


    /* Create the one shot timer software timer for onboard HMI events handling */
    long_press_rel_tmr_hdle = xTimerCreate("long_pr_rel_tmr", LONG_PRESS_REL_TIMER_TICKS, pdFALSE, 0, long_press_rel_tmr_cb);

    /* Create the one shot timer software timer for onboard HMI events handling */
    short_press_tmr_hdle = xTimerCreate("short_pr_tmr", SHORT_PRESS_TIMER_TICKS, pdFALSE, 0, short_press_tmr_cb);
#endif


    /* Create the periodic software timer for blinking HMI segments */
    blink_ctrl_timer_hdle = xTimerCreate("blink_tmr", BLINK_TIMER_TICKS, pdTRUE, 0, obhmi_blink_tmr_cb);

    /* Initialize the device Onboard HMI */
    initialize_dev_onboardhmi();
    //ESP32_timer_config

    timer_config_parameter.timer_configuration.alarm_en = true;
    timer_config_parameter.timer_configuration.counter_en = false;
    timer_config_parameter.timer_configuration.intr_type = HAL_HW_TIMER_INT_MODE;
    timer_config_parameter.timer_configuration.counter_dir = HAL_HW_TIMER_COUNT_DIR;
    timer_config_parameter.timer_configuration.auto_reload = true;
    timer_config_parameter.timer_configuration.divider = HAL_HW_TIMER_DIVIDER;
    timer_config_parameter.timer_cb = &inv_timer_cb;
    timer_config_parameter.alarm_interval = HAL_HW_TIMER_INTERVAL;
    timer_config_parameter.timer_group = INV_TIMER_GRP;
    timer_config_parameter.timer_group_num = INV_TIMER_NUM;

    hal_init_timer(&timer_config_parameter);

    timer_queue = xQueueCreate(10, sizeof(hal_timer_event_t));

    /* Publish the class availability to the broker */
    install_parameters();

    // Instantiate, initialize and start inventory handler object
    inventory_handler_init(&l_ih, &inv_storage, inventory_handler_cb, NULL);

    /* Subscribe the all the needed DDMP parameters to the broker */
    start_subscribe();
    start_publish();

    /* Create Task for connector OnboardHMI to handle/process all the ddmp related activities */
    TRUE_CHECK(osal_task_create(conn_onboardhmi_process_task, CONNECTOR_ONBOARDHMI_PROCESS_TASK_NAME, CONNECTOR_ONBOARD_HMI_PROCESS_STACK_DEPTH, NULL, CONNECTOR_ONBOARD_HMI_PROCESS_PRIORITY, NULL));
	TRUE_CHECK(osal_task_create(conn_onboardhmi_ctrl_task, CONNECTOR_ONBOARDHMI_PWR_CTRL_TASK_NAME, CONNECTOR_ONBOARD_HMI_CTRL_TASK_STACK_DEPTH, obhmi_pwr_ctrl_que_hdl, CONNECTOR_ONBOARD_HMI_CTRL_TASK_PRIORITY, NULL));

    TRUE_CHECK(osal_task_create(conn_obdhmi_btn_task, CONNECTOR_OBHMI_BTN_TASK_NAME, CONNECTOR_OBHMI_BTN_TASK_STACK_DEPTH, timer_queue, CONNECTOR_OBHMI_BTN_TASK_TASK_PRIORITY, NULL));

    /* At board startup set the max duty cycle for HMI backlight */
    hmi_backlight_set_duty(ONBOARD_HMI_MAX_DUTY_CYCLE);
    /* Change the state */
    change_onhmi_ctrl_state(onbrd_hmi_ctrl_sm.onbrd_hmi_ctrl_state);

    return 1;
}

/**
  * @brief  Function to subscribe the DDMP parameters
  * @param  none.
  * @retval none.
  */
static void start_subscribe(void)
{
    conn_onboardhmi_param_t *ptr_param_db;
    uint16_t db_idx;
    uint8_t num_elements = ELEMENTS(conn_onboardhmi_param_db);

    for (db_idx = 0; db_idx < num_elements; db_idx++)
    {
        ptr_param_db = &conn_onboardhmi_param_db[db_idx];

        /* Check the DDM parameter need subscription */
        if (ptr_param_db->sub)
        {
            if (DDM2_PARAMETER_GROUP(ptr_param_db->ddm_parameter) != DDM2_PARAMETER_GROUP(SNODE0))
            {
                TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SUBSCRIBE, ptr_param_db->ddm_parameter, NULL, 0, connector_onboard_hmi.connector_id, (TickType_t)portMAX_DELAY));
            }
            else
            {
                // Add to inventory handler
                inventory_handler_add(&l_ih, DDM2_PARAMETER_CLASS_INSTANCE(ptr_param_db->ddm_parameter));
            }
        }
    }
}

/**
  * @brief  Function to publish the DDMP parameters
  * @param  none.
  * @retval none.
  */
static void start_publish(void)
{
    conn_onboardhmi_param_t *ptr_param_db;
    uint16_t db_idx;
    uint8_t num_elements = ELEMENTS(conn_onboardhmi_param_db);

    for (db_idx = 0; db_idx < num_elements; db_idx++)
    {
        ptr_param_db = &conn_onboardhmi_param_db[db_idx];

        /* Check the DDM parameter need to publish, owned parameters */
        if (ptr_param_db->pub)
        {
            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, ptr_param_db->ddm_parameter, &ptr_param_db->i32Value, sizeof(int32_t), connector_onboard_hmi.connector_id, portMAX_DELAY));
        }
    }
}

/**
  * @brief  Function to process the set and publish parameter from broker
  * @param  DDM parameter.
  * @param  i32value.
  * @param  req_type refer enum DDMP2_CONTROL_ENUM.
  * @retval none.
  */
static void process_set_and_publish_request(uint32_t ddm_param, int32_t i32value, DDMP2_CONTROL_ENUM req_type)
{
    int32_t i32Index;
    int32_t i32Factor;
    int32_t pub_value = i32value;
	uint16_t db_idx;
	conn_onboardhmi_param_t* param_db;

	/* Validate the DDM parameter received */
	db_idx = get_ddm_index_from_db(ddm_param);

	if (DDMP_UNAVAILABLE != db_idx)
	{
        if (req_type == DDMP2_CONTROL_SET)
        {
            /* Frame and send the publish request */
            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, ddm_param, &pub_value, sizeof(int32_t),
                   connector_onboard_hmi.connector_id, portMAX_DELAY));
        }

		param_db = &conn_onboardhmi_param_db[db_idx];

        i32Index = ddm2_parameter_list_lookup(DDM2_PARAMETER_BASE_INSTANCE(ddm_param));

        if (-1 != i32Index)
        {
            if (i32Index < DDM2_PARAMETER_COUNT)
            {
                i32Factor = Ddm2_unit_factor_list[Ddm2_parameter_list_data[i32Index].in_unit];
            }
            else
            {
                i32Factor = 1;
            }
            if (i32Factor > 0)
            {
                i32value = i32value / i32Factor;
            }

            if (i32value != param_db->i32Value)
            {
                /* Update the received value in the database table*/
	  	        param_db->i32Value = i32value;
		        /* Check callback function registered for this DDM parameter */
		        if (NULL != param_db->cb_func)
		        {
                    /* Execute the callback function */
                    param_db->cb_func(ddm_param, i32value);
		        }
            }
        }
	}
}

/**
  * @brief  Task for Connector OnboardHMI to hanle the DDMP request
  * @param  pvParameter.
  * @retval none.
  */
static void conn_onboardhmi_process_task(void *pvParameter)
{
    DDMP2_FRAME *pframe;
	size_t frame_size;

	inventory_handler_start(&l_ih, &connector_onboard_hmi);

	while (1)
	{
        TRUE_CHECK (pframe = xRingbufferReceive(connector_onboard_hmi.to_connector, &frame_size, portMAX_DELAY));
        if (!inventory_handler_update(&l_ih, pframe))
        {
            switch (pframe->frame.control)
            {
                case DDMP2_CONTROL_PUBLISH:
                    process_set_and_publish_request(pframe->frame.publish.parameter, pframe->frame.publish.value.int32, pframe->frame.control);
                    break;

                case DDMP2_CONTROL_SET:
                    if (((IVPMGR0STATE_STORAGE == ptr_hmi_ctrl_sm->inv_state) && (pframe->frame.set.parameter == IV0PWRON)) ||
                            ((IVPMGR0STATE_STORAGE == ptr_hmi_ctrl_sm->inv_state) && (pframe->frame.set.parameter == IV0MODE)))
                    {
                        /* power button functionalities and short press for mode button is not required in storage mode */
                    }
                    else
                    {
                        process_set_and_publish_request(pframe->frame.set.parameter, pframe->frame.set.value.int32, pframe->frame.control);
                    }
                    break;

                case DDMP2_CONTROL_SUBSCRIBE:
                    process_subscribe_request(pframe->frame.subscribe.parameter);
                    break;

                default:
    #if CONN_ONBHMI_DEBUG_LOG
                    LOG(W, "UNHANDLED frame %02x from broker!", pframe->frame.control);
    #endif
                    break;
            }
        }

        vRingbufferReturnItem(connector_onboard_hmi.to_connector, pframe);
    }
}

/**
  * @brief  Function to handle the blinking of segments
  * @param  ptr_seg_stat Pointer to the blink enabled segemnt
  * @retval none.
  */
static void handle_hmi_blink_segment(void)
{
    blink_timer_counter += OBHMI_BLINK_CTRL_TASK_DELAY_MSEC;

    if (blink_timer_counter >= FAST_BLINK_TIME_INTERVAL)
    {
        blink_timer_counter = 0;

        for (uint8_t seg = 0; seg < ONBOARD_HMI_MAX_SEGEMENT; seg++)
        {
            if (g_blink_segment_status & (1 << seg))
            {
                uc1510c_set_segment(seg, blink_flag);
            }
            else
            {
                /* Restore the segment to its previous status */
                if (prev_seg_stat[seg] != seg_stat[seg])
                {
                    uc1510c_set_segment(seg, seg_stat[seg]);
                    prev_seg_stat[seg] = seg_stat[seg];
                }
            }
        }

        blink_flag = !blink_flag;
    }
}

static void restore_seg(void)
{

    for (uint8_t seg = 0; seg < ONBOARD_HMI_MAX_SEGEMENT; seg++)
    {
        uc1510c_set_segment(seg, seg_stat[seg]);
        prev_seg_stat[seg] = seg_stat[seg];
    }

}
/**
  * @brief  Function to update the blink segment status based on received error code
  *
  * @param  error_code.
  * @retval none.
  */
void update_blink_info(const uint32_t error_code)
{
    uint8_t index;
    uint32_t blink_seg_curr_status = g_blink_segment_status;

    g_error_code = g_error_code | error_code;
    for (index = 0; index < MAX_NUM_ERROR_CODES; index++)
    {
        if ((error_code & (1 << index)) == 0)
        {
           g_error_ack &= ~(1 << index);
        }

        if ((error_code & (1 << index)) == 0  || ((g_error_ack & (1 << index)) > 0))
        {
            switch (index)
            {
                //Errors acknowledged by USER
                case DP_SENSOR_BOARD_BATTERY_LOW:
                    blink_seg_curr_status &= ~(1 << SEG_WARNING_STATUS);
                    blink_seg_curr_status &= ~(1 << SEG_SOLAR_BATTERY_STATUS);
                    break;

                //Errors acknowledged by USER and SERVICE
                case DP_SENSOR_BOARD_DISCONNECTED:

                    if (user_errack < USER_ERRACK_MAX)
                    {
                        blink_seg_curr_status &= ~(1 << SEG_BLE_STATUS);
                        blink_seg_curr_status &= ~(1 << SEG_WARNING_STATUS);
                    }
                    else
                    {
                        blink_seg_curr_status &= ~(1 << SEG_BLE_STATUS);
                        blink_seg_curr_status &= ~(1 << SEG_MODE_BUTTON);
                        blink_seg_curr_status &= ~(1 << SEG_WARNING_STATUS);
                    }
                    restore_seg();
                    break;

                //Errors acknowledged by SERVICE
                case VOC_SENSOR_COMMUNICATION_ERROR:
                    //Hanlde VOC sensor communication error
                    //fallthrough
                case VOC_SENSOR_DATA_PLAUSIBLE_ERROR:
                    //Hanlde VOC sensor Data error
                    //fallthrough
                case DP_SENSOR_DATA_PLAUSIBLE_ERROR:
                    //Handle DP sensor data error
                    //fallthrough
                case FAN1_RPM_MISMATCH:
                    //process FAN1 RPM error
                    //fallthrough
                case FAN1_TACHO_READ_DEVICE_INACTIVE:
                    //process FAN1 Tacho reading error when device is in in-active state
                    //fallthrough
                case FAN1_NO_TACHO_DEVICE_ACTIVE:
                    //process FAN1 Tacho when device is in active state
                    //fallthrough
                case FAN2_RPM_MISMATCH:
                    //process FAN2 RPM error
                    //fallthrough
                case FAN2_TACHO_READ_DEVICE_INACTIVE:
                    //process FAN2 Tacho reading error when device is in in-active state
                    //fallthrough
                case FAN2_NO_TACHO_DEVICE_ACTIVE:
                    //process FAN2 Tacho reading error when device is in active state
                    //fallthrough
                case MOTOR_RPM_MISMATCH:
                    //process Motor RPM error
                    //fallthrough
                case MOTOR_TACHO_READ_DEVICE_INACTIVE:
                    //process Motor Tacho reading error when device is in in-active state
                    //fallthrough
                case MOTOR_NO_TACHO_DEVICE_ACTIVE:
                    //process Motor Tacho reading error when device is in active state
                    //fallthrough
                case BACKUP_BATTERY_LOW:
                    //Process backup Battery low condition
                    //fallthrough
                case COMM_ERROR_WITH_BATTERY_IC:
                    //Handle Battery ic communication error
                    //fallthrough
                case BATTERY_OVER_HEATING:
                    //Handle Battery IC temperature status
                    //fallthrough
                case BATTERY_EXPIRED:
                    //Handle back up battery voltage status
                    //fallthrough
                case COMM_ERROR_WITH_LCD_DRIVER_IC:
                    //Handle LCD driver communication
                    //fallthrough
                case TOUCH_BUTTON_EVENT_PROCESS_ERROR:
                    //Handle Touch error events
                    //fallthrough
                case CAN_COMMUNICATION_ERROR:
                    //Handle CAN communication error
                    //fallthrough
                case LIN_COMMUNICATION_ERROR:
                    //Handle LIN communication error
                    blink_seg_curr_status &= ~(1 << SEG_MODE_BUTTON);
                    blink_seg_curr_status &= ~(1 << SEG_WARNING_STATUS);
                    break;
                case  UNUSED_3:
                //fallthrough
                case  UNUSED_6:
                //fallthrough:
                case UNUSED_17:
                //fallthrough
                case UNUSED_18:
                    /*Unused error codes*/
                    break;
                default:
                    /*Do Nothing*/
#if CONN_ONBHMI_DEBUG_LOG
                    LOG(W, "Clear: Unhandled error code %d", index);
#endif
                    break;
            }
        }
    }

    for (index = 0; index < MAX_NUM_ERROR_CODES; index++)
    {
        if ((error_code & (1 << index)) && ((g_error_ack & (1 << index)) == 0))
        {
            switch (index)
            {
                case DP_SENSOR_BOARD_BATTERY_LOW:
                    blink_seg_curr_status |= (1 << SEG_WARNING_STATUS);
                    blink_seg_curr_status |= (1 << SEG_SOLAR_BATTERY_STATUS);
                    break;

                case DP_SENSOR_BOARD_DISCONNECTED:
                    if (user_errack < USER_ERRACK_MAX)
                    {
                        blink_seg_curr_status |= (1 << SEG_BLE_STATUS);
                        blink_seg_curr_status |= (1 << SEG_WARNING_STATUS);
                        user_errack++;
                        restore_seg();              //Restore LCD segments
                    }
                    else
                    {
                        blink_seg_curr_status &= ~(1 << SEG_BLE_STATUS);
                        blink_seg_curr_status |= (1 << SEG_MODE_BUTTON);
                        blink_seg_curr_status |= (1 << SEG_WARNING_STATUS);
                        restore_seg();          //Restore LCD segments
                    }
                    break;

                case VOC_SENSOR_COMMUNICATION_ERROR:
                    //Handle VOC sensor communication error
                    //fallthrough
                case VOC_SENSOR_DATA_PLAUSIBLE_ERROR:
                    //Handle VOC sensor Data error
                    //fallthrough
                case DP_SENSOR_DATA_PLAUSIBLE_ERROR:
                    //process FAN1 RPM error
                    //fallthrough
                case FAN1_RPM_MISMATCH:
                    //process FAN1 RPM error
                    //fallthrough
                case FAN1_TACHO_READ_DEVICE_INACTIVE:
                    //process FAN1 Tacho reading error when device is in in-active state
                    //fallthrough
                case FAN1_NO_TACHO_DEVICE_ACTIVE:
                    //process FAN1 Tacho when device is in active state
                    //fallthrough
                case FAN2_RPM_MISMATCH:
                    //process FAN2 RPM error
                    //fallthrough
                case FAN2_TACHO_READ_DEVICE_INACTIVE:
                    //process FAN2 Tacho reading error when device is in in-active state
                    //fallthrough
                case FAN2_NO_TACHO_DEVICE_ACTIVE:
                    //process FAN2 Tacho reading error when device is in active state
                    //fallthrough
                case MOTOR_RPM_MISMATCH:
                    //process Motor RPM error
                    //fallthrough
                case MOTOR_TACHO_READ_DEVICE_INACTIVE:
                    //process Motor Tacho reading error when device is in in-active state
                    //fallthrough
                case MOTOR_NO_TACHO_DEVICE_ACTIVE:
                    //process Motor Tacho reading error when device is in active state
                    //fallthrough
                case BACKUP_BATTERY_LOW:
                    //Process backup Battery low condition
                    //fallthrough
                case COMM_ERROR_WITH_BATTERY_IC:
                    //Handle Battery ic communication error
                    //fallthrough
                case BATTERY_OVER_HEATING:
                    //Handle Battery IC temperature status
                    //fallthrough
                case BATTERY_EXPIRED:
                    //Handle back up battery voltage status
                    //fallthrough
                case COMM_ERROR_WITH_LCD_DRIVER_IC:
                    //Handle LCD driver communication
                    //fallthrough
                case TOUCH_BUTTON_EVENT_PROCESS_ERROR:
                //Handle Touch error events
                //fallthrough
                case CAN_COMMUNICATION_ERROR:
                    //Handle CAN communication error
                    //fallthrough
                case LIN_COMMUNICATION_ERROR:                                   //Handle LIN communication error
                    blink_seg_curr_status |= (1 << SEG_MODE_BUTTON);
                    blink_seg_curr_status |= (1 << SEG_WARNING_STATUS);
                    restore_seg();
                    break;

                default:
                    /*Do Nothing*/
#if CONN_ONBHMI_DEBUG_LOG
                    LOG(W, "Set: Unhandled error code %d", index);
#endif
                    break;
            }
        }
    }

    g_blink_segment_status = blink_seg_curr_status;
}

/**
 * @brief error code check and acknowledge when user press any button.
 *  Clear segments showing the Error code.
 */
void error_check_ack(void)
{
    uint8_t index;

    for (index = 0; index < MAX_NUM_ERROR_CODES; index++)
    {
         if ((g_error_code & (1 << index)) > 0)
         {
            g_error_ack = g_error_ack | (1 << index);
         }
    }
}

/**
  * @brief  Task to control the onboard HMI
  * @param  pvParameter.
  * @retval none.
  */
static void conn_onboardhmi_ctrl_task(void *pvParameter)
{
    uint8_t update_seg = 0;
    uint8_t check_timer_stat = 0;
    uint32_t calc_resol = 0;
    int32_t ble_on_off_value = 0;

    while (1)
    {
        /* Queue will be in blocked state untill data recevied */
        if (osal_success == osal_queue_receive((osal_queue_handle_t)pvParameter, (void *)&ptr_hmi_ctrl_sm->data_frame, portMAX_DELAY))
        {
            /* Parse the data received from queue */
            switch (ptr_hmi_ctrl_sm->data_frame.data_id)
            {
                case INV_STATE:
                    ptr_hmi_ctrl_sm->inv_state = ptr_hmi_ctrl_sm->data_frame.data;
                    break;

                case ONBOARD_HMI_CTRL_STATE_CHANGE:
                    /* Change the state */
                    ptr_hmi_ctrl_sm->onbrd_hmi_ctrl_state = ptr_hmi_ctrl_sm->data_frame.data;
                    break;

                case BTN_PRESSED_EVENT:
                    hmi_backlight_set_duty(ONBOARD_HMI_MAX_DUTY_CYCLE);
                    reset_hmi_wait_timer();
                    if (hmi_standby_mode == 1)
                    {
                        hmi_standby_mode = 0;
                    }
                    else
                    {
                        if (ONBOARD_HMI_STATE_STARTUP != ptr_hmi_ctrl_sm->onbrd_hmi_ctrl_state)
                        {
                            /* Handle button events */
                            handle_onboard_hmi_button_event((uint16_t)ptr_hmi_ctrl_sm->data_frame.data, ptr_hmi_ctrl_sm->inv_state, ptr_hmi_ctrl_sm->invent_error_status);
                        }
                    }
                    break;

                case HMI_BACK_LIGHT_TEST:
                    if (ptr_hmi_ctrl_sm->data_frame.data < HMI_BACKLIGHT_MIN_DUTY_CYCLE)
                    {
                        ptr_hmi_ctrl_sm->data_frame.data = HMI_BACKLIGHT_MIN_DUTY_CYCLE;
                    }
                    else if (ptr_hmi_ctrl_sm->data_frame.data > HMI_BACKLIGHT_MAX_DUTY_CYCLE)
                    {
                        ptr_hmi_ctrl_sm->data_frame.data = HMI_BACKLIGHT_MAX_DUTY_CYCLE;
                    }
                    else
                    {

                    }

                    calc_resol = calc_linear_interpolation(HMI_BACKLIGHT_MIN_DUTY_CYCLE, HMI_BACKLIGHT_MAX_DUTY_CYCLE, LEDC_PWM_MIN_DUTY_CYCLE, LEDC_PWM_MAX_DUTY_CYCLE, ptr_hmi_ctrl_sm->data_frame.data);
#if CONN_ONBHMI_DEBUG_LOG
                    LOG(I, "calc_resol = %d", calc_resol);
#endif
                    /* Set the backlight in the onboard HMI */
                    hmi_backlight_set_duty(calc_resol);
                    break;

                /*30 seconds timer when no button is pressed has expired*/
                case HMI_TIMER_EXPIRED:
                    if (ONBOARD_HMI_STATE_STARTUP != ptr_hmi_ctrl_sm->onbrd_hmi_ctrl_state)
                    {
#if CONN_ONBHMI_DEBUG_LOG
                        LOG(W, "HMI_TIMER_EXPIRED mode ");
#endif
                        if (ptr_hmi_ctrl_sm->selected_mode == INV_SLEEP)
                        {
                            hmi_backlight_set_duty(ONBOARD_HMI_5_DUTY_CYCLE);
                        }
                        else if (ptr_hmi_ctrl_sm->selected_mode == INV_STORAGE)
                        {
                            hmi_backlight_set_duty(ONBOARD_HMI_5_DUTY_CYCLE);
                        }
                        else
                        {
                            hmi_backlight_set_duty(ONBOARD_HMI_MIN_DUTY_CYCLE);
                        }
                        hmi_standby_mode = 1;
                        /* Closing the window for mobile App pairing */
                    }
                    break;

                case INV_ERROR_STATUS:
                    ptr_hmi_ctrl_sm->invent_error_status = ptr_hmi_ctrl_sm->data_frame.data;

                    if (ptr_hmi_ctrl_sm->invent_error_status != ptr_hmi_ctrl_sm->invent_prev_err_status)
                    {
                        ptr_hmi_ctrl_sm->invent_prev_err_status = ptr_hmi_ctrl_sm->invent_error_status;

                        update_blink_info(ptr_hmi_ctrl_sm->invent_error_status);

                        check_timer_stat = 1;
                    }
                    break;

                default:
                    break;
            }

            if (ptr_hmi_ctrl_sm->onbrd_hmi_ctrl_state != ONBOARD_HMI_STATE_ACTIVE)
            {
                /* Store the status of HMI segments without updating in HMI */
                obhmi_update_var(ptr_hmi_ctrl_sm->data_frame.data_id, ptr_hmi_ctrl_sm->data_frame.data);
            }

            /* Process the received data */
            switch (ptr_hmi_ctrl_sm->onbrd_hmi_ctrl_state)
            {
                case ONBOARD_HMI_STATE_IDLE:
                    stop_obhmi_blink_timer();
                    /* Start timer */
                    start_hmi_wait_timer(OBHMI_NO_HMI_ACTION_WAIT_TIME_MSEC);
                    /* Reset the HMI btn variables */
                    reset_hmi_btn_ctrl_variables();
                    /* Update the Power Mode status and Mode to the broker */
                    update_and_send_value_to_broker(IV0MODE, IV0MODE_OFF);
                    update_and_send_value_to_broker(IV0PWRON, IV0PWRON_OFF);
                    /* Update segments */
                    onboard_hmi_update_segments(POWERED_OFF_STATE_SEG);
                    /* Change the state */
                    change_onhmi_ctrl_state(ONBOARD_HMI_STATE_OFF);
                    break;

                case ONBOARD_HMI_STATE_OFF:
#if CONN_ONBHMI_DEBUG_LOG
                    LOG(I, "ONBOARD_HMI_STATE_OFF inv_state = %d seg_state = %d", ptr_hmi_ctrl_sm->inv_state, ptr_hmi_ctrl_sm->seg_state);
#endif
                    if (IVPMGR0STATE_ACTIVE == ptr_hmi_ctrl_sm->inv_state)
                    {
                        /* Set the segement state */
                        ptr_hmi_ctrl_sm->seg_state = ENABLE_ALL_SEGEMENT;
                        /* Change the state */
                        change_onhmi_ctrl_state(ONBOARD_HMI_STATE_STARTUP);
                    }
                    break;

                case ONBOARD_HMI_STATE_STARTUP:
                    /* Start the Inventilate Powered ON Indication sequence in HMI */
                    // Step 1 : Enable all segments in Onboard HMI
                    // Step 2 : Wait for 2 seconds
                    // Step 3 : Disable all segments in Onboard HMI
                    // Step 4 : Wait for 2 seconds
                    // Step 5 : Enable the segments as per the status
                    switch (ptr_hmi_ctrl_sm->seg_state)
                    {
                        case ENABLE_ALL_SEGEMENT:
                            /* Update segments */
                            onboard_hmi_update_segments(ptr_hmi_ctrl_sm->seg_state);
                            /* Start timer */
                            start_hmi_wait_timer(OBHMI_WAIT_TIME_MSEC);
                            /* set the segment state */
                            ptr_hmi_ctrl_sm->seg_state = DISABLE_ALL_SEGEMENT;
                            ble_on_off_value = BT0PAIR_IN_PAIRING_MODE_OFF;
                            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, BT0PAIR, &ble_on_off_value, sizeof(int32_t),
                                            connector_onboard_hmi.connector_id, portMAX_DELAY));
                            break;

                        case DISABLE_ALL_SEGEMENT:
                            if (HMI_TIMER_EXPIRED == ptr_hmi_ctrl_sm->data_frame.data_id)
                            {
                                /* Set LCD backight duty cycle */
                                hmi_backlight_set_duty(ONBOARD_HMI_OFF_DUTY_CYCLE);
                                /* Update segments */
                                onboard_hmi_update_segments(ptr_hmi_ctrl_sm->seg_state);
                                /* Start timer */
                                start_hmi_wait_timer(OBHMI_WAIT_TIME_MSEC);
                                /* set the segment state */
                                ptr_hmi_ctrl_sm->seg_state = POWERED_ON_STATE_SEG;
                            }
                            break;

                        case POWERED_ON_STATE_SEG:
                            if (HMI_TIMER_EXPIRED == ptr_hmi_ctrl_sm->data_frame.data_id)
                            {
                                /* Set LCD backight duty cycle */
                                hmi_backlight_set_duty(ONBOARD_HMI_MAX_DUTY_CYCLE);
                                /* Update segments */
                                onboard_hmi_update_segments(ptr_hmi_ctrl_sm->seg_state);
                                /* Update the default mode selection to the broker */
                                update_and_send_value_to_broker(IV0MODE, IV0MODE_AUTO);
                                /* Start timer */
                                start_hmi_wait_timer(OBHMI_NO_HMI_ACTION_WAIT_TIME_MSEC);
                                /* Change the state */
                                change_onhmi_ctrl_state(ONBOARD_HMI_STATE_ACTIVE);
                            }
                            break;

                        default:
                            break;
                    }
                    break;

                case ONBOARD_HMI_STATE_ACTIVE:
#if CONN_ONBHMI_DEBUG_LOG
                    //LOG(I, "ONBOARD_HMI_STATE_ACTIVE inv_state = %d", ptr_hmi_ctrl_sm->inv_state);
#endif
                    /* Process received data frame from queue */
                    switch (ptr_hmi_ctrl_sm->data_frame.data_id)
                    {
                        case BLINK_SEG_BLE_STAT:
                            //Blink BLE icon on Bluetooth Communication Error
                            check_timer_stat = 1;
                            break;

                        case BLINK_SEG_FILT_STAT:
                            /*Handle filter status*/
                            check_timer_stat = 1;
                            break;

                        case UPDATE_SEG_BLE:            //Show / hide BLE icon on connect/disconnect events
                        case UPDATE_SEG_DP_SENS_STAT:
                            LOG(W, "DP Sensor connectivity changed..Update BLE Status id %d data %d", ptr_hmi_ctrl_sm->data_frame.data_id, ptr_hmi_ctrl_sm->data_frame.data );
                            update_seg = 1;
                            break;

                        case UPDATE_SEG_AQ:               //Show / Hide IAQ segments when air quality changes
                            //fallthrough
                        case UPDATE_SEG_FILT_STAT:        //Show filter icon on Expiry of filter timing(500hours)
                            //fallthrough
                        case UPDATE_SEG_WARN:             //Blink warning icon for error coded.
                            //fallthrough
                        case UPDATE_SEG_PWRSRC:           //Show / Hide Solar/Battery icon
                            //fallthrough
                        case UPDATE_SEG_WIFI:             //Show / Hide Wifi icon depending on the availablity of WIFI
                            //fallthrough
                        case UPDATE_SEG_IONIZER:          //Show / Hide Inonizer icon when ever necessary
                            //fallthrough
                        case UPDATE_SEG_MODE:
                            // "Update Seg " pointed by ptr_hmi_ctrl_sm->data_frame.data_id)
                            if (ptr_hmi_ctrl_sm->data_frame.data_id == UPDATE_SEG_PWRSRC)
                            {
                                ptr_hmi_ctrl_sm->selected_pwr_src = ptr_hmi_ctrl_sm->data_frame.data_id;
                            }
#if CONN_ONBHMI_DEBUG_LOG
                            LOG(W, "Update SEG Status AQ,Filter,warn,Pwrsrc,wifi,Ionizer");
#endif
                            update_seg = 1;
                            break;

                        case BLINK_TIMER_EXPIRED:
                            {
                                /* Handle the segement blink based on periodic timer expire event */
                                handle_hmi_blink_segment();

                                if (g_blink_segment_status == 0)
                                {
                                    if (blink_flag == 0)
                                    {
                                        restore_seg();
                                        ptr_hmi_ctrl_sm->blink_tmr_status = false;
                                        stop_obhmi_blink_timer();
                                    }
                                }
                            }
                            break;

                        case INV_STATE:
                            switch (ptr_hmi_ctrl_sm->data_frame.data)
                            {
                                case IVPMGR0STATE_STORAGE:
                                    ptr_hmi_ctrl_sm->selected_mode = INV_STORAGE;
                                    obhmi_set_segment(UPDATE_SEG_VEHMOD, IV0STORAGE_ACTIVATE);
                                    obhmi_set_segment(UPDATE_SEG_MODE, IV0MODE_OFF);
                                    obhmi_set_segment(UPDATE_SEG_AQ_OFF, IV0AQST_AIR_QUALITY_UNKNOWN);

                                    update_and_send_value_to_broker(IV0MODE, IV0MODE_OFF);
                                    update_and_send_value_to_broker(IV0AQST, IV0AQST_AIR_QUALITY_UNKNOWN);
                                    update_and_send_value_to_broker(DIM0LVL, DIM_LVL_DUTY_CYCLE_0);                             /*Turn off LED*/
                                    check_timer_stat = 1;
                                    break;

                                case IVPMGR0STATE_ACTIVE:
                                    obhmi_set_segment(UPDATE_SEG_VEHMOD, IV0STORAGE_DEACTIVATE);
                                    obhmi_set_segment(UPDATE_SEG_MODE, IV0MODE_AUTO);
                                    obhmi_set_segment(UPDATE_SEG_WIFI, inv_wifi_status);
                                    obhmi_set_segment(UPDATE_SEG_BLE, inv_ble_status);
                                    obhmi_set_segment(UPDATE_SEG_AQ, inv_iaq_status);

                                    update_and_send_value_to_broker(IV0MODE, IV0MODE_AUTO);
                                    check_timer_stat = 1;
                                    break;

                                case IVPMGR0STATE_STANDBY:
                                    /* Reset the segment state */
                                    ptr_hmi_ctrl_sm->seg_state = ENABLE_ALL_SEGEMENT;
                                    /* Change the HMI ctrl state */
                                    change_onhmi_ctrl_state(ONBOARD_HMI_STATE_IDLE);
                                    break;

                                default:
                                    break;
                            }
                            break;

                        default:
                            break;
                    }
                    break;

                default:
                    break;
            }

            if (check_timer_stat)
            {
                if (ptr_hmi_ctrl_sm->blink_tmr_status == false)
                {
                    start_obhmi_blink_timer();
                    ptr_hmi_ctrl_sm->blink_tmr_status = true;
                }

                check_timer_stat = 0;
            }

            if (update_seg)
            {
                if (ptr_hmi_ctrl_sm->data_frame.data != IV0MODE_OFF)
                {
                    ptr_hmi_ctrl_sm->selected_mode = ptr_hmi_ctrl_sm->data_frame.data;
                }
                obhmi_set_segment(ptr_hmi_ctrl_sm->data_frame.data_id, ptr_hmi_ctrl_sm->data_frame.data);
                update_seg = 0;
            }

            ptr_hmi_ctrl_sm->data_frame.data_id = INVALID_DATA_ID;
        }
    }
}


/**
  * @brief  Function to get ddm index from database table
  * @param  DDMP Parameter.
*/
static void conn_obdhmi_btn_task(void *pvParameter)
{
    hal_timer_event_t btn_evt;
    uint16_t button_press_event = BTN_NO_EVENTS;
    bool blink1_ok = 0;
    bool blink2_ok = 0;
    int32_t i32_ble_mode = 0;

    while (1)
    {
        /* Queue will be in blocked state untill data recevied */
        if (osal_success == osal_queue_receive((osal_queue_handle_t)pvParameter, (void *)&btn_evt, portMAX_DELAY))
        {
            ZERO_CHECK(uc1510c_get_touch_panel_status((uint8_t*)&lcd_btn_press_evt));
            if ((btn_evt.timer_counter_value == 1))
            {
                /* Short press */
                button_press_event = lcd_btn_press_evt | (BUTTON_EVT_SHORT_PRESS << 8);
                /* Push the data into the queue */
                push_data_to_the_queue(button_press_event, BTN_PRESSED_EVENT);
#if CONN_ONBHMI_DEBUG_LOG
                LOG(W, "short Press! %d", lcd_btn_press_evt);
#endif
            }
            else if ((btn_evt.timer_counter_value == 2) && (blink1_ok == 1))
            {
                //App pairing
                /*Long press#1 */;
#if CONN_ONBHMI_DEBUG_LOG
                LOG(W, "Long Press! LP#1 = %d", lcd_btn_press_evt);
#endif
                if (lcd_btn_press_evt == BTN_LIGHT)
                {
                    blink1_ok = 0;
                    i32_ble_mode = 1;
                    //TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, BT0SCAN, &i32_ble_mode, sizeof(int32_t),
//                                connector_onboard_hmi.connector_id, portMAX_DELAY));

                    i32_ble_mode = BT0PAIR_IN_PAIRING_MODE_ON;
                    TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, BT0PAIR, &i32_ble_mode, sizeof(int32_t),connector_onboard_hmi.connector_id, portMAX_DELAY));
                }
                else
                {
                    button_press_event = lcd_btn_press_evt | (BUTTON_EVT_LONG_PRESS << 8);
                    /* Push the data into the queue */
                    push_data_to_the_queue(button_press_event, BTN_PRESSED_EVENT);
                }
            }
            else if ((btn_evt.timer_counter_value == 3) && (blink2_ok == 1))
            {
                //DP sensor Pairing
                /* Long press#2 */;
                blink2_ok = 0;
#if CONN_ONBHMI_DEBUG_LOG
                LOG(W, "Long Press! LP#2 = %d", lcd_btn_press_evt);
#endif
                /* Push the data into the queue */
                prev_btn_press_evt = lcd_btn_press_evt;
                button_press_event = lcd_btn_press_evt | (BUTTON_EVT_LONG_PRESS << 8);
                push_data_to_the_queue(button_press_event, BTN_PRESSED_EVENT);
                lcd_btn_state = BTN_LONG_PRESS_RELEASED;

            }
            else if ((btn_evt.timer_counter_value == 4))
            {
                /*Blink#1 */;
                blink_led_bk_light(BLINK_LED_BK_LIGHT);
                blink1_ok = 1;
            }
            else if (btn_evt.timer_counter_value == 5)
            {
                /* Blink#2 */;
                if (lcd_btn_press_evt == BTN_LIGHT)
                {
                    blink_led_bk_light(BLINK_POWER);
                }
                blink2_ok = 1;
            }
            else if (btn_evt.timer_counter_value == 8)
            {
                /* Bsec_critical_error */;
#if CONN_ONBHMI_DEBUG_LOG
                LOG(I, "[BME68x Critical error occured]");
#endif
            }
            btn_press_wait = BTN_REL_WAIT_TIME / 2;

        }
    }
}

/**
  * @brief  Function to blink led back light as an acknowledgement of button events
  * @param  sel_blink selects icon to be blinked
*/
static void blink_led_bk_light(uint8_t sel_blink)
{
    uint8_t i_led_bk = 0 ;
    uint8_t blnk_seg = SEG_POWER_BUTTON;
    uint8_t blink_count = 1;

    if (sel_blink == BLINK_POWER)
    {
       for (i_led_bk = 0; i_led_bk < blink_count; i_led_bk++)
        {

            uc1510c_set_segment(blnk_seg, SEG_OFF);
            vTaskDelay(pdMS_TO_TICKS(LCD_BK_LIT_BLINK_DELAY));
            uc1510c_set_segment(blnk_seg, SEG_ON);
            vTaskDelay(pdMS_TO_TICKS(LCD_BK_LIT_BLINK_DELAY));
            btn_press_wait = BTN_REL_WAIT_TIME;
        }
    }
    else
    {
        for (i_led_bk = 0; i_led_bk < blink_count; i_led_bk++)
        {
            hmi_backlight_set_duty(ONBOARD_HMI_10_DUTY_CYCLE);
            btn_press_wait = BTN_REL_WAIT_TIME;
            vTaskDelay(500 / portTICK_RATE_MS);
            hmi_backlight_set_duty(ONBOARD_HMI_MAX_DUTY_CYCLE);
            vTaskDelay(500 / portTICK_RATE_MS);
            btn_press_wait = BTN_REL_WAIT_TIME;
        }
    }


    uc1510c_set_segment(blnk_seg, SEG_ON);
    hmi_backlight_set_duty(ONBOARD_HMI_MAX_DUTY_CYCLE);
}

/**
  * @brief  Function to get ddm index from database table
  * @param  DDMP Parameter.
  * @retval none.
  */
static uint8_t get_ddm_index_from_db(uint32_t ddm_param)
{
	conn_onboardhmi_param_t* param_db;
	uint8_t db_idx = DDMP_UNAVAILABLE;
	uint8_t index;

	for (index = 0u; index < conn_onbhmi_db_elements; index++)
 	{
		param_db = &conn_onboardhmi_param_db[index];

		/* Validate the DDM parameter received */
		if (param_db->ddm_parameter == ddm_param)
	  	{
			db_idx = index;
			break;
		}
	}

	return db_idx;
}

/**
  * @brief  Initialize the Device onboardHMI
  * @param  none.
  * @retval none.
  */
static void initialize_dev_onboardhmi(void)
{
	error_type result;
    uc_drv_bus_conf uc1510c_bus_conf;

    uc1510c_bus_conf.i2c.port = I2C_MASTER0_PORT;
    uc1510c_bus_conf.i2c.sda = I2C_MASTER0_SDA;
    uc1510c_bus_conf.i2c.scl = I2C_MASTER0_SCL;
    uc1510c_bus_conf.i2c.bitrate = I2C_MASTER0_FREQ;

    /* Initialize the LCD Driver IC UC1510C */
    result = init_uc1510c(&uc1510c_bus_conf);

    if (RES_PASS != result)
    {
#if CONN_ONBHMI_DEBUG_LOG
        LOG(W, "UC1510C Driver init failed = %d", result);
#endif
    }
}

/**
  * @brief  Function to publish the available classes in connector onboard HMI to the broker
  * @param  none.
  * @retval none.
  */
static void install_parameters(void)
{
	uint32_t device_class = IV0;
	/* Register Onboard Inventilate and Button Class */
	int instance = broker_register_instance(&device_class, connector_onboard_hmi.connector_id);
	ASSERT(instance != -1);
}

/**
  * @brief  Function to process the subscribe request received from the broker
            Every subscribtion will be followed by publish of current value of the subscribed parameter
  * @param  DDM parameter.
  * @retval none.
  */
static void process_subscribe_request(uint32_t ddm_param)
{
	uint16_t db_idx;
    int index;
    int32_t value = 0;
    int factor = 0;
	conn_onboardhmi_param_t* param_db;

    /* Validate the DDM parameter received */
    db_idx = get_ddm_index_from_db(ddm_param);

    if (DDMP_UNAVAILABLE != db_idx)
    {
        param_db = &conn_onboardhmi_param_db[db_idx];

        index = ddm2_parameter_list_lookup(DDM2_PARAMETER_BASE_INSTANCE(ddm_param));

        if (-1 != index)
        {
            factor = Ddm2_unit_factor_list[Ddm2_parameter_list_data[index].out_unit];

            if (factor == 0)
            {
                factor = 1;
            }

            /* Multiply with the factor */
            value = param_db->i32Value * factor;

            /* Frame and send the publish request */
            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, ddm_param, &value, sizeof(int32_t), connector_onboard_hmi.connector_id, portMAX_DELAY));
        }
        else
        {
#if CONN_ONBHMI_DEBUG_LOG
            LOG(W, "DDMP 0x%x not found in ddm2_parameter_list_lookup", ddm_param);
#endif
        }
    }
    else
    {
#if CONN_ONBHMI_DEBUG_LOG
        LOG(E, "Invalid DDMP Request ddm_param 0x%x", ddm_param);
#endif
    }
}

/**
  * @brief  Timer callback function which indicates the time elapsed to inventilate control state machine
  * @param  none.
  * @retval none.
  */
void change_onhmi_ctrl_state(ONBRDHMI_CTRL_STATE onbrd_hmi_ctrl_state)
{
    push_data_to_the_queue(onbrd_hmi_ctrl_state, ONBOARD_HMI_CTRL_STATE_CHANGE);
}

/**
  * @brief  Timer callback function to handle time elapse event
  * @param  none.
  * @retval none.
  */
static void onboard_hmi_timer_cb_func( TimerHandle_t xTimer )
{
    /*Time elapsed send data to queue*/
    push_data_to_the_queue(0, HMI_TIMER_EXPIRED);
}



#ifdef LCD_DRIVER_IC_INIT_VERSION_200623
/**
  * @brief LCD NEW init code Timer callback function to handle long press time elapse event
  * @param  none.
  * @retval none.
  * interrupt every 4ms @ 80MHz/4096 divider configuration
  */
static void inv_timer_cb(BaseType_t *data_in)
{
	hal_timer_event_t evt;

#ifdef ESP_HW_TIMER
    if (bsec_critical_error > 0)
    {
        bsec_critical_error++;
        if(bsec_critical_error > TIME_60SEC)
        {
            bsec_critical_error = 0;
            blink_event = 0;
            evt.timer_counter_value = 8;        //Trigger LCD blink for 1
            xQueueSendFromISR(timer_queue, &evt, NULL);
        }
    }
    if (button_first_press == 1)
    {
        btn_dbnc_count++;
        tch_timer_cnt++;
        pbutton_first_press = button_first_press;

        if ((tch_timer_cnt > TIME_3000MS) && (tch_timer_cnt < TIME_6000MS) && ((blink_event & 0x01) == 0))
        {
            blink_event = 1;
            evt.timer_counter_value = 4;        //Trigger LCD blink for 1
            xQueueSendFromISR(timer_queue, &evt, data_in);
        }
        else if ((tch_timer_cnt > TIME_6000MS) && ((blink_event & 0x02) == 0))
        {
            blink_event = 2;
            evt.timer_counter_value = 5;        //Trigger power button blink
            xQueueSendFromISR(timer_queue, &evt, data_in);
        }

        if ((btn_dbnc_count > TIME_500MS) && (btn_dbnc_count < TIME_508MS))
        {
            button_released = 1;

            btn_dbnc_count = 0;
            blink_event = 0;


            //Calculate_touch_timings   blink_event
            if (tch_timer_cnt < TIME_500MS)
            {
                tch_btn_event = 0;
            }
            else if ((tch_timer_cnt > TIME_500MS) && (tch_timer_cnt < TIME_3000MS) && ((tch_btn_event & 0x01) == 0))
            {
                //Short press
                button_event = 1;
                tch_btn_event = tch_btn_event | 0x01;
                evt.timer_counter_value = button_event;
                xQueueSendFromISR(timer_queue, &evt, data_in);
            }
            else if ((tch_timer_cnt > TIME_3000MS) && (tch_timer_cnt < TIME_6000MS) && ((tch_btn_event & 0x02) == 0))
            {
                //Long press 1
                button_event = 2;
                tch_btn_event = tch_btn_event | 0x02;
                evt.timer_counter_value = button_event;
                xQueueSendFromISR(timer_queue, &evt, data_in);

            }
            else if ((tch_timer_cnt > TIME_6000MS) && ((tch_btn_event & 0x04) == 0))
            {
                //Long press 2
                button_event = 3;
                tch_btn_event = tch_btn_event | 0x04;
                evt.timer_counter_value = button_event;
                xQueueSendFromISR(timer_queue, &evt, data_in);
            }

            button_first_press = 0;
            tch_timer_cnt = 0;
            tch_btn_event = 0;
        }

    }

#endif
}

#endif

#ifndef ESP_HW_TIMER

/**
  * @brief  Timer callback function to handle long press time elapse event
  * @param  none.
  * @retval none.
  */
static void long_press_tmr_cb( TimerHandle_t xTimer )
{
    uint16_t long_press_event = BTN_NO_EVENTS;
    TRUE_CHECK(xTimerStop(short_press_tmr_hdle, 0));
    prev_btn_press_evt = lcd_btn_press_evt;
    long_press_event = lcd_btn_press_evt | (BUTTON_EVT_LONG_PRESS << 8);
    push_data_to_the_queue(long_press_event, BTN_PRESSED_EVENT);
    lcd_btn_state = BTN_LONG_PRESS_ONGOING;
    TRUE_CHECK(xTimerStart(long_press_rel_tmr_hdle, 0));
}

/**
  * @brief  Timer callback function to handle long press time elapse event
  * @param  none.
  * @retval none.
  */
static void long_press_rel_tmr_cb( TimerHandle_t xTimer )
{
    /*Long press Released*/
    lcd_btn_state = BTN_LONG_PRESS_RELEASED;
}



/**
  * @brief  Timer callback function to handle short press time elapse event
  * @param  none.
  * @retval none.
  */
static void short_press_tmr_cb( TimerHandle_t xTimer )
{
    uint16_t short_press_event = BTN_NO_EVENTS;

    if (lcd_btn_state == BTN_PRESSED)
    {
        /* Stop the long press timer */
        TRUE_CHECK(xTimerStop(long_press_tmr_hdle, 0));
        /* Frame the short press event */
        short_press_event = lcd_btn_press_evt | (BUTTON_EVT_SHORT_PRESS << 8);
        /* Push the data into the queue */
        push_data_to_the_queue(short_press_event, BTN_PRESSED_EVENT);
        /* Change the state */
        lcd_btn_state = BTN_RELEASED;
    }

}

#endif

/**
  * @brief  Callback function to handle onboard hmi btn press interrupt
  * @param  device id.
  * @param  port number.
  * @param  pin number.
  * @retval none.
  */
void onboard_hmi_interrupt_cb(int device, int port, int pin)
{
    button_press_flag = 1;
    button_press_expired = BUTTON_EXPIRED_COUNT;    //set button expiry timer50
    int_touch = 1;
    btn_dbnc_delay = KEY_DEBOUNCE_COUNT ;  //1
    btn_dbnc_count = 0;
    if (button_first_press == 0)
    {
        button_first_press = 1;
    }

#ifndef ESP_HW_TIMER
    switch (lcd_btn_state)
    {
        case BTN_RELEASED:                  //process Button release events
            //fallthrough
        case BTN_LONG_PRESS_RELEASED:
            ZERO_CHECK(uc1510c_get_touch_panel_status((uint8_t*)&lcd_btn_press_evt));
            TRUE_CHECK(xTimerStart(long_press_tmr_hdle, 0));
            TRUE_CHECK(xTimerStart(short_press_tmr_hdle, 0));
            lcd_btn_state = BTN_PRESSED;
            break;

        case BTN_PRESSED:                   //Process button press events
            TRUE_CHECK(xTimerReset(short_press_tmr_hdle, 0));
            break;

        case BTN_LONG_PRESS_ONGOING:        // process Long press events
            ZERO_CHECK(uc1510c_get_touch_panel_status((uint8_t*)&lcd_btn_press_evt));
            /* Validate the button press event */
            if ( lcd_btn_press_evt != prev_btn_press_evt )
            {
                TRUE_CHECK(xTimerStop(long_press_rel_tmr_hdle, 0));
                TRUE_CHECK(xTimerStart(short_press_tmr_hdle, 0));
                TRUE_CHECK(xTimerStart(long_press_tmr_hdle, 0));
                lcd_btn_state = BTN_PRESSED;
            }
            else
            {
                /*Long Press*/
                TRUE_CHECK(xTimerReset(long_press_rel_tmr_hdle, 0));
            }
            break;

        default:
            break;
    }
#endif
}

/**
  * @brief  Timer callback function to handle blink of HMI segments
  * @param  none.
  * @retval none.
  */
static void obhmi_blink_tmr_cb(TimerHandle_t xTimer)
{
    push_data_to_the_queue(0, BLINK_TIMER_EXPIRED);
}

/**
  * @brief  Function to push the data into the queue
  * @param  data.
  * @retval none
  */
static void push_data_to_the_queue(int32_t data, OBHMI_CTRL_DATA_ID data_id)
{
    OBHMI_CTRL_DATA_FRAME obhmi_ctrl_data_frame;

    /* Update the data frame */
    obhmi_ctrl_data_frame.data = data;
    obhmi_ctrl_data_frame.data_id = data_id;

    /* Append the data in the Queue */
    osal_base_type_t ret = osal_queue_send(obhmi_pwr_ctrl_que_hdl, &obhmi_ctrl_data_frame, 0);

    if (osal_success != ret)
    {
#if CONN_ONBHMI_DEBUG_LOG
        LOG(W, "Queue error ret = %d", ret);
#endif
    }
}

/**
  * @brief  Function used to convert the DDMP param to DATA ID which understanding by onboard HMI control task
  * @param  ddm_param.
  * @retval data_id refer the enum OBHMI_CTRL_DATA_ID.
  */
OBHMI_CTRL_DATA_ID convert_ddmp_to_data_id(int32_t data, uint32_t ddm_param)
{
    OBHMI_CTRL_DATA_ID data_id = INV_STATE;

    switch (ddm_param)
    {
        case IVPMGR0STATE:
            data_id = INV_STATE;
            break;

        case IV0AQST:
            inv_iaq_status = data;
            data_id = UPDATE_SEG_AQ;
            break;

        case SBMEB0AQR:
            //get_bm68x_accuracy_level
            inv_acqrc_level = data;
            data_id = UPDATE_SEG_AQ;
            break;

        case IV0FILST:
            data_id = ( data == IV0FILST_FILTER_RESET ) ? BLINK_SEG_FILT_STAT : UPDATE_SEG_FILT_STAT;
            break;

        case IV0WARN:
            data_id = UPDATE_SEG_WARN;
            break;

        case IV0MODE:
            data_id = UPDATE_SEG_MODE;
            break;

        case IV0STORAGE:
            data_id = UPDATE_SEG_VEHMOD;
            break;

        case IV0PWRSRC:
            data_id = UPDATE_SEG_PWRSRC;
            break;

        case WIFI0STS:
            inv_wifi_status = data;
            data_id = UPDATE_SEG_WIFI;
            break;

        case BT0PAIR:                       /* BT0PAIR Response received*/
            data_id = UPDATE_SEG_BLE;
            inv_ble_status = data;
            break;

        case SDP0AVL:						/*SDP0AVL received*/
            data_id = UPDATE_SEG_DP_SENS_STAT;
            ble_dp_con_sts = data;
            break;

        case IV0BLREQ:
            data_id = BLINK_SEG_BLE_STAT;
            break;

        case IV0IONST:
            data_id = UPDATE_SEG_IONIZER;
            break;

        case IV0HMITST:
            data_id = HMI_BACK_LIGHT_TEST;
            break;

        case IV0ERRST:
            data_id = INV_ERROR_STATUS;
            set_err_ackstate(data_id);
            break;
        default:
            data_id = INVALID_DATA_ID;
            break;
    }

    return data_id;
}

/**
  * @brief  Callback function to handle all the subscribed parameters.
  * @param  ddm_param - DDM Parameter with instance
  * @param  data      - Value for the DDM Parameter
  * @retval none
  */
static void handle_hmi_ctrl_sub_data(uint32_t ddm_param, int32_t data)
{
    OBHMI_CTRL_DATA_ID data_id;

    if (ddm_param == SDP0DP)
    {
        LOG(I, "[BLE DP data received_OBHMI data = %d]", data);
        // TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SUBSCRIBE, SDP0DP, NULL, 0, connector_onboard_hmi.connector_id, portMAX_DELAY));
        /* update BLE icon in LCD */
        inv_ble_status = 1;
        push_data_to_the_queue(inv_ble_status, UPDATE_SEG_DP_SENS_STAT);
    }

    /* Update button control variables based on the received DDMP request */
    if (((IVPMGR0STATE_STORAGE == ptr_hmi_ctrl_sm->inv_state) && (ddm_param == IV0PWRON))
        || ((IVPMGR0STATE_STORAGE == ptr_hmi_ctrl_sm->inv_state) && (ddm_param == IV0MODE)))
    {
        LOG(I, "[ storage mode skip %d event]", ddm_param);
    }
    else if (((ptr_hmi_ctrl_sm->invent_error_status & (1 << BACKUP_BATTERY_LOW)) > 0) &&
              ((ddm_param == IV0PWRON) || (ddm_param == IV0MODE) || (ddm_param == DIM0LVL)))
    {
        /*skip for battery low*/
         LOG(I, "[ Battery low skip %d event]", ddm_param);
    }
    else
    {
        update_hmi_btn_ctrl_variables(ddm_param, data);

        data_id = convert_ddmp_to_data_id(data, ddm_param);

#if CONN_ONBHMI_DEBUG_LOG
        LOG(I, "ddm_param = 0x%x, data = %d", ddm_param, data);
#endif
        if (INVALID_DATA_ID != data_id)
        {
            push_data_to_the_queue(data, data_id);
        }
    }
}

/**
  * @brief  Function to start the timer
  * @param  none.
  * @retval none.
  */
void start_hmi_wait_timer(uint32_t period_ms)
{
    osal_ubase_type_t ret;

    if (configured_time_ms != period_ms)
    {
        /* Store the new period */
        configured_time_ms = period_ms;
        /* Change timer period */
        ret = xTimerChangePeriod(obHmiOneShotTimer, pdMS_TO_TICKS(period_ms), 0);

        if (ret != pdPASS)
        {
#if CONN_ONBHMI_DEBUG_LOG
            LOG(I, "Timer period change failed = %d", ret);
#endif
        }
    }
    else
    {
        /* Start timer */
        ret = xTimerStart(obHmiOneShotTimer, 0);

        if (ret != pdPASS)
        {
#if CONN_ONBHMI_DEBUG_LOG
            LOG(I, "Timer start failed = %d", ret);
#endif
        }
    }
}

/**
  * @brief  Function to start the blink timer
  * @param  none.
  * @retval none.
  */
static void start_obhmi_blink_timer(void)
{
    osal_ubase_type_t ret = xTimerStart(blink_ctrl_timer_hdle, 0);

    if (ret != pdPASS)
    {
#if CONN_ONBHMI_DEBUG_LOG
        LOG(I, "Timer start failed = %d", ret);
#endif
    }
}

/**
  * @brief  Function to stop the blink timer
  * @param  none.
  * @retval none.
  */
static void stop_obhmi_blink_timer(void)
{
    osal_ubase_type_t  xTimerStoped = xTimerStop(blink_ctrl_timer_hdle, 0);

    if (xTimerStoped != pdPASS)
    {
#if CONN_ONBHMI_DEBUG_LOG
        LOG(I, "Timer stop Failed");
#endif
    }
}

/**
  * @brief  Function to stop the timer
  * @param  none.
  * @retval none.
  */
void stop_hmi_wait_timer(void)
{
    osal_ubase_type_t  xTimerStoped = xTimerStop(obHmiOneShotTimer, 0);

    if (xTimerStoped != pdPASS)
    {
#if CONN_ONBHMI_DEBUG_LOG
        LOG(E, "Timer stop Failed");
#endif
    }
}

/**
  * @brief  Function to stop the timer
  * @param  none.
  * @retval none.
  */
void reset_hmi_wait_timer(void)
{
    osal_ubase_type_t  xTimerReset = xTimerReset(obHmiOneShotTimer, 0);

    if (xTimerReset != pdPASS)
    {
#if CONN_ONBHMI_DEBUG_LOG
        LOG(E, "Timer reset Failed");
#endif
    }
}

/**
  * @brief  Update the new value in database table and publish to broker
  * @param  ddm_parameter.
  * @param  i32Value.
  * @retval none.
  */
void update_and_send_value_to_broker(uint32_t ddm_parameter, int32_t value)
{
    DDMP2_CONTROL_ENUM control;
	conn_onboardhmi_param_t* param_db;
    uint8_t db_idx = get_ddm_index_from_db(ddm_parameter);

	if (DDMP_UNAVAILABLE != db_idx)
   	{
		param_db = &conn_onboardhmi_param_db[db_idx];

		/* Update the value in db table */
		param_db->i32Value = value;

        if (DIM0LVL == ddm_parameter)
        {
            control = DDMP2_CONTROL_SET;
        }
        else
        {
            control = DDMP2_CONTROL_PUBLISH;
        }

        if ((IV0BLREQ == ddm_parameter) || (IV0FILST == ddm_parameter)  || (IV0MODE == ddm_parameter))
        {
            OBHMI_CTRL_DATA_ID data_id = convert_ddmp_to_data_id(value, ddm_parameter);

            push_data_to_the_queue(value, data_id);
        }

		/* Send data to the broker */
        connector_send_frame_to_broker(control, ddm_parameter, &value, sizeof(value), connector_onboard_hmi.connector_id, (TickType_t)portMAX_DELAY);
   	}
}

/**
  * @brief  Function to update the HMI button variables
  * @param  ddmp ddmp_parameter.
  * @param  data data hold by the ddmp_parameter.
  * @retval none.
  */
static void update_hmi_btn_ctrl_variables(uint32_t ddmp, int32_t data)
{
    BTN_PRESSED_EVT hmi_btn_evt = BTN_NO_EVENTS;
    BTN_PRESS_EVENT_TYPE event_type;
    ONBOARD_HMI_BUTTON_EVENT* btn_event_list;
    const hmi_domain_to_ddm_system_t* hmi_domain_to_ddm_system;
    uint8_t index = 0;
    uint8_t idx2 = 0;

    switch (ddmp)
    {
        case IV0PWRON:
            if (IVPMGR0STATE_STORAGE == ptr_hmi_ctrl_sm->inv_state)
            {
                hmi_btn_evt = BTN_NO_EVENTS;
                lcd_btn_press_evt = BTN_NO_EVENTS;
                LOG(I,"Skip DDMP Event in storage mode");
            }
            else
            {
                hmi_btn_evt = BTN_POWER;
                event_type = BUTTON_EVT_SHORT_PRESS;
                inv_pwr_state = data;                    // Access using semaphore
                lcd_btn_press_evt = BTN_POWER;               // Access using semaphore
            }
            break;

        case IV0BLREQ:
            /*Process BLE request*/
            hmi_btn_evt = BTN_LIGHT;
            event_type = BUTTON_EVT_LONG_PRESS;
            break;

        case DIM0LVL:
            /*Handle LED brightness control events*/
            hmi_btn_evt = BTN_LIGHT;
            event_type = BUTTON_EVT_SHORT_PRESS;
            break;

        case IV0MODE:
            /*handle Mode button short press events */
            hmi_btn_evt = BTN_MODE;
            event_type = BUTTON_EVT_SHORT_PRESS;
            break;

        case SDP0AVL:
            /*Handle Dp sensor data*/
            if (DP_SENSOR_NOT_AVAILABLE == data)
            {
                LOG(W, "Subscribe request for DDMP SDP0AVL");
                // Whenever the availability of sensor node received then the subscribtion should be done again
                TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SUBSCRIBE, SDP0AVL, NULL, 0,
                           connector_onboard_hmi.connector_id, portMAX_DELAY));
            }

            if (DP_SENSOR_AVAILABLE == data)
            {
                TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SUBSCRIBE, SDP0AVL, NULL, 0,
                           connector_onboard_hmi.connector_id, portMAX_DELAY));

            }
            break;

        default:
            break;
    }

    if (BTN_NO_EVENTS != hmi_btn_evt)
    {
        for (index = 0; index < ONBOARD_HMI_EVT_TABLE_SIZE; index++)
        {
            btn_event_list = &onboard_hmi_btn_evt[index];

            if (btn_event_list->hmi_btn_evt == hmi_btn_evt)
            {
                hmi_domain_to_ddm_system = btn_event_list->data_conversion.hmi_domain_to_ddm_system;

                for (idx2 = 0; idx2 < btn_event_list->data_conversion.size; idx2++)
                {
                    if ((hmi_domain_to_ddm_system[idx2].ddm_parameter == ddmp) &&
                         (hmi_domain_to_ddm_system[idx2].ddm_system_value == data))
                    {
                        if (BUTTON_EVT_SHORT_PRESS == event_type)
                        {
                            btn_event_list->short_press_evt_cnt = hmi_domain_to_ddm_system[idx2].hmi_domain_value;
                        }
                        else
                        {
                            btn_event_list->long_press_evt_cnt = hmi_domain_to_ddm_system[idx2].hmi_domain_value;
                        }

                        index = ONBOARD_HMI_EVT_TABLE_SIZE;
                        break;
                    }
                }
            }
        }
    }
}

#endif
