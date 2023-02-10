/*! \file connector_onboardhmi.c
	\brief Connector for Onboard HMI
	\Author Sundaramoorthy-LTTS
 */

/** Includes ******************************************************************/
#include "configuration.h"

#ifdef CONNECTOR_ONBOARD_HMI
#include "iGeneralDefinitions.h"

#include "connector.h"
#include "connector_onboardhmi.h"

#include "app_api.h"

#ifdef DEVICE_UC1510C
#include "drv_uc1510c.h"
#endif

#include "sorted_list.h"
#include "linear_interpolation.h"
#include "hal_ledc.h"
#include "osal.h"

/** Defines ******************************************************************/
#define CONN_OBHMI_SUB_DEPTH		        ((uint8_t) 20u)
#define DDMP_UNAVAILABLE                    ((uint8_t) 0xFFu)
#define ONBOARD_HMI_EVENT_QUEUE_LEN         15
#define ONBOARD_HMI_EVENT_QUEUE_SIZE        sizeof(int)
#define ONBOARD_HMI_BTN_TASK_DELAY          ((uint32_t) 500u)
#define ONBOARD_HMI_PWR_CTRL_TASK_DELAY     ((uint32_t) 500u)
#define OBHMI_PWR_CTRL_TASK_QUE_LEN	        ((osal_base_type_t) 30)
#define OBHMI_PWR_CTRL_TASK_QUE_ITEM_SIZE   ((osal_base_type_t) sizeof(OBHMI_CTRL_DATA_FRAME))
#define CONN_ONBHMI_DEBUG_LOG               1
#define NUM_AVL_ELEMENTS                    ((uint8_t) 2u)

/* Blink functionality macro definitions */
#define FAST_BLINK_TIME_INTERVAL             500u
#define SLOW_BLINK_TIME_INTERVAL            1000u
#define OBHMI_BLINK_CTRL_TASK_DELAY_MSEC     500u
#define BLINK_RUN_TIME_INFINITE              ((uint16_t)  0xFFFF)

#ifndef INVENT_EOL_TESTING
#define FILTER_RESET_STATUS_RUN_TIME_MSEC    ((uint16_t)  30000u)
#define BT_PAIR_STATUS_RUN_TIME_MSEC         ((uint16_t)  60000u)
#define STORAGE_MODE_STATUS_RUN_TIME_MSEC    ((uint16_t)  10000u)
#else
#define FILTER_RESET_STATUS_RUN_TIME_MSEC    ((uint16_t)  10000u)
#define BT_PAIR_STATUS_RUN_TIME_MSEC         ((uint16_t)  10000u)
#define STORAGE_MODE_STATUS_RUN_TIME_MSEC    ((uint16_t)  10000u)
#endif

/** Static Function declarations ********************************************/
static int initialize_connector_onboardhmi(void);
static void install_parameters(void);
static void initialize_dev_onboardhmi();
static void conn_onboardhmi_process_task(void *pvParameter);
static void conn_onboardhmi_ctrl_task(void *pvParameter);
static int add_subscription(DDMP2_FRAME *pframe);
static void process_subscribe_request(uint32_t ddm_param);
static uint8_t get_ddm_index_from_db(uint32_t ddm_param);
static void process_set_and_publish_request(uint32_t ddm_param, int32_t i32value, DDMP2_CONTROL_ENUM req_type);
static void handle_hmi_ctrl_sub_data(uint32_t ddm_param, int32_t data);
static void onboard_hmi_timer_cb_func( TimerHandle_t xTimer );
static void start_publish(void);
static void start_subscribe(void);
void change_onhmi_ctrl_state(ONBRDHMI_CTRL_STATE onbrd_hmi_ctrl_state);
static void push_data_to_the_queue(int32_t data, OBHMI_CTRL_DATA_ID data_id);
static void long_press_tmr_cb( TimerHandle_t xTimer );
static void short_press_tmr_cb( TimerHandle_t xTimer );
static void long_press_rel_tmr_cb( TimerHandle_t xTimer );
void update_and_send_value_to_broker(uint32_t ddm_parameter, int32_t value);
static void update_hmi_btn_ctrl_variables(uint32_t ddmp, int32_t data);
static void obhmi_blink_tmr_cb( TimerHandle_t xTimer );
static void start_obhmi_blink_timer(void);
static void stop_obhmi_blink_timer(void);

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
static INV_PWR_STATE inv_pwr_state = PWR_STATE_OFF;

static ONBRD_HMI_CTRL_SM* ptr_hmi_ctrl_sm = &onbrd_hmi_ctrl_sm;

/* Structure for Connector OnboardHMI */
CONNECTOR connector_onboard_hmi =
{
	.name = "OnboardHMI Connector",
	.initialize = initialize_connector_onboardhmi
};

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
    {IV0IONST    ,       DDM2_TYPE_INT32_T,    1,        0,                      IV0IONST_OFF,   handle_hmi_ctrl_sub_data},
    {IV0ERRST    ,       DDM2_TYPE_INT32_T,    1,        0,                 IV0ERRST_NO_ERROR,                       NULL},
    {IV0BLREQ    ,       DDM2_TYPE_INT32_T,    1,        0,                     IV0BLREQ_IDLE,   handle_hmi_ctrl_sub_data},
    {IV0HMITST   ,       DDM2_TYPE_INT32_T,    1,        0,                                 0,   handle_hmi_ctrl_sub_data},
    {IVPMGR0STATE,       DDM2_TYPE_INT32_T,    0,        1,              IVPMGR0STATE_STANDBY,   handle_hmi_ctrl_sub_data},   
    {DIM0LVL     ,       DDM2_TYPE_INT32_T,    0,        1,              DIM_LVL_DUTY_CYCLE_0, 	 handle_hmi_ctrl_sub_data},
    {BT0PAIR     ,       DDM2_TYPE_INT32_T,    0,        1,           BT0PAIR_OUT_NOT_PAIRING, 	 handle_hmi_ctrl_sub_data},
    {SDP0AVL     ,       DDM2_TYPE_INT32_T,    0,        1,           DP_SENSOR_NOT_AVAILABLE,   handle_hmi_ctrl_sub_data},
    {IV0PRST     ,       DDM2_TYPE_INT32_T,    1,        0,      IV0PRST_PRESS_STATUS_UNKNOWN,                       NULL}
};

static const uint32_t conn_onbhmi_db_elements = ELEMENTS(conn_onboardhmi_param_db);

DECLARE_SORTED_LIST_EXTRAM(conn_obhmi_table, CONN_OBHMI_SUB_DEPTH);       //!< \~ Subscription table storage

static BLINK_SEG_INFO blink_seg_info[ONBOARD_HMI_MAX_SEGEMENT] = 
{
//  blink_cur_action       blink_end_action      blink_run_time_ms     timer_counter   run_time_counter     segment_flag
    {BLINK_DEACTIVATE,   SEGMENT_STATE_OFF,    BLINK_RUN_TIME_INFINITE,      0,                0,               0       }, // SEG_AIR_QUALITY_LEVEL_1_LOW
    {BLINK_DEACTIVATE,   SEGMENT_STATE_OFF,    BLINK_RUN_TIME_INFINITE,      0,                0,               0       }, // SEG_AIR_QUALITY_LEVEL_2_MID
    {BLINK_DEACTIVATE,   SEGMENT_STATE_OFF,    BLINK_RUN_TIME_INFINITE,      0,                0,               0       }, // SEG_AIR_QUALITY_LEVEL_3_HIGH
    {BLINK_DEACTIVATE,   SEGMENT_STATE_OFF,    BLINK_RUN_TIME_INFINITE,      0,                0,               0       }, // SEG_IONIZER_STATUS
    {BLINK_DEACTIVATE,   SEGMENT_STATE_OFF,    BLINK_RUN_TIME_INFINITE,      0,                0,               0       }, // SEG_FILTER_STATUS
    {BLINK_DEACTIVATE,   SEGMENT_STATE_OFF,    BLINK_RUN_TIME_INFINITE,      0,                0,               0       }, // SEG_WARNING_STATUS
    {BLINK_DEACTIVATE,   SEGMENT_STATE_OFF,    BLINK_RUN_TIME_INFINITE,      0,                0,               0       }, // SEG_MODE_AUTO
    {BLINK_DEACTIVATE,   SEGMENT_STATE_OFF,    BLINK_RUN_TIME_INFINITE,      0,                0,               0       }, // SEG_MODE_TURBO
    {BLINK_DEACTIVATE,   SEGMENT_STATE_OFF,    BLINK_RUN_TIME_INFINITE,      0,                0,               0       }, // SEG_MODE_SLEEP
    {BLINK_DEACTIVATE,   SEGMENT_STATE_OFF,    BLINK_RUN_TIME_INFINITE,      0,                0,               0       }, // SEG_MODE_MENU_LINE
    {BLINK_DEACTIVATE,   SEGMENT_STATE_OFF,    BLINK_RUN_TIME_INFINITE,      0,                0,               0       }, // SEG_STORAGE_MODE
    {BLINK_DEACTIVATE,   SEGMENT_STATE_OFF,    BLINK_RUN_TIME_INFINITE,      0,                0,               0       }, // SEG_SOLAR_BATTERY_STATUS 
    {BLINK_DEACTIVATE,   SEGMENT_STATE_OFF,    BLINK_RUN_TIME_INFINITE,      0,                0,               0       }, // SEG_BLE_STATUS
    {BLINK_DEACTIVATE,   SEGMENT_STATE_OFF,    BLINK_RUN_TIME_INFINITE,      0,                0,               0       }, // SEG_WIFI_STATUS
    {BLINK_DEACTIVATE,   SEGMENT_STATE_OFF,    BLINK_RUN_TIME_INFINITE,      0,                0,               0       }, // SEG_LIGHT_BUTTON
    {BLINK_DEACTIVATE,   SEGMENT_STATE_OFF,    BLINK_RUN_TIME_INFINITE,      0,                0,               0       }, // SEG_MODE_BUTTON
    {BLINK_DEACTIVATE,   SEGMENT_STATE_OFF,    BLINK_RUN_TIME_INFINITE,      0,                0,               0       }, // SEG_POWER_BUTTON
};

/**
  * @brief  Initialize the connector for Onboard HMI
  * @param  none.
  * @retval none.
  */
static int initialize_connector_onboardhmi(void)
{
    LOG(I, "Start Init");
    /* Initialize the onboard hmi state machine structure variables */
    onbrd_hmi_ctrl_sm.onbrd_hmi_ctrl_state = ONBOARD_HMI_STATE_IDLE;
    onbrd_hmi_ctrl_sm.seg_state            = ENABLE_ALL_SEGEMENT;

    /* Create queue for invent control task */
    obhmi_pwr_ctrl_que_hdl = osal_queue_create(OBHMI_PWR_CTRL_TASK_QUE_LEN, OBHMI_PWR_CTRL_TASK_QUE_ITEM_SIZE);

    if ( NULL == obhmi_pwr_ctrl_que_hdl )
    {
        LOG(E, "Queue creation failed for inventilate onboard hmi conrol task");
    }

    /* Create the one shot timer software timer for onboard HMI events handling */
    obHmiOneShotTimer = xTimerCreate("obHmiOneShotTimer", OBHMI_WAIT_TIME_TICKS, pdFALSE, 0, onboard_hmi_timer_cb_func);

    /* Create the one shot timer software timer for onboard HMI events handling */
    long_press_tmr_hdle = xTimerCreate("long_pr_tmr", LONG_PRESS_TIMER_TICKS, pdFALSE, 0, long_press_tmr_cb);

    /* Create the one shot timer software timer for onboard HMI events handling */
    long_press_rel_tmr_hdle = xTimerCreate("long_pr_rel_tmr", LONG_PRESS_REL_TIMER_TICKS, pdFALSE, 0, long_press_rel_tmr_cb);

    /* Create the one shot timer software timer for onboard HMI events handling */
    short_press_tmr_hdle = xTimerCreate("short_pr_tmr", SHORT_PRESS_TIMER_TICKS, pdFALSE, 0, short_press_tmr_cb);

    /* Create the periodic software timer for blinking HMI segments */
    blink_ctrl_timer_hdle = xTimerCreate("blink_tmr", BLINK_TIMER_TICKS, pdTRUE, 0, obhmi_blink_tmr_cb);

    /* Initialize the device Onboard HMI */
    initialize_dev_onboardhmi();

    /* Create Task for connector OnboardHMI to handle/process all the ddmp related activities */
    TRUE_CHECK(osal_task_create(conn_onboardhmi_process_task, CONNECTOR_ONBOARDHMI_PROCESS_TASK_NAME, CONNECTOR_ONBOARD_HMI_PROCESS_STACK_DEPTH, NULL, CONNECTOR_ONBOARD_HMI_PROCESS_PRIORITY, NULL));
	TRUE_CHECK(osal_task_create(conn_onboardhmi_ctrl_task,    CONNECTOR_ONBOARDHMI_PWR_CTRL_TASK_NAME, CONNECTOR_ONBOARD_HMI_CTRL_TASK_STACK_DEPTH, obhmi_pwr_ctrl_que_hdl, CONNECTOR_ONBOARD_HMI_CTRL_TASK_PRIORITY, NULL));
    
    /* Publish the class availbility to the broker */
    install_parameters();

    /* Subsribe the all the needed DDMP parameters to the broker */
    start_subscribe();
    start_publish();

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
    
    for ( db_idx = 0; db_idx < num_elements; db_idx++ )
    {
        ptr_param_db = &conn_onboardhmi_param_db[db_idx];

        /* Check the DDM parameter need subscribtion */
        if ( ptr_param_db->sub )
        {
            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SUBSCRIBE, ptr_param_db->ddm_parameter, NULL, 0, connector_onboard_hmi.connector_id, (TickType_t)portMAX_DELAY));
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
    
    for ( db_idx = 0; db_idx < num_elements; db_idx++ )
    {
        ptr_param_db = &conn_onboardhmi_param_db[db_idx];

        /* Check the DDM parameter need to publish */
        if ( ptr_param_db->pub ) 
        {
            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, ptr_param_db->ddm_parameter, &ptr_param_db->i32Value, sizeof(int32_t), connector_onboard_hmi.connector_id, portMAX_DELAY));
        }
    }
}

/**
  * @brief  Add device to inventory if it does not already exists
  * @param  DDMP Frame.
  * @retval result 0 - Succesfully added to list / 1 - Fail.
  */
static int add_subscription(DDMP2_FRAME *pframe)
{
    SORTED_LIST_KEY_TYPE     key = pframe->frame.subscribe.parameter;
    SORTED_LIST_VALUE_TYPE value = 1;

    return sorted_list_single_add(&conn_obhmi_table, key, value);
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

#if CONN_ONBHMI_DEBUG_LOG
    LOG(I, "Received ddm_param = 0x%x i32value = %d", ddm_param, i32value);
#endif

	/* Validate the DDM parameter received */
	db_idx = get_ddm_index_from_db(ddm_param);
 
	if ( DDMP_UNAVAILABLE != db_idx )
	{
        if ( req_type == DDMP2_CONTROL_SET )
        {
            /* Frame and send the publish request */
            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, ddm_param, &pub_value, sizeof(int32_t), \
                       connector_onboard_hmi.connector_id, portMAX_DELAY));
        }
        
		param_db = &conn_onboardhmi_param_db[db_idx];

#if CONN_ONBHMI_DEBUG_LOG		
		LOG(I, "Valid DDMP parameter");
#endif
        i32Index = ddm2_parameter_list_lookup(DDM2_PARAMETER_BASE_INSTANCE(ddm_param));

        if ( -1 != i32Index )
        {

#if CONN_ONBHMI_DEBUG_LOG
            LOG(I, "i32Index = %d", i32Index);
#endif
            if (  i32Index < DDM2_PARAMETER_COUNT )
                i32Factor = Ddm2_unit_factor_list[Ddm2_parameter_list_data[i32Index].in_unit];
            else
                i32Factor = 1;

            if ( i32Factor > 0 )
            {
                i32value = i32value / i32Factor;
            }
                
#if CONN_ONBHMI_DEBUG_LOG
            LOG(I, "After factored i32value = %d", i32value);
#endif
            if ( i32value != param_db->i32Value )
            {
                /* Update the received value in the database table*/
	  	        param_db->i32Value = i32value;
		        /* Check callback function registered for this DDM parameter */
		        if  ( NULL != param_db->cb_func )
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

	while (1)
	{
        TRUE_CHECK ( pframe = xRingbufferReceive(connector_onboard_hmi.to_connector, &frame_size, portMAX_DELAY) );
        
        switch (pframe->frame.control)
        {
            case DDMP2_CONTROL_PUBLISH:
#if CONN_ONBHMI_DEBUG_LOG
                LOG(I, "DDMP2_CONTROL_PUBLISH");
#endif
                process_set_and_publish_request(pframe->frame.publish.parameter, pframe->frame.publish.value.int32, pframe->frame.control);
                break;
                
            case DDMP2_CONTROL_SET:
#if CONN_ONBHMI_DEBUG_LOG
                LOG(I, "DDMP2_CONTROL_SET");
#endif
                process_set_and_publish_request(pframe->frame.set.parameter, pframe->frame.set.value.int32, pframe->frame.control);
		        break;
                
            case DDMP2_CONTROL_SUBSCRIBE:
#if CONN_ONBHMI_DEBUG_LOG
                LOG(I, "DDMP2_CONTROL_SUBSCRIBE");
#endif
                add_subscription(pframe);
                process_subscribe_request(pframe->frame.subscribe.parameter);
			    break;
                
            default:
				LOG(E, "UNHANDLED frame %02x from broker!",pframe->frame.control);
			    break;
        }

        vRingbufferReturnItem(connector_onboard_hmi.to_connector, pframe);
    }
}

/**
  * @brief  Function to update the blink segment info
  * @param  segment    -  Segment Number 
  * @param  blink_stat -  Blink Action ACTIVATE/DEACTIVATE
  * @param  blink_run_time_ms.
  * @retval none.
  */
static void update_blink_seg_info(ONBOARDHMI_SEGMENT segment, BLINK_ACTION blink_action, uint16_t blink_run_time_ms, SEGMENT_STATE blink_end_state)
{
    blink_seg_info[segment].blink_action      = blink_action;
    blink_seg_info[segment].blink_run_time_ms = blink_run_time_ms;
    blink_seg_info[segment].blink_end_state   = blink_end_state;
}

/**
  * @brief  Function to handle the blinking of segments
  * @param  ptr_seg_stat Pointer to the blink enabled segemnt
  * @retval none.
  */
static void handle_hmi_blink_segment(uint32_t* ptr_seg_stat)
{
    for ( uint8_t seg = 0; seg < ONBOARD_HMI_MAX_SEGEMENT; seg++ )
    {
        if ( BLINK_ACTIVATE == blink_seg_info[seg].blink_action )
        {
            if ( blink_seg_info[seg].timer_counter >= FAST_BLINK_TIME_INTERVAL )
            {
                blink_seg_info[seg].timer_counter = 0;
                blink_seg_info[seg].segment_flag = !blink_seg_info[seg].segment_flag;
                uc1510c_set_segment(seg, blink_seg_info[seg].segment_flag);
            }

            blink_seg_info[seg].timer_counter    += OBHMI_BLINK_CTRL_TASK_DELAY_MSEC;
            blink_seg_info[seg].run_time_counter += OBHMI_BLINK_CTRL_TASK_DELAY_MSEC;

            if ( blink_seg_info[seg].run_time_counter != BLINK_RUN_TIME_INFINITE )
            {
                if ( blink_seg_info[seg].run_time_counter >= blink_seg_info[seg].blink_run_time_ms )
                {
                    LOG(W, "blink_end_state = %d", blink_seg_info[seg].blink_end_state)
                    blink_seg_info[seg].run_time_counter  = 0;
                    update_blink_seg_info(seg, BLINK_DEACTIVATE, BLINK_RUN_TIME_INFINITE, blink_seg_info[seg].blink_end_state);
                    uc1510c_set_segment(seg, blink_seg_info[seg].blink_end_state);
                }
            }

            *ptr_seg_stat |=  1 << seg;
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
    uint8_t update_seg       = 0;
    uint8_t check_timer_stat = 0;
    uint32_t calc_resol      = 0;
    uint32_t blink_seg_stat  = 0;

    while (1)
    {
        /* Queue will be in blocked state untill data recevied */
        if ( osal_success == osal_queue_receive((osal_queue_handle_t)pvParameter, (void *)&ptr_hmi_ctrl_sm->data_frame, portMAX_DELAY) )
        {
            LOG(I, "Data received from queue data_id = %d data = %d", \
                ptr_hmi_ctrl_sm->data_frame.data_id, ptr_hmi_ctrl_sm->data_frame.data);

            /* Parse the data received from queue */
            switch ( ptr_hmi_ctrl_sm->data_frame.data_id )
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

                    if ( ONBOARD_HMI_STATE_STARTUP != ptr_hmi_ctrl_sm->onbrd_hmi_ctrl_state )
                    {
                        /* Handle button events */
                        handle_onboard_hmi_button_event((uint16_t)ptr_hmi_ctrl_sm->data_frame.data, ptr_hmi_ctrl_sm->inv_state);
                    }
                    break;

                case HMI_BACK_LIGHT_TEST:
                    if ( ptr_hmi_ctrl_sm->data_frame.data < HMI_BACKLIGHT_MIN_DUTY_CYCLE )
                    {
                        ptr_hmi_ctrl_sm->data_frame.data = HMI_BACKLIGHT_MIN_DUTY_CYCLE;
                    }
                    else if ( ptr_hmi_ctrl_sm->data_frame.data > HMI_BACKLIGHT_MAX_DUTY_CYCLE )
                    {
                        ptr_hmi_ctrl_sm->data_frame.data = HMI_BACKLIGHT_MAX_DUTY_CYCLE;
                    }
                    else
                    {

                    }

                    calc_resol = calc_linear_interpolation(HMI_BACKLIGHT_MIN_DUTY_CYCLE, HMI_BACKLIGHT_MAX_DUTY_CYCLE, LEDC_PWM_MIN_DUTY_CYCLE, LEDC_PWM_MAX_DUTY_CYCLE, ptr_hmi_ctrl_sm->data_frame.data);

                    LOG(I, "calc_resol = %d", calc_resol);
                    /* Set the backlight in the onboard HMI */
                    hmi_backlight_set_duty(calc_resol);
                    break;

                case HMI_TIMER_EXPIRED:
                    if ( ONBOARD_HMI_STATE_STARTUP != ptr_hmi_ctrl_sm->onbrd_hmi_ctrl_state )
                    {
                        LOG(W, "HMI_TIMER_EXPIRED");
                        hmi_backlight_set_duty(ONBOARD_HMI_MIN_DUTY_CYCLE);
                    }
                    break;

                default:
                    break; 
            }

            if ( ptr_hmi_ctrl_sm->onbrd_hmi_ctrl_state != ONBOARD_HMI_STATE_ACTIVE )
            {
                /* Store the status of HMI segments without updating in HMI */
                obhmi_update_var(ptr_hmi_ctrl_sm->data_frame.data_id, ptr_hmi_ctrl_sm->data_frame.data);
            }

            /* Process the received data */
            switch ( ptr_hmi_ctrl_sm->onbrd_hmi_ctrl_state )
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
                    LOG(I, "ONBOARD_HMI_STATE_OFF inv_state = %d seg_state = %d", \
                            ptr_hmi_ctrl_sm->inv_state, ptr_hmi_ctrl_sm->seg_state);

                    if ( IVPMGR0STATE_ACTIVE == ptr_hmi_ctrl_sm->inv_state )
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
                    switch ( ptr_hmi_ctrl_sm->seg_state )
                    {
                        case ENABLE_ALL_SEGEMENT:
                            /* Update segments */
                            onboard_hmi_update_segments(ptr_hmi_ctrl_sm->seg_state);
                            /* Start timer */
                            start_hmi_wait_timer(OBHMI_WAIT_TIME_MSEC);
                            /* set the segment state */
                            ptr_hmi_ctrl_sm->seg_state = DISABLE_ALL_SEGEMENT;
                            break;

                        case DISABLE_ALL_SEGEMENT:
                            if ( HMI_TIMER_EXPIRED == ptr_hmi_ctrl_sm->data_frame.data_id )
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
                            if ( HMI_TIMER_EXPIRED == ptr_hmi_ctrl_sm->data_frame.data_id )
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
                    LOG(I, "ONBOARD_HMI_STATE_ACTIVE inv_state = %d", ptr_hmi_ctrl_sm->inv_state);
                    /* Process received data frame from queue */
                    switch ( ptr_hmi_ctrl_sm->data_frame.data_id )
                    {
                        case BLINK_SEG_BLE_STAT:
                            update_blink_seg_info(SEG_BLE_STATUS, BLINK_ACTIVATE, BT_PAIR_STATUS_RUN_TIME_MSEC, SEGMENT_STATE_OFF);
                            check_timer_stat = 1;
                            break;

                        case BLINK_SEG_FILT_STAT:
                            update_blink_seg_info(SEG_FILTER_STATUS, BLINK_ACTIVATE, FILTER_RESET_STATUS_RUN_TIME_MSEC, SEGMENT_STATE_OFF);
                            check_timer_stat = 1;
                            break;

                        case UPDATE_SEG_BLE:
                        case UPDATE_SEG_DP_SENS_STAT:
                            LOG(W, "DP Sensor connectivity changed..Update BLE Status");
                            update_blink_seg_info(SEG_BLE_STATUS, BLINK_DEACTIVATE, BLINK_RUN_TIME_INFINITE, SEGMENT_STATE_OFF);
                            update_seg = 1;
                            break;
                            
                        case UPDATE_SEG_AQ:
                        case UPDATE_SEG_FILT_STAT:
                        case UPDATE_SEG_WARN:
                        case UPDATE_SEG_PWRSRC:
                        case UPDATE_SEG_WIFI:
                        case UPDATE_SEG_IONIZER:
                        case UPDATE_SEG_MODE:
                            LOG(W, "Update Seg = %d", ptr_hmi_ctrl_sm->data_frame.data_id);
                            update_seg = 1;
                            break;

                        case BLINK_TIMER_EXPIRED:
                            {
                                blink_seg_stat = 0;

                                /* Handle the segement blink based on periodic timer expire event */
                                handle_hmi_blink_segment(&blink_seg_stat);

                                if ( blink_seg_stat == 0 )
                                {
                                    LOG(W, "blink_seg_stat = %d", blink_seg_stat);
                                    ptr_hmi_ctrl_sm->blink_tmr_status = false;
                                    stop_obhmi_blink_timer();
                                }
                            }
                            break;

                        case INV_STATE:
                            switch ( ptr_hmi_ctrl_sm->data_frame.data )
                            {
                                case IVPMGR0STATE_STORAGE:
                                    obhmi_set_segment(UPDATE_SEG_VEHMOD, IV0STORAGE_ACTIVATE);
                                    obhmi_set_segment(UPDATE_SEG_MODE, IV0MODE_OFF);
                                    //update_blink_seg_info(SEG_STORAGE_MODE, BLINK_ACTIVATE, STORAGE_MODE_STATUS_RUN_TIME_MSEC, SEGMENT_STATE_ON);
                                    check_timer_stat = 1;
                                    break;

                                case IVPMGR0STATE_ACTIVE:
                                    obhmi_set_segment(UPDATE_SEG_VEHMOD, IV0STORAGE_DEACTIVATE);
                                    obhmi_set_segment(UPDATE_SEG_MODE, IV0MODE_AUTO);
                                    update_and_send_value_to_broker(IV0MODE, IV0MODE_AUTO);
                                    //update_blink_seg_info(SEG_STORAGE_MODE, BLINK_ACTIVATE, STORAGE_MODE_STATUS_RUN_TIME_MSEC, SEGMENT_STATE_OFF);
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

            if ( check_timer_stat )
            {
                if ( ptr_hmi_ctrl_sm->blink_tmr_status == false )
                {
                    start_obhmi_blink_timer();
                    ptr_hmi_ctrl_sm->blink_tmr_status = true;
                }

                check_timer_stat = 0;
            }

            if ( update_seg )
            {
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
  * @retval none.
  */
static uint8_t get_ddm_index_from_db(uint32_t ddm_param)
{
	conn_onboardhmi_param_t* param_db;
	uint8_t db_idx = DDMP_UNAVAILABLE; 
	uint8_t index;
	bool avail = false;

	for ( index = 0u; ( ( index < conn_onbhmi_db_elements ) && ( avail == false ) ); index++ )
 	{
		param_db = &conn_onboardhmi_param_db[index];
      	
		/* Validate the DDM parameter received */
		if ( param_db->ddm_parameter == ddm_param )
	  	{
			db_idx = index;
			avail = true;
		}
	}
	
	return db_idx;
}

/**
  * @brief  Initialize the Device onboardHMI
  * @param  none.
  * @retval none.
  */
static void initialize_dev_onboardhmi()
{
	error_type result;
    drv_bus_conf uc1510c_bus_conf;

    uc1510c_bus_conf.i2c.port    = I2C_MASTER0_PORT;
    uc1510c_bus_conf.i2c.sda     = I2C_MASTER0_SDA;
    uc1510c_bus_conf.i2c.scl     = I2C_MASTER0_SCL;
    uc1510c_bus_conf.i2c.bitrate = I2C_MASTER0_FREQ;
    
    /* Initialize the LCD Driver IC UC1510C */
    result = init_uc1510c(&uc1510c_bus_conf);
    
    if ( RES_PASS != result )
    {
        LOG(E, "UC1510C Driver init failed = %d",  result);
    }
}

/**
  * @brief  Function to publish the available classes in connedtor onboard HMI to the broker
  * @param  none.
  * @retval none.
  */
static void install_parameters(void)
{
	int32_t available = 1;
	
	/* Connector Onboard HMI publish the available of Inventilate Class and Button Class */
    /* Create the DDMP frame to Publish Inventilate "Available" to broker */
    TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, IV0AVL, &available, sizeof(int32_t), \
               connector_onboard_hmi.connector_id, portMAX_DELAY));
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
    uint32_t list_value = 0;
    SORTED_LIST_RETURN_VALUE ret = sorted_list_unique_get(&list_value, &conn_obhmi_table, ddm_param, 0);

#if CONN_ONBHMI_DEBUG_LOG
    LOG(I, "Received ddm_param = 0x%x match = %d", ddm_param, ret);
#endif

    if ( SORTED_LIST_FAIL != ret )
	{
		/* Validate the DDM parameter received */
		db_idx = get_ddm_index_from_db(ddm_param);

		if ( DDMP_UNAVAILABLE != db_idx )
	  	{
			param_db = &conn_onboardhmi_param_db[db_idx];

            index = ddm2_parameter_list_lookup(DDM2_PARAMETER_BASE_INSTANCE(ddm_param));

#if CONN_ONBHMI_DEBUG_LOG
            LOG(I, "ddm2_parameter_list_lookup index = %d", index);
#endif
            if ( -1 != index )
			{
                factor = Ddm2_unit_factor_list[Ddm2_parameter_list_data[index].out_unit];

                if ( factor == 0 )
                {
                    factor = 1;
                }
                
                /* Multiply with the factor */
                value = param_db->i32Value * factor;
#if CONN_ONBHMI_DEBUG_LOG
                LOG(I, "After factored i32value = %d", value);
#endif
                /* Frame and send the publish request */
                TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, ddm_param, &value, sizeof(int32_t), connector_onboard_hmi.connector_id, portMAX_DELAY));
            }
            else
            {
                LOG(E, "DDMP 0x%x not found in ddm2_parameter_list_lookup", ddm_param);
            }
		}
        else
        {
            LOG(E, "Invalid DDMP Request ddm_param 0x%x", ddm_param);
        }
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
#if CONN_ONBHMI_DEBUG_LOG
    LOG(I, "Time elapsed");
#endif
    push_data_to_the_queue(0, HMI_TIMER_EXPIRED);
}

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
    LOG(W, "Long Press! = %d", lcd_btn_press_evt);
}

/**
  * @brief  Timer callback function to handle long press time elapse event
  * @param  none.
  * @retval none.
  */
static void long_press_rel_tmr_cb( TimerHandle_t xTimer )
{
    LOG(W, "Long press released");
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

    if ( lcd_btn_state == BTN_PRESSED )
    {
        /* Stop the long press timer */
        TRUE_CHECK(xTimerStop(long_press_tmr_hdle, 0));
        /* Frame the short press event */
        short_press_event = lcd_btn_press_evt | (BUTTON_EVT_SHORT_PRESS << 8);
        /* Push the data into the queue */
        push_data_to_the_queue(short_press_event, BTN_PRESSED_EVENT);
        /* Change the state */
        lcd_btn_state = BTN_RELEASED;
        LOG(W, "Short Press! = %d", lcd_btn_press_evt);
    }
    else
    {
        LOG(W, "Unhandle");
    }

}

/**
  * @brief  Callback function to handle onboard hmi btn press interrupt
  * @param  device id.
  * @param  port number.
  * @param  pin number.
  * @retval none.
  */
void onboard_hmi_interrupt_cb(int device, int port, int pin)
{
    switch (lcd_btn_state)
    {
        case BTN_RELEASED:
        case BTN_LONG_PRESS_RELEASED:
            ZERO_CHECK(uc1510c_get_touch_panel_status((uint8_t*)&lcd_btn_press_evt));
            TRUE_CHECK(xTimerStart(long_press_tmr_hdle, 0));
            TRUE_CHECK(xTimerStart(short_press_tmr_hdle, 0));
            lcd_btn_state = BTN_PRESSED;
            break;

        case BTN_PRESSED:
            TRUE_CHECK(xTimerReset(short_press_tmr_hdle, 0));
            break;

        case BTN_LONG_PRESS_ONGOING:
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
                LOG(W, "Long Press.....");
                TRUE_CHECK(xTimerReset(long_press_rel_tmr_hdle, 0));
            }
            break;

        default:
            break;
    }
}

/**
  * @brief  Timer callback function to handle blink of HMI segments
  * @param  none.
  * @retval none.
  */
static void obhmi_blink_tmr_cb( TimerHandle_t xTimer )
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
    obhmi_ctrl_data_frame.data    = data;
    obhmi_ctrl_data_frame.data_id = data_id;

    /* Append the data in the Queue */
    osal_base_type_t ret = osal_queue_send (obhmi_pwr_ctrl_que_hdl, &obhmi_ctrl_data_frame, 0);

    if ( osal_success != ret )
    {
        LOG(E, "Queue error ret = %d", ret);
    }
}

/**
  * @brief  Function used to convert the DDMP param to DATA ID which understanding by onboard HMI control task
  * @param  ddm_param.
  * @retval data_id refer the enum OBHMI_CTRL_DATA_ID.
  */
OBHMI_CTRL_DATA_ID convert_ddmp_to_data_id(int32_t data, uint32_t ddm_param)
{
    OBHMI_CTRL_DATA_ID data_id;

    switch (ddm_param)
    {
        case IVPMGR0STATE:
            data_id = INV_STATE;
            break;

        case IV0AQST:
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
            data_id = UPDATE_SEG_WIFI;
            break;

        case BT0PAIR:
            LOG(W, "BT0PAIR Response received");
            data_id = UPDATE_SEG_BLE;
            break;

        case SDP0AVL:
            LOG(W, "SDP0AVL received");
            data_id = UPDATE_SEG_DP_SENS_STAT;
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

    /* Update button control variables based on the received DDMP request */
    update_hmi_btn_ctrl_variables(ddm_param, data);

    data_id = convert_ddmp_to_data_id(data, ddm_param);

#if CONN_ONBHMI_DEBUG_LOG
    LOG(I, "ddm_param = 0x%x, data = %d", ddm_param, data);
#endif
    if ( INVALID_DATA_ID != data_id )
    {
        push_data_to_the_queue(data, data_id);
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

    if ( configured_time_ms != period_ms )
    {
        /* Store the new period */
        configured_time_ms = period_ms;
        /* Change timer period */
        ret = xTimerChangePeriod(obHmiOneShotTimer, pdMS_TO_TICKS(period_ms), 0);
        
        if ( ret != pdPASS )
        {
		    LOG(E, "Timer period change failed = %d", ret);
        }  
    }
    else
    {
        /* Start timer */
        ret = xTimerStart( obHmiOneShotTimer, 0 );
        
        if ( ret != pdPASS )
        {
            LOG(E, "Timer start failed = %d", ret);
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
    osal_ubase_type_t ret = xTimerStart( blink_ctrl_timer_hdle, 0 );
    
    if ( ret != pdPASS )
    {
        LOG(E, "Timer start failed = %d", ret);
    }
}

/**
  * @brief  Function to stop the blink timer
  * @param  none.
  * @retval none.
  */
static void stop_obhmi_blink_timer(void)
{
    osal_ubase_type_t  xTimerStoped = xTimerStop( blink_ctrl_timer_hdle, 0 );
	
    if ( xTimerStoped != pdPASS )
    {
		LOG(E, "Timer stop Failed");
    }
}

/**
  * @brief  Function to stop the timer
  * @param  none.
  * @retval none.
  */
void stop_hmi_wait_timer(void)
{
    osal_ubase_type_t  xTimerStoped = xTimerStop( obHmiOneShotTimer, 0 );
	
    if ( xTimerStoped != pdPASS )
    {
		LOG(E, "Timer stop Failed");
    }
}

/**
  * @brief  Function to stop the timer
  * @param  none.
  * @retval none.
  */
void reset_hmi_wait_timer(void)
{
    osal_ubase_type_t  xTimerReset = xTimerReset( obHmiOneShotTimer, 0 );
	
    if ( xTimerReset != pdPASS )
    {
		LOG(E, "Timer reset Failed");
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
	
	if ( DDMP_UNAVAILABLE != db_idx )
   	{
		param_db = &conn_onboardhmi_param_db[db_idx];
		
		/* Update the value in db table */
		param_db->i32Value = value;

        if ( DIM0LVL == ddm_parameter )
        {
            control = DDMP2_CONTROL_SET;
            LOG(W, "DDMP2_CONTROL_SET for ddm_parameter = 0x%x", ddm_parameter);
        }
        else
        {
            control = DDMP2_CONTROL_PUBLISH;
        }

        if ( ( IV0BLREQ == ddm_parameter ) || ( IV0FILST == ddm_parameter )  || ( IV0MODE == ddm_parameter ) )
        {
            OBHMI_CTRL_DATA_ID data_id = convert_ddmp_to_data_id(value, ddm_parameter);

            push_data_to_the_queue(value, data_id);
        }
		
		/* Send data to the broker */
        connector_send_frame_to_broker(control, ddm_parameter, &value, sizeof(value), \
                                   connector_onboard_hmi.connector_id, (TickType_t)portMAX_DELAY);
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

    LOG(W, "ddmp = 0x%x data = %d", ddmp, data);

    switch (ddmp)
    {
        case IV0PWRON:
            hmi_btn_evt       = BTN_POWER;
            event_type        = BUTTON_EVT_SHORT_PRESS;
            inv_pwr_state     = data;                    // Access using semaphore
            lcd_btn_press_evt = BTN_POWER;               // Access using semaphore
            break;

        case IV0BLREQ:
            hmi_btn_evt = BTN_LIGHT;
            event_type  = BUTTON_EVT_LONG_PRESS;
            break;

        case DIM0LVL:
            hmi_btn_evt = BTN_LIGHT;
            event_type  = BUTTON_EVT_SHORT_PRESS;
            break;
        
        case IV0MODE:
            hmi_btn_evt = BTN_MODE;
            event_type  = BUTTON_EVT_SHORT_PRESS;
            break;

        case SDP0AVL:
            if ( DP_SENSOR_NOT_AVAILABLE == data )
            {
                LOG(W, "Subscribe request for DDMP SDP0AVL");
                // Whenever the availability of sensor node received then the subscribtion should be done again
                TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SUBSCRIBE, SDP0AVL, NULL, 0, \
                           connector_onboard_hmi.connector_id, portMAX_DELAY));
            }
            break;

        default:
            break;
    }

    if ( BTN_NO_EVENTS != hmi_btn_evt )
    {
        for ( index = 0; index < ONBOARD_HMI_EVT_TABLE_SIZE; index++ )
        {
            btn_event_list = &onboard_hmi_btn_evt[index];

            if ( btn_event_list->hmi_btn_evt == hmi_btn_evt )
            {
                hmi_domain_to_ddm_system = btn_event_list->data_conversion.hmi_domain_to_ddm_system;

                for ( idx2 = 0; idx2 < btn_event_list->data_conversion.size; idx2++ )
                {
                    if ( ( hmi_domain_to_ddm_system[idx2].ddm_parameter    == ddmp ) && 
                         ( hmi_domain_to_ddm_system[idx2].ddm_system_value == data ) )
                    {
                        if ( BUTTON_EVT_SHORT_PRESS == event_type )
                        { 
                            btn_event_list->short_press_evt_cnt = hmi_domain_to_ddm_system[idx2].hmi_domain_value;
                            LOG(W, "Short Press cnt = %d", btn_event_list->short_press_evt_cnt);
                        }
                        else
                        {
                            btn_event_list->long_press_evt_cnt = hmi_domain_to_ddm_system[idx2].hmi_domain_value;
                            LOG(W, "Long Press cnt = %d", btn_event_list->long_press_evt_cnt);
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
