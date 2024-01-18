
/*! \file connector_pwm_fan_motor.c
	\brief Connector for Fan and Motor
	\Author Sundaramoorthy-LTTS
 */

/** Includes ******************************************************************/
#include "configuration.h"

#ifdef CONNECTOR_PWM_FAN_MOTOR

#include "connector_pwm_fan_motor.h"
#include "app_api.h"
#include "ddm2_parameter_list.h"
#include "osal.h"
#include "hal_pwm.h"
#include "sorted_list.h"
#include "app_error_code.h"

#define CONN_PWM_FAN_MTR_SUB_DEPTH     50
#define DP_BAT_LIMIT                   2700

// Defining local parameter space
#define MTR0_NEW_TACHO_DATA_EVENT	    (DDM2_PARAMETER_CLASS(MTR0AVL) | DDM2_PARAMETER_PROPERTY_FIELD(0xf0))

// Minimal time between sending new tacho values from interrupt, to not flood the system
#define TACHO_SEND_MIN_INTERNAL_MS      (1000u)
// Minimal time between sending zero tacho value from interrupt, when no new interrupts has been received
#define TACHO_ERROR_MIN_INTERNAL_MS     (2000u)

/* Function pointer declaration */
typedef void (*conn_mtr_param_changed_t)(uint32_t dev_id, int32_t i32Value);

/* Struct Type Definitions */
typedef struct
{
    uint32_t ddm_parameter;
    DDM2_TYPE_ENUM type;
    uint8_t pub;
    uint8_t sub;
    int32_t i32Value;
    conn_mtr_param_changed_t cb_func;
} conn_fan_motor_parameter_t;


//! Holds Tacho Information
typedef struct 
{
    uint32_t dev_rpm;
} capture_info_t;


// Defining event data type
typedef struct _mtr_tacho_data_event_t
{
    capture_info_t capture_info;
    uint8_t capture_signal;
} mtr_tacho_data_event_t;

static uint32_t min_limit_rpm = 0;
static uint32_t max_limit_rpm = 0;
static int32_t  rated_speed = 0;

static DDMP2_FRAME frame_event_to_send;
volatile mtr_tacho_data_event_t tacho_event;

/* Static Function declarations */
static int initialize_connector_fan_motor(void);
static void conn_fan_motor_process_task(void *pvParameter);
static void conn_fan_mtr_ctrl_task(void *pvParameter);
static void install_parameters(void);
static void start_subscribe(void);
static void start_publish(void);
static void process_set_and_publish_request(uint32_t ddm_param, int32_t i32value, DDMP2_CONTROL_ENUM req_type);
static void process_subscribe_request(uint32_t ddm_param);
static int add_subscription(DDMP2_FRAME *pframe);
static uint8_t get_ddm_index_from_db(uint32_t ddm_param);
static void handle_invsub_data(uint32_t ddm_param, int32_t data);
static void iv_periodic_tmr_cb( TimerHandle_t xTimer );
static void iv_one_shot_tmr_cb( TimerHandle_t xTimer );
static void change_storage_timer_period(uint32_t time_period_msec);
static void start_storage_timer(void);
static void stop_storage_timer(void);
static void storage_cb_func( TimerHandle_t xTimer );
static void storage_humid_chk_cb_func(TimerHandle_t xTimer);
static bool pwm_cap_isr_cb(uint8_t unit, uint8_t capture_signal, uint32_t value);

//! Quene handle definition
osal_queue_handle_t iv_ctrl_que_handle;

static TimerHandle_t peridic_tmr_hdle;
static TimerHandle_t one_shot_tmr_hdle;
static TimerHandle_t xStorageTimer;
static TimerHandle_t xStorageHumid_chk_Timer;
INVENTILATE_CONTROL_ALGO* ptr_ctrl_algo = &iv_ctrl_algo;
static uint8_t storage_humid_timer = 0;

static uint32_t tmr_period = 0;
static uint8_t  iaq_run_state = 0;
static uint32_t iaqprev_30min= 0;

static volatile TickType_t time_now[MAX_NUM_DEVICE];
static volatile TickType_t last_tick[MAX_NUM_DEVICE] = {0, 0, 0};
static volatile TickType_t time_diff[MAX_NUM_DEVICE];
static volatile uint32_t time_diff_ms = 0;
static volatile uint32_t capture_counter[MAX_NUM_DEVICE];
static volatile uint8_t pulse_count_per_revol[MAX_NUM_DEVICE];
static volatile uint32_t current_cap_value[MAX_NUM_DEVICE];
static volatile uint32_t previous_cap_value[MAX_NUM_DEVICE];
static volatile capture_info_t dev_capt[MAX_NUM_DEVICE];
uint32_t dev_rpm[MAX_NUM_DEVICE];

extern EXT_RAM_ATTR fan_motor_control_info_db_t fan_motor_control_db[DEV_RPM_DB_TABLE_SIZE];

//#define TACHO_READ_QUEUE_LENGTH 				  ((osal_base_type_t) MAX_NUM_DEVICE)
//#define TACHO_READ_QUEUE_SIZE   				  ((osal_base_type_t) (MAX_NUM_DEVICE * sizeof(capture_info_t)))

#define IV_CONTROL_TASK_QUEUE_LENGTH			  ((osal_base_type_t) 200)
#define IV_CONTROL_TASK_QUEUE_ITEM_SIZE   		  ((osal_base_type_t) sizeof(IV_DATA))

#define CONN_FAN_MTR_ENABLE_LOGS                   0u

/* Structure for Connector fan motor */
CONNECTOR connector_pwm_fan_motor =
{
	.name = "Fan Motor connector",
	.initialize = initialize_connector_fan_motor,
};

/* DDM Parameter table for connector fan motor */
static conn_fan_motor_parameter_t conn_fan_motor_param_db[] =
{
	//  ddm_parameter                               type       pub sub      i32Value                 cb_func
    {MTR0DEVID|DDM2_PARAMETER_INSTANCE(0) , DDM2_TYPE_INT32_T, 	1, 	0, 	     DEV_FAN1_AIR_IN,                      NULL},
	{MTR0DEVID|DDM2_PARAMETER_INSTANCE(1) , DDM2_TYPE_INT32_T, 	1, 	0,      DEV_FAN2_AIR_OUT,                      NULL},
	{MTR0DEVID|DDM2_PARAMETER_INSTANCE(2) , DDM2_TYPE_INT32_T, 	1, 	0, 	           DEV_MOTOR,                      NULL},
    {MTR0SETSPD|DDM2_PARAMETER_INSTANCE(0), DDM2_TYPE_INT32_T, 	1, 	1, 	                   0,        handle_invsub_data},
	{MTR0SETSPD|DDM2_PARAMETER_INSTANCE(1), DDM2_TYPE_INT32_T, 	1, 	1, 	                   0,        handle_invsub_data},
	{MTR0SETSPD|DDM2_PARAMETER_INSTANCE(2), DDM2_TYPE_INT32_T, 	1, 	1, 	                   0,        handle_invsub_data},	
    {MTR0TACHO|DDM2_PARAMETER_INSTANCE(0) , DDM2_TYPE_INT32_T, 	1, 	0, 	                   0,        handle_invsub_data},
	{MTR0TACHO|DDM2_PARAMETER_INSTANCE(1) , DDM2_TYPE_INT32_T, 	1, 	0, 	                   0,        handle_invsub_data},
	{MTR0TACHO|DDM2_PARAMETER_INSTANCE(2) , DDM2_TYPE_INT32_T, 	1, 	0, 	                   0,        handle_invsub_data},
    {MTR0MINSPD|DDM2_PARAMETER_INSTANCE(0), DDM2_TYPE_INT32_T, 	1, 	0, 	    DEV_FAN1_MIN_RPM,        handle_invsub_data},
	{MTR0MINSPD|DDM2_PARAMETER_INSTANCE(1), DDM2_TYPE_INT32_T, 	1, 	0, 	    DEV_FAN2_MIN_RPM,        handle_invsub_data},
	{MTR0MINSPD|DDM2_PARAMETER_INSTANCE(2), DDM2_TYPE_INT32_T, 	1, 	0, 	   DEV_MOTOR_MIN_RPM,        handle_invsub_data},
	{MTR0MAXSPD|DDM2_PARAMETER_INSTANCE(0), DDM2_TYPE_INT32_T, 	1, 	0, 	    DEV_FAN1_MAX_RPM,        handle_invsub_data},
	{MTR0MAXSPD|DDM2_PARAMETER_INSTANCE(1), DDM2_TYPE_INT32_T, 	1, 	0, 	    DEV_FAN2_MAX_RPM,        handle_invsub_data},
	{MTR0MAXSPD|DDM2_PARAMETER_INSTANCE(2), DDM2_TYPE_INT32_T, 	1, 	0, 	   DEV_MOTOR_MAX_RPM,        handle_invsub_data},
    {MTR0TACHO|DDM2_PARAMETER_INSTANCE(0) , DDM2_TYPE_INT32_T, 	1, 	0, 	                   0,                      NULL},
	{MTR0TACHO|DDM2_PARAMETER_INSTANCE(1) , DDM2_TYPE_INT32_T, 	1, 	0, 	                   0,                      NULL},
	{MTR0TACHO|DDM2_PARAMETER_INSTANCE(2) , DDM2_TYPE_INT32_T, 	1, 	0, 	                   0,                      NULL},
    
    {MTR0DIR|DDM2_PARAMETER_INSTANCE(0)   , DDM2_TYPE_INT32_T, 	1, 	0, 	                   0,        handle_invsub_data},
	{MTR0DIR|DDM2_PARAMETER_INSTANCE(1)   , DDM2_TYPE_INT32_T, 	1, 	0, 	                   0,        handle_invsub_data},
	{MTR0DIR|DDM2_PARAMETER_INSTANCE(2)   , DDM2_TYPE_INT32_T, 	1, 	0, 	                   0,        handle_invsub_data},
    {IVAQR0MIN|DDM2_PARAMETER_INSTANCE(0) , DDM2_TYPE_INT32_T, 	1, 	0, 	                   0,        handle_invsub_data},
    {IVAQR0MIN|DDM2_PARAMETER_INSTANCE(1) , DDM2_TYPE_INT32_T, 	1, 	0, 	                   0,        handle_invsub_data},
    {IVAQR0MIN|DDM2_PARAMETER_INSTANCE(2) , DDM2_TYPE_INT32_T, 	1, 	0, 	                   0,        handle_invsub_data},
    {IVAQR0MAX|DDM2_PARAMETER_INSTANCE(0) , DDM2_TYPE_INT32_T, 	1, 	0, 	                   0,        handle_invsub_data},
    {IVAQR0MAX|DDM2_PARAMETER_INSTANCE(1) , DDM2_TYPE_INT32_T, 	1, 	0, 	                   0,        handle_invsub_data},
    {IVAQR0MAX|DDM2_PARAMETER_INSTANCE(2) , DDM2_TYPE_INT32_T, 	1, 	0, 	                   0,        handle_invsub_data},
	{IVAQR0MIN|DDM2_PARAMETER_INSTANCE(0) , DDM2_TYPE_INT32_T,  1,  0,                     0, 	     handle_invsub_data},
    {IVAQR0MIN|DDM2_PARAMETER_INSTANCE(1) , DDM2_TYPE_INT32_T,  1,  0,                     0, 	     handle_invsub_data},
    {IVAQR0MIN|DDM2_PARAMETER_INSTANCE(2) , DDM2_TYPE_INT32_T,  1,  0,                     0, 	     handle_invsub_data},
    {IVAQR0MAX|DDM2_PARAMETER_INSTANCE(0) , DDM2_TYPE_INT32_T,  1,  0,                     0, 	     handle_invsub_data},
    {IVAQR0MAX|DDM2_PARAMETER_INSTANCE(1) , DDM2_TYPE_INT32_T,  1,  0,                     0, 	     handle_invsub_data},
    {IVAQR0MAX|DDM2_PARAMETER_INSTANCE(2) , DDM2_TYPE_INT32_T,  1,  0,                     0, 	     handle_invsub_data},
    {SBMEB0IAQ				              , DDM2_TYPE_INT32_T, 	0, 	1, 	                   0,        handle_invsub_data},
    {SBMEB0AQR                            , DDM2_TYPE_INT32_T, 	0, 	1, 	                   0,        handle_invsub_data},
    {SBMEB0HUM                            , DDM2_TYPE_INT32_T, 	0, 	1, 	                   0,        handle_invsub_data},
    {SDP0AVL                              , DDM2_TYPE_INT32_T,  0,  1,                     0,        handle_invsub_data},
    {SDP0DP                               , DDM2_TYPE_INT32_T, 	0, 	1, 	                   0,        handle_invsub_data},
    {IV0MODE                              , DDM2_TYPE_INT32_T, 	0, 	1, 	                   0,        handle_invsub_data},
    {IVPMGR0STATE                         , DDM2_TYPE_INT32_T, 	0, 	1, 	                   0,        handle_invsub_data},
    {IV0IONST                             , DDM2_TYPE_INT32_T, 	1, 	0, 	         IV0IONST_ON,        handle_invsub_data},
    {IV0ERRST                             , DDM2_TYPE_INT32_T, 	0, 	1, 	                   0,        handle_invsub_data},
    {IV0SETT                              , DDM2_TYPE_INT32_T, 	0, 	1, 	                   0,        handle_invsub_data},
    {IV0STGT                       , DDM2_TYPE_INT32_T, 	0, 	1, 	                   0,        NULL},
    {IV0PWRSRC                            , DDM2_TYPE_INT32_T, 	0, 	1, 	                   0,        handle_invsub_data},
};

/* Calculate the connector fan motor database table num elements */
static const uint32_t conn_fanmot_db_elements = ELEMENTS(conn_fan_motor_param_db);

DECLARE_SORTED_LIST_EXTRAM(conn_pwm_fan_mtr_table, CONN_PWM_FAN_MTR_SUB_DEPTH);       //!< \~ Subscription table storage 

/**
  * @brief  Initialize the connector fan motor
  * @param  none.
  * @retval none.
  */
static int initialize_connector_fan_motor(void)
{    
    /* Create queue for invent control task */
    iv_ctrl_que_handle = osal_queue_create(IV_CONTROL_TASK_QUEUE_LENGTH, IV_CONTROL_TASK_QUEUE_ITEM_SIZE);

    if ( NULL != iv_ctrl_que_handle )
    {
        LOG(I, "Queue creation done for invent control task");
    }

	/* Initialize the RPM ranges of devices FAN Motor */
	read_data_from_nvs();
#if CONN_PWM_DEBUG_LOG
    LOG(I,"[IV0Sett = %d ]",ivsett_config.byte );
    if(ivsett_config.EN_DIS_IONIZER == true)
    {
        LOG(I,"Ionizer Enabled");
    }
    else
    {
        LOG(I,"Ionizer Disbled");
    }

    if(ivsett_config.EN_DIS_SOLAR == true)
    {
        LOG(I,"Solar Enabled");
    }
    else
    {
        LOG(I,"Solar Disbled");
    }
#endif

	/* Initialize the fan and motor */
    initialize_fan_motor();

    /* Initilaize inventilate control logic */
    init_iv_control_algo(&iv_ctrl_algo);

    /* Create the periodic timer for inventilate control logic */
    peridic_tmr_hdle = xTimerCreate("iv_prd_tmr", IV_IAQ_PROCESSING_TICKS, pdTRUE, 0, iv_periodic_tmr_cb);

    /* Create the one shot timer software timer */
    one_shot_tmr_hdle = xTimerCreate("one_sh_tmr", RV_IDLE_COND_SETTLE_WAIT_TIME_TICKS, pdFALSE, 0, iv_one_shot_tmr_cb);

    /* Create the one shot timer software timer for inventilate control logic, storage mode. */
    xStorageTimer = xTimerCreate("xStorageTimer", ptr_ctrl_algo->storage_tmr_val_ticks[ptr_ctrl_algo->storage_timer_config], pdFALSE, 0, storage_cb_func);
    
    xStorageHumid_chk_Timer =  xTimerCreate("xStorage_HumChkTmr", STORAGE_HUM_CHK_TMR_TICKS, pdTRUE, 0, storage_humid_chk_cb_func);
    /* Task for handling the DDMP request */
	TRUE_CHECK(osal_task_create(conn_fan_motor_process_task, CONNECTOR_FAN_MOTOR_PROCESS_TASK_NAME, CONNECTOR_FAN_MOTOR_PROCESS_STACK_DEPTH, NULL, CONNECTOR_FAN_MOTOR_PROCESS_TASK_PRIORITY, NULL));

    /* Task for control the RPM of Fan and Motor by an algorithm designed based on IAQ and Differential pressure sensor data */
    TRUE_CHECK(osal_task_create(conn_fan_mtr_ctrl_task, CONNECTOR_FAN_MOTOR_CONTROL_TASK_NAME, CONNECTOR_FAN_MOTOR_TACHO_READ_STACK_DEPTH, iv_ctrl_que_handle, CONNECTOR_FAN_MOTOR_CTRL_TASK_PRIORITY, NULL));

	/* Install parameters in the Inventory of broker */
	install_parameters();

	/* Subscribe DDMP parameters */
	start_subscribe();

    /* Publish all DDMP parameters */
    start_publish();

	return 1;
}

/**
  * @brief  Function to publish Connector PWM FAN Motor provided available classes to the broker
  * @param  none.
  * @retval none.
  */
static void install_parameters(void)
{
    uint8_t index;

    // Send the request for register FAN1, FAN2 and Motor devices instance to broker
    for ( index = 0; index < MAX_NUM_DEVICE; index++ )
    {
        TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_REG, DDM2_PARAMETER_CLASS(MTR0AVL), NULL, 0, connector_pwm_fan_motor.connector_id, portMAX_DELAY));
    }

    // Send the request for register AIR quality levels
    for ( index = 0; index < IAQ_RANGE_LEVELS; index++ )
    {
        TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_REG, DDM2_PARAMETER_CLASS(IVAQR0AVL), NULL, 0, connector_pwm_fan_motor.connector_id, portMAX_DELAY));
    }
}

/**
  * @brief  Function to subscribe the DDMP parameters needed for connector fan and motor
  * @param  none.
  * @retval none.
  */
static void start_subscribe(void)
{
	conn_fan_motor_parameter_t *ptr_param_db;
	uint8_t db_idx;

	for ( db_idx = 0; db_idx < conn_fanmot_db_elements; db_idx++ )
	{
		ptr_param_db = &conn_fan_motor_param_db[db_idx];
        
        /* Check the DDM parameter need subscribtion */
		if ( ptr_param_db->sub )
		{
            LOG(I, "Subscribed DDMP for %s is 0x%x", connector_pwm_fan_motor.name, ptr_param_db->ddm_parameter);
            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SUBSCRIBE, ptr_param_db->ddm_parameter, NULL, 0, connector_pwm_fan_motor.connector_id, portMAX_DELAY));
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
    conn_fan_motor_parameter_t *ptr_param_db;
    uint16_t db_idx;
    
    for ( db_idx = 0; db_idx < conn_fanmot_db_elements; db_idx++ )
    {
        ptr_param_db = &conn_fan_motor_param_db[db_idx];

        /* Check the DDM parameter need to publish */
        if ( ptr_param_db->pub ) 
        {
            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, ptr_param_db->ddm_parameter, &ptr_param_db->i32Value, sizeof(int32_t), connector_pwm_fan_motor.connector_id, portMAX_DELAY));
        }
    }
}

/**
  * @brief  Function to update the Air Quality status to broker
  * @param  iaq_status Refer enum IV0AQST_ENUM.
  * @retval none.
  */
void update_iaq_status_to_broker(const IV0AQST_ENUM iaq_status)
{
    int32_t i32Value = iaq_status;

    TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, IV0AQST, &i32Value, \
                sizeof(int32_t), connector_pwm_fan_motor.connector_id, portMAX_DELAY));
}

/**
  * @brief  Function to update the differential pressure status to broker
  * @param  press_status Refer enum IV0DPST_ENUM.
  * @retval none.
  */
void update_dp_status_to_broker(const IV0PRST_ENUM press_status)
{
    int32_t i32Value = press_status;

    TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, IV0PRST, &i32Value, \
                sizeof(int32_t), connector_pwm_fan_motor.connector_id, portMAX_DELAY));
}

/**
  * @brief  Function to update the value of set RPM in the FAN1, FAN2 and motor to broker
  * @param  dev_id Refer enum invent_device_id_t.
  * @param  rpm    RPM set to the corresponding device ID.
  * @retval none.
  */
void update_set_rpm_to_broker(const invent_device_id_t dev_id, const uint32_t rpm)
{
    int32_t i32Value = rpm;

    TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, MTR0SETSPD|DDM2_PARAMETER_INSTANCE(dev_id), &i32Value, \
                sizeof(int32_t), connector_pwm_fan_motor.connector_id, portMAX_DELAY));
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

    return sorted_list_single_add(&conn_pwm_fan_mtr_table, key, value);
}

/**
  * @brief  Task for connector fan motor for processing the DDMP request received from broker
  * @param  pvParameter.
  * @retval none.
  */
static void conn_fan_motor_process_task(void *pvParameter)
{
	DDMP2_FRAME *pframe;
	size_t frame_size;
    int32_t available = 1;

	while (1)
	{
		TRUE_CHECK ( pframe = xRingbufferReceive(connector_pwm_fan_motor.to_connector, &frame_size, portMAX_DELAY) );

		switch (pframe->frame.control)
		{
		case DDMP2_CONTROL_PUBLISH:
#if CONN_PWM_DEBUG_LOG
		    LOG(I, "Received DDMP2_CONTROL_PUBLISH");
#endif 
			process_set_and_publish_request(pframe->frame.publish.parameter, pframe->frame.publish.value.int32, pframe->frame.control);
			break;

		case DDMP2_CONTROL_SET:
            if (MTR0_NEW_TACHO_DATA_EVENT == pframe->frame.set.parameter)
            {
                mtr_tacho_data_event_t     *p_dev_capt = (mtr_tacho_data_event_t *)pframe->frame.set.value.raw;
                /* Send the RPM values to the broker */
                update_and_send_val_to_broker(MTR0TACHO|DDM2_PARAMETER_INSTANCE(p_dev_capt->capture_signal), p_dev_capt->capture_info.dev_rpm);
            }
            else
            {
#if CONN_PWM_DEBUG_LOG
                LOG(I, "Received DDMP2_CONTROL_SET");
#endif 
                process_set_and_publish_request(pframe->frame.set.parameter, pframe->frame.set.value.int32, pframe->frame.control);
            }
            break;

		case DDMP2_CONTROL_SUBSCRIBE:
#if CONN_PWM_DEBUG_LOG
		    LOG(I, "Received DDMP2_CONTROL_SUBSCRIBE");
#endif 
			add_subscription(pframe);
			process_subscribe_request(pframe->frame.subscribe.parameter);
			break;

        case DDMP2_CONTROL_REG:
#if CONN_PWM_DEBUG_LOG
            LOG(I, "Received DDMP2_CONTROL_REG device_class = 0x%x", pframe->frame.reg.device_class);
#endif
            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, pframe->frame.reg.device_class, &available, 
                        sizeof(available), connector_pwm_fan_motor.connector_id, portMAX_DELAY));
            break;

		default:
			LOG(E, "UNHANDLED frame %02x from broker!",pframe->frame.control);
			break;
		}

		vRingbufferReturnItem(connector_pwm_fan_motor.to_connector, pframe);
    }
}


//#ifndef CONNECTOR_EOL_SERVICE
/**
  * @brief  Task to control Fan and Motor based on IAQ and Differential pressure sensor
  * @param  pvParameter.
  * @retval none.
  */
static void conn_fan_mtr_ctrl_task(void *pvParameter)
{
    uint32_t tacho_read = 0;
    int32_t iaq_value = 0;

    while (1)
    {
        /* Queue will be in blocked state untill data recevied */
        if ( pdPASS == xQueueReceive((osal_queue_handle_t)pvParameter, (void *)&ptr_ctrl_algo->iv_data, portMAX_DELAY) )
		{
#if CONN_PWM_DEBUG_LOG
            LOG(I, "Data ID %d data = %d", ptr_ctrl_algo->iv_data.data_id, ptr_ctrl_algo->iv_data.data);
#endif

#ifndef CONNECTOR_EOL_SERVICE
            /* Parse the received data */
            parse_received_data();

            /* Check if the timer for IAQ process control has expired or not - IV_IAQ_PROCESSING_TICKS (5sec) */
            if ( true == ptr_ctrl_algo->per_tmr_exp )
            {
                /* Reset the timer flag */
                ptr_ctrl_algo->per_tmr_exp   = false;
                ptr_ctrl_algo->data_received = true;

                /* Average the accumulated data of IAQ and DP */
                calc_avg_for_iaq_dp(ptr_ctrl_algo);
                
                /* Find the IAQ status and Pressure status from the accumulated data */
                ptr_ctrl_algo->curr_pr_stat  = find_press_comp_state(ptr_ctrl_algo);                        
                ptr_ctrl_algo->curr_iaq_stat = find_air_quality_status(ptr_ctrl_algo);

                if ( ptr_ctrl_algo->curr_iaq_stat != ptr_ctrl_algo->prev_iaq_stat )
                {
                    update_iaq_status_to_broker(ptr_ctrl_algo->curr_iaq_stat);
                    ptr_ctrl_algo->prev_iaq_stat = ptr_ctrl_algo->curr_iaq_stat;
                }
                
                if ( ptr_ctrl_algo->curr_pr_stat != ptr_ctrl_algo->prev_pr_stat )
                {
                    update_dp_status_to_broker(ptr_ctrl_algo->curr_pr_stat);
                    ptr_ctrl_algo->prev_pr_stat = ptr_ctrl_algo->curr_pr_stat;
                }

                ptr_ctrl_algo->min_counter += IV_CONTROL_PROCESSING_INTERVAL_MIN;

                if ( ptr_ctrl_algo->min_counter >= 1 )//FAN_MOTOR_VALIDATION_INTERVAL_MIN )
                {
                    ptr_ctrl_algo->min_counter = 0;

                    ptr_ctrl_algo->fan_mtr_dev_curr_stat = ptr_ctrl_algo->invent_error_status;

                    for ( invent_device_id_t dev_id = 0; dev_id < MAX_NUM_DEVICE; dev_id++ )
                    {
                        if ( ptr_ctrl_algo->set_rpm[dev_id] != 0 )
                        {
                            LOG(I,"[calculate tolerance value to check RPM mismatch error]");
                            rated_speed   = (int32_t)((float)ptr_ctrl_algo->set_rpm[dev_id] * ( (float)ptr_ctrl_algo->rated_speed_percent[dev_id] / (float)100.0f ));
                            max_limit_rpm = ptr_ctrl_algo->set_rpm[dev_id] + rated_speed;
                            min_limit_rpm = ptr_ctrl_algo->set_rpm[dev_id] - rated_speed;

                            if ( ptr_ctrl_algo->dev_tacho[dev_id] == 0 )
                            {
                                // Error : No tacho is received when the device is active
                                tacho_read = ptr_ctrl_algo->dev_tacho[dev_id];
                                update_and_send_val_to_broker(MTR0TACHO|DDM2_PARAMETER_INSTANCE(dev_id), tacho_read);
                                switch(dev_id)
                                {
                                    case 0:
                                        /*FAN1 No Tacho*/
                                        ptr_ctrl_algo->fan_mtr_dev_curr_stat |= 1 << ( FAN1_NO_TACHO_DEVICE_ACTIVE );
                                        break;
                                    case 1:
                                        /*FAN2 No Tacho*/
                                        ptr_ctrl_algo->fan_mtr_dev_curr_stat |= 1 << (  FAN2_NO_TACHO_DEVICE_ACTIVE );                                   
                                        break;
                                    case 2:
                                        /*Motor No Tacho*/
                                        ptr_ctrl_algo->fan_mtr_dev_curr_stat |= 1 << ( MOTOR_NO_TACHO_DEVICE_ACTIVE );
                                        break;
                                    case MAX_NUM_DEVICE:
                                        break;    
                                    default:
                                        /* invalid device*/
                                        break;    

                                }
                            }
                            else if ( ( ptr_ctrl_algo->dev_tacho[dev_id] < min_limit_rpm ) || ( ptr_ctrl_algo->dev_tacho[dev_id] > max_limit_rpm ) )
                            {
                                // Error : Tacho reading is not matched with the expected RPM..
                                switch(dev_id)
                                {
                                    case 0:
                                        /*FAN2 RPM Mismatch*/
                                        ptr_ctrl_algo->fan_mtr_dev_curr_stat |= 1 << ( FAN1_RPM_MISMATCH );
                                        break;
                                    case 1:
                                        /*FAN2 RPM Mismatch*/
                                        ptr_ctrl_algo->fan_mtr_dev_curr_stat |= 1 << (  FAN2_RPM_MISMATCH );
                                        break;
                                    case 2:
                                        /*Motor Mismatch*/
                                        ptr_ctrl_algo->fan_mtr_dev_curr_stat |= 1 << ( MOTOR_RPM_MISMATCH );
                                        break;
                                    case MAX_NUM_DEVICE:
                                        break;    

                                }
                            }
                            else
                            {
                                // Error resolved : No tacho is received when the device is active
                                // Error resolved : Tacho reading is not matched with the expected RPM..
                                switch(dev_id)
                                {
                                    case 0:
                                        ptr_ctrl_algo->fan_mtr_dev_curr_stat &= ~( 1 << ( FAN1_RPM_MISMATCH ) );
                                        ptr_ctrl_algo->fan_mtr_dev_curr_stat &= ~( 1 << (FAN1_NO_TACHO_DEVICE_ACTIVE ) );
                                        break;
                                    case 1:
                                        ptr_ctrl_algo->fan_mtr_dev_curr_stat &= ~( 1 << ( FAN2_RPM_MISMATCH ) );
                                        ptr_ctrl_algo->fan_mtr_dev_curr_stat &= ~( 1 << (FAN2_NO_TACHO_DEVICE_ACTIVE ) );
                                        break;
                                    case 2:
                                        ptr_ctrl_algo->fan_mtr_dev_curr_stat &= ~( 1 << ( MOTOR_RPM_MISMATCH ) );
                                        ptr_ctrl_algo->fan_mtr_dev_curr_stat &= ~( 1 << (MOTOR_NO_TACHO_DEVICE_ACTIVE ) );
                                        break;
                                    case MAX_NUM_DEVICE:
                                        break;    

                                }
                            }
                        }
                        else
                        {
                            if ( ptr_ctrl_algo->dev_tacho[dev_id] != 0 )
                            {
                                // Error : Tacho data received when the device is inactive / OFF
                                switch(dev_id)
                                {
                                    case 0:
                                        ptr_ctrl_algo->fan_mtr_dev_curr_stat |= 1 <<  FAN1_TACHO_READ_DEVICE_INACTIVE ;
                                        LOG(W,"[FAN1_INACT %d]",ptr_ctrl_algo->dev_tacho[dev_id]);
                                        break;
                                    case 1:
                                        ptr_ctrl_algo->fan_mtr_dev_curr_stat |= 1 <<  FAN2_TACHO_READ_DEVICE_INACTIVE ;
                                        LOG(W,"[FAN2_INACT %d]",ptr_ctrl_algo->dev_tacho[dev_id]);
                                        break;
                                    case 2:
                                        ptr_ctrl_algo->fan_mtr_dev_curr_stat |= 1 << MOTOR_TACHO_READ_DEVICE_INACTIVE ;
                                        LOG(W,"[MOTOR_INACT %d]",ptr_ctrl_algo->dev_tacho[dev_id]);
                                        break;
                                    case MAX_NUM_DEVICE:
                                        break;    

                                }
                            }
                            else
                            {
                                // Error resolved : Tacho data received when the device is inactive / OFF
                                switch(dev_id)
                                {
                                    case 0:
                                        ptr_ctrl_algo->fan_mtr_dev_curr_stat &= ~ ( 1 << FAN1_TACHO_READ_DEVICE_INACTIVE  );
                                        break;
                                    case 1:
                                        ptr_ctrl_algo->fan_mtr_dev_curr_stat &= ~ ( 1 <<  FAN2_TACHO_READ_DEVICE_INACTIVE  );
                                        break;
                                    case 2:
                                        ptr_ctrl_algo->fan_mtr_dev_curr_stat &= ~ ( 1 <<  MOTOR_TACHO_READ_DEVICE_INACTIVE );
                                        break;
                                    case MAX_NUM_DEVICE:
                                        break;    

                                }
                            }
                        }

                        LOG(I, "dev%d set_rpm = %d tacho = %d min_exp_rpm = %d", \
                            dev_id, ptr_ctrl_algo->set_rpm[dev_id], ptr_ctrl_algo->dev_tacho[dev_id], min_limit_rpm);

                        // Temp fix to find the no tacho event
                        ptr_ctrl_algo->dev_tacho[dev_id] = 0;
                    
                    }

                    if ( ptr_ctrl_algo->fan_mtr_dev_curr_stat != ptr_ctrl_algo->fan_mtr_dev_prev_stat )
                    {
                        LOG(W, "Fan/Motor fault status = 0x%x", ptr_ctrl_algo->fan_mtr_dev_curr_stat);

                        ptr_ctrl_algo->fan_mtr_dev_prev_stat = ptr_ctrl_algo->fan_mtr_dev_curr_stat;
                        
                        TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, IV0ERRST, &ptr_ctrl_algo->fan_mtr_dev_curr_stat, sizeof(int32_t), \
                                   connector_pwm_fan_motor.connector_id, portMAX_DELAY));
                    }

                }
            iaq_value = ptr_ctrl_algo->curr_avg_iaq_value;
#if INV_ALGO_DEBUG 
                LOG(I, "iaq_st=%d pr_st=%d cur_avg_iaq_val = %d", ptr_ctrl_algo->curr_iaq_stat, ptr_ctrl_algo->curr_pr_stat,ptr_ctrl_algo->curr_avg_iaq_value);
#endif
            }

#if INV_ALGO_DEBUG 
                LOG(I, "cur_st=%d iaq_val = %d ", ptr_ctrl_algo->curr_state, iaq_value) ;
#endif

            /* State machine to control the inventilate */                       
            switch ( ptr_ctrl_algo->curr_state )
            {
                case INVENTILATE_STATE_STANDBY:
                    {
                        if ( IVPMGR0STATE_ACTIVE == ptr_ctrl_algo->iv_pwr_state )
                        {
                            start_periodic_timer();
                            reset_accumulated_data(ptr_ctrl_algo);
                            /* Reset the prev accumulated data */                           
                            ptr_ctrl_algo->cur_sel_mode      = IV0MODE_AUTO;
                            /* Reset the dev configuration */                            
                            reset_dev_config();
#if ( INVENT_HARWARE_VERSION >= HW_VERSION_4_1 )
                            // Power ON Ionizer when the inventilate is powered ON by user 
#ifdef EN_IONIZER_FLAG

                            if(ivsett_config.EN_DIS_IONIZER == true)
                            {
                                EN_IONIZER(1);
                                ptr_ctrl_algo->ionizer_status    = (int32_t)IV0IONST_ON;
                            }
                            else
                            {
                                EN_IONIZER(0);
                                ptr_ctrl_algo->ionizer_status    = (int32_t)IV0IONST_OFF;
                            }
#else
                                EN_IONIZER(1);
                                ptr_ctrl_algo->ionizer_status    = (int32_t)IV0IONST_ON;

#endif
#endif                          

                            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, IV0IONST, &ptr_ctrl_algo->ionizer_status, sizeof(int32_t), \
                                        connector_pwm_fan_motor.connector_id, portMAX_DELAY));
                            /* Reset the accuracy status */
                            ptr_ctrl_algo->sens_acc          = BME6X_STABILIZATION_ONGOING;
                            /* Set the state to be transfer */    
                            ptr_ctrl_algo->curr_state        = INVENTILATE_STATE_PROCESS_PARAM; 
                        }
                    }
                    break;

                case INVENTILATE_STATE_PROCESS_PARAM:
                    {
                        if ( IV0MODE_AUTO      !=  ptr_ctrl_algo->cur_sel_mode  )
                        {
                            reset_dev_config();
                            LOG(I,"rst_dev curr_mode %d prev_mode %d ",ptr_ctrl_algo->cur_sel_mode, ptr_ctrl_algo->prev_sel_mode );

                        }
                        else
                        {
                            if ( ( IV0PRST_VALID_PRESS_LEVEL    != ptr_ctrl_algo->curr_pr_stat ) && 
                                ( IV0PRST_PRESS_STATUS_UNKNOWN != ptr_ctrl_algo->curr_pr_stat ) &&  
                                ( IV0MODE_AUTO               == ptr_ctrl_algo->cur_sel_mode  ))
                            {
                                /* When the program control reached this point which means pressure exceeds the acceptable range 
                                The timer is start here for a defined period of time to validate the 
                                pressure compensation control, after this timer expires */

                                /* Reset the timer flag before starting the timer */
                                ptr_ctrl_algo->wait_tmr_exp = false;
                                /* Set the DP compensation threshold */
                                update_dp_comp_threshold_val(ptr_ctrl_algo);
                                /* Start wait timer */
                                start_wait_tmr(MIN_TO_MSEC(RV_PRESS_COMP_WAIT_TIME_MIN));
                                /* Update state */
                                ptr_ctrl_algo->curr_state = INVENTILATE_STATE_PRESS_CTRL;
                            }
                            else if ( ( IV0MODE_AUTO == ptr_ctrl_algo->cur_sel_mode  ) && 
                                    ( ( ( IV0AQST_AIR_QUALITY_GOOD != ptr_ctrl_algo->curr_iaq_stat ) && ( IV0AQST_AIR_QUALITY_UNKNOWN != ptr_ctrl_algo->curr_iaq_stat ) ) ||  
                                    (iaq_value > IAQ_DEF_GOOD_MAX) ) )
                            {
                                /* Update state */
                                iaq_run_state = 0;
                                ptr_ctrl_algo->curr_state = INVENTILATE_STATE_AQ_CTRL;
                            }
                            else if ( ptr_ctrl_algo->cur_sel_mode != ptr_ctrl_algo->prev_sel_mode  )
                            {
                                ///reset_dev_config();
                                //LOG(I,"rst_dev curr_mode %d prev_mode %d ",ptr_ctrl_algo->cur_sel_mode, ptr_ctrl_algo->prev_sel_mode );
                            }
                            else
                            {
                                LOG(I,"pp_els curr_mode %d prev_mode %d ",ptr_ctrl_algo->cur_sel_mode, ptr_ctrl_algo->prev_sel_mode );
                                if ( ( IV0AQST_AIR_QUALITY_GOOD   == ptr_ctrl_algo->curr_iaq_stat ) && 
                                    ( IV0PRST_VALID_PRESS_LEVEL  == ptr_ctrl_algo->curr_pr_stat  ) && 
                                    ( IV0MODE_AUTO               == ptr_ctrl_algo->cur_sel_mode  ) )
                                {
                                    /* Reset the timer flag before starting the timer */
                                    ptr_ctrl_algo->wait_tmr_exp = false;
                                    /* Start timer to wait for 30 min */
                                    start_wait_tmr(MIN_TO_MSEC(RV_IDLE_COND_WAIT_TIME_MIN));
                                    /* Set the state to be transfer */
                                    ptr_ctrl_algo->curr_state = INVENTILATE_STATE_WAIT_FOR_IDLE_SETTLE;
                                }
                            }
                        }
                    }
                    break;

                case INVENTILATE_STATE_PRESS_CTRL:
                    if ( ( true == ptr_ctrl_algo->data_received ) && ( IV0MODE_AUTO == ptr_ctrl_algo->cur_sel_mode ) )
                    {
                        /* Reset data received flag */
                        ptr_ctrl_algo->data_received = false;
                        /* Execute pressure control routine */
                        ptr_ctrl_algo->curr_state = press_control_routine(ptr_ctrl_algo);
                        /* Store the current value of DP */
                        ptr_ctrl_algo->prev_avg_dp_value = ptr_ctrl_algo->curr_avg_dp_value;
                        iaq_run_state = 0;
                    }
                    else if ( IV0MODE_AUTO != ptr_ctrl_algo->cur_sel_mode ) 
                    {
                        ptr_ctrl_algo->curr_state = INVENTILATE_STATE_PROCESS_PARAM;
                        LOG(I,"press mode %d",ptr_ctrl_algo->cur_sel_mode);
                    }
                    break;

                case INVENTILATE_STATE_AQ_CTRL:
                    /* 1. Air quality will be control only when the pressure inside the RV is valid or unknown
                       2. BME68X Sensor accuracy should be high ( 3 ) */
                    
                    if ( ( IV0MODE_AUTO == ptr_ctrl_algo->cur_sel_mode ) && 
                         ( ( IV0PRST_VALID_PRESS_LEVEL    == ptr_ctrl_algo->curr_pr_stat  ) || 
                           ( IV0PRST_PRESS_STATUS_UNKNOWN == ptr_ctrl_algo->curr_pr_stat  ) ) )
                    {
                        LOG(I,"[IAQ_control_state iaq_rn_st: %d accu %d curr_avg_iaq %d]",iaq_run_state, ptr_ctrl_algo->sens_acc, ptr_ctrl_algo->curr_avg_iaq_value);
                        /*
                        if( (IV0AQST_CALIBRATION_ONGOING != ptr_ctrl_algo->prev_iaq_stat ) && (IV0AQST_CALIBRATION_ONGOING == ptr_ctrl_algo->curr_iaq_stat) )
                        {
                            iaq_run_state = 0;
                        }
                        */

                        if ( ( IV0AQST_CALIBRATION_ONGOING  == ptr_ctrl_algo->curr_iaq_stat ) && (ptr_ctrl_algo->curr_avg_iaq_value > IAQ_DEF_GOOD_MAX ) )         // iaq_value  for accuracy level 0 to 3
                        { 
                            if(iaq_run_state == 0)
                            {
                                LOG(I,"[IAQ_control_start_timer");
                                ptr_ctrl_algo->wait_tmr_exp = false;
                                start_wait_tmr(MIN_TO_MSEC(IV_IAQ_ACC0_WAIT_MIN));
                                iaq_run_state = 1;
                                iaqprev_30min = *ptr_ctrl_algo->ptr_prev_data;
                                LOG(I, "30min IAQ:%d", iaqprev_30min);
                            }
                            
                            if( ( true == ptr_ctrl_algo->wait_tmr_exp ) && (iaq_run_state == 1) )
                            {
                                iaq_run_state = 2;
                                stop_wait_tmr();
                                LOG(I,"[IAQ_control_stop_timer_rs:2]");
                                *ptr_ctrl_algo->ptr_prev_data = iaqprev_30min;
                                LOG(W,"iaq_prev %d iaq30min %d", *ptr_ctrl_algo->ptr_prev_data, iaqprev_30min);
                            }
                        }
                        else
                        {
                            iaq_run_state = 2;
                            stop_wait_tmr();
                            LOG(I,"[IAQ_control_stop_timer_rs:2E]");
                            //ptr_ctrl_algo->ptr_prev_data = iaqprev_30min;
                        }

                        if(iaq_run_state == 2)
                        {
                            if ( ( true           == ptr_ctrl_algo->data_received ) && 
                                ( IV0MODE_AUTO   == ptr_ctrl_algo->cur_sel_mode  ) )
                            {
                                 LOG(I,"[IAQ_control_st:aqcr]");
                                /* Reset data received flag */
                                ptr_ctrl_algo->data_received = false;
                                /* Execute AQ control routine */
                                ptr_ctrl_algo->curr_state = aq_control_routine(ptr_ctrl_algo);
                                /* Store the current value */
                                *ptr_ctrl_algo->ptr_prev_data = *ptr_ctrl_algo->ptr_curr_data;
                            }
                        }
                    }
                    else
                    {
                        /* Set the state to be transfer */
                        ptr_ctrl_algo->curr_state = INVENTILATE_STATE_PROCESS_PARAM;
                    }
                    break;

                case INVENTILATE_STATE_PRESS_CTRL_EX_LIMIT:
                    if ( ( IV0PRST_VALID_PRESS_LEVEL   == ptr_ctrl_algo->curr_pr_stat  ) || 
                         ( ptr_ctrl_algo->wait_tmr_exp == true                         ) )
                    {
                        LOG(W, "Exit for exceed limit state");
                        stop_wait_tmr();
                        /* Reset the timer flag */
                        ptr_ctrl_algo->wait_tmr_exp = false;
                        /* Reset the dev compensation configuration */
                        reset_dev_config(); 
                        /* Set the state to be transfer */
                        ptr_ctrl_algo->curr_state = INVENTILATE_STATE_PROCESS_PARAM;
                    }
                    else if ( ptr_ctrl_algo->cur_sel_mode != ptr_ctrl_algo->prev_sel_mode  )
                    {
#if CONN_PWM_DEBUG_LOG
                            LOG(I, "Mode changed");
#endif
                        reset_dev_config();
                        ptr_ctrl_algo->curr_state = INVENTILATE_STATE_PROCESS_PARAM;
                    }
                    break;

                case INVENTILATE_STATE_WAIT_FOR_IDLE_SETTLE:
                    if ( ( ( IV0PRST_VALID_PRESS_LEVEL != ptr_ctrl_algo->curr_pr_stat  ) && ( IV0PRST_PRESS_STATUS_UNKNOWN != ptr_ctrl_algo->curr_pr_stat  ) ) ||
                         ( IAQ_DEF_GOOD_MAX < ptr_ctrl_algo->curr_avg_iaq_value  ) || 
                         ( ptr_ctrl_algo->cur_sel_mode != ptr_ctrl_algo->prev_sel_mode ) )
                    {
#if ( INVENT_HARWARE_VERSION >= HW_VERSION_4_1 )
                        // Power ON Ionizer
#ifdef EN_IONIZER_FLAG

                            if(ivsett_config.EN_DIS_IONIZER == true)
                            {
                                EN_IONIZER(1);
                                ptr_ctrl_algo->ionizer_status    = (int32_t)IV0IONST_ON;
                            }
                            else
                            {
                                EN_IONIZER(0);
                                ptr_ctrl_algo->ionizer_status    = (int32_t)IV0IONST_OFF;
                            }
#else
                                EN_IONIZER(1);
                                ptr_ctrl_algo->ionizer_status    = (int32_t)IV0IONST_ON;

#endif
#endif
                        

                        TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, IV0IONST, &ptr_ctrl_algo->ionizer_status, sizeof(int32_t), \
                                    connector_pwm_fan_motor.connector_id, portMAX_DELAY));
                        /* Stop the wait timer */
                        stop_wait_tmr();
                        /* Reset the dev compensation configuration */
                        reset_dev_config();
                        /* Reset the timer flag */
                        ptr_ctrl_algo->wait_tmr_exp = false;
                        /* Set the state to be transfer */
                        ptr_ctrl_algo->curr_state = INVENTILATE_STATE_PROCESS_PARAM; 
                    }
                    else if ( true == ptr_ctrl_algo->wait_tmr_exp )
                    {
#if ( INVENT_HARWARE_VERSION >= HW_VERSION_4_1 )
                        // Power OFF Ionizer
                        EN_IONIZER(0);
#endif
                        ptr_ctrl_algo->ionizer_status = (int32_t)IV0IONST_OFF;

                        TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, IV0IONST, &ptr_ctrl_algo->ionizer_status, sizeof(int32_t), \
                                    connector_pwm_fan_motor.connector_id, portMAX_DELAY));
                        /* 30 minutes wait timer expired and still the AQ and DP are in good range 
                           So Turn-OFF the FAN and Motor..To reduce the power consumption */
                        /* Reset the timer flag */
                        ptr_ctrl_algo->wait_tmr_exp = false;
                        /* Set zero RPM */
                        ptr_ctrl_algo->set_rpm[DEV_FAN1_AIR_IN]         = 0;
                        ptr_ctrl_algo->set_rpm[DEV_FAN2_AIR_OUT]          = 0;
                        /* Set the compensation configuration to IDLE */
                        ptr_ctrl_algo->dev_comp_config[DEV_FAN1_AIR_IN] = IDLE_COMP_DEV;
                        ptr_ctrl_algo->dev_comp_config[DEV_FAN2_AIR_OUT]  = IDLE_COMP_DEV;
                        /* Set the state to be transfer */
                        ptr_ctrl_algo->curr_state = INVENTILATE_STATE_IDLE;
                    }
                    break;

                case INVENTILATE_STATE_IDLE:
                    if ( ( ( IV0PRST_VALID_PRESS_LEVEL   != ptr_ctrl_algo->curr_pr_stat  ) && ( IV0PRST_PRESS_STATUS_UNKNOWN != ptr_ctrl_algo->curr_pr_stat ) )||
                         ( IAQ_DEF_GOOD_MAX < ptr_ctrl_algo->curr_avg_iaq_value ) || 
                         ( ptr_ctrl_algo->cur_sel_mode != ptr_ctrl_algo->prev_sel_mode ) )
                    {
                        /* Reset the dev compensation configuration */
                        reset_dev_config();
                        /* Set the state to be transfer */
                        ptr_ctrl_algo->curr_state = INVENTILATE_STATE_PROCESS_PARAM; 
                    }
                    break;

                case INVENTILATE_STATE_PROCESS_STANDBY:
                    /* Stop the periodic timer */
                    stop_periodic_timer();
                    /* Stop the wait timer */
                    stop_wait_tmr();
                    /* Stop storage timer */
                    stop_storage_timer();
                    /* Reset the timer flags */
                    ptr_ctrl_algo->wait_tmr_exp    = false;
                    ptr_ctrl_algo->per_tmr_exp     = false;
                    ptr_ctrl_algo->storage_tmr_exp = false;
                    ptr_ctrl_algo->data_received   = false;
                    ptr_ctrl_algo->change_dev      = false;
                    /* Reset the data count */
                    ptr_ctrl_algo->iaq_data_count = 0;
                    ptr_ctrl_algo->dp_data_count  = 0;
                    /* Overwrite user selected mode as OFF */
                    ptr_ctrl_algo->cur_sel_mode   = IV0MODE_OFF;

                    if ( IVPMGR0STATE_STORAGE == ptr_ctrl_algo->iv_pwr_state )
                    {
                        ptr_ctrl_algo->prev_sel_mode = ptr_ctrl_algo->cur_sel_mode;
                        /* when the user select storage mode..Then devcie will start the storage from 24 hrs sleep */
                        ptr_ctrl_algo->storage_timer_config = STORAGE_TIMER_21H;
                        /* Configure the timer based on the storage timer selction */
                        change_storage_timer_period(ptr_ctrl_algo->storage_tmr_val_ticks[ptr_ctrl_algo->storage_timer_config]);
                        /* Start the timer */
                        start_storage_timer();
                        /* Set the state to be trasfer */
                        ptr_ctrl_algo->curr_state = INVENTILATE_STATE_STORAGE;
                    }
                    else
                    {
                        /* Set the state to be trasfer */                 
                        ptr_ctrl_algo->curr_state = INVENTILATE_STATE_STANDBY;
                    }
                    break;

                case INVENTILATE_STATE_STORAGE:
                    if ( IVPMGR0STATE_ACTIVE == ptr_ctrl_algo->iv_pwr_state )
                    {
                        /* Stop storage timer */
                        stop_storage_timer();

                        ptr_ctrl_algo->storage_tmr_exp = false;

                        /* Set the state to be transfer */
                        ptr_ctrl_algo->curr_state = INVENTILATE_STATE_STANDBY; 
                    }
                    else if ( true == ptr_ctrl_algo->storage_tmr_exp )
                    {
                        /* Reset the timer expired flag */
                        ptr_ctrl_algo->storage_tmr_exp = false;

                        if ( STORAGE_TIMER_21H == ptr_ctrl_algo->storage_timer_config )
                        {
                            /* The 24 hrs timer is expired..so now run the FANs for next 3 hours */
                            ptr_ctrl_algo->cur_sel_mode          = IV0MODE_STORAGE;
                            ptr_ctrl_algo->storage_timer_config  = STORAGE_TIMER_03H;
                        }
                        else
                        {
                            /* The 3 hrs timer is expired..so turn off the fans and start the 24 hrs timer */
                            ptr_ctrl_algo->cur_sel_mode          = IV0MODE_OFF;
                            ptr_ctrl_algo->storage_timer_config  = STORAGE_TIMER_21H;
                        }
                        
                        /* Memorize the current configured mode */
                        ptr_ctrl_algo->storage_mode_ctrl = ptr_ctrl_algo->cur_sel_mode;

                        /* Set the FAN1 and FAN2 rpm */
                        if(ptr_ctrl_algo->selected_power_source == IV0PWRSRC_12V_CAR_BATTERY_INPUT)
                        {
                            /* Running in CAR battery run fan at full speed*/
                            ptr_ctrl_algo->set_rpm[DEV_FAN1_AIR_IN]  = fan_motor_control_db[DEV_FAN1_AIR_IN].rpm_range[ptr_ctrl_algo->cur_sel_mode].min_rpm;
                            ptr_ctrl_algo->set_rpm[DEV_FAN2_AIR_OUT]   = fan_motor_control_db[DEV_FAN2_AIR_OUT].rpm_range[ptr_ctrl_algo->cur_sel_mode].min_rpm;
                        }
                        else
                        {
                            /*Solar is enabled */
                            ptr_ctrl_algo->set_rpm[DEV_FAN1_AIR_IN]  = (fan_motor_control_db[DEV_FAN1_AIR_IN].rpm_range[ptr_ctrl_algo->cur_sel_mode].min_rpm)/2;
                            ptr_ctrl_algo->set_rpm[DEV_FAN2_AIR_OUT]   = (fan_motor_control_db[DEV_FAN2_AIR_OUT].rpm_range[ptr_ctrl_algo->cur_sel_mode].min_rpm)/2;
                        }
                        ptr_ctrl_algo->set_rpm[DEV_MOTOR] = 0;
                        LOG(I,"[Storage_RPM %d %d]",ptr_ctrl_algo->set_rpm[DEV_FAN1_AIR_IN],ptr_ctrl_algo->set_rpm[DEV_FAN2_AIR_OUT]);
                        /* Configure the timer based on the storage timer selection */
                        change_storage_timer_period(ptr_ctrl_algo->storage_tmr_val_ticks[ptr_ctrl_algo->storage_timer_config]);
                    }
                    else if ( ( ptr_ctrl_algo->cur_sel_mode != ptr_ctrl_algo->storage_mode_ctrl ) && ( ptr_ctrl_algo->cur_sel_mode != IV0MODE_OFF ) )
                    {
                        /* User mode selection will not be apply while in storage mode..
                           Overwrite user selected mode */
                        ptr_ctrl_algo->cur_sel_mode = ptr_ctrl_algo->storage_mode_ctrl;
                    }
                    else
                    {

                    }
                    break;

                case INVENTILATE_STATE_VALIDATE_DEV_STATUS:
                    break;
                    
                default:
                    LOG(E, "err uhst=%d", ptr_ctrl_algo->curr_state);
                    break;
            }

            /* Update the calculated RPM */
            update_dev_rpm(ptr_ctrl_algo, ptr_ctrl_algo->cur_sel_mode);
            if ( ptr_ctrl_algo->cur_sel_mode != ptr_ctrl_algo->prev_sel_mode )
            {
                ptr_ctrl_algo->prev_sel_mode = ptr_ctrl_algo->cur_sel_mode;
            }
        
            if ( ptr_ctrl_algo->curr_state != ptr_ctrl_algo->prev_state )
            {
                change_state(ptr_ctrl_algo->curr_state);
                /* Memorize the current state */
                ptr_ctrl_algo->prev_state = ptr_ctrl_algo->curr_state;
            }
#endif
        }

    }
}
//#endif

#ifndef CONNECTOR_EOL_SERVICE
/**
  * @brief  Function to process the newly received data from the queue
  * @retval none.
  */
void parse_received_data(void)
{
	switch ( ptr_ctrl_algo->iv_data.data_id ) 
    {
        case IV_STATE_CHANGE:
            //LOG(W, "iv_state changed = %d", dframe->data);
            ptr_ctrl_algo->curr_state = ptr_ctrl_algo->iv_data.data;                 /* Change the inventilate control state machine state */
            break;

        case IV_PWR_STATE:
            ptr_ctrl_algo->iv_pwr_state = ptr_ctrl_algo->iv_data.data;               /* Get the newly requested power mode */

            if ( ( IVPMGR0STATE_STANDBY == ptr_ctrl_algo->iv_pwr_state ) || ( IVPMGR0STATE_STORAGE == ptr_ctrl_algo->iv_pwr_state ) )
            { 
                ptr_ctrl_algo->curr_state          = INVENTILATE_STATE_PROCESS_STANDBY;
                ptr_ctrl_algo->prev_state          = INVENTILATE_STATE_PROCESS_STANDBY;
                
#if ( INVENT_HARWARE_VERSION >= HW_VERSION_4_1 )
                // Power OFF Ionizer when the inventilate is powered OFF by user 
                EN_IONIZER(0);
#endif
                ptr_ctrl_algo->ionizer_status = (int32_t)IV0IONST_OFF;

                TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, IV0IONST, &ptr_ctrl_algo->ionizer_status, sizeof(int32_t), \
                        connector_pwm_fan_motor.connector_id, portMAX_DELAY));
            }
            break;

        case DP_SENSOR_STATUS:
            ptr_ctrl_algo->dp_senor_status = (DP_SENS_STATUS)ptr_ctrl_algo->iv_data.data;
            break;

        case IAQ_DATA: 
            ptr_ctrl_algo->iaq_data_count++;                                         /* Increment the data counter */
            ptr_ctrl_algo->accum_iaq_value += ptr_ctrl_algo->iv_data.data; 
            break;

        case IV_HUMIDITY_DATA:
            ptr_ctrl_algo->humidity_data_count++;
            ptr_ctrl_algo->curr_avg_hum_value += ptr_ctrl_algo->iv_data.data; 
            break;

        case IV_VOC_SENSOR_ACC:
            ptr_ctrl_algo->sens_acc = ptr_ctrl_algo->iv_data.data;                   /* Update the VOC sensor accuracy status */
            break;
        
        case IV_DP_BATTERY_LEVEL:
                LOG(I,"[DP_Battery_level_data_received %d]",ptr_ctrl_algo->iv_data.data); 
            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, IV0ERRST, &ptr_ctrl_algo->fan_mtr_dev_curr_stat, sizeof(int32_t), \
                                   connector_pwm_fan_motor.connector_id, portMAX_DELAY));
            
            if (ptr_ctrl_algo->iv_data.data < DP_BAT_LIMIT)
            {
                //Send Error Code for DP sensor board.
                TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, IV0ERRST, &ptr_ctrl_algo->fan_mtr_dev_curr_stat, sizeof(int32_t), \
                                   connector_pwm_fan_motor.connector_id, portMAX_DELAY));

            }
            break;

        case DP_DATA: 
            //Check if the value is within valid range -25 to +25 (-25000 to +25000)
            if ( ( (ptr_ctrl_algo->iv_data.data < DPSENS_PLAUSIBLE_UPRANGE) && (ptr_ctrl_algo->iv_data.data > 0) ) || \
                 ( (ptr_ctrl_algo->iv_data.data > DPSENS_PLAUSIBLE_DOWNRANGE) && (ptr_ctrl_algo->iv_data.data < 0) ) )
            {
                ptr_ctrl_algo->iv_data.data = ( ptr_ctrl_algo->iv_data.data != 0 ) ? ( ptr_ctrl_algo->iv_data.data / ptr_ctrl_algo->dp_resol_factor ) : 0;
                ptr_ctrl_algo->curr_avg_dp_value += ptr_ctrl_algo->iv_data.data;        /* Add the new value with previous value */
                ptr_ctrl_algo->dp_data_count++;                                         /* Increment the data counter */
            }
            break;

        case IV_MODE:
            ptr_ctrl_algo->cur_sel_mode = ptr_ctrl_algo->iv_data.data;             /* Get the user selected mode */
            break;

        case IV_PER_TMR_EXP:
            ptr_ctrl_algo->per_tmr_exp = true;                                      /* Timer expired */
            break;

        case IV_RPM_STEP_LVL1:                                                      //Set RPM in steps                   
         //fall through
       case IV_RPM_STEP_LVL2:                                                      
        //fall through
        case IV_RPM_STEP_LVL3:                                                      
        //fall through
        case IV_RPM_STEP_LVL4:                                                      
        //fall through
        case IV_RPM_STEP_LVL5:
            /* Set the rpm step */
            set_iv_rpm_step_level((ptr_ctrl_algo->iv_data.data_id - IV_RPM_STEP_LVL1), ptr_ctrl_algo->iv_data.data);
            break;

        case IV_FAN1_RPM_MIN:                                                       //Update minimum level RPM for Fan1 in NVS 
        //fall through
        case IV_FAN2_RPM_MIN:                                                       //Update minimum level RPM for Fan2 in NVS
        //fall through
        case IV_MTR_RPM_MIN:                                                        //Update minimum level RPM for Motor in NVS
        //fall through
        case IV_FAN1_RPM_MAX:                                                       //Update maximum level RPM for Fan1 in NVS
        //fall through
        case IV_FAN2_RPM_MAX:                                                       //Update maximum level RPM for Fan2 in NVS
        //fall through
        case IV_MTR_RPM_MAX:                                                        //Update maximum level RPM for Motor in NVS
            /* Update the min RPM range database table */
            update_data_in_nvm(ptr_ctrl_algo->iv_data.data_id, ptr_ctrl_algo->iv_data.data);
            /* Calc and update min and max rpm corresponding to mode */
            calc_mode_min_max_rpm();
            break;

        case IV_FAN1_TACHO:
        //fall through
        case IV_FAN2_TACHO:
        //fall through
        case IV_MTR_TACHO:
#if CONN_PWM_DEBUG_LOG        
            LOG(I, "Tacho read dev%d = %d", ptr_ctrl_algo->iv_data.data_id - IV_FAN1_TACHO, ptr_ctrl_algo->iv_data.data);
#endif            
            ptr_ctrl_algo->dev_tacho[ptr_ctrl_algo->iv_data.data_id - IV_FAN1_TACHO] = ptr_ctrl_algo->iv_data.data;
            break;

        case IV_IAQ_GOOD_MIN:                                                       //Update IAQ Good mimimum level in NVS
        //fall through
        case IV_IAQ_FAIR_MIN:                                                       //Update IAQ Fair minimum level in NVS
        //fall through
        case IV_IAQ_BAD_MIN:                                                        //Update IAQ Bad minimum level in NVS
        //fall through
        case IV_IAQ_GOOD_MAX:                                                       //Update IAQ Good maximum level in NVS
        //fall through
        case IV_IAQ_FAIR_MAX:                                                       //Update IAQ Fair maximum level in NVS
        //fall through
        case IV_IAQ_BAD_MAX:                                                        //Update IAQ Bad maximum level in NVS
            /* Update the received IAQ range in the NVM and variables */
            update_data_in_nvm(ptr_ctrl_algo->iv_data.data_id, ptr_ctrl_algo->iv_data.data);
            break;

        case IV_FAN1_SET_RPM:                                                       //Set FAN1 RPM
        //fall through
        case IV_FAN2_SET_RPM:                                                       //Set FAN2 RPM    
        //fall through
        case IV_MTR_SET_RPM:                                                        //Set MOTOR RPM
            /* set the requested RPM to the corresponding device */
            set_fan_motor_rpm((invent_device_id_t)(ptr_ctrl_algo->iv_data.data_id - IV_FAN1_SET_RPM), ptr_ctrl_algo->iv_data.data);
            break;

        case IV_ONE_SHOT_TMR_EXP:
            ptr_ctrl_algo->wait_tmr_exp = true;
            break;

        case IV_STORAGE_TMR_EXP:
            ptr_ctrl_algo->storage_tmr_exp = true;
            break;

        case IV_ERROR_STATUS:
            ptr_ctrl_algo->invent_error_status = ptr_ctrl_algo->iv_data.data;
            break;

        case IV_IVSETT:
            /*IV set config Enable/Disable Solar*/
            break;
        case IV_POWER_SOURCE:
            /*Active power source*/
            LOG(I,"[Act_pwr_src %d ]",ptr_ctrl_algo->iv_data.data) 
            ptr_ctrl_algo->selected_power_source = ptr_ctrl_algo->iv_data.data;
            break;

        case INVALID_DATA_RECEIVED:
            /* Skip it*/                                      
            break;

        default:
            LOG(E, "Invalid data id = %d", ptr_ctrl_algo->iv_data.data_id);
            break;
    }
}
#endif

/**
  * @brief  Function to process the set and publish parameter from broker
  * @param  DDM parameter.
  * @retval none.
  */
static void process_set_and_publish_request(uint32_t ddm_param, int32_t i32value, DDMP2_CONTROL_ENUM req_type)
{
    int32_t i32Index;
    int32_t i32Factor;
    int32_t pub_value = i32value;
	uint16_t db_idx;
	conn_fan_motor_parameter_t* param_db;

#if CONN_PWM_DEBUG_LOG
    LOG(I, "Received ddm_param = 0x%x i32value = %d", ddm_param, i32value);
#endif

    if ( SDP0DP == ddm_param )
    {
        LOG(I, "ddmp 0x%x Received DP data i32value = %d", ddm_param ,i32value);
    }

	/* Validate the DDM parameter received */
	db_idx = get_ddm_index_from_db(ddm_param);
 
	if ( DDMP_UNAVAILABLE != db_idx )
	{
        if ( DDMP2_CONTROL_SET == req_type )
        {
                /* Frame and send the publish request */
                TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, ddm_param, &pub_value, sizeof(int32_t), \
                            connector_pwm_fan_motor.connector_id, portMAX_DELAY));
        }

		param_db = &conn_fan_motor_param_db[db_idx];

        i32Index = ddm2_parameter_list_lookup(DDM2_PARAMETER_BASE_INSTANCE(ddm_param));

        if ( -1 != i32Index )
        {
            i32Factor = Ddm2_unit_factor_list[Ddm2_parameter_list_data[i32Index].in_unit];
            
            /* Differential pressure value should be handle with the decimal point resolution 
               For example 1.24 (pascal) value will be received as 1240 (i.e 1.24 * 1000 )
               So if we again divide this by factor(1000) then the resolution could be lost, So
               to avoid this, Divide by factor operation should be skipped for IVSDP0DP and SDP0DP
            */
            i32Factor = ( ( i32Factor == 0 ) || ( SDP0DP == ddm_param ) ) ? 1 : i32Factor;

            i32value  = i32value / i32Factor;  /* Divide by factor */
            if ( i32value != param_db->i32Value )
            {
                /* Update the received value in the database table*/
	  	        param_db->i32Value = i32value;
		        /* Check callback function registered for this DDM parameter */
		        if ( NULL != param_db->cb_func )
		        {
                    /* Execute the callback function */
                    param_db->cb_func(ddm_param, i32value);
		        }
            }
        }
	}
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
	conn_fan_motor_parameter_t* param_db;
    uint32_t list_value = 0;
    SORTED_LIST_RETURN_VALUE ret = sorted_list_unique_get(&list_value, &conn_pwm_fan_mtr_table, ddm_param, 0);

#if CONN_PWM_DEBUG_LOG
    LOG(I, "Received ddm_param = 0x%x ret = %d", ddm_param, ret);
#endif

    if ( SORTED_LIST_FAIL != ret )
	{
		/* Validate the DDM parameter received */
		db_idx = get_ddm_index_from_db(ddm_param);

		if ( DDMP_UNAVAILABLE != db_idx )
	  	{
			param_db = &conn_fan_motor_param_db[db_idx];

            index = ddm2_parameter_list_lookup(DDM2_PARAMETER_BASE_INSTANCE(ddm_param));

            if ( -1 != index )
			{
                factor = Ddm2_unit_factor_list[Ddm2_parameter_list_data[index].out_unit];

                if ( factor == 0 )
                {
                    factor = 1;
                }
                
                /* Multiply with the factor */
                value = param_db->i32Value * factor;
                /* Frame and send the publish request */
                TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, ddm_param, &value, sizeof(int32_t), connector_pwm_fan_motor.connector_id, portMAX_DELAY));
            }
		}
	}
	else
	{
		LOG(I, "sorted_list_key_search ddm_param 0x%x not found", ddm_param);
	}
}

/**
  * @brief  Update the new value in database table and publish to broker
  * @param  ddm_parameter.
  * @param  i32Value.
  * @retval none.
  */
void update_and_send_val_to_broker(uint32_t ddm_parameter, int32_t value)
{
	conn_fan_motor_parameter_t* param_db;
    uint8_t db_idx = get_ddm_index_from_db(ddm_parameter);
	int index;
    int32_t factor_value = 0;
    int factor;
    uint32_t ddmp = 0;

#if CONN_PWM_DEBUG_LOG
    LOG(I, "ddm_parameter = 0x%x value = %d", ddm_parameter, value);
#endif

	if ( DDMP_UNAVAILABLE != db_idx )
   	{
		param_db = &conn_fan_motor_param_db[db_idx];

		/* Update the value in db table */
		param_db->i32Value = value;
        
        index = ddm2_parameter_list_lookup(DDM2_PARAMETER_BASE_INSTANCE(ddm_parameter));

        if ( -1 != index )
		{
            factor = Ddm2_unit_factor_list[Ddm2_parameter_list_data[index].out_unit];
            
            if ( factor == 0 )
            {
                factor = 1;
            }
                
            /* Multiply with the factor */
            factor_value = param_db->i32Value * factor;
            ddmp = DDM2_PARAMETER_BASE_INSTANCE(ddm_parameter);

            if ( ( MTR0TACHO == ddmp ) && ( NULL != param_db->cb_func ) )
            {
                /* Execute the callback function */
                param_db->cb_func(ddm_parameter, param_db->i32Value);
            }
            /* Frame and send the publish request */
            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, ddm_parameter, &factor_value, \
                       sizeof(int32_t), connector_pwm_fan_motor.connector_id, portMAX_DELAY));
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
	conn_fan_motor_parameter_t* param_db;
	uint8_t db_idx = DDMP_UNAVAILABLE; 
	uint8_t index;
	bool avail = false;

	for ( index = 0u; ( ( index < conn_fanmot_db_elements ) && ( avail == false ) ); index ++ )
 	{
		param_db = &conn_fan_motor_param_db[index];
      	
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
  * @brief  Callback function to handle all the subscribed parameters.
  *         Few parameters are used for algorithm
  * @param  ddm_param - DDM Parameter with instance
  * @param  data      - Value for the DDM Parameter
  * @retval none
  */
static void handle_invsub_data(uint32_t ddm_param, int32_t data)
{
    IV_DATA iv_data;
    uint32_t dp_plausible_errst = 0;

    /* set the data */
    iv_data.data = data;

    switch (ddm_param)
    {
        case SBMEB0IAQ:
            iv_data.data_id = IAQ_DATA;
            break;

        case SBMEB0HUM:
            iv_data.data_id = IV_HUMIDITY_DATA;
            bme68x_humid_value =iv_data.data;
            break;

        case SDP0AVL:
            
            if ( DP_SENSOR_AVAILABLE == iv_data.data )/* Check the DP sensor avalabity status */
            {
                LOG(W, "Subscribe request for DDMP SDP0DP");
                // Whenever the availability of sensor node received then the subscribtion should be done again
                TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SUBSCRIBE, SDP0DP, NULL, 0, connector_pwm_fan_motor.connector_id, portMAX_DELAY));

                // Whenever the availability of sensor node received then the subscribtion should be done again
                TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SUBSCRIBE, SNODE0BATTLVL, NULL, 0, connector_pwm_fan_motor.connector_id, portMAX_DELAY));
            }
            else if ( DP_SENSOR_NOT_AVAILABLE == iv_data.data )
            {
                LOG(W, "Subscribe request for DDMP SDP0AVL");
                // Whenever the availability of sensor node received then the subscribtion should be done again
                TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SUBSCRIBE, SDP0AVL, NULL, 0, connector_pwm_fan_motor.connector_id, portMAX_DELAY));

                LOG(W, "Subscribe request for DDMP SNODE0AVL");
                // Whenever the availability of sensor node received then the subscribtion should be done again
                TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SUBSCRIBE, SNODE0AVL, NULL, 0, connector_pwm_fan_motor.connector_id, portMAX_DELAY));
            }
            else
            {
                
            }

            iv_data.data_id = DP_SENSOR_STATUS;
            break;

        case SDP0DP:

            iv_data.data_id = DP_DATA;
            if ((iv_data.data > DPSENS_PLAUSIBLE_UPRANGE) && (iv_data.data > 0))
            {
                dp_plausible_errst |= 1 << DP_SENSOR_DATA_PLAUSIBLE_ERROR;
                TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, IV0ERRST, &dp_plausible_errst, sizeof(int32_t), \
                connector_pwm_fan_motor.connector_id, portMAX_DELAY));
#ifdef DP_ERR_DEBUG
                LOG(I, "DP PLAUSIBLE ERRSET OVER= %d", iv_data.data);
#endif
            }
            else if ((iv_data.data < DPSENS_PLAUSIBLE_DOWNRANGE) && (iv_data.data < 0))
            {
                dp_plausible_errst |= 1 << DP_SENSOR_DATA_PLAUSIBLE_ERROR;
                TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, IV0ERRST, &dp_plausible_errst, sizeof(int32_t), \
                connector_pwm_fan_motor.connector_id, portMAX_DELAY));
#ifdef DP_ERR_DEBUG
                LOG(I, "DP PLAUSIBLE ERRSET UNDER= %d", iv_data.data);
#endif
            }
            else if (((iv_data.data > 0) && (iv_data.data < DPSENS_PLAUSIBLE_UPRANGE)) || ((iv_data.data < 0) && (iv_data.data > DPSENS_PLAUSIBLE_DOWNRANGE)))
            {
                dp_plausible_errst &= ~ ( 1 <<  DP_SENSOR_DATA_PLAUSIBLE_ERROR );
                TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, IV0ERRST, &dp_plausible_errst, sizeof(int32_t), \
                connector_pwm_fan_motor.connector_id, portMAX_DELAY));
#ifdef DP_ERR_DEBUG
                LOG(I, "DP PLAUSIBLE VALID = %d", iv_data.data);
#endif
            }
#ifdef DP_ERR_DEBUG
            LOG(I, "DP DATA = %d", iv_data.data);
#endif
            break;
        case SBMEB0AQR:
            iv_data.data_id = IV_VOC_SENSOR_ACC;
            break;

        case IV0MODE:
            iv_data.data_id = IV_MODE;
            break;

        case IVPMGR0STATE:
            iv_data.data_id = IV_PWR_STATE;
            break;

        case MTR0SETSPD|DDM2_PARAMETER_INSTANCE(0):
            iv_data.data_id = IV_FAN1_SET_RPM;
            break;

        case MTR0SETSPD|DDM2_PARAMETER_INSTANCE(1):
            iv_data.data_id = IV_FAN2_SET_RPM;
            break;

        case MTR0SETSPD|DDM2_PARAMETER_INSTANCE(2):
            iv_data.data_id = IV_MTR_SET_RPM;
            break;

        case MTR0MINSPD|DDM2_PARAMETER_INSTANCE(0):
            iv_data.data_id = IV_FAN1_RPM_MIN;
            break;

        case MTR0MINSPD|DDM2_PARAMETER_INSTANCE(1):
            iv_data.data_id = IV_FAN2_RPM_MIN;
            break;

        case MTR0MINSPD|DDM2_PARAMETER_INSTANCE(2):
            iv_data.data_id = IV_MTR_RPM_MIN;
            break;

        case MTR0MAXSPD|DDM2_PARAMETER_INSTANCE(0):
            iv_data.data_id = IV_FAN1_RPM_MAX;
            break;

        case MTR0MAXSPD|DDM2_PARAMETER_INSTANCE(1):
            iv_data.data_id = IV_FAN2_RPM_MAX;
            break;

        case MTR0MAXSPD|DDM2_PARAMETER_INSTANCE(2):
            iv_data.data_id = IV_MTR_RPM_MAX;
            break;

        case MTR0TACHO|DDM2_PARAMETER_INSTANCE(0):
            iv_data.data_id = IV_FAN1_TACHO;
            break;

        case MTR0TACHO|DDM2_PARAMETER_INSTANCE(1):
            iv_data.data_id = IV_FAN2_TACHO;
            break;

        case MTR0TACHO|DDM2_PARAMETER_INSTANCE(2):
            iv_data.data_id = IV_MTR_TACHO;
            break;

        case IVAQR0MIN|DDM2_PARAMETER_INSTANCE(0):
            iv_data.data_id = IV_IAQ_GOOD_MIN;
            break;

        case IVAQR0MIN|DDM2_PARAMETER_INSTANCE(1):
            iv_data.data_id = IV_IAQ_FAIR_MIN;
            break;

        case IVAQR0MIN|DDM2_PARAMETER_INSTANCE(2):
            iv_data.data_id = IV_IAQ_BAD_MIN;
            break;

        case IVAQR0MAX|DDM2_PARAMETER_INSTANCE(0):
            iv_data.data_id = IV_IAQ_GOOD_MAX;
            break;

        case IVAQR0MAX|DDM2_PARAMETER_INSTANCE(1):
            iv_data.data_id = IV_IAQ_FAIR_MAX;
            break;

        case IVAQR0MAX|DDM2_PARAMETER_INSTANCE(2):
            iv_data.data_id = IV_IAQ_BAD_MAX;
            break;

        case IV0ERRST:
            iv_data.data_id = IV_ERROR_STATUS;
            break;

        case IV0SETT:
            iv_data.data_id = IV_IVSETT;
            break;
        case IV0PWRSRC:
            iv_data.data_id = IV_POWER_SOURCE;
            break;            
        default:
            iv_data.data_id = INVALID_DATA_RECEIVED;
            break;
    }

    push_data_in_queue(iv_data.data, iv_data.data_id);
}
 
/**
  * @brief  Initialize dev fan and motor
  * @param  none.
  * @retval none.
  */
void initialize_fan_motor(void)
{
    // Get config from application config
    hal_pwm_init_gpio(CONNECTOR_PWM_FAN_MOTOR_CAPTURE_UNIT);
    gpio_pulldown_en(MCPWM_UNIT_0_GPIO_CAP0_IN);                                      //Enable pull down on CAP0   signal
    gpio_pulldown_en(MCPWM_UNIT_0_GPIO_CAP1_IN);                                      //Enable pull down on CAP1   signal
    gpio_pulldown_en(MCPWM_UNIT_0_GPIO_CAP2_IN);                                      //Enable pull down on CAP2   signal

    pulse_count_per_revol[DEV_FAN1_AIR_IN] = CONNECTOR_PWM_FAN_MOTOR_CAPTURE_CHANNEL_0_0;
    pulse_count_per_revol[DEV_FAN2_AIR_OUT] = CONNECTOR_PWM_FAN_MOTOR_CAPTURE_CHANNEL_0_1;
    pulse_count_per_revol[DEV_MOTOR] = CONNECTOR_PWM_FAN_MOTOR_CAPTURE_CHANNEL_0_2;

	hal_pwm_init(CONNECTOR_PWM_FAN_MOTOR_CAPTURE_UNIT);
    /* Turn OFF the FAN and Motor at start of the system */
    hal_pwm_capture_enable(CONNECTOR_PWM_FAN_MOTOR_CAPTURE_UNIT, DEV_FAN1_AIR_IN, pwm_cap_isr_cb);
    hal_pwm_set_duty_cycle(CONNECTOR_PWM_FAN_MOTOR_CAPTURE_UNIT, DEV_FAN1_AIR_IN, 100);  /* FAN1 and FAN2 need inverted PWM signal */
    hal_pwm_capture_enable(CONNECTOR_PWM_FAN_MOTOR_CAPTURE_UNIT, DEV_FAN2_AIR_OUT, pwm_cap_isr_cb);
    hal_pwm_set_duty_cycle(CONNECTOR_PWM_FAN_MOTOR_CAPTURE_UNIT, DEV_FAN2_AIR_OUT,  100);
    hal_pwm_capture_enable(CONNECTOR_PWM_FAN_MOTOR_CAPTURE_UNIT, DEV_MOTOR, pwm_cap_isr_cb);
    hal_pwm_set_duty_cycle(CONNECTOR_PWM_FAN_MOTOR_CAPTURE_UNIT, DEV_MOTOR,          0);  /* Motor need non-inverted PWM signal */
}

/**
  * @brief  Timer callback function for inventilate control algorithm
  * @param  none.
  * @retval none.
  */
void iv_periodic_tmr_cb( TimerHandle_t xTimer )
{
    push_data_in_queue(0, IV_PER_TMR_EXP);
}

/**
  * @brief  One shot timer callback function
  * @param  none.
  * @retval none.
  */
void iv_one_shot_tmr_cb( TimerHandle_t xTimer )
{
    push_data_in_queue(0, IV_ONE_SHOT_TMR_EXP);
}

/**
  * @brief  Function to add the inventilate state changed request in queue 
  * @param  iv_ctrl_state State to be transfer.
  * @retval none.
  */
void change_state(INVENT_CONTROL_STATE iv_ctrl_state)
{
    IV_DATA iv_data;
    iv_data.data    = iv_ctrl_state;
    iv_data.data_id = IV_STATE_CHANGE;

    // Append the data in the queue
    osal_base_type_t ret = xQueueSendToFront (iv_ctrl_que_handle, &iv_data, portMAX_DELAY);

    if ( osal_success != ret )
    {
        LOG(E, "Queue error ret = %d", ret);
    }
}

/**
  * @brief  Function to push the data frame into the queue
  * @param  data.
  * @retval data_id refer the enum DATA_ID.
  */
void push_data_in_queue(int32_t data, DATA_ID data_id)
{
    IV_DATA iv_data;
    iv_data.data    = data;
    iv_data.data_id = data_id;

    // Append the data in the queue
    osal_base_type_t ret = osal_queue_send (iv_ctrl_que_handle, &iv_data, portMAX_DELAY);

    if ( osal_success != ret )
    {
        LOG(E, "Queue error ret = %d", ret);
    }
}

/**
  * @brief  Function to start the timer
  * @param  none.
  * @retval none.
  */
void start_periodic_timer(void)
{
    osal_ubase_type_t peridic_tmr = xTimerStart( peridic_tmr_hdle, portMAX_DELAY );
	
    if ( peridic_tmr != pdPASS )
    {
		LOG(E, "peridic_tmr failed to start");
    }
}

/**
  * @brief  Function to stop the timer
  * @param  none.
  * @retval none.
  */
void stop_periodic_timer(void)
{
    osal_ubase_type_t  peridic_tmr = xTimerStop( peridic_tmr_hdle, portMAX_DELAY );
	
    if ( peridic_tmr != pdPASS )
    {
		LOG(E, "peridic_tmr failed to stop");
    }
}

/**
  * @brief  Function to start the timer
  * @param  none.
  * @retval none.
  */
void start_wait_tmr(uint32_t period_ms)
{
    osal_ubase_type_t ret;

    if ( tmr_period != period_ms )
    {
        /* Store the new period */
        tmr_period = period_ms;

        /* Change timer period */
        ret = xTimerChangePeriod( one_shot_tmr_hdle, pdMS_TO_TICKS(period_ms), portMAX_DELAY );
        
        if ( ret != pdPASS )
        {
		    LOG(E, "st err=%d", ret);
        }  
    }
    else
    {
        /* Start timer */
        ret = xTimerStart( one_shot_tmr_hdle, portMAX_DELAY );
        
        if ( ret != pdPASS )
        {
            LOG(E, "err stop=%d", ret);
        }
    }
}

/**
  * @brief  Function to stop the wait timer
  * @param  none.
  * @retval none.
  */
void stop_wait_tmr(void)
{
    osal_ubase_type_t  ret = xTimerStop( one_shot_tmr_hdle, portMAX_DELAY );
	
    if ( ret != pdPASS )
    {
		LOG(E, "press_cmp_tmr failed to stop");
    }
}

/**
  * @brief  Function to change the time period for storage mode timer
  * @param  change_period.
  * @retval none.
  */
static void change_storage_timer_period(TickType_t time_period_ticks)
{
    osal_ubase_type_t xStorageTimerChange;  
    int32_t i32_ddmp_value = 0;

    xStorageTimerChange = xTimerChangePeriod( xStorageTimer, time_period_ticks, portMAX_DELAY);

    if ( xStorageTimerChange != pdPASS )
    {
		LOG(E, "Storage Timer period change failed");
    }
    if(time_period_ticks == pdMS_TO_TICKS(MIN_TO_MSEC(STORAGE_MODE_RUN_TIME_03_HR)))
    {
        i32_ddmp_value =IV0STGT_RUN_3HRS;// IV0STORAGE_ACTIVE_3HOURS;
        TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, IV0STGT, &i32_ddmp_value, \
                            sizeof(int32_t), connector_pwm_fan_motor.connector_id, portMAX_DELAY));
        storage_humid_timer = 0;
        xStorageTimerChange = xTimerStart( xStorageHumid_chk_Timer, portMAX_DELAY );
        
        if ( xStorageTimerChange != pdPASS )
        {
            LOG(E, "Storage humid chk Timer start failed");
        }
    }
}

/**
  * @brief  Function to start the storage mode timer
  * @param  none.
  * @retval none.
  */
static void start_storage_timer(void)
{
    osal_ubase_type_t  xStorageTimerStarted;

    xStorageTimerStarted = xTimerStart( xStorageTimer, portMAX_DELAY );
	
    if ( xStorageTimerStarted != pdPASS )
    {
		LOG(E, "Storage Timer start failed");
    }
}

/**
  * @brief  Function to stop the storage mode timer
  * @param  none.
  * @retval none.
  */
static void stop_storage_timer(void)
{
    osal_ubase_type_t   xStorageTimerStopped;
    int32_t i32_ddmp_value = 0;

    xStorageTimerStopped = xTimerStop( xStorageTimer, portMAX_DELAY );
	
    if ( xStorageTimerStopped != pdPASS )
    {
		LOG(E, "Storage Timer stop failed");
    }

    xStorageTimerStopped = xTimerStop( xStorageHumid_chk_Timer, portMAX_DELAY );
	
    if ( xStorageTimerStopped != pdPASS )
    {
		LOG(E, "Storage humid check Timer stop failed");
    }

    i32_ddmp_value = IV0STGT_IDLE_21HRS;
    TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, IV0STGT, &i32_ddmp_value, \
                        sizeof(int32_t), connector_pwm_fan_motor.connector_id, portMAX_DELAY));

}

/**
  * @brief  Timer callback function handle the storge timer expire event
  * @param  none.
  * @retval none.
  */
static void storage_cb_func( TimerHandle_t xTimer )
{
    IV_DATA iv_data;
    iv_data.data    = 0;
    iv_data.data_id = IV_STORAGE_TMR_EXP;

    LOG(W, "Storage Time elapsed");

    // Append the STORAGE_TMR_EXP data in the Queue
    osal_base_type_t ret = osal_queue_send (iv_ctrl_que_handle, &iv_data, 0);

    if ( osal_success != ret )
    {
        LOG(E, "Queue error ret = %d", ret);
    }
}

/*
This function is called every 1 minute from timer interrupt 
Timer handler:xStorageHumid_chk_Timer
*/
static void storage_humid_chk_cb_func(TimerHandle_t xTimer)
{
    IV_DATA iv_data;
    iv_data.data    = 0;
    iv_data.data_id = IV_STORAGE_HUMID_CHK_TMR_EXP;
    storage_humid_timer++;
#if CONN_PWM_DEBUG_LOG    
    LOG(C,"[Storage_tmr_count %d present humid %d initial_humid %d curr %d]",storage_humid_timer,bme68x_humid_value, bm_humid_prev,bm_humid_curr);
#endif
    if(storage_humid_timer == STORAGE_RUN_START_TIME)
    {
        bm_humid_prev = bme68x_humid_value;
        bm_humid_curr = bme68x_humid_value;
#if CONN_PWM_DEBUG_LOG        
        LOG(C,"[humid_init_value = %d]",bm_humid_prev);
#endif
    }
    else if(storage_humid_timer >= STORAGE_RUN_END_TIME)
    {
        bm_humid_curr = bme68x_humid_value;
#if CONN_PWM_DEBUG_LOG
        LOG(I,"[humid_prev %d  hum_curr %d]",bm_humid_prev,bm_humid_curr);
#endif        
        if((bm_humid_curr - bm_humid_prev ) >= 10)
        {
            //Stop fan
#if CONN_PWM_DEBUG_LOG
            LOG(C,"[humid value increase. stop FAN1 and FAN2]");
#endif            
            iv_data.data = 0;
            iv_data.data_id = IV_FAN1_SET_RPM;
            push_data_in_queue(iv_data.data, iv_data.data_id);
            iv_data.data = 0;
            iv_data.data_id = IV_FAN2_SET_RPM;
            push_data_in_queue(iv_data.data, iv_data.data_id);
        }
        storage_humid_timer = 0;

    }
#if CONN_PWM_DEBUG_LOG    
    LOG(I,"[Storage_chk_ tmr %d humid present %d initial_humid %d, curr %d]",storage_humid_timer,bme68x_humid_value, bm_humid_prev,bm_humid_curr);
#endif
}

static bool pwm_cap_isr_cb(uint8_t unit, uint8_t capture_signal, uint32_t value)
{
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
    bool yield = false;

    // Increase counter to indicate we have a valid interrupt
    capture_counter[capture_signal] = capture_counter[capture_signal] + 1u;
    // Calculate time since last interrupt. Need to handle overflow.
    time_now[capture_signal]     = xTaskGetTickCountFromISR();
    time_diff[capture_signal]    = time_now[capture_signal] - last_tick[capture_signal];
    time_diff_ms = pdTICKS_TO_MS(time_diff[capture_signal]);

    if ( time_diff_ms >= TACHO_SEND_MIN_INTERNAL_MS )
    {
        if (capture_counter[capture_signal] != previous_cap_value[capture_signal])
        {
            /* To taken care of overflow of pulse counter in Interrupt handler */
            if (capture_counter[capture_signal] >= previous_cap_value[capture_signal])
            {
                current_cap_value[capture_signal] = capture_counter[capture_signal] - previous_cap_value[capture_signal];
            }
            else
            {
                current_cap_value[capture_signal] = (HAL_PWM_CAP_COUNTER_LIMIT - previous_cap_value[capture_signal]) + capture_counter[capture_signal];
            }

            // Update the previous capture value	
            previous_cap_value[capture_signal] = capture_counter[capture_signal];

            // To prevent divide by zero
            if (current_cap_value[capture_signal] > 0)
            {
                // Find rpm from the pulse per second
                dev_capt[capture_signal].dev_rpm  = (current_cap_value[capture_signal]  * NUM_SECONDS_PER_MINUTE) / pulse_count_per_revol[capture_signal];
                dev_rpm[capture_signal] = dev_capt[capture_signal].dev_rpm;
            }
        }
        // Send tacho event MTR0_NEW_TACHO_DATA_EVENT
        tacho_event.capture_signal = capture_signal;
        tacho_event.capture_info.dev_rpm = dev_capt[capture_signal].dev_rpm;
        ddmp2_create_set(&frame_event_to_send, MTR0_NEW_TACHO_DATA_EVENT, (const void*)&tacho_event, sizeof(tacho_event), connector_pwm_fan_motor.connector_id);
        if (pdFALSE == xRingbufferSendFromISR(connector_pwm_fan_motor.to_connector, &frame_event_to_send, ddmp2_full_frame_size(DDMP2_CONTROL_SET, sizeof(tacho_event)), &pxHigherPriorityTaskWoken))
        {
            /* Do Nothing */
        }
        // Check if we need to yield in caller function
        if (pxHigherPriorityTaskWoken == pdTRUE)
        {
            yield = true;
        }
        // Reset to handle case when no tacho has been received for a long time
        dev_capt[capture_signal].dev_rpm = 0;
		// Save last tick value for sending tacho value
		last_tick[capture_signal] = time_now[capture_signal];
    }

    // Now check the others quickly for error timeout
    for (int i = 0; i < MAX_NUM_DEVICE; i++)
    {
        if (i == capture_signal)
        {
            // Skip current interrupt
            continue;
        }
        time_diff[i] = time_now[capture_signal] - last_tick[i];
        time_diff_ms = pdTICKS_TO_MS(time_diff[i]);
        if (time_diff_ms >= TACHO_ERROR_MIN_INTERNAL_MS)
        {
            if (capture_counter[i] == previous_cap_value[i])
            {
                // No new interrupts received for TACHO_ERROR_MIN_INTERNAL_MS. Send 0 as rpm
                dev_capt[i].dev_rpm = 0;
                mtr_tacho_data_event_t tacho_event;
                tacho_event.capture_signal = i;
                tacho_event.capture_info.dev_rpm = 0u;
                ddmp2_create_set(&frame_event_to_send, MTR0_NEW_TACHO_DATA_EVENT, (const void*)&tacho_event, sizeof(tacho_event), connector_pwm_fan_motor.connector_id);
                pxHigherPriorityTaskWoken = pdFALSE;
                if (pdFALSE == xRingbufferSendFromISR(connector_pwm_fan_motor.to_connector, &frame_event_to_send, ddmp2_full_frame_size(DDMP2_CONTROL_SET, sizeof(tacho_event)), &pxHigherPriorityTaskWoken))
                {
                	/* Do Nothing */
                }
                // Check if we need to yield in caller function
                if (pxHigherPriorityTaskWoken == pdTRUE)
                {
                    yield = true;
                }
            }
            // Save last tick value for sending tacho value
            last_tick[i] = time_now[capture_signal];
        }
    }

    return yield;
}
#endif /* CONNECTOR_PWM_FAN_MOTOR */ 
