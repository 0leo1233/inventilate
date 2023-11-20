/*! \file connector_dp_sens_service.c
	\brief Connector for Differential Pressure sensor
	\Author Sundaramoorthy-LTTS
 */

/** Includes ******************************************************************/
#include "configuration.h"

#include <stdint.h>
#include "osal.h"
#include "connector_dp_sens_service.h"
#include "hal_i2c_master.h"
#include "ddm2_parameter_list.h"
#include "miniddm_parameter_list.h"
#include "sorted_list.h"

#ifdef CONNECTOR_BLE
#include "connector_ble.h"
#include "bluetooth_le.h"
#include "ble_central.h"
#endif

#include "app_error_code.h"

#define CONN_DP_SENS_SUB_DEPTH		                   ((uint8_t) 20u)
#define WAIT_COUNT_FOR_NEXT_READ_ACTION                ((uint8_t)  1u)
#define DDMP_UNAVAILABLE                               ((uint8_t) 0xFFu)
#define DIFF_PRESS_DB_INDEX                            ((uint8_t) 0u)
#define TEMP_PRESS_DB_INDEX                            ((uint8_t) 1u)
#define DIFF_PRESS_SCALE_FAC_DB_INDEX                  ((uint8_t) 2u)
#define TEMP_SCALE_FAC_DB_INDEX                        ((uint8_t) 3u)
#define DP_SENS_QUE_LEN			                       ((osal_base_type_t) 10)
#define DP_SENS_QUE_ITEM_SIZE   	                   ((osal_base_type_t) sizeof(QUE_DATA))
#define CONN_DP_SENS_SERV_LOGS                         1
#define BLE_SCAN_WAIT_TIME_TICKS                       pdMS_TO_TICKS(SCAN_DURATION)
#define BLE_SCAN_MAX_RETRY_COUNT                       3u
#define ENABLE_SEND_SDP_SENSOR_DATA                    1
#define DISABLE_SEND_SDP_SENSOR_DATA                   0
#define SDP_SENSOR_SAMPLING_INTERNAL_LONG              65000
#define SDP_SENSOR_VALID_SAMPLING_INTERVAL             5000
#define DP_SENSOR_AVAILABLE                            ((uint8_t)  1u)
#define DP_SENSOR_NOT_AVAILABLE                        ((uint8_t)  0u)
#define DP_SENSOR_BAT_MIN                              ((uint16_t)1000u)
#define DP_SENSOR_BAT_MAX                              ((uint16_t)2700u)

#if ( SDP3X_SENS_BOARD_COMM == COMM_I2C )

typedef enum __data_id
{
    PWR_MGR_DATA  = 0,
    DATA_INVALID  = 3
}DATA_ID;

typedef union __sensor_data
{
    sdp3x_sensor_data sdp3x_data_output;
}sensor_data;

typedef enum __diff_press_sm_state
{
    STATE_IDLE = 0,
    STATE_START_MEASUREMENT = 1,
    STATE_READ_PRESSURE_DATA = 2,
    STATE_WAIT_TO_SEND_NEXT_COMMAND = 3
}diff_press_sm_state;

typedef struct __diff_press_read_sm
{
  IVPMGR0ST_ENUM       pmgr_state;
  diff_press_sm_state  state;
  uint16_t             command;
  sensor_data          data_output;
  uint32_t             wait_counter;
  uint8_t              dev_status;
}diff_press_read_sm;

#elif ( SDP3X_SENS_BOARD_COMM == COMM_BLE )	

typedef enum __data_id
{
    PWR_MGR_DATA                = 0,
    BLE_REQUEST                 = 1,
    DP_SENS_STAT                = 2,
    SENS_NODE_AVL               = 3,
    SENS_NODE_MFGR              = 4,
    SENS_NODE_PROD_TYPE         = 5,
    SENS_NODE_TYPE              = 6,
    BLE_SCAN_TIME_EXP           = 7,
    DP_SENSOR_STATUS            = 8,
    DP_SENS_SERV_SM_STATE       = 9,
    DATA_INVALID                = 10
}DATA_ID;

typedef enum __diff_press_sm_state
{
    STATE_STANDBY             = 0,
    STATE_WAIT_FOR_BL_REQ     = 1,
    STATE_DP_SENS_BLE_PAIRING = 2,
    STATE_DP_SENS_PAIRED      = 3
}DIFF_PRESS_SM_STATE;

typedef enum __dp_sens_stat
{
    DP_SENS_STAT_UNKNOWN     = 0,
    DP_SENS_NODE_NOT_FOUND   = 1,
    DP_SENS_NODE_FOUND       = 2
}DP_SENS_STATUS;

typedef struct __diff_press_read_sm
{
    bool                 ble_scan_timer_exp;
    bool                 ble_conn_periodic_tmr_exp;
    IVPMGR0STATE_ENUM    pmgr_state;
    IV0BLREQ_ENUM        bl_req;
    DP_SENS_STATUS       dp_sens_node_stat;
    DIFF_PRESS_SM_STATE  state;
    uint8_t              retry_count;
    int32_t              dp_sensor_availablity;
}diff_press_read_sm;
#endif
typedef struct __que_data
{
    int32_t     data;
    DATA_ID  data_id;
}QUE_DATA;

static uint32_t invent_dp_error_stat = 0;
static DPSENS_ERROR dps_curr_err_stat = DPSENS_NO_ERROR;
static DPSENS_ERROR dps_prev_err_stat = DPSENS_NO_ERROR;

/* static function declarations */
static int initialize_connector_dp_sens_service(void);
static int add_subscription(DDMP2_FRAME *pframe);
static void install_parameters(void);
static void process_set_and_publish_request(uint32_t ddm_param, int32_t i32value, DDMP2_CONTROL_ENUM req_type);
static void process_subscribe_request(uint32_t ddm_param);
static uint8_t get_ddm_index_from_db(uint32_t ddm_param);
static void pwr_state_callback(int32_t i32Value);
static void start_publish(void);
static void start_subscribe(void);
static void conn_diffpress_process_task(void *pvParameter);
static void conn_diffpress_read_task(void *pvParameter);
static void push_data_to_que(DATA_ID data_id, int32_t i32Value);
#if ( SDP3X_SENS_BOARD_COMM == COMM_I2C )
static void init_dev_diffpress_sensor(diff_press_read_sm* ptr_sm);
static uint8_t read_diff_pressure_sm(void);
static void update_data_to_broker(diff_press_read_sm* ptr_inst);
static void convert_and_send_domain_val_to_ddm_sys_value(uint32_t ddm_param);
#elif ( SDP3X_SENS_BOARD_COMM == COMM_BLE )	
static void process_ble_scan_data(const uint8_t * const value);
static void bl_req_callback(int32_t i32Value);
static void parse_queue_data(diff_press_read_sm *ptr_inst , QUE_DATA* qdata);
static void ble_scan_time_exp_cb( TimerHandle_t xTimer );
static void start_ble_scan_timer(void);
static void stop_ble_scan_timer(void);
static void dp_sens_avl_callback(int32_t i32Value);
static void snode_avl_callback(int32_t i32Value);
static void change_diff_state(DIFF_PRESS_SM_STATE sm_state);
static void configure_sdp_sensor(int32_t send_dp, int32_t sample_int);
static void diffpress_error_code(const DPSENS_ERROR error);
#endif

/* Instance for State Machine */
static diff_press_read_sm sm_instane;
static osal_queue_handle_t dp_sens_queue;
#if ( SDP3X_SENS_BOARD_COMM == COMM_BLE )	
static TimerHandle_t ble_scan_timer_hdle;
#endif

/* Structure for Connector VOC Sensor */
CONNECTOR connector_diffpress_sensor =
{
	.name       = "DiffPress Sensor Connector",
	.initialize = initialize_connector_dp_sens_service,
};

/* DDM Parameter table for connector differential pressure sensor */
static conn_diff_press_sensor_param_t conn_diffpress_sensor_param_db[] =
{
#if ( SDP3X_SENS_BOARD_COMM == COMM_I2C )
	//  ddm_parameter         type           pub       sub              data                  cb_func
    {IVSDP0DP        , DDM2_TYPE_INT32_T,     1,        0,  					 0,  			    NULL},
    {IVSDP0TEMP      , DDM2_TYPE_INT32_T,     1,        0,  				     0,  			    NULL},
    {IVSDP0DPSCFAC   , DDM2_TYPE_INT32_T,     1,        0,  					 0,  			    NULL},
    {IVSDP0TPSCFAC   , DDM2_TYPE_INT32_T,     1,        0,  					 0,  			    NULL},
#elif ( SDP3X_SENS_BOARD_COMM == COMM_BLE )	  
    {BT0SCAN         , DDM2_TYPE_OTHER,       0,        1,                       0,                 NULL},       
    {SDP0AVL         , DDM2_TYPE_INT32_T,     0,        1,                       0, dp_sens_avl_callback}, 
    {SNODE0BATTLVL   , DDM2_TYPE_INT32_T,     0,        1,                       0, snode_avl_callback},       
    {IV0BLREQ        , DDM2_TYPE_INT32_T,     0,        1,  					 0,      bl_req_callback},
#endif
    {IVPMGR0STATE    , DDM2_TYPE_INT32_T,     0,        1,						 0,   pwr_state_callback},
    {IV0ERRST        , DDM2_TYPE_INT32_T, 	  0,        1, 	                     0,  NULL},
};

/* Calculate the connector fan motor database table num elements */
static const uint32_t conn_diff_press_db_elements = ELEMENTS(conn_diffpress_sensor_param_db);

DECLARE_SORTED_LIST_EXTRAM(conn_dp_sens_sub_table, CONN_DP_SENS_SUB_DEPTH);       //!< \~ Subscription table storage

/**
  * @brief  Initialize the connector for differential pressure sensor service
  * @param  none.
  * @retval none.
  */
static int initialize_connector_dp_sens_service(void)
{
    /* Initialize the state machine state */
    sm_instane.state = STATE_STANDBY;

    /* Create queue for invent control task */
    dp_sens_queue = osal_queue_create(DP_SENS_QUE_LEN, DP_SENS_QUE_ITEM_SIZE);

    if ( NULL != dp_sens_queue )
    {
        LOG(I, "Queue creation done for DP sensor");
    }

#if ( SDP3X_SENS_BOARD_COMM == COMM_I2C )
	/* Initialize the device voc sensor */
	init_dev_diffpress_sensor(&sm_instane);
#endif

#if ( SDP3X_SENS_BOARD_COMM == COMM_BLE )
    /* Create the one shot timer software timer for BLE scan time */
    ble_scan_timer_hdle = xTimerCreate("ble_scan_timer_hdle", BLE_SCAN_WAIT_TIME_TICKS, pdFALSE, 0, ble_scan_time_exp_cb);
#endif

	TRUE_CHECK(osal_task_create(conn_diffpress_process_task, CONNECTOR_DIFF_PRESS_PROCESS_TASK_NAME, 
                CONNECTOR_DIFFPRESS_PROCESS_STACK_DEPTH, NULL, CONNECTOR_DIFFPRESS_PROCESS_TASK_PRIORITY, NULL));

	TRUE_CHECK(osal_task_create(conn_diffpress_read_task, CONNECTOR_DIFF_PRESS_READ_TASK_NAME, 
                CONNECTOR_DIFFPRESS_READ_STACK_DEPTH, NULL, CONNECTOR_DIFFPRESS_READ_TASK_PRIORITY, NULL));

    /* install ddmp parameters */
    install_parameters();

    start_subscribe();

    start_publish();

	return 1;
}

/**
  * @brief  Function to subscribe the DDMP parameters needed for connector fan and motor
  * @param  none.
  * @retval none.
  */
static void start_subscribe(void)
{
	conn_diff_press_sensor_param_t *ptr_param_db;
	uint8_t db_idx;
    uint8_t num_elements = ELEMENTS(conn_diffpress_sensor_param_db);

	for ( db_idx = 0; db_idx < num_elements; db_idx++ )
	{
		ptr_param_db = &conn_diffpress_sensor_param_db[db_idx];
        
        /* Check the DDM parameter need subscribtion */
		if ( ptr_param_db->sub )
		{
            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SUBSCRIBE, ptr_param_db->ddm_parameter, \
                       &ptr_param_db->i32Value, sizeof(int32_t), connector_diffpress_sensor.connector_id, portMAX_DELAY));
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
    conn_diff_press_sensor_param_t *ptr_param_db;
    uint16_t db_idx;
    uint8_t num_elements = ELEMENTS(conn_diffpress_sensor_param_db);
    
    for ( db_idx = 0; db_idx < num_elements; db_idx++ )
    {
        ptr_param_db = &conn_diffpress_sensor_param_db[db_idx];

        /* Check the DDM parameter need to publish */
        if ( ptr_param_db->pub ) 
        {
            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, ptr_param_db->ddm_parameter, &ptr_param_db->i32Value, \
                        sizeof(int32_t), connector_diffpress_sensor.connector_id, portMAX_DELAY));
        }
    }
}

/**
  * @brief  Task for connector differential pressure sensor to proces ddmp request
  * @param  pvParameter.
  * @retval none.
  */
static void conn_diffpress_process_task(void *pvParameter)
{
	DDMP2_FRAME *pframe;
	size_t frame_size;

	while (1)
	{
		TRUE_CHECK ( pframe = xRingbufferReceive(connector_diffpress_sensor.to_connector, &frame_size, portMAX_DELAY) );

		switch (pframe->frame.control)
		{
		case DDMP2_CONTROL_PUBLISH:/* Send the changed data to broker */
            if ( BT0SCAN != pframe->frame.publish.parameter )
            {
                process_set_and_publish_request(pframe->frame.publish.parameter, pframe->frame.publish.value.int32,pframe->frame.control);
            }
            else
            {
                size_t payload_size = ddmp2_value_size(pframe);

                if ( payload_size > 0 )
                {
                    process_ble_scan_data(pframe->frame.publish.value.raw);
                }
                else
                {
                    LOG(W, "Invalid BLE device");
                }
            }
            break;

		case DDMP2_CONTROL_SET:
#if CONN_DP_SENS_SERV_LOGS
            LOG(I, "Set Request received parameter = 0x%x value = %d", pframe->frame.set.parameter, pframe->frame.set.value.int32);
#endif
            process_set_and_publish_request(pframe->frame.set.parameter, pframe->frame.set.value.int32,pframe->frame.control);
			break;

		case DDMP2_CONTROL_SUBSCRIBE:/* Send the request for the data */
#if CONN_DP_SENS_SERV_LOGS
            LOG(I, "Subcribe Request received");
#endif
			add_subscription(pframe);
            process_subscribe_request(pframe->frame.subscribe.parameter);
			break;

		default:
			LOG(E, "Connector diff press sensor sensor received UNHANDLED frame %02x from broker!",pframe->frame.control);
			break;
		}

		vRingbufferReturnItem(connector_diffpress_sensor.to_connector, pframe);
    }
}

/**
  * @brief  Task for read Differential Pressure data
  * @param  pvParameter.
  * @retval none.
  */
static void conn_diffpress_read_task(void *pvParameter)
{
#if ( SDP3X_SENS_BOARD_COMM == COMM_I2C )
    TickType_t task_frequency  = (TickType_t)pdMS_TO_TICKS(CONN_DIFF_PRESS_READ_TASK_DELAY_MSEC);
    TickType_t last_wake_time  = xTaskGetTickCount();
#elif ( SDP3X_SENS_BOARD_COMM == COMM_BLE )
    QUE_DATA qdata;
    int32_t i32Value = 1;
    diff_press_read_sm *ptr_inst = &sm_instane;
    DIFF_PRESS_SM_STATE sm_next_state = ptr_inst->state;
#endif

    while (1)
	{
#if ( SDP3X_SENS_BOARD_COMM == COMM_I2C )
        /* State Machine for reading the pressure data from the sensor via I2C */
        read_diff_pressure_sm();

        /* Task delay */
        vTaskDelayUntil(&last_wake_time, task_frequency);

#elif ( SDP3X_SENS_BOARD_COMM == COMM_BLE )

        //send battery voltage
        TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SUBSCRIBE, SNODE0BATTLVL, NULL, 0, \
                connector_diffpress_sensor.connector_id, portMAX_DELAY));

        //command to send DP sensor data            
        TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SUBSCRIBE, SNODE0MDL, NULL, 0, \
                connector_diffpress_sensor.connector_id, portMAX_DELAY));


        if ( pdPASS == xQueueReceive(dp_sens_queue, (void *)&qdata, portMAX_DELAY) )
        {
    #if CONN_DP_SENS_SERV_LOGS
            LOG(I, "Data received from que data=%d data_id=%d", qdata.data, qdata.data_id);
    #endif
            parse_queue_data(ptr_inst, &qdata);
        }
        
        switch ( ptr_inst->state )
        {
            case STATE_STANDBY:
                if ( IVPMGR0STATE_ACTIVE == ptr_inst->pmgr_state )
                {
                    /* Reset the BL Request flag */
                    ptr_inst->bl_req = IV0BLREQ_IDLE;
                    /* Change the state */
                    sm_next_state = STATE_WAIT_FOR_BL_REQ;
                }
                else if ( DP_SENSOR_AVAILABLE == ptr_inst->dp_sensor_availablity )
                {
                    /* Inventilate system is not Turned ON by user, So no need for DP sensor data */
                    configure_sdp_sensor(DISABLE_SEND_SDP_SENSOR_DATA, SDP_SENSOR_SAMPLING_INTERNAL_LONG);
                }
                else
                {

                }
                break;

            case STATE_WAIT_FOR_BL_REQ:
                LOG(W, "STATE_WAIT_FOR_BL_REQ bl_req = %d dp_sensor_availablity = %d", ptr_inst->bl_req, ptr_inst->dp_sensor_availablity);
                if ( IVPMGR0STATE_STANDBY == ptr_inst->pmgr_state )
                {
                    stop_ble_scan_timer();
                    /* Change the state */
                    sm_next_state = STATE_STANDBY;
                }
                else if ( DP_SENSOR_AVAILABLE == ptr_inst->dp_sensor_availablity )
                {
                    LOG(W, "Paired with existing DP sensor node");
                    stop_ble_scan_timer();
                    /* Reset the BL Request flag */
                    ptr_inst->bl_req = IV0BLREQ_IDLE;
                    /* Configure the SDP sensor */
                    configure_sdp_sensor(ENABLE_SEND_SDP_SENSOR_DATA, SDP_SENSOR_VALID_SAMPLING_INTERVAL);
                    /* Change the state */
                    sm_next_state = STATE_DP_SENS_PAIRED;
                }
                else if ( ( IV0BLREQ_SCAN == ptr_inst->bl_req ) || ( IV0BLREQ_PAIR == ptr_inst->bl_req ) )
                {
                    LOG(I, "DP Sensor BLE scan (BT0SCAN) req sent");
                    /* Reset the BL Request flag */
                    ptr_inst->bl_req = IV0BLREQ_IDLE;
                    /* Send the request */
                    TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, BT0SCAN, &i32Value, sizeof(int32_t), \
                                connector_diffpress_sensor.connector_id, portMAX_DELAY));
                    /* Start the wait timer */
                    start_ble_scan_timer();
                    /* Change the state */
                }
                else
                {
                    /* Do nothing */
                }
                break;

            case STATE_DP_SENS_BLE_PAIRING:
                if ( IVPMGR0STATE_STANDBY == ptr_inst->pmgr_state )
                {
                    stop_ble_scan_timer();
                    ptr_inst->retry_count        = 0;
                    ptr_inst->ble_scan_timer_exp = false;
                    sm_next_state                = STATE_STANDBY;
                }
                else if ( ( DP_SENS_NODE_FOUND  == ptr_inst->dp_sens_node_stat     ) && 
                          ( DP_SENSOR_AVAILABLE == ptr_inst->dp_sensor_availablity ) )
                {
                    LOG(W, "DP Sensor node found and pairing done succesfully");
                    stop_ble_scan_timer();
                    /* Configure the SDP sensor */
                    configure_sdp_sensor(ENABLE_SEND_SDP_SENSOR_DATA, SDP_SENSOR_VALID_SAMPLING_INTERVAL);
                    /* Reset the flag and retry count */
                    ptr_inst->retry_count        = 0;
                    ptr_inst->ble_scan_timer_exp = false;
                    /* Reset the BL Request flag */
                    ptr_inst->bl_req = IV0BLREQ_IDLE;
                    /* Change the state */
                    sm_next_state    = STATE_DP_SENS_PAIRED;
                }
                else if ( true == ptr_inst->ble_scan_timer_exp )
                {
                    LOG(E, "DP Sensor node not found..Check DP sensor board powered ON ?");
                    /* Reset the flag */
                    ptr_inst->ble_scan_timer_exp = false;
                    
                    if ( ptr_inst->retry_count < BLE_SCAN_MAX_RETRY_COUNT )
                    {
                        ptr_inst->retry_count++;
                        LOG(E, "RetryCount=%d", ptr_inst->retry_count);
                        /* Send the BT scan request */
                        TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, BT0SCAN, &i32Value, sizeof(int32_t), \
                                connector_diffpress_sensor.connector_id, portMAX_DELAY)); 
                        /* Start the BLE scan timer */
                        start_ble_scan_timer();
                    }
                    else
                    {
                        LOG(E, "DP sensor node pairing failed");
                        /* Reset the BL Request flag */
                        ptr_inst->bl_req      = IV0BLREQ_IDLE;
                        /* Reset the flag and retry count */
                        ptr_inst->retry_count = 0;
                        /* Change the state */
                        sm_next_state         = STATE_WAIT_FOR_BL_REQ;
                        dps_curr_err_stat     = DPSENS_BOARD_CONN_RETRY;
                        diffpress_error_code(dps_curr_err_stat); 
                    }
                }
                else
                {
                    /* Do nothing */
                }
                break;

            case STATE_DP_SENS_PAIRED:
                if ( ( IVPMGR0STATE_STANDBY == ptr_inst->pmgr_state ) || 
                     ( IVPMGR0STATE_STORAGE == ptr_inst->pmgr_state ) )
                {
                    /* Inventilate system is Turned OFF by user, So no need for the DP data */
                    //configure_sdp_sensor(DISABLE_SEND_SDP_SENSOR_DATA, SDP_SENSOR_SAMPLING_INTERNAL_LONG);
                    /* Change the state */
                    sm_next_state = STATE_STANDBY;
                }
                else if ( DP_SENSOR_NOT_AVAILABLE == ptr_inst->dp_sensor_availablity )
                {
                    //DP_Sensor not vaialable or disconnected.
                    LOG(I,"[DP_Sensor_not_available]");
                    /* Reset the BL Request flag */
                    ptr_inst->bl_req = IV0BLREQ_IDLE;
                    /* Change the state */
                    sm_next_state = STATE_WAIT_FOR_BL_REQ;
                    diffpress_error_code(DPSENS_BOARD_DISCONNECTED);
                }
                else if ( ptr_inst->bl_req != IV0BLREQ_IDLE )
                {
                    // Ignore the BLE pair/scan request
                    LOG(W, "DP Sensor paired already");
                }
                else
                {
                    /* Do nothing */
                }
                break;

            default:
                break;
        }

        if ( sm_next_state != ptr_inst->state )
        {
            change_diff_state(sm_next_state);
        }

    #if CONN_DP_SENS_SERV_LOGS
        LOG(I, "state = %d", ptr_inst->state);
    #endif
#endif
    }
}

#if ( SDP3X_SENS_BOARD_COMM == COMM_I2C )
/**
  * @brief  State Machine to read the differential pressure sensor data from SDP3x
  * @param  none.
  * @retval error code.
  */
static uint8_t read_diff_pressure_sm(void)
{
    int ret = 0;
    QUE_DATA qdata;
    diff_press_read_sm *ptr_inst = &sm_instane;
    
    if ( pdPASS == xQueueReceive(dp_sens_queue, (void *)&qdata, 0) )
    {
        if ( PWR_MGR_DATA == qdata.data_id )
        {
            ptr_inst->pmgr_state = qdata.data;
        }
    }

    switch (ptr_inst->state)
    {
        case STATE_IDLE:
            if ( ( SUCCESS == ptr_inst->dev_status ) && ( IVPMGR0ST_ACTIVE == ptr_inst->pmgr_state ) )
            {
                /* Change the state */
                ptr_inst->state = STATE_START_MEASUREMENT;
            }
            break;

        case STATE_START_MEASUREMENT:
            /* Start the measurement */
            ret = start_measurement();
            
            if ( ret == 0 )
            {
                /* Change the state */
                ptr_inst->state = STATE_WAIT_TO_SEND_NEXT_COMMAND;
            }
            else
            {
                LOG(E, "Error in start measurement = %d", ret);
                /* Reset the counter */
                ptr_inst->wait_counter = 0u;
                /* Change the state */
                ptr_inst->state        = STATE_IDLE;
            }
            break;

        case STATE_READ_PRESSURE_DATA:
            /* Read the data from the sensor */
            get_diff_pressure_data(&ptr_inst->data_output.sdp3x_data_output);
            LOG(I, "Differential Pressure = %f Pascal", ptr_inst->data_output.sdp3x_data_output.diff_pressure_data);
            LOG(I, "Temperature = %f DegC", ptr_inst->data_output.sdp3x_data_output.temperature_data);
            LOG(I, "Pressure sensor scale factor = %d", ptr_inst->data_output.sdp3x_data_output.scale_factor_diff_pressure);
            update_data_to_broker(ptr_inst);
            /* Change the state */
            ptr_inst->state = STATE_WAIT_TO_SEND_NEXT_COMMAND;
            break;

        case STATE_WAIT_TO_SEND_NEXT_COMMAND:
            if ( IVPMGR0ST_ACTIVE == ptr_inst->pmgr_state )
            {
                /* Increment the counter */
                ptr_inst->wait_counter++;

                if ( ptr_inst->wait_counter >= WAIT_COUNT_FOR_NEXT_READ_ACTION )
                {
                    /* Reset the counter */
                    ptr_inst->wait_counter = 0u;
                    /* Change the state */
                    ptr_inst->state        = STATE_READ_PRESSURE_DATA;
                }
            }
            else
            {
                /* Reset the data */
                ptr_inst->data_output.sdp3x_data_output.diff_pressure_data         = 0;
                ptr_inst->data_output.sdp3x_data_output.temperature_data           = 0;
                ptr_inst->data_output.sdp3x_data_output.scale_factor_diff_pressure = 0;
                ptr_inst->data_output.sdp3x_data_output.scale_factor_temperature   = 0;
                /* Update the reset data to broker */
                update_data_to_broker(ptr_inst);
                /* Stop the measurement */
                stop_measurement();
                /* Change the state */
                ptr_inst->state = STATE_IDLE;
            }
            break;

        default:
            break;
    }
    
    return 0;
}

/**
  * @brief  Function to update the DP sensor data to broker
  * @param  ptr_inst Pointer to the structue instance of type diff_press_read_sm.
  * @retval none.
  */
static void update_data_to_broker(diff_press_read_sm* ptr_inst)
{
    uint8_t  loop_idx;
    int      index;
    int      factor = 0;
    int32_t  ddm_param[2] = { IVSDP0DP, IVSDP0TEMP };
    int32_t* value[2] = { &conn_diffpress_sensor_param_db[DIFF_PRESS_DB_INDEX].i32Value, 
                          &conn_diffpress_sensor_param_db[TEMP_PRESS_DB_INDEX].i32Value };

    for ( loop_idx = 0; loop_idx < 2; loop_idx++ )
    {
        index = ddm2_parameter_list_lookup(DDM2_PARAMETER_BASE_INSTANCE(ddm_param[loop_idx]));

        if ( SORTLIST_INVALID_VALUE != index )
        {
            factor = Ddm2_unit_factor_list[Ddm2_parameter_list_data[index].out_unit];
        }

        factor = ( factor > 0 ) ? factor : 1;

        *value[loop_idx] = (int32_t) ( (float)ptr_inst->data_output.sdp3x_data_output.diff_pressure_data * (float)factor );
    }

    /* Store the data */
    conn_diffpress_sensor_param_db[DIFF_PRESS_SCALE_FAC_DB_INDEX].i32Value = (int32_t)ptr_inst->data_output.sdp3x_data_output.scale_factor_diff_pressure;
    conn_diffpress_sensor_param_db[TEMP_SCALE_FAC_DB_INDEX].i32Value       = (int32_t)ptr_inst->data_output.sdp3x_data_output.scale_factor_temperature;
    
    /* Publish the data to broker */            
    convert_and_send_domain_val_to_ddm_sys_value(IVSDP0DP);
    convert_and_send_domain_val_to_ddm_sys_value(IVSDP0TEMP);
    convert_and_send_domain_val_to_ddm_sys_value(IVSDP0DPSCFAC);
    convert_and_send_domain_val_to_ddm_sys_value(IVSDP0TPSCFAC);
}

/**
  * @brief  Init the device differential pressure sensor
  * @param  none.
  * @retval none.
  */
static void init_dev_diffpress_sensor(diff_press_read_sm* ptr_sm)
{
    int result;

    /* Reset the struct instance variables */
    ptr_sm->state        = STATE_IDLE;
    ptr_sm->wait_counter = 0u;

    /* SDP32 I2C Bus Configurations */
    const SDP32_BUS_CONF sdp32_conf = 
    {
        .type = SDP32_BUS_CONF_TYPE_I2C,
        .i2c =
        {
            .port    = I2C_MASTER0_PORT,
            .sda     = I2C_MASTER0_SDA,
            .scl     = I2C_MASTER0_SCL,
            .bitrate = I2C_MASTER0_FREQ,
        },
        .irq = 
        {
            .port = 0,
            .pin  = 0,
        }
    };
    
    result = sdp32_seq_init(&sdp32_conf);

    if ( SUCCESS == result )
    {
        LOG(I, "SDP3x Device Initialized Succesfully result = 0x%x", result);
    }
    else
    {
        LOG(E, "SDP3x Device Initialization Failed result = 0x%x", result);
    }

    ptr_sm->dev_status = result;
}

#elif ( SDP3X_SENS_BOARD_COMM == COMM_BLE )

/**
  * @brief  Funtion to parse the queue data and update in the structure variables 
  * @param  ptr_inst Pointer to the structure diff_press_read_sm.
  * @retval qdata Pointer to the structure QUE_DATA.
  */
static void parse_queue_data(diff_press_read_sm *ptr_inst , QUE_DATA* qdata)
{
    switch (qdata->data_id)
    {
        case PWR_MGR_DATA:
            ptr_inst->pmgr_state = qdata->data;
            break;

        case DP_SENS_SERV_SM_STATE:
            ptr_inst->state = qdata->data;
            break;

        case BLE_REQUEST:
            ptr_inst->bl_req = qdata->data;
            break;

        case DP_SENS_STAT:
            ptr_inst->dp_sens_node_stat = qdata->data;
            break;

        case BLE_SCAN_TIME_EXP:
            /*BLE Scan_timer expired*/                     
            ptr_inst->ble_scan_timer_exp = true;
            break;

        case DP_SENSOR_STATUS:
            ptr_inst->dp_sensor_availablity = qdata->data;
            break;
        
        default:
            break;
    }
}

/**
  * @brief  Function to add the inventilate state changed request in queue 
  * @param  iv_ctrl_state State to be transfer.
  * @retval none.
  */
static void change_diff_state(DIFF_PRESS_SM_STATE sm_state)
{
    QUE_DATA qdata;
    qdata.data    = sm_state;
    qdata.data_id = DP_SENS_SERV_SM_STATE;

    // Append the data in the queue
    osal_base_type_t ret = xQueueSendToFront (dp_sens_queue, &qdata, 0);

    if ( osal_success != ret )
    {
        LOG(E, "Queue error ret = %d", ret);
    }
}

/**
  * @brief  Callback function to handle bl request
  * @param  i32Value.
  * @retval none.
  */
static void bl_req_callback(int32_t i32Value)
{
    push_data_to_que(BLE_REQUEST, i32Value);
}

/**
  * @brief  Callback function to handle bl request
  * @param  i32Value.
  * @retval none.
  */
static void dp_sens_avl_callback(int32_t i32Value)
{
    if ( 0 == i32Value )
    {
        LOG(I, "Subscribe DDMP SDP0AVL");
        // Whenever the availability of sensor node received then the subscribtion should be done again
        TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SUBSCRIBE, SDP0AVL, NULL, 0, \
                    connector_diffpress_sensor.connector_id, portMAX_DELAY));
    }
    /* Push the data into the queue */
    push_data_to_que(DP_SENSOR_STATUS, i32Value);
}

/**
  * @brief  Callback function to handle bl request
  * @param  i32Value.
  * @retval none.
  */
static void snode_avl_callback(int32_t i32Value)
{
    if( (i32Value > DP_SENSOR_BAT_MIN) && (i32Value < DP_SENSOR_BAT_MAX) )
    {
        //Send Battery low Error code
        diffpress_error_code(DPSENS_BOARD_BATTERY_LOW); 
    }
    //Subscribe to get DP sensor battery voltage            
    TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SUBSCRIBE, SNODE0BATTLVL, NULL, 0, \
                connector_diffpress_sensor.connector_id, portMAX_DELAY));            

}

/**
  * @brief  Callback function to handle the BLE scan time expired and 
  *         push the event into the queue for further processing
  * @param  TimerHandle_t timer handle.
  * @retval none.
  */
static void ble_scan_time_exp_cb( TimerHandle_t xTimer )
{
    push_data_to_que(BLE_SCAN_TIME_EXP, 0);
}

/**
  * @brief  Function to start the timer for BLE scan
  * @param  none.
  * @retval none.
  */
static void start_ble_scan_timer(void)
{
    osal_ubase_type_t tmr_started = xTimerStart( ble_scan_timer_hdle, 0 );
	
    if ( tmr_started == pdPASS )
    {
		LOG(I, "BLE scan timer started");
    }
}

/**
  * @brief  Function to stop the timer for BLE scan
  * @param  none.
  * @retval none.
  */
static void stop_ble_scan_timer(void)
{
    osal_ubase_type_t  tmr_stopped = xTimerStop( ble_scan_timer_hdle, 0 );
	
    if ( tmr_stopped == pdPASS )
    {
		LOG(I, "BLE scan timer stopped");
    }
}

/**
  * @brief  Function to process the BLE scan data
  * @param  none.
  * @retval none.
  */
static void process_ble_scan_data(const uint8_t * const value)
{
    uint8_t wl_addr[7];
    uint8_t index = 0;
    DP_SENS_STATUS scan_stat = DP_SENS_NODE_NOT_FOUND;
	const BLE_DEVICE_ENUM_FRAME *ble_enum = (BLE_DEVICE_ENUM_FRAME*) value;

    LOG(I, "Received BT0SCAN list");
	LOG(W, "\tmfg=%04x node_type=%02x node_id=%02x\n", ble_enum->manufacturer, ble_enum->node_id, ble_enum->node_type);
	LOG(W, "\tble_id=%u:" MACSTR "\n", ble_enum->ble_address_type , MAC2STR(ble_enum->ble_address));
	LOG(W, "\trssi=%d\n", ble_enum->rssi);
	LOG(W, "\tname=%.16s\n", ble_enum->name);

    if ( ( ble_enum->manufacturer == DOMETIC_BLE_ID             ) && 
         ( ble_enum->node_type    == NODE_TYPE_DOMETIC          ) && 
         ( ble_enum->node_id      == NODE_EXTENDED_SENSOR ) 
       )
    {
        LOG(I, "Found valid DP sensor");
        wl_addr[index] = ble_enum->ble_address_type;

        memcpy((void*)&wl_addr[1], (const void*)ble_enum->ble_address, 6);
        
        TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, BT0ADDWL, (const void*)wl_addr, 7, \
                    connector_diffpress_sensor.connector_id, portMAX_DELAY));

        scan_stat = DP_SENS_NODE_FOUND;

        push_data_to_que(DP_SENS_STAT, scan_stat);
    }
}

#endif

/**
  * @brief  Function to publish Inventilate available to the broker
  * @param  none.
  * @retval none.
  */
static void install_parameters(void)
{
    int32_t available = 1;
#if ( SDP3X_SENS_BOARD_COMM == COMM_I2C )

    TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, IVSDP0AVL, &available, sizeof(int32_t), \
                connector_diffpress_sensor.connector_id, portMAX_DELAY));
#endif

    LOG(W, "Subscribe request for DDMP SNODE0AVL from conn_dp_sens");
    // Whenever the availability of sensor node received then the subscribtion should be done again
    TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SUBSCRIBE, SNODE0AVL, &available,sizeof(int32_t), \
            connector_diffpress_sensor.connector_id, portMAX_DELAY));
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

    return sorted_list_single_add(&conn_dp_sens_sub_table, key, value);
}

/**
  * @brief  Function to process the set and publish parameter from broker
  * @param  DDM parameter.
  * @retval none.
  */
static void process_set_and_publish_request(uint32_t ddm_param, int32_t i32value, DDMP2_CONTROL_ENUM req_type)
{
    int32_t i32Index;
    int32_t i32Factor;
	uint16_t db_idx;
	conn_diff_press_sensor_param_t* param_db;

#if CONN_DP_SENS_SERV_LOGS
    LOG(I, "Received ddm_param = 0x%x i32value = %d", ddm_param, i32value);
#endif

	/* Validate the DDM parameter received */
	db_idx = get_ddm_index_from_db(ddm_param);
 
	if ( DDMP_UNAVAILABLE != db_idx )
	{
#if CONN_DP_SENS_SERV_LOGS
        LOG(I, "Valid DDMP parameter ");
#endif

        if ( DDMP2_CONTROL_SET == req_type )
        {
            
                /* Frame and send the publish request */
                TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, ddm_param, &i32value, sizeof(int32_t), \
                            connector_diffpress_sensor.connector_id, portMAX_DELAY));
        }

		param_db = &conn_diffpress_sensor_param_db[db_idx];
		
        i32Index = ddm2_parameter_list_lookup(DDM2_PARAMETER_BASE_INSTANCE(ddm_param));

        if ( -1 != i32Index )
        {
            i32Factor = Ddm2_unit_factor_list[Ddm2_parameter_list_data[i32Index].in_unit];
      
            /* Differential pressure value should be handle with the decimal point resolution 
               For example 1.24 (pascal) value will be received as 1240 (i.e 1.24 * 1000 )
               So if we again divide this by factor(1000) then the resolution could be lost, So
               to avoid this, Divide by factor operation should be skipped for IVSDP0DP
            */
            i32Factor = ( i32Factor == 0 ) ? 1 : i32Factor;
            
            i32value = i32value / i32Factor;
            
            if ( i32value != param_db->i32Value )
            {
                /* Update the received value in the database table*/
	  	        param_db->i32Value = i32value;
		        /* Check callback function registered for this DDM parameter */
		        if  ( NULL != param_db->cb_func )
		        {
		   	        /* Execute the callback function */
                    param_db->cb_func(param_db->i32Value);
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
    uint32_t list_value = 0;
    int index;
    int32_t value = 0;
    int factor = 0;
	conn_diff_press_sensor_param_t* param_db;
    SORTED_LIST_RETURN_VALUE ret = sorted_list_unique_get(&list_value, &conn_dp_sens_sub_table, ddm_param, 0);

    if ( SORTED_LIST_FAIL != ret )
	{
		/* Validate the DDM parameter received */
		db_idx = get_ddm_index_from_db(ddm_param);

		if ( DDMP_UNAVAILABLE != db_idx )
	  	{
			param_db = &conn_diffpress_sensor_param_db[db_idx];

            index = ddm2_parameter_list_lookup(DDM2_PARAMETER_BASE_INSTANCE(ddm_param));

            if ( -1 != index )
			{
                factor = Ddm2_unit_factor_list[Ddm2_parameter_list_data[index].out_unit];

                /* The differential pressure value is stored in the database with factor multiplication
                   to avoid the loss of decimal resolution, So while sending no need to multiply with factor again */
                factor = ( factor == 0 ) ? 1 : factor;
                
                /* Multiply with the factor */
                value = param_db->i32Value * factor;
                /* Frame and send the publish request */
                TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, ddm_param, &value, sizeof(int32_t), \
                            connector_diffpress_sensor.connector_id, portMAX_DELAY));
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
	else
	{
		LOG(E, "SORTLIST_INVALID_VALUE ddm_param 0x%x", ddm_param);
	}
}

#if ( SDP3X_SENS_BOARD_COMM == COMM_I2C )
/**
  * @brief  Function to convert the domain value to DDM system value
  * @param  ddm_param - DDMP Parameter.
  * @retval Result of conversion .
  */
static void convert_and_send_domain_val_to_ddm_sys_value(uint32_t ddm_param)
{
	uint16_t db_idx;
    int index;
    int32_t i32Value = 0;
    int factor = 0;
	conn_diff_press_sensor_param_t* param_db;
	
    /* Validate the DDM parameter received */
	db_idx = get_ddm_index_from_db(ddm_param);

    if ( DDMP_UNAVAILABLE != db_idx )
    {
        param_db = &conn_diffpress_sensor_param_db[db_idx];

        index = ddm2_parameter_list_lookup(DDM2_PARAMETER_BASE_INSTANCE(ddm_param));
        if ( SORTLIST_INVALID_VALUE != index )
	    {
            factor = Ddm2_unit_factor_list[Ddm2_parameter_list_data[index].out_unit];

            if ( factor == 0 )
            {
                factor = 1;
            }
        
            if ( ( IVSDP0DP != ddm_param ) && ( IVSDP0TEMP != ddm_param ) )
            {
                /* Multiply with the factor */
                i32Value = param_db->i32Value * factor;
            }
            else
            {
                /* The data is already multiplied by the factor for the IVSDP0DP and IVSDP0TEMP */
                i32Value = param_db->i32Value;
            }
            /* Frame and send the publish request */
		    connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, ddm_param, &i32Value, sizeof(i32Value), \
                                connector_diffpress_sensor.connector_id, (TickType_t)portMAX_DELAY);
	    }
        else
        {
            LOG(E, "DDMP 0x%x not found in ddm2_parameter_list_lookup", ddm_param);
        }
    }
}
#endif

/**
  * @brief  Function to get ddm index from database table
  * @param  DDMP Parameter.
  * @retval none.
  */
static uint8_t get_ddm_index_from_db(uint32_t ddm_param)
{
	conn_diff_press_sensor_param_t* param_db;
	uint8_t db_idx = DDMP_UNAVAILABLE; 
	uint8_t index;
	bool avail = false;

	for ( index = 0u; ( ( index < conn_diff_press_db_elements ) && ( avail == false ) ); index++ )
 	{
		param_db = &conn_diffpress_sensor_param_db[index];
      	
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
  * @brief  Callback function to handle subscribed data
  * @param  i32Value.
  * @retval none.
  */
static void pwr_state_callback(int32_t i32Value)
{
    push_data_to_que(PWR_MGR_DATA, i32Value);
}


/**
  * @brief  Callback function to handle dp sensor error condition
  * @param  table_index
  * @param  i32Value.
  * @retval none.
  */
static void inv_dp_err_stat_callback(uint8_t table_index, int32_t i32Value)
{
    /* Update error status value */
    invent_dp_error_stat = i32Value;
}
/**
  * @brief  Function to push data frame into the queue
  * @param  data_id  Refer the enum DATA_ID.
  * @param  i32Value Value corresponding to the data ID.
  * @retval none.
  */
static void push_data_to_que(DATA_ID data_id, int32_t i32Value)
{
    QUE_DATA qdata;
    qdata.data    = i32Value;
    qdata.data_id = data_id;

    // push the data in the Queue
    osal_base_type_t ret = osal_queue_send(dp_sens_queue, &qdata, portMAX_DELAY);

    if ( osal_success != ret )
    {
        LOG(E, "Queue error ret = %d", ret);
    }
}

/**
  * @brief  Function to configure SDP sensor
  * @param  send_dp    1 - Send DP Data / 0 - Not send DP data
  * @param  sample_int Data sending interval
  * @retval none.
  */
static void configure_sdp_sensor(int32_t send_dp, int32_t sample_int)
{
    TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, SDP0SENDDP, (const void*)&send_dp, sizeof(int32_t), \
                connector_diffpress_sensor.connector_id, portMAX_DELAY));
    TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, SDP0SAMPP, (const void*)&sample_int, sizeof(int32_t), \
                connector_diffpress_sensor.connector_id, portMAX_DELAY));
    
    //command to send DP sensor data            
    TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SUBSCRIBE, SNODE0MDL, NULL, 0, \
                connector_diffpress_sensor.connector_id, portMAX_DELAY));
      
    TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SUBSCRIBE, SNODE0BATTLVL, NULL, 0, \
                connector_diffpress_sensor.connector_id, portMAX_DELAY));            
}


/**
  * @brief  Callback function to handle subscribed data
  * @param  table_index
  * @param  i32Value.
  * @retval none.
  */
static void diffpress_error_code(const DPSENS_ERROR error)
{
    uint32_t err_frame = invent_dp_error_stat;

    switch (error)
    {
        case DPSENS_BOARD_DISCONNECTED:
            err_frame |=  1 << DP_SENSOR_BOARD_DISCONNECTED;
            break;

        case DPSENS_DATA_PLAUSIBLE_ERROR:
            err_frame |=  1 << DP_SENSOR_DATA_PLAUSIBLE_ERROR;
            break;

        case DPSENS_BOARD_BATTERY_LOW:
            err_frame |=  1 << DP_SENSOR_BOARD_BATTERY_LOW;
            break;

        case DPSENS_NO_ERROR:
            err_frame &= ~( 1 << DP_SENSOR_BOARD_DISCONNECTED);
            err_frame &= ~( 1 << DP_SENSOR_DATA_PLAUSIBLE_ERROR);
            err_frame &= ~( 1 << DP_SENSOR_BOARD_BATTERY_LOW);
            break;

        default:
            LOG(E, "Unhandled error code = %d", error);
            break;
    }

    if ( err_frame != invent_dp_error_stat )
    {
        connector_send_frame_to_broker(DDMP2_CONTROL_SET, IV0ERRST, &err_frame, sizeof(err_frame), connector_diffpress_sensor.connector_id, (TickType_t)portMAX_DELAY);
    }
}

