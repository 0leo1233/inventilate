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
#include "sensor_connect.h"

#ifdef CONNECTOR_BLE
#include "connector_ble.h"
#include "bluetooth_le.h"
#include "ble_central.h"
#endif

#include "app_error_code.h"

typedef enum _dpsens_err
{
    DPSENS_NO_ERROR = 0,
    DPSENS_BOARD_DISCONNECTED = 1,
    DPSENS_BOARD_CONN_RETRY = 2,
    DPSENS_NO_DATA_ERROR = 3,
    DPSENS_BOARD_CONNECTED = 4,
    DPSENS_DATA_PLAUSIBLE_ERROR = 5,
    DPSENS_BOARD_BATTERY_LOW = 6
} DPSENS_ERROR;


typedef void (*conn_diff_param_changed_t)(uint32_t param_id, int32_t i32Value);

typedef struct
{
    uint32_t ddm_parameter;
    DDM2_TYPE_ENUM type;
    uint8_t pub;
    uint8_t sub;
    int32_t i32Value;
    conn_diff_param_changed_t cb_func;
} conn_diff_press_sensor_param_t;


#define CONN_DP_SENS_SERV_LOGS  1
#define DP_ERR_DEBUG            1

#define DDMP_UNAVAILABLE                        ((uint8_t) 0xFFu)
#define DP_SENS_QUE_LEN                         ((osal_base_type_t)10)                  //DP sensor Queue length
#define DP_SENS_QUE_ITEM_SIZE                   ((osal_base_type_t) sizeof(QUE_DATA))   //DPsensor Queue size
#define BLE_SENSOR_POLL_TIME_TICKS              pdMS_TO_TICKS(60*1000)                  //BLE sensor poll time, 60 s
#define BLE_SCAN_MAX_RETRY_COUNT                3u                                      //BLE scan retry count
#define ENABLE_SEND_SDP_SENSOR_DATA             1                                       //Send pressure data enable bit
#define DISABLE_SEND_SDP_SENSOR_DATA            0                                       //send pressure data disable bit
#define SDP_SENSOR_SAMPLING_INTERNAL_LONG       65000
#define SDP_SENSOR_VALID_SAMPLING_INTERVAL      5000
#define DP_SENSOR_AVAILABLE                     ((uint8_t)1u)                           //DP sensor is available
#define DP_SENSOR_NOT_AVAILABLE                 ((uint8_t)0u)                           //NO DP sensor found
#define DP_SENSOR_BAT_MIN                       ((uint16_t)1800u)	                    //1000 changed to 1800
#define DP_SENSOR_BAT_MAX                       ((uint16_t)3600u)	                    //2700

/* Enum to map the received data in the queue with a DATA ID */
typedef enum _data_id
{
    DP_SENS_STAT = 1,
    BLE_SCAN_TIME_EXP = 2,
    DP_SENS_SERV_SM_STATE = 3,
    BLE_REQUEST = 4,
    DP_SENS_AVAIL_STAT = 5,
	DP_SENS_DATA = 6,
	DP_SENS_BATTERYLVL = 7,
    PWR_MGR_DATA = 8,
	INV_ERRSTAT = 9,
    DATA_INVALID = 10
} DATA_ID;

/* Enum for the states in the state machine */
typedef enum _diff_press_sm_state
{
    STATE_STANDBY = 0,
    STATE_STANDBY_CONNECTED = 1,
    STATE_WAIT_FOR_BL_REQ = 2,
    STATE_DP_SENS_BLE_PAIRING = 3,
    STATE_DP_SENS_PAIRED = 4
} DIFF_PRESS_SM_STATE;

typedef enum _dp_sens_stat
{
    DP_SENS_STAT_UNKNOWN = 0,
    DP_SENS_NODE_NOT_FOUND = 1,
    DP_SENS_NODE_FOUND = 2
} DP_SENS_STATUS;

/* Structure having the DP sensor related Information */
typedef struct _diff_press_read_sm
{
    bool ble_conn_periodic_tmr_exp;
    uint8_t retry_count;
    DP_SENS_STATUS dp_sens_node_stat;
    bool ble_scan_timer_exp;
    DIFF_PRESS_SM_STATE state;
    IV0BLREQ_ENUM bl_req;
    int32_t dp_sensor_availablity;
    uint32_t dp_sensor_data;
    int32_t dp_sensor_battlvl;
    IVPMGR0STATE_ENUM pmgr_state;
    uint32_t inv_dperr_stat;
    uint32_t instance;              // SNODE0 instance
    uint32_t sensor_class_instance; // SDP0 class instance
    void *sensor_connect_object;
} diff_press_read_sm;

/* Structure for the queued data */
typedef struct _que_data
{
    int32_t data;
    DATA_ID data_id;
} QUE_DATA;

static uint32_t invent_dp_error_stat = 0;
static DPSENS_ERROR dps_curr_err_stat = DPSENS_NO_ERROR;

/* Instance for State Machine */
static diff_press_read_sm sm_instance;
static osal_queue_handle_t dp_sens_queue;               //Handle for the queue used in this connector
static TimerHandle_t ble_sensor_poll_handle;            //Handle for 15sec timer

/* static function declarations */
static int initialize_connector_dp_sens_service(void);
static void start_subscribe_non_sensor_parameters(void);
static void start_subscribe_sensor_parameters(void);
static void install_parameters(void);
static void process_set_and_publish_request(uint32_t ddm_param, int32_t i32value, DDMP2_CONTROL_ENUM req_type);
static uint8_t get_ddm_index_from_db(uint32_t ddm_param);

static void conn_diffpress_process_task(void *pvParameter);
static void conn_diffpress_read_task(void *pvParameter);
static void push_data_to_que(DATA_ID data_id, int32_t i32Value);
static void parse_queue_data(diff_press_read_sm *ptr_inst , QUE_DATA* qdata);

static void change_diff_state(DIFF_PRESS_SM_STATE sm_state);
static void ble_sensor_poll_time_exp_cb(TimerHandle_t xTimer);
static void start_sensor_poll_timer(void);
static void stop_sensor_poll_timer(void);
static void configure_sdp_sensor(int32_t send_dp, int32_t sample_int);

static void handle_dpsens_data(uint32_t ddm_param, int32_t data);

static void inv_dp_err_stat_callback(uint32_t table_index, int32_t i32Value);
static void dpsens_connect_errchk(DPSENS_ERROR dpconnect_err);
static void dpsens_battlvl_errchk(int32_t battlvl);
static void dpsens_plausible_errchk(int32_t dp_data);


/* Structure for Connector VOC Sensor */
CONNECTOR connector_diffpress_sensor =
{
	.name       = "DiffPress Sensor Connector",
	.initialize = initialize_connector_dp_sens_service,
};

/* DDM Parameter table for connector differential pressure sensor */
static conn_diff_press_sensor_param_t conn_diffpress_sensor_param_db[] =
{
  //ddm_parameter      type             pub     sub     i32Value    cb_func
    {SDP0DP,        DDM2_TYPE_INT32_T,  0,      1, 	    0,          handle_dpsens_data},
    {SNODE0BATTLVL, DDM2_TYPE_INT32_T,  0,      1,      0,          handle_dpsens_data},
    {IV0BLREQ,      DDM2_TYPE_INT32_T,  0,      1,  	0,          handle_dpsens_data},
    {IVPMGR0STATE,  DDM2_TYPE_INT32_T,  0,      1,		0,          handle_dpsens_data},
    {IV0ERRST,      DDM2_TYPE_INT32_T,  0,      1, 	    0,          inv_dp_err_stat_callback}
};

/* Calculate the connector fan motor database table num elements */
static const uint32_t conn_diff_press_db_elements = ELEMENTS(conn_diffpress_sensor_param_db);

/**
  * @brief  Initialize the connector for differential pressure sensor service
  * @param  none.
  * @retval none.
  */
static int initialize_connector_dp_sens_service(void)
{
    /* Initialize the state machine state */
    sm_instance.state = STATE_STANDBY;

    /* Create queue for invent control task */
    dp_sens_queue = osal_queue_create(DP_SENS_QUE_LEN, DP_SENS_QUE_ITEM_SIZE);

    if (NULL != dp_sens_queue)
    {
#if CONN_DP_SENS_SERV_LOGS
        LOG(I, "Queue creation done for DP sensor");
#endif
    }

    /* Create the one shot timer software timer for BLE sensor poll timer */
    ble_sensor_poll_handle = xTimerCreate("ble_sensor_poll_handle", BLE_SENSOR_POLL_TIME_TICKS, pdFALSE, NULL, ble_sensor_poll_time_exp_cb);


    /* install ddmp parameters */
    install_parameters();

    start_subscribe_non_sensor_parameters();

    TRUE_CHECK(osal_task_create(conn_diffpress_process_task, CONNECTOR_DIFF_PRESS_PROCESS_TASK_NAME,
                CONNECTOR_DIFFPRESS_PROCESS_STACK_DEPTH, NULL, CONNECTOR_DIFFPRESS_PROCESS_TASK_PRIORITY, NULL));

    TRUE_CHECK(osal_task_create(conn_diffpress_read_task, CONNECTOR_DIFF_PRESS_READ_TASK_NAME,
                CONNECTOR_DIFFPRESS_READ_STACK_DEPTH, NULL, CONNECTOR_DIFFPRESS_READ_TASK_PRIORITY, NULL));

	return 1;
}

/**
  * @brief  Function to subscribe the DDMP parameters
  * @param  none.
  * @retval none.
  */
static void start_subscribe_non_sensor_parameters(void)
{
	conn_diff_press_sensor_param_t *ptr_param_db;
	uint8_t db_idx;
    uint8_t num_elements = ELEMENTS(conn_diffpress_sensor_param_db);

	for (db_idx = 0; db_idx < num_elements; db_idx++)
	{
		ptr_param_db = &conn_diffpress_sensor_param_db[db_idx];

        /* Check the DDM parameter need subscribtion */
		if (ptr_param_db->sub)
		{
		    // Cannot subscribe to any sensor related parameters here as we don't know which instances are allocated
		    if (DDM2_PARAMETER_GROUP(ptr_param_db->ddm_parameter) != DDM2_PARAMETER_GROUP(SNODE0))
		    {
		        TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SUBSCRIBE, ptr_param_db->ddm_parameter, NULL, 0,
                    connector_diffpress_sensor.connector_id, portMAX_DELAY));
		    }
        }
	}
}

static void start_subscribe_sensor_parameters(void)
{
    conn_diff_press_sensor_param_t *ptr_param_db;
    uint8_t db_idx;
    uint8_t num_elements = ELEMENTS(conn_diffpress_sensor_param_db);

    for (db_idx = 0; db_idx < num_elements; db_idx++)
    {
        ptr_param_db = &conn_diffpress_sensor_param_db[db_idx];

        /* Check the DDM parameter need subscription */
        if (ptr_param_db->sub)
        {
            // Subscribe to any sensor related parameters here as we now know which instances are allocated
            if (DDM2_PARAMETER_CLASS(ptr_param_db->ddm_parameter) == DDM2_PARAMETER_CLASS(SNODE0))
            {
                TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SUBSCRIBE, ptr_param_db->ddm_parameter | DDM2_PARAMETER_INSTANCE(sm_instance.instance), NULL, 0,
                    connector_diffpress_sensor.connector_id, portMAX_DELAY));
            }
            else if (DDM2_PARAMETER_CLASS(ptr_param_db->ddm_parameter) == DDM2_PARAMETER_CLASS(SDP0))
            {
                TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SUBSCRIBE, DDM2_PARAMETER_BASE_INSTANCE(ptr_param_db->ddm_parameter) | DDM2_PARAMETER_INSTANCE_PART(sm_instance.sensor_class_instance), NULL, 0,
                    connector_diffpress_sensor.connector_id, portMAX_DELAY));
            }
        }
    }
}

/**
  * @brief  Function to publish Inventilate available to the broker
  * @param  none.
  * @retval none.
  */
static void install_parameters(void)
{
    // Instantiate, initialize and start sensor_connect object
    sm_instance.sensor_connect_object = sensor_connect_create();
    ASSERT(sm_instance.sensor_connect_object != NULL);
    sensor_connect_init(sm_instance.sensor_connect_object, &connector_diffpress_sensor, "dp_sensor");
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

	if (DDMP_UNAVAILABLE != db_idx)
	{
#if CONN_DP_SENS_SERV_LOGS
        LOG(I, "Valid DDMP parameter ");
#endif

        if (DDMP2_CONTROL_SET == req_type)
        {
            /* Frame and send the publish request */
            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, ddm_param, &i32value, sizeof(int32_t),
                        connector_diffpress_sensor.connector_id, portMAX_DELAY));
        }

		param_db = &conn_diffpress_sensor_param_db[db_idx];

        i32Index = ddm2_parameter_list_lookup(DDM2_PARAMETER_BASE_INSTANCE(ddm_param));

        if (-1 != i32Index)
        {
            i32Factor = Ddm2_unit_factor_list[Ddm2_parameter_list_data[i32Index].in_unit];

            /* Differential pressure value should be handle with the decimal point resolution
               For example 1.24 (pascal) value will be received as 1240 (i.e 1.24 * 1000 )
               So if we again divide this by factor(1000) then the resolution could be lost, So
               to avoid this, Divide by factor operation should be skipped for IVSDP0DP
            */
            i32Factor = (i32Factor == 0) ? 1 : i32Factor;

            i32value = i32value / i32Factor;

            if (i32value != param_db->i32Value)
            {
                /* Update the received value in the database table*/
	  	        param_db->i32Value = i32value;
		        /* Check callback function registered for this DDM parameter */
		        if  (NULL != param_db->cb_func)
		        {
		   	        /* Execute the callback function */
                    param_db->cb_func(param_db->ddm_parameter, param_db->i32Value);
		        }
            }
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
	conn_diff_press_sensor_param_t* param_db;
	uint8_t db_idx = DDMP_UNAVAILABLE;
	uint8_t index;
	bool avail = false;

	for (index = 0u; ((index < conn_diff_press_db_elements) && (avail == false)); index++ )
 	{
		param_db = &conn_diffpress_sensor_param_db[index];

		/* Validate the DDM parameter received */
		if (param_db->ddm_parameter == DDM2_PARAMETER_BASE_INSTANCE(ddm_param))
	  	{
			db_idx = index;
			avail = true;
		}
	}

	return db_idx;
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

	sensor_connect_start(sm_instance.sensor_connect_object);
	while (1)
	{
		TRUE_CHECK (pframe = xRingbufferReceive(connector_diffpress_sensor.to_connector, &frame_size, portMAX_DELAY));
#if CONN_DP_SENS_SERV_LOGS
        ESP_LOG_BUFFER_HEXDUMP("Conn_dp_sens ", pframe, frame_size, ESP_LOG_INFO);
#endif
        if (!sensor_connect_update(sm_instance.sensor_connect_object, pframe))
        {
            // Check if not already handled by sensor_connect handler
            switch (pframe->frame.control)
            {
            case DDMP2_CONTROL_GENERIC:
                if (pframe->frame.generic.id == SENSOR_CONNECT_STATUS_GENERIC_PARAMETER)
                {
                    QUE_DATA iv_data;
                    iv_data.data_id = DP_SENS_AVAIL_STAT;
                    iv_data.data = 0;
                    sensor_connect_status_t *value = (sensor_connect_status_t*)pframe->frame.generic.data;
                    /* set the data */
                    if (value->status == SENSOR_AVAILABLE)
                    {
                        LOG(D, "Sensor available: %x (%x)", value->sensor, value->classes[0]);
                        iv_data.data = DP_SENSOR_AVAILABLE;
                        // Save instances for this sensor
                        sm_instance.instance = value->sensor;
                        sm_instance.sensor_class_instance = value->classes[0];
                    }
                    else if (value->status == SENSOR_NOT_AVAILABLE)
                    {
                        LOG(D, "Sensor not available");
                        iv_data.data = DP_SENSOR_NOT_AVAILABLE;
                    }
                    else if (value->status == SENSOR_CONNECT_FAILED)
                    {
                        LOG(D, "Sensor connection failed");
                        dps_curr_err_stat = DPSENS_BOARD_CONN_RETRY;
                        dpsens_connect_errchk(dps_curr_err_stat);
                        iv_data.data_id = DATA_INVALID;
                    }
                    push_data_to_que(iv_data.data_id, iv_data.data);
                }
                break;
            case DDMP2_CONTROL_PUBLISH:/* Send the changed data to broker */
                process_set_and_publish_request(pframe->frame.publish.parameter, pframe->frame.publish.value.int32, pframe->frame.control);
                break;

//            case DDMP2_CONTROL_SET:
//                process_set_and_publish_request(pframe->frame.set.parameter, pframe->frame.set.value.int32,pframe->frame.control);
//                break;

//            case DDMP2_CONTROL_SUBSCRIBE:
//                process_subscribe_request(pframe->frame.subscribe.parameter);
//                break;

            default:
    #if CONN_DP_SENS_SERV_LOGS
                LOG(W, "Connector diff press sensor sensor received UNHANDLED frame %02x from broker!",pframe->frame.control);
    #endif
                break;
            }
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
    QUE_DATA qdata;
    diff_press_read_sm *ptr_inst = &sm_instance;
    DIFF_PRESS_SM_STATE sm_next_state = ptr_inst->state;

    while (1)
	{
        if (pdPASS == xQueueReceive(dp_sens_queue, (void *)&qdata, portMAX_DELAY))
        {
#if CONN_DP_SENS_SERV_LOGS
            LOG(I, "Data received from que data=%d data_id=%d", qdata.data, qdata.data_id);
#endif
            parse_queue_data(ptr_inst, &qdata);
        }

        switch (ptr_inst->state)
        {
            case STATE_STANDBY:
                if (IVPMGR0STATE_ACTIVE == ptr_inst->pmgr_state)
                {
                    /* Reset the BL Request flag */
                    ptr_inst->bl_req = IV0BLREQ_IDLE;
                    /* Change the state */
                    sm_next_state = STATE_WAIT_FOR_BL_REQ;
                }
                else if (DP_SENSOR_AVAILABLE == ptr_inst->dp_sensor_availablity)
                {
                    /* Inventilate system is not Turned ON by user, So no need for DP sensor data */
                    configure_sdp_sensor(DISABLE_SEND_SDP_SENSOR_DATA, SDP_SENSOR_SAMPLING_INTERNAL_LONG);
                    /* Change the state */
                    sm_next_state = STATE_STANDBY_CONNECTED;
                    // Start timer to request new data
                    stop_sensor_poll_timer();
                    start_sensor_poll_timer();
                }
                else
                {
                    /* Do nothing */
                }
                break;
            case STATE_STANDBY_CONNECTED:
                if (IVPMGR0STATE_ACTIVE == ptr_inst->pmgr_state)
                {
                    /* Reset the BL Request flag */
                    ptr_inst->bl_req = IV0BLREQ_IDLE;
                    /* Change the state */
                    sm_next_state = STATE_WAIT_FOR_BL_REQ;
                }
                else if (DP_SENSOR_NOT_AVAILABLE == ptr_inst->dp_sensor_availablity)
                {
                    /* Change the state */
                    sm_next_state = STATE_STANDBY;
                }
                else
                {
                    /* Do nothing */
                }
                break;
            case STATE_WAIT_FOR_BL_REQ:
#if CONN_DP_SENS_SERV_LOGS
                LOG(W, "STATE_WAIT_FOR_BL_REQ bl_req = %d dp_sensor_availablity = %d", ptr_inst->bl_req, ptr_inst->dp_sensor_availablity);
#endif
                if (IVPMGR0STATE_STANDBY == ptr_inst->pmgr_state)
                {
                    // stop_sensor_poll_timer();
                    /* Change the state */
                    sm_next_state = STATE_STANDBY;
                }
                else if (DP_SENSOR_AVAILABLE == ptr_inst->dp_sensor_availablity)
                {
#if CONN_DP_SENS_SERV_LOGS
                    LOG(W, "Paired with existing DP sensor node");
#endif
                    // Start timer to request new data
                    stop_sensor_poll_timer();
                    start_sensor_poll_timer();

                    /* Reset the BL Request flag */
                    ptr_inst->bl_req = IV0BLREQ_IDLE;
                    /* Configure the SDP sensor */
                    configure_sdp_sensor(ENABLE_SEND_SDP_SENSOR_DATA, SDP_SENSOR_VALID_SAMPLING_INTERVAL);
                    /* Change the state */
                    sm_next_state = STATE_DP_SENS_PAIRED;
                }
                else if ( ( IV0BLREQ_SCAN == ptr_inst->bl_req ) /*|| ( IV0BLREQ_PAIR == ptr_inst->bl_req )*/ )
                {
#if CONN_DP_SENS_SERV_LOGS
                    LOG(I, "DP Sensor BLE scan (BT0SCAN) req sent");
#endif
                    /* Reset the BL Request flag */
                    ptr_inst->bl_req = IV0BLREQ_IDLE;
                    /* Send the request */
                    uint8_t sensors[1] = { SENSOR_DIFFERENTIAL_PRESSURE_SENSOR };
                    sensor_connect_request(sm_instance.sensor_connect_object, NODE_CLASS_DOMETIC, sensors, 1);
                }
                else
                {
                    /* Do nothing */
                }
                break;

            case STATE_DP_SENS_PAIRED:
                if ((IVPMGR0STATE_STANDBY == ptr_inst->pmgr_state) ||
                    (IVPMGR0STATE_STORAGE == ptr_inst->pmgr_state))
                {
                    /* Change the state */
                    sm_next_state = STATE_STANDBY;
                }
                else if (DP_SENSOR_NOT_AVAILABLE == ptr_inst->dp_sensor_availablity)
                {
                    //DP_Sensor not available or disconnected.
                    stop_sensor_poll_timer();
                    /* Reset the BL Request flag */
                    ptr_inst->bl_req = IV0BLREQ_IDLE;
                    /* Change the state */
                    sm_next_state = STATE_WAIT_FOR_BL_REQ;
                    dpsens_connect_errchk(DPSENS_BOARD_DISCONNECTED);
                }
                else if (ptr_inst->bl_req != IV0BLREQ_IDLE)
                {
                    // Ignore the BLE pair/scan request
#if CONN_DP_SENS_SERV_LOGS
                    LOG(W, "DP Sensor paired already");
#endif
                }
                else
                {
                    dpsens_connect_errchk(DPSENS_BOARD_CONNECTED);
                    /* Do nothing */
                }
                break;

            default:
                break;
        }

        if (sm_next_state != ptr_inst->state)
        {
            change_diff_state(sm_next_state);
        }

#if CONN_DP_SENS_SERV_LOGS
        LOG(I, "state = %d", ptr_inst->state);
#endif
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
    qdata.data = sm_state;
    qdata.data_id = DP_SENS_SERV_SM_STATE;

    // Append the data in the queue
    osal_base_type_t ret = xQueueSendToFront(dp_sens_queue, &qdata, portMAX_DELAY);

    if (osal_success != ret)
    {
#if CONN_DP_SENS_SERV_LOGS
        LOG(E, "Queue error ret = %d", ret);
#endif
    }
}

/**
  * @brief  Callback function to handle the BLE sensor poll time expired and
  *         request new sensor data
  * @param  xTimer timer handle.
  * @retval none.
  */
static void ble_sensor_poll_time_exp_cb(TimerHandle_t xTimer)
{
    // read battery voltage
    TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SUBSCRIBE, SNODE0BATTLVL | DDM2_PARAMETER_INSTANCE(sm_instance.instance), NULL, 0,
                connector_diffpress_sensor.connector_id, portMAX_DELAY));
    start_sensor_poll_timer();
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
    qdata.data = i32Value;
    qdata.data_id = data_id;

    // push the data in the Queue
    osal_base_type_t ret = osal_queue_send(dp_sens_queue, &qdata, portMAX_DELAY);

    if (osal_success != ret)
    {
        LOG(W, "DP Sensor Queue error ret = %d", ret);
    }
}

/**
  * @brief  Callback function to handle all the subscribed parameters.
  *         Few parameters are used for algorithm
  * @param  ddm_param - DDM Parameter with instance
  * @param  data      - Value for the DDM Parameter
  * @retval none
  */
static void handle_dpsens_data(uint32_t ddm_param, int32_t data)
{
    QUE_DATA iv_data;

    /* set the data */
    iv_data.data = data;
    iv_data.data_id = 0;

    switch (ddm_param)
    {
        case IV0BLREQ:
            iv_data.data_id = BLE_REQUEST;
            break;

        case SDP0DP:
            iv_data.data_id = DP_SENS_DATA;
#ifdef DP_ERR_DEBUG
            LOG(I, "DP DATA = %d", iv_data.data);
#endif
            dpsens_plausible_errchk(iv_data.data);
            break;

        case SNODE0BATTLVL:
            iv_data.data_id = DP_SENS_BATTERYLVL;
            dpsens_battlvl_errchk(iv_data.data);
            break;

        case IVPMGR0STATE:
            iv_data.data_id = PWR_MGR_DATA;
            break;

        case IV0ERRST:
            iv_data.data_id = INV_ERRSTAT;
            break;

        default:
            iv_data.data_id = DATA_INVALID;
            break;
    }

    push_data_to_que(iv_data.data_id, iv_data.data);
}

/**
  * @brief  Funtion to parse the queue data and update in the structure variables
  * @param  ptr_inst Pointer to the structure diff_press_read_sm.
  * @retval qdata Pointer to the structure QUE_DATA.
  */
static void parse_queue_data(diff_press_read_sm *ptr_inst , QUE_DATA* qdata)
{
    switch (qdata->data_id)
    {
        case DP_SENS_STAT:
            ptr_inst->dp_sens_node_stat = qdata->data;
            break;

        case BLE_SCAN_TIME_EXP:                         /* BLE Scan_timer expired */
            ptr_inst->ble_scan_timer_exp = true;
            break;

        case DP_SENS_SERV_SM_STATE:
            ptr_inst->state = qdata->data;
            break;

        case BLE_REQUEST:
            ptr_inst->bl_req = qdata->data;
            break;

        case DP_SENS_AVAIL_STAT:
            if (ptr_inst->dp_sensor_availablity == DP_SENSOR_NOT_AVAILABLE)
            {
                ptr_inst->dp_sensor_availablity = qdata->data;
                if (ptr_inst->dp_sensor_availablity == DP_SENSOR_AVAILABLE)
                {
                    // start subscribe to sensor parameters
                    start_subscribe_sensor_parameters();
                }
            }
            else
            {
                if (ptr_inst->dp_sensor_availablity == DP_SENSOR_AVAILABLE)
                {
                    if (qdata->data == DP_SENSOR_NOT_AVAILABLE)
                    {
                        // Stop timer to request new data
                        stop_sensor_poll_timer();
                    }
                }
                ptr_inst->dp_sensor_availablity = qdata->data;
            }
            break;

        case DP_SENS_DATA:
            ptr_inst->dp_sensor_data = qdata->data;
            break;

        case DP_SENS_BATTERYLVL:
            ptr_inst->dp_sensor_battlvl = qdata->data;
            break;

        case PWR_MGR_DATA:
            ptr_inst->pmgr_state = qdata->data;
            break;

        case INV_ERRSTAT:
            ptr_inst->inv_dperr_stat = qdata->data;
            break;

        default:
            break;
    }
}

/**
  * @brief  Function to start the timer for BLE scan
  * @param  none.
  * @retval none.
  */
static void start_sensor_poll_timer(void)
{
    BaseType_t tmr_started = xTimerStart(ble_sensor_poll_handle, portMAX_DELAY);

    if (tmr_started == pdPASS)
    {
		LOG(D, "BLE sensor poll timer started");
    }
}

/**
  * @brief  Function to stop the timer for BLE sensor poll
  * @param  none.
  * @retval none.
  */
static void stop_sensor_poll_timer(void)
{
    BaseType_t tmr_stopped = xTimerStop(ble_sensor_poll_handle, portMAX_DELAY);

    if (tmr_stopped == pdPASS)
    {
		LOG(D, "BLE sensor poll timer stopped");
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
    TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, SDP0SENDDP | DDM2_PARAMETER_INSTANCE_PART(sm_instance.sensor_class_instance), (const void*)&send_dp, sizeof(int32_t),
                connector_diffpress_sensor.connector_id, portMAX_DELAY));
    TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, SDP0SAMPP | DDM2_PARAMETER_INSTANCE_PART(sm_instance.sensor_class_instance), (const void*)&sample_int, sizeof(int32_t),
                connector_diffpress_sensor.connector_id, portMAX_DELAY));
}

/**
  * @brief  Function to check the DP sensor data is within the valid range and update the error status
  * @param  dp_data
  * @retval none.
  */
static void dpsens_plausible_errchk(int32_t dp_data)
{
    uint32_t err_frame = invent_dp_error_stat;  //inv_dperr_stat

    if ((dp_data > DPSENS_PLAUSIBLE_UPRANGE) && (dp_data > 0))
    {
        err_frame |= 1 << DP_SENSOR_DATA_PLAUSIBLE_ERROR;
#ifdef DP_ERR_DEBUG
        LOG(I, "DP PLAUSIBLE ERRSET OVER");
#endif
    }
    else if ((dp_data < DPSENS_PLAUSIBLE_DOWNRANGE) && (dp_data < 0))
    {
        err_frame |= (1 << DP_SENSOR_DATA_PLAUSIBLE_ERROR);
#ifdef DP_ERR_DEBUG
        LOG(I, "DP PLAUSIBLE ERRSET UNDER");
#endif
    }
    else if (((dp_data > 0) && (dp_data < DPSENS_PLAUSIBLE_UPRANGE)) || ((dp_data < 0) && (dp_data > DPSENS_PLAUSIBLE_DOWNRANGE)))
    {
        err_frame &= ~ (1 <<  DP_SENSOR_DATA_PLAUSIBLE_ERROR);
#ifdef DP_ERR_DEBUG
        LOG(I, "No DP PLAUSIBLE ERR");
#endif
    }

    if (err_frame != invent_dp_error_stat)
    {
        TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, IV0ERRST, &err_frame, sizeof(int32_t),
                connector_diffpress_sensor.connector_id, portMAX_DELAY));
    }
    invent_dp_error_stat = err_frame;
}

/**
  * @brief  Function to check the DP sensor battery level and update the error status
  * @param  battlvl
  * @retval none.
  */
static void dpsens_battlvl_errchk(int32_t battlvl)
{
    uint32_t err_frame = invent_dp_error_stat;  //inv_dperr_stat

    /* Battery level within the range - clear the error */
    if ((battlvl > DP_SENSOR_BAT_MIN) && (battlvl < DP_SENSOR_BAT_MAX))
    {
        err_frame &= ~(1 << DP_SENSOR_BOARD_BATTERY_LOW);  //Clear Battery Low error code
    }
    else        /* Battery level out of range - set the error */
    {
        err_frame |= (1 << DP_SENSOR_BOARD_BATTERY_LOW);     //Set Battery low Error code
    }

    if (err_frame != invent_dp_error_stat)
    {
        TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, IV0ERRST, &err_frame, sizeof(int32_t),
                connector_diffpress_sensor.connector_id, portMAX_DELAY));
    }
    invent_dp_error_stat = err_frame;
}

/**
  * @brief  Function to check the DP connection and update the error status
  * @param  dpconnect_err - DPSENS_BOARD_DISCONNECTED, DPSENS_BOARD_CONN_RETRY, DPSENS_BOARD_CONNECTED
  * @retval none.
  */
static void dpsens_connect_errchk(DPSENS_ERROR dpconnect_err)
{
    uint32_t err_frame = invent_dp_error_stat;  //inv_dperr_stat

    if ((DPSENS_BOARD_DISCONNECTED == dpconnect_err) || (DPSENS_BOARD_CONN_RETRY == dpconnect_err))
    {
        err_frame |= (1 << DP_SENSOR_BOARD_DISCONNECTED);     //DP Sensor board is disconnected
    }
    else if (DPSENS_BOARD_CONNECTED == dpconnect_err)
    {
        err_frame &= ~(1 << DP_SENSOR_BOARD_DISCONNECTED); //DP Sensor board is connected
    }

    if (err_frame != invent_dp_error_stat)
    {
        LOG(D, "err_frame: %x", err_frame);
        TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, IV0ERRST, &err_frame, sizeof(int32_t),
                connector_diffpress_sensor.connector_id, portMAX_DELAY));
    }
    invent_dp_error_stat = err_frame;
}

/**
  * @brief  Callback function to handle dp sensor error condition
  * @param  table_index
  * @param  i32Value.
  * @retval none.
  */
static void inv_dp_err_stat_callback(uint32_t table_index, int32_t i32Value)
{
    /* Update error status value */
    invent_dp_error_stat = i32Value;
}

