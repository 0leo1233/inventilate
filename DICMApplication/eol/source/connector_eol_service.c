/*! \file connector_eol_service.c
	\brief Connector for eol service
	\Author Sundaramoorthy-LTTS
 */

/** Includes ******************************************************************/
#include "configuration.h"

#ifdef CONNECTOR_EOL_SERVICE
#include "connector.h"
#include "ble_central.h"
#include "connector_eol_service.h"
#include "connector_lin_comm_test.h"
#include "ddm2_parameter_list.h"
#include "osal.h"
#include "hal_pwm.h"
#include "hal_ledc.h"
#include "sorted_list.h"
#include "linear_interpolation.h"
#include "app_api.h"
#include "hal/can_types.h"
#include "driver/can.h"

#define EOL_CONTROL_TASK_QUEUE_LENGTH			  ((osal_base_type_t) 10)
#define EOL_CONTROL_TASK_QUEUE_ITEM_SIZE   		  ((osal_base_type_t) sizeof(EOL_DATA_FRAME))
#define BT_SCAN_TIMER_PERIOD                      pdMS_TO_TICKS(5000)

osal_queue_handle_t eol_ctrl_que_handle;
static TimerHandle_t one_shot_tmr_hdle;

/* Static Function declarations */
static int initialize_connector_eol(void);
static void conn_eol_process_task(void *pvParameter);
static void install_parameters(void);
static void start_subscribe(void);
static void start_publish(void);
static void conn_eol_ctrl_task(void *pvParameter);
static void handle_eol_req(uint32_t ddm_param, int32_t data);
static void handle_eol_data(uint32_t ddm_param, int32_t data);
static uint8_t get_ddm_index_from_db(uint32_t ddm_param);
static void process_subscribe_request(uint32_t ddm_param);
static int add_subscription(DDMP2_FRAME *pframe);
#if 0
void parse_eol_data(EOL_CONTROL_ALGO* eol_ctrl, EOL_DATA_FRAME* eol_data_frame);
#endif
static void push_data_eol_queue(EOL_DATA_FRAME* eol_data_frame);
static void process_set_and_publish_request(uint32_t ddm_param, int32_t i32value, DDMP2_CONTROL_ENUM req_type);
static void handle_tacho_data(uint32_t ddm_param, int32_t data);
static void initialize_eol(void);
static void process_ble_scan_data(const uint8_t * const value);
static WIFI0STS_ENUM check_wifi(int8_t *rssi_value);
static void stop_eol_test(void);
static void bt_scan_tmr_expired_cb( TimerHandle_t xTimer );
void can_call(void);


//extern void test_lin_transmit(void);
//extern int rs485_tranmit_data(char* data, uint16_t data_len);

uint16_t fan_rpm_eol[3] = {EOL_FAN_MIN_RPM, EOL_FAN_MID_RPM, EOL_FAN_MAX_RPM};
uint32_t tacho_val[3];

typedef struct eol_serv_status
{
    WIFI0STS_ENUM wifi_status;
    uint16_t        iaq_value;
    uint8_t       num_ble_dev;
}EOL_SERV_STATUS;

static EOL_SERV_STATUS eol_serv_status;

/* Structure for Connector eol*/
CONNECTOR connector_eol =
{
	.name = "Connector EOL",
	.initialize = initialize_connector_eol,
};

/* DDM Parameter table for connector eol*/
static conn_eol_parameter_t conn_eol_param_db[] =
{
    {.ddm_parameter = IVEOL0REQ,               .type = DDM2_TYPE_INT32_T, .pub = 0, .sub = 1, .i32Value =                0, .cb_func =      handle_eol_req},
    {.ddm_parameter = IVEOL0RESP,              .type = DDM2_TYPE_INT32_T, .pub = 1, .sub = 0, .i32Value =                0, .cb_func =                 NULL},
    {.ddm_parameter = MTR0TACHO|DDM2_PARAMETER_INSTANCE(0),   .type = DDM2_TYPE_INT32_T, .pub = 0, .sub = 1, .i32Value =                0, .cb_func =    handle_tacho_data},
    {.ddm_parameter = MTR0TACHO|DDM2_PARAMETER_INSTANCE(1),   .type = DDM2_TYPE_INT32_T, .pub = 0, .sub = 1, .i32Value =                0, .cb_func =    handle_tacho_data},
    {.ddm_parameter = MTR0TACHO|DDM2_PARAMETER_INSTANCE(2),   .type = DDM2_TYPE_INT32_T, .pub = 0, .sub = 1, .i32Value =                0, .cb_func =    handle_tacho_data},
    {.ddm_parameter = SBMEB0IAQ,               .type = DDM2_TYPE_INT32_T, .pub = 0, .sub = 1, .i32Value =                0, .cb_func =      handle_eol_data},
    {.ddm_parameter = BT0SCAN,                 .type = DDM2_TYPE_OTHER,   .pub = 0, .sub = 1, .i32Value =                0, .cb_func =                 NULL},
    {.ddm_parameter = WIFI0STS,                .type = DDM2_TYPE_INT32_T, .pub = 0, .sub = 1, .i32Value = WIFI0STS_UNKNOWN, .cb_func =      handle_eol_data},
};

/* Calculate the connector eol database table num elements */
static const uint32_t conn_eol_db_elements = ELEMENTS(conn_eol_param_db);

DECLARE_SORTED_LIST_EXTRAM(conn_eol_table, CONN_EOL_SUB_DEPTH);       //!< \~ Subscription table storage 

/**
  * @brief  Initialize the connector eol
  * @param  none.
  * @retval none.
  */
static int initialize_connector_eol(void)
{
    osal_ubase_type_t ret;

    /* Create queue for invent control task */
    eol_ctrl_que_handle = osal_queue_create(EOL_CONTROL_TASK_QUEUE_LENGTH, EOL_CONTROL_TASK_QUEUE_ITEM_SIZE);

    if ( NULL != eol_ctrl_que_handle )
    {
        LOG(I, "Queue creation done for eol control task");
    }

	/* Initialize the eol */
    initialize_eol();

    /* Create the one shot timer software timer */
    one_shot_tmr_hdle = xTimerCreate("one_sh_tmr", BT_SCAN_TIMER_PERIOD, pdFALSE, 0, bt_scan_tmr_expired_cb);

    /* Task for handling the DDMP request */
	TRUE_CHECK(osal_task_create(conn_eol_process_task, CONNECTOR_EOL_PROCESS_TASK_NAME, CONNECTOR_EOL_PROCESS_STACK_DEPTH, NULL, CONNECTOR_EOL_PROCESS_TASK_PRIORITY, NULL));

    /* Task for control the EOL by an algorithm designed based on UART data */
    TRUE_CHECK(osal_task_create(conn_eol_ctrl_task, CONNECTOR_EOL_CONTROL_TASK_NAME, CONNECTOR_EOL_CONTROL_STACK_DEPTH, eol_ctrl_que_handle, CONNECTOR_EOL_CTRL_TASK_PRIORITY, NULL));

	/* Install parameters in the Inventory of broker */
	install_parameters();

	/* Subscribe DDMP parameters */
	start_subscribe();

    /* Publish all DDMP parameters */
    start_publish();
    
    
    ret = xTimerStart( one_shot_tmr_hdle, portMAX_DELAY );
        
    if ( ret != pdPASS )
    {
        LOG(E, "Timer start failed = %d", ret);
    }

	return 1;
}

/**
  * @brief  Function to subscribe the DDMP parameters needed for connector eol
  * @param  none.
  * @retval none.
  */
static void start_subscribe(void)
{
	conn_eol_parameter_t *ptr_param_db;
	uint8_t db_idx;

	for ( db_idx = 0; db_idx < conn_eol_db_elements; db_idx++ )
	{
		ptr_param_db = &conn_eol_param_db[db_idx];
        
        /* Check the DDM parameter need subscribtion */
		if ( ptr_param_db->sub )
		{
            LOG(I, "Subscribed DDMP for %s is 0x%x", connector_eol.name, ptr_param_db->ddm_parameter);
            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SUBSCRIBE, ptr_param_db->ddm_parameter, &ptr_param_db->i32Value, \
                       sizeof(int32_t), connector_eol.connector_id, portMAX_DELAY));
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
    conn_eol_parameter_t *ptr_param_db;
    uint16_t db_idx;
    
    for ( db_idx = 0; db_idx < conn_eol_db_elements; db_idx++ )
    {
        ptr_param_db = &conn_eol_param_db[db_idx];

        /* Check the DDM parameter need to publish */
        if ( ptr_param_db->pub ) 
        {
            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, ptr_param_db->ddm_parameter, &ptr_param_db->i32Value, \
                       sizeof(int32_t), connector_eol.connector_id, portMAX_DELAY));
        }
    }
}

/**
  * @brief  Task for connector eol for processing the DDMP request received from broker
  * @param  pvParameter.
  * @retval none.
  */
static void conn_eol_process_task(void *pvParameter)
{
    DDMP2_FRAME *pframe;
	size_t frame_size;

	while (1)
	{
		TRUE_CHECK ( pframe = xRingbufferReceive(connector_eol.to_connector, &frame_size, portMAX_DELAY) );

		switch (pframe->frame.control)
		{
		case DDMP2_CONTROL_PUBLISH:
#if CONN_EOL_DEBUG_LOG
		    LOG(I, "Received DDMP2_CONTROL_PUBLISH");
#endif 
            if ( BT0SCAN != pframe->frame.publish.parameter )
            {
                process_set_and_publish_request(pframe->frame.publish.parameter, pframe->frame.publish.value.int32,pframe->frame.control);
            }
            else
            {
                size_t payload_size = ddmp2_value_size(pframe);
                LOG(I, "Received DDMP2_CONTROL_PUBLISH: BT0SCAN payload_size = %d", payload_size);
                
                if ( payload_size > 0 )
                {
                    process_ble_scan_data(pframe->frame.publish.value.raw);
                }
                
            }
			break;

		case DDMP2_CONTROL_SET:
#if CONN_EOL_DEBUG_LOG
		    LOG(I, "Received DDMP2_CONTROL_SET");
#endif 
		    process_set_and_publish_request(pframe->frame.set.parameter, pframe->frame.set.value.int32, pframe->frame.control);
			break;

		case DDMP2_CONTROL_SUBSCRIBE:
#if CONN_EOL_DEBUG_LOG
		    LOG(I, "Received DDMP2_CONTROL_SUBSCRIBE");
#endif 
			add_subscription(pframe);
			process_subscribe_request(pframe->frame.subscribe.parameter);
			break;

		default:
			LOG(E, "UNHANDLED frame %02x from broker!",pframe->frame.control);
			break;
		}

		vRingbufferReturnItem(connector_eol.to_connector, pframe);
    }
}

/**
  * @brief  Function to publish Connector EOL provided available classes to the broker
  * @param  none.
  * @retval none.
  */
static void install_parameters(void)
{
    int32_t available = 1;

    // Send the request for register eol devices instance to broker
    TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, IVEOL0AVL, &available, sizeof(int32_t), \
               connector_eol.connector_id, portMAX_DELAY));
}

/**
  * @brief  Initialize dev fan and motor
  * @param  none.
  * @retval none.
  */
static void initialize_eol(void)
{
    LOG(I, "EOL INITIALIZE.");
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
    int32_t pub_value = i32value;
	uint16_t db_idx;
	conn_eol_parameter_t* param_db;

#if CONN_EOL_DEBUG_LOG
    LOG(I, "Received ddm_param = 0x%x i32value = %d", ddm_param, i32value);
#endif

	/* Validate the DDM parameter received */
	db_idx = get_ddm_index_from_db(ddm_param);
 
	if ( DDMP_UNAVAILABLE != db_idx )
	{
        if ( DDMP2_CONTROL_SET == req_type )
        {
            /* Frame and send the publish request */
            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, ddm_param, &pub_value, sizeof(int32_t), \
                        connector_eol.connector_id, portMAX_DELAY));
        }

		param_db = &conn_eol_param_db[db_idx];

#if CONN_EOL_DEBUG_LOG		
		LOG(I, "Valid DDMP parameter");
#endif
        i32Index = ddm2_parameter_list_lookup(DDM2_PARAMETER_BASE_INSTANCE(ddm_param));

        if ( -1 != i32Index )
        {
            if (  i32Index < DDM2_PARAMETER_COUNT )
                i32Factor = Ddm2_unit_factor_list[Ddm2_parameter_list_data[i32Index].in_unit];
            else
                i32Factor = 1;

            if ( i32Factor > 0 )
            {
                i32value = i32value / i32Factor;
            }
                
#if CONN_EOL_DEBUG_LOG
            LOG(I, "After factored i32value = %d", i32value);
#endif
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
    uint32_t list_value = 0;
	conn_eol_parameter_t* param_db;
    SORTED_LIST_RETURN_VALUE ret = sorted_list_unique_get(&list_value, &conn_eol_table, ddm_param, 0);

#if CONN_EOL_DEBUG_LOG
    LOG(I, "Received ddm_param = 0x%x ret = %d", ddm_param, ret);
#endif

    if ( SORTED_LIST_FAIL != ret )
	{
		/* Validate the DDM parameter received */
		db_idx = get_ddm_index_from_db(ddm_param);

		if ( DDMP_UNAVAILABLE != db_idx )
	  	{
			param_db = &conn_eol_param_db[db_idx];

            index = ddm2_parameter_list_lookup(DDM2_PARAMETER_BASE_INSTANCE(ddm_param));

#if CONN_EOL_DEBUG_LOG
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
#if CONN_PWM_DEBUG_LOG
                LOG(I, "After factored i32value = %d", value);
#endif
                /* Frame and send the publish request */
                TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, ddm_param, &value, sizeof(int32_t), \
                           connector_eol.connector_id, portMAX_DELAY));
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

/**
  * @brief  Add device to inventory if it does not already exists
  * @param  DDMP Frame.
  * @retval result 0 - Succesfully added to list / 1 - Fail.
  */
static int add_subscription(DDMP2_FRAME *pframe)
{
    SORTED_LIST_KEY_TYPE     key = pframe->frame.subscribe.parameter;
    SORTED_LIST_VALUE_TYPE value = 1;

    return sorted_list_single_add(&conn_eol_table, key, value);
}

/**
  * @brief  Function to get ddm index from database table
  * @param  DDMP Parameter.
  * @retval none.
  */
static uint8_t get_ddm_index_from_db(uint32_t ddm_param)
{
	conn_eol_parameter_t* param_db;
	uint8_t db_idx = DDMP_UNAVAILABLE; 
	uint8_t index;
	bool avail = false;

	for ( index = 0u; ( ( index < conn_eol_db_elements ) && ( avail == false ) ); index ++ )
 	{
		param_db = &conn_eol_param_db[index];
      	
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
  * @brief  Function to push the data frame into the queue
  * @param  data.
  * @retval data_id refer the enum DATA_ID.
  */
static void push_data_eol_queue(EOL_DATA_FRAME* eol_data_frame) 
{
    // Append the data in the queue
    osal_base_type_t ret = osal_queue_send (eol_ctrl_que_handle, eol_data_frame, 0);

    if ( osal_success != ret )
    {
        LOG(E, "Queue error ret = %d", ret);
    }
}

#if 0 
/**
  * @brief  Task to control eol functionalities
  * @param  pvParameter.
  * @retval none.
  */
static void conn_eol_ctrl_task(void *pvParameter)
{
    EOL_DATA_FRAME eol_data_frame;
    uint8_t resp_data[4] = {0x00,0x00, 0x00, 0x00};
    uint32_t u32data = 0;
    int8_t rssi = 0;
    uint8_t index = 0;

    while (1)
    {
        if ( pdPASS == xQueueReceive((osal_queue_handle_t)pvParameter, (void *)&eol_data_frame, portMAX_DELAY) )
		{
            LOG(I, "EOL Test ID = %d EOL TEST DATA = %d", eol_data_frame.data_id, eol_data_frame.data);

            if ( eol_data_frame.eol_ack == PCBA_EOL_NO_ERR )
            {
                if ( ( eol_data_frame.data_id >= START_TESTID ) && ( eol_data_frame.data_id <= END_TESTID ) )
                {
                    resp_data[0] = eol_data_frame.data_id;
                    resp_data[1] = 0x00;
                    resp_data[2] = 0x00;

                    //parse_eol_data(eol_ctrl, &eol_data_frame);
                    switch ( eol_data_frame.data_id & CONVERT_DATAID )
                    {
                        case TEST_START:
                            LOG(W, "TEST_START");
                            int32_t i32Value = 1;

                            EN_IONIZER(1);

                            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, BT0SCAN, &i32Value, sizeof(int32_t), \
                                                        connector_eol.connector_id, portMAX_DELAY)); //BLE_SCAN

                            resp_data[3] = PCBA_EOL_NO_ERR;
                            break;

                        case TEST_SWVER:
                            LOG(W, "TEST_SWVER");
                            resp_data[1] = FIRMWARE_MAJOR;
                            resp_data[2] = FIRMWARE_MINOR;
                            resp_data[3] = PCBA_EOL_NO_ERR;                                                 //no error
                            break;

                        case TEST_HWVER:
                            LOG(W, "TEST_HWVER");
                            resp_data[1] = HARDWARE_MAJOR;
                            resp_data[2] = HARDWARE_MINOR;
                            resp_data[3] = PCBA_EOL_NO_ERR;                                                 //no error
                            break;

                        case TEST_DISP:
                            LOG(W, "TEST_DISP");
                            displaytest(&eol_data_frame);
                            resp_data[3] = eol_data_frame.eol_ack;
                            break;

                        case TEST_FANMTRRPM:
                            LOG(W, "TEST_FANMTRRPM");
                            for ( index = 0; index < MAX_NUM_DEVICE; index++ )
                            {
                                set_fan_motor_rpm(index, fan_rpm_eol[index]);
                            }
                            /* Update the response */
                            resp_data[3] = PCBA_EOL_NO_ERR;
                            break;

                        case TEST_TACHO:
                            LOG(W, "TEST_TACHO");
                            validate_fan_mtr_rpm(&eol_data_frame, &fan_rpm_eol[0]);
                            resp_data[2] = eol_data_frame.data;
                            resp_data[3] = eol_data_frame.eol_ack;                            
                            break;

                        case TEST_LEDSTRIP:
                            LOG(W, "TEST_LEDSTRIP");
                            light_dimmer_set_duty(LED_DIMMER_ON_DUTY_CYCLE);
                            resp_data[3] = PCBA_EOL_NO_ERR;
                            break;

                        case TEST_VOC:
                            LOG(W, "TEST_VOC");
                            int8_t bme68x_status = get_bmestatus();

                            if ( BME68X_OK != bme68x_status )
                            {
                                resp_data[1] = 0xFF;
                                resp_data[2] = 0xFF;
                                resp_data[3] = PCBA_EOL_ERR_COMM;
                            }
                            else
                            {
                                LOG(W, "BME680 Comm stat = %d  IAQ %d", bme68x_status, eol_serv_status.iaq_value);
                                resp_data[1] = (eol_serv_status.iaq_value >> 8) & 0xFF;
                                resp_data[2] = eol_serv_status.iaq_value & 0xFF;//bme68x_status;
                                resp_data[3] = PCBA_EOL_NO_ERR;
                            }
                            break;
                        
                        case TEST_BLE:
                            
                            eol_serv_status.num_ble_dev = get_ble_adv_dev_count();

                            LOG(W, "TEST_BLE BLE device count = %d", eol_serv_status.num_ble_dev);

                            resp_data[1] = eol_serv_status.num_ble_dev;

                            if ( eol_serv_status.num_ble_dev > 0 )
                            {
                                resp_data[3] = PCBA_EOL_NO_ERR;
                            }
                            else
                            {
                                resp_data[3] = PCBA_EOL_ERR_TIMEOUT;
                            }
                            break;

                        case TEST_WIFI:
                            LOG(W, "TEST_WIFI");
                            resp_data[1] = check_wifi(&rssi);
                            resp_data[2] = rssi;
                            resp_data[3] = PCBA_EOL_NO_ERR;
                            break;
                     
                        case TEST_END:
                            LOG(W, "TEST_END");
                            resp_data[3] = PCBA_EOL_NO_ERR;
                            break;

                        case TEST_STOP:
                            LOG(W, "TEST_STOP");
                            stop_eol_test();
                            resp_data[3] = PCBA_EOL_NO_ERR;
                            break;
                        
                        case TEST_CAN:
                            LOG(W, "TEST_CAN");
                            can_call();
                            resp_data[3] = PCBA_EOL_NO_ERR;
                            break;

                        case TEST_LIN:
                            LOG(W, "TEST_LIN");
#ifdef CONNECTOR_LIN_COMM_TEST
                            LIN_TX_FRAME lin_tx_frame;

                            lin_tx_frame.device_id = 0x0A;

                            for ( index = 0; index < 8; index++ )
                            {
                                lin_tx_frame.data[index] = index + 1;
                            }

                            lin_tx_frame.size = 8;

                            lin_tx_data(&lin_tx_frame);

                            resp_data[3] = PCBA_EOL_NO_ERR;
#else
                            resp_data[3] = PCBA_EOL_ERR_TIMEOUT;

#endif
                            break;
#if 0

                        case TEST_BATTERY:
                            LOG(W, "TEST_BATTERY");
#ifdef DEVICE_BQ25792
                            REG48_PART_INFO_REG     bqic_part_info;
                            result = bq25792_read_reg(PART_INFORMATION_REG48H, &bqic_part_info.byte, ONE_BYTE);
                            
                            if ( RES_PASS == result  )
                            {
                                if ( bqic_part_info.PART_NUMBER == BQ25792_PART_NUMBER )
                                {
                                    resp_data[3] = PCBA_EOL_NO_ERR;
                                }
                                else
                                {
                                    resp_data[3] = PCBA_EOL_ERR_DATAMISMATCH;
                                }
                            }
                            else
                            {
                                resp_data[3] = PCBA_EOL_ERR_COMM;
                            }
#endif
                            break;
#endif
                        default:
                            resp_data[3] = PCBA_EOL_ERR_DATAMISMATCH; 
                            LOG(E, "EOL CONNECT RESPONSE ERR = %d", resp_data[3]);
                            break;
                    }
                                                                                                                        //switch case data id 
                    u32data = ((resp_data[0] << 24)) | ((resp_data[1]) << 16) | ((resp_data[2] << 8)) | (resp_data[3]);
                    TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, IVEOL0RESP, &u32data,  sizeof(int32_t), connector_eol.connector_id, portMAX_DELAY));
                    
                }
            }
            else
            {
                LOG(E,"Queue data unhandled");
            }                                                                                                             //eol_status_flag
        } 
    }
}
#endif


/**
  * @brief  Task to control eol functionalities
  * @param  pvParameter.
  * @retval none.
  */
static void conn_eol_ctrl_task(void *pvParameter)
{
    EOL_DATA_FRAME eol_data_frame;
    uint8_t resp_data[4] = {0x00,0x00, 0x00, 0x00};
    uint32_t u32data = 0;
    int8_t rssi = 0;
    uint8_t index = 0;

    while (1)
    {
        if ( pdPASS == xQueueReceive((osal_queue_handle_t)pvParameter, (void *)&eol_data_frame, portMAX_DELAY) )
		{
            LOG(I, "EOL Test ID = %d EOL TEST DATA = %d", eol_data_frame.data_id, eol_data_frame.data);

            if ( eol_data_frame.eol_ack == PCBA_EOL_NO_ERR )
            {
                if ( ( eol_data_frame.data_id >= START_TESTID ) && ( eol_data_frame.data_id <= END_TESTID ) )
                {
                    resp_data[0] = eol_data_frame.data_id;
                    resp_data[1] = 0x00;
                    resp_data[2] = 0x00;

                    //parse_eol_data(eol_ctrl, &eol_data_frame);
                    switch ( eol_data_frame.data_id & CONVERT_DATAID )
                    {
                        case TEST_START:
                            LOG(W, "TEST_START");
                            int32_t i32Value = 1;

                            EN_IONIZER(1);

                            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, BT0SCAN, &i32Value, sizeof(int32_t), \
                                                        connector_eol.connector_id, portMAX_DELAY)); //BLE_SCAN

                            resp_data[3] = PCBA_EOL_NO_ERR;
                            break;

                        case TEST_SWVER:
                            LOG(W, "TEST_SWVER");
                            resp_data[1] = FIRMWARE_MAJOR;
                            resp_data[2] = FIRMWARE_MINOR;
                            resp_data[3] = PCBA_EOL_NO_ERR;                                                 //no error
                            break;

                        case TEST_HWVER:
                            LOG(W, "TEST_HWVER");
                            resp_data[1] = HARDWARE_MAJOR;
                            resp_data[2] = HARDWARE_MINOR;
                            resp_data[3] = PCBA_EOL_NO_ERR;                                                 //no error
                            break;

                        case TEST_DISP_SEG:         //!< 0xD4
                            LOG(W, "TEST_DISP");
                            displaytest(&eol_data_frame);
                            resp_data[3] = eol_data_frame.eol_ack;
                            break;

                        case TEST_FAN1:             //!< 0xD5
                            LOG(W, "TEST_FANMTRRPM");

                            //get 2 and 3 byte for RPM - common func
                            set_fan_motor_rpm(index, fan_rpm_eol[index]);

                            LOG(W, "TEST_TACHO");
                            validate_fan_mtr_rpm(&eol_data_frame, &fan_rpm_eol[0]);

                            resp_data[2] = eol_data_frame.data;
                            resp_data[3] = eol_data_frame.eol_ack;                            

                            /* Update the response */
                            resp_data[3] = PCBA_EOL_NO_ERR;
                            break;

                        case TEST_FAN2:             //!< 0xD6
                            LOG(W, "TEST_FANMTRRPM");

                            //get 2 and 3 byte for RPM - common func
                            set_fan_motor_rpm(index, fan_rpm_eol[index]);

                            LOG(W, "TEST_TACHO");
                            validate_fan_mtr_rpm(&eol_data_frame, &fan_rpm_eol[0]);

                            resp_data[2] = eol_data_frame.data;
                            resp_data[3] = eol_data_frame.eol_ack;                            

                            /* Update the response */
                            resp_data[3] = PCBA_EOL_NO_ERR;
                            break;


                        case TEST_MOTOR:             //!< 0xD7
                            LOG(W, "TEST_FANMTRRPM");

                            //get 2 and 3 byte for RPM - common func
                            set_fan_motor_rpm(index, fan_rpm_eol[index]);

                            LOG(W, "TEST_TACHO");
                            validate_fan_mtr_rpm(&eol_data_frame, &fan_rpm_eol[0]);

                            resp_data[2] = eol_data_frame.data;
                            resp_data[3] = eol_data_frame.eol_ack;                            

                            /* Update the response */
                            resp_data[3] = PCBA_EOL_NO_ERR;
                            break;

                        case TEST_LEDSTRIP:          //!< 0xD8
                            LOG(W, "TEST_LEDSTRIP");
                            //get 2 and 3 byte for brightness - common func
                            light_dimmer_set_duty(LED_DIMMER_ON_DUTY_CYCLE);
                            resp_data[3] = PCBA_EOL_NO_ERR;
                            break;

                        case TEST_VOC:              //!< 0xD9
                            LOG(W, "TEST_VOC");
                            int8_t bme68x_status = get_bmestatus();

                            //IAQ to be send - Todo
                            if ( BME68X_OK != bme68x_status )
                            {
                                resp_data[1] = 0xFF;
                                resp_data[2] = 0xFF;
                                resp_data[3] = PCBA_EOL_ERR_COMM;
                            }
                            else
                            {
                                LOG(W, "BME680 Comm stat = %d  IAQ %d", bme68x_status, eol_serv_status.iaq_value);
                                resp_data[1] = (eol_serv_status.iaq_value >> 8) & 0xFF;
                                resp_data[2] = eol_serv_status.iaq_value & 0xFF;//bme68x_status;
                                resp_data[3] = PCBA_EOL_NO_ERR;
                            }
                            break;
                        
                        case TEST_BLE:
                            
                            eol_serv_status.num_ble_dev = get_ble_adv_dev_count();

                            LOG(W, "TEST_BLE BLE device count = %d", eol_serv_status.num_ble_dev);

                            resp_data[1] = eol_serv_status.num_ble_dev;

                            if ( eol_serv_status.num_ble_dev > 0 )
                            {
                                resp_data[3] = PCBA_EOL_NO_ERR;
                            }
                            else
                            {
                                resp_data[3] = PCBA_EOL_ERR_TIMEOUT;
                            }
                            break;

                        case TEST_WIFI:
                            LOG(W, "TEST_WIFI");
                            resp_data[1] = check_wifi(&rssi);
                            resp_data[2] = rssi;
                            resp_data[3] = PCBA_EOL_NO_ERR;
                            break;
                     
                        case TEST_CAN:
                            LOG(W, "TEST_CAN");
                            can_call();
                            resp_data[3] = PCBA_EOL_NO_ERR;
                            break;

                        case TEST_LIN:
                            LOG(W, "TEST_LIN");
#ifdef CONNECTOR_LIN_COMM_TEST
                            LIN_TX_FRAME lin_tx_frame;

                            lin_tx_frame.device_id = 0x0A;

                            for ( index = 0; index < 8; index++ )
                            {
                                lin_tx_frame.data[index] = index + 1;
                            }

                            lin_tx_frame.size = 8;

                            lin_tx_data(&lin_tx_frame);

                            resp_data[3] = PCBA_EOL_NO_ERR;
#else
                            resp_data[3] = PCBA_EOL_ERR_TIMEOUT;

#endif
                            break;
#if 0

                        case TEST_BATTERY:
                            LOG(W, "TEST_BATTERY");
#ifdef DEVICE_BQ25792
                            REG48_PART_INFO_REG     bqic_part_info;
                            result = bq25792_read_reg(PART_INFORMATION_REG48H, &bqic_part_info.byte, ONE_BYTE);
                            
                            if ( RES_PASS == result  )
                            {
                                if ( bqic_part_info.PART_NUMBER == BQ25792_PART_NUMBER )
                                {
                                    resp_data[3] = PCBA_EOL_NO_ERR;
                                }
                                else
                                {
                                    resp_data[3] = PCBA_EOL_ERR_DATAMISMATCH;
                                }
                            }
                            else
                            {
                                resp_data[3] = PCBA_EOL_ERR_COMM;
                            }
#endif
                            break;
#endif
                        case TEST_END:
                            LOG(W, "TEST_END");
                            resp_data[3] = PCBA_EOL_NO_ERR;
                            break;

                        case TEST_STOP:
                            LOG(W, "TEST_STOP");
                            stop_eol_test();
                            resp_data[3] = PCBA_EOL_NO_ERR;
                            break;
                        
                        default:
                            resp_data[3] = PCBA_EOL_ERR_DATAMISMATCH; 
                            LOG(E, "EOL CONNECT RESPONSE ERR = %d", resp_data[3]);
                            break;
                    }
                                                                                                                        //switch case data id 
                    u32data = ((resp_data[0] << 24)) | ((resp_data[1]) << 16) | ((resp_data[2] << 8)) | (resp_data[3]);
                    TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, IVEOL0RESP, &u32data,  sizeof(int32_t), connector_eol.connector_id, portMAX_DELAY));
                    
                }
            }
            else
            {
                LOG(E,"Queue data unhandled");
            }                                                                                                             //eol_status_flag
        } 
    }
}


/**
  * @brief  Callback function to handle all the subscribed parameters.
  *         Few parameters are used for algorithm
  * @param  ddm_param - DDM Parameter with instance
  * @param  data      - Value for the DDM Parameter
  * @retval none
  */
static void handle_eol_req(uint32_t ddm_param, int32_t data)
{
    EOL_DATA_FRAME eol_data_frame;
    uint32_t u32data = data;

    eol_data_frame.eol_ack  =  u32data & 0xFF;
    eol_data_frame.data     =  ( u32data >> 8  ) & 0xFF;
    eol_data_frame.data_id  =  ( u32data >> 24 ) & 0xFFFF;

    LOG(I, "handle ddm_parameter = 0x%x value = %d", ddm_param, data);
    LOG(I, " dev_ack = 0x%x", eol_data_frame.eol_ack  );
    LOG(I, " Data = 0x%x", eol_data_frame.data);
    LOG(I, " data_id = 0x%x", eol_data_frame.data_id);
    
    LOG(I, "ddm_parameter = 0x%x u32data = 0x%x dev_ack = 0x%x data = %d data_id = %d", ddm_param, u32data, eol_data_frame.eol_ack , eol_data_frame.data, eol_data_frame.data_id);

    push_data_eol_queue(&eol_data_frame);
}

/**
 * @brief Callback function to handle all the subscribed parameters.
 *        Few parameters are used for algorithm
 * @param ddm_param - DDM Parameter with instance
 * @param data - Value for the DDM Parameter
 */
static void handle_tacho_data(uint32_t ddm_param, int32_t data)
{
    uint8_t inst_val = 0;

    inst_val = DDM2_PARAMETER_INSTANCE_PART(ddm_param) >> 8;

    set_tacho(inst_val, data);
}

/**
 * @brief Callback function to handle all the subscribed parameters.
 *        Few parameters are used for algorithm
 * @param ddm_param 
 * @param data 
 */
static void handle_eol_data(uint32_t ddm_param, int32_t data)
{
    switch (ddm_param)
    {
    case SBMEB0IAQ:
        eol_serv_status.iaq_value = data;
        LOG(W, "IAQ = %d", eol_serv_status.iaq_value);
        break;

    case WIFI0STS:
        eol_serv_status.wifi_status = data;
        LOG(W, "WIFI0STS = %d", eol_serv_status.wifi_status);
        break;
    
    default:
        break;
    }

}

/**
  * @brief  Function to process the BLE scan data
  * @param  none.
  * @retval none.
  */
static void process_ble_scan_data(const uint8_t * const value)
{
	const BLE_DEVICE_ENUM_FRAME *ble_enum = (BLE_DEVICE_ENUM_FRAME*) value;

    LOG(I, "Received BT0SCAN list");
	LOG(W, "\tmfg=%04x node_type=%02x node_id=%02x\n", ble_enum->manufacturer, ble_enum->node_id, ble_enum->node_type);
	LOG(W, "\tble_id=%u:" MACSTR "\n", ble_enum->ble_address_type , MAC2STR(ble_enum->ble_address));
	LOG(W, "\trssi=%d\n", ble_enum->rssi);
	LOG(W, "\tname=%.16s\n", ble_enum->name);

    LOG(W, "BLE Device count = %d", eol_serv_status.num_ble_dev);
}

static WIFI0STS_ENUM check_wifi(int8_t *rssi_value)
{
    WIFI0STS_ENUM wifi_stat = eol_serv_status.wifi_status;
    wifi_ap_record_t ap;

    if ( WIFI0STS_CONNECTED == eol_serv_status.wifi_status )
    {
        esp_wifi_sta_get_ap_info(&ap);

        LOG(W,"[Rssi %d ssid %s Mac 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x]", \
            ap.rssi, ap.ssid, ap.bssid[0],ap.bssid[1],ap.bssid[2],ap.bssid[3],ap.bssid[4],ap.bssid[5]);

        *rssi_value = ap.rssi;
        LOG(W,"[WIFI Strength: %d ...... %d]",ap.rssi, *rssi_value);
    }
    else
    {
        LOG(E, "Wifi Not connected");
    }

    return wifi_stat;
}

static void stop_eol_test(void)
{
    uint8_t index;

    for ( index = 0; index < MAX_NUM_DEVICE; index++ )
    {
        set_fan_motor_rpm(index, 0);

        tacho_val[index] = 0;
    }

    light_dimmer_set_duty(LED_DIMMER_OFF_DUTY_CYCLE);
    
    EN_IONIZER(0);
}

/**
 * @brief Set the tacho object
 * 
 * @param dev_id 
 * @param data 
 */
void set_tacho(uint8_t dev_id, int32_t data)
{
    LOG(I,"The set tacho data: %d", tacho_val[dev_id]); 
    tacho_val[dev_id] = data;
}

/**
 * @brief Function to get and set the fan motor RPM
 * 
 * @param eol_data_frame data, data_id, eol_acknowledge
 * @return EOL_ERROR_FRAME 
 */
EOL_ERROR_FRAME validate_fan_mtr_rpm(EOL_DATA_FRAME* ptr_eol_frame, uint16_t* fan_rpm)
{
    uint8_t index = 0;
    uint8_t tacho_data = 0;

    for ( index = 0; index < MAX_NUM_DEVICE; index++ )
    {
        LOG(W, "Dev %d : SET RPM %d | TACHO %d", index, fan_rpm[index], tacho_val[index]);

        if ( tacho_val[index] != 0u )
        {
            LOG(W, "Tacho feedback success for devid %d", index);
            tacho_data &= ~(1 << index);
        }
        else 
        {
            LOG(W, "Tacho feedback failed for devid %d", index);
            tacho_data |= (1 << index);
        }
    }

    if ( tacho_data != 0 )
    {
        ptr_eol_frame->eol_ack = PCBA_EOL_ERR_DATARX;
    }
    else
    {
        ptr_eol_frame->eol_ack = PCBA_EOL_NO_ERR;
    }

    ptr_eol_frame->data = tacho_data;

    return ptr_eol_frame->eol_ack;
}

void can_call(void)
{
    can_message_t msg = {0};

	msg.identifier += 1;
    msg.data_length_code = 8;

	for (int i = 0;i < msg.data_length_code; i++)
	{
		msg.data[i] = i + 1;
	}

	can_transmit(&msg, 0);
}

/**
  * @brief  One shot timer callback function
  * @param  none.
  * @retval none.
  */
static void bt_scan_tmr_expired_cb( TimerHandle_t xTimer )
{
    LOG(W, "Timer Expired BLE dev count = %d", eol_serv_status.num_ble_dev);

    //if ( eol_serv_status.num_ble_dev == 0 )
    {
        int32_t i32Value = 1;
        osal_ubase_type_t ret;

        TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, BT0SCAN, &i32Value, sizeof(int32_t), \
                                                        connector_eol.connector_id, portMAX_DELAY));

        ret = xTimerStart( one_shot_tmr_hdle, 0 );

        if ( ret != pdPASS )
        {
            LOG(E, "Timer start failed");
        }
    }

}





#endif /*CONNECTOR_EOL*/ 