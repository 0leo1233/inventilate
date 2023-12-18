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
#include "connector_pwr_ctrl_service.h"
#include "ddm2_parameter_list.h"
#include "osal.h"
#include "hal_pwm.h"
#include "hal_ledc.h"
#include "sorted_list.h"
#include "linear_interpolation.h"
#include "app_api.h"
#include "bme68x_defs.h"
#include "eolpcba_invent.h"
#include "drv_uc1510c.h"
#include "driver/twai.h"
#include "hal/twai_types.h"
#ifdef DEVICE_BQ25798
#include "drv_bq25798.h"
#endif
#ifdef DEVICE_BQ25792
#include "drv_bq25792.h"
#endif
#include "esp_wifi.h"
#include "esp_wifi_types.h"

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
static void push_data_eol_queue(EOL_DATA_FRAME* eol_data_frame);
static void process_set_and_publish_request(uint32_t ddm_param, int32_t i32value, DDMP2_CONTROL_ENUM req_type);
static void handle_tacho_data(uint32_t ddm_param, int32_t data);
static void initialize_eol(void);
static void process_ble_scan_data(const uint8_t * const value);
static WIFI0STS_ENUM check_wifi(int8_t *rssi_value);
static void stop_eol_test(void);
static void bt_scan_tmr_expired_cb( TimerHandle_t xTimer );
void can_call(void);
EOL_ERROR_FRAME validate_fan_mtr_rpm(EOL_DATA_FRAME* ptr_eol_frame, uint32_t* fan_rpm);
EOL_ERROR_FRAME display_seg_test(EOL_DATA_FRAME* eol_data_frame);
//static int16_t eol_power_consumption(EOL_DATA_FRAME* eol_data_frame, uint8_t case_val, uint8_t dev_id, uint32_t data);
void set_tacho(uint8_t dev_id, int32_t data);
void send_datato_broker(uint8_t resp_data[]);
static void get_macid();
static int16_t eol_power_consumption(EOL_DATA_FRAME* eol_data_frame);
#ifdef INVENTPCBA_TEST
EOL_ERROR_FRAME displaytest(EOL_DATA_FRAME* eol_data_frame);
#endif

//extern void test_lin_transmit(void);
//extern int rs485_tranmit_data(char* data, uint16_t data_len);

uint32_t fan_rpm_eol[3] = {EOL_FAN_MIN_RPM, EOL_FAN_MID_RPM, EOL_FAN_MAX_RPM};
uint32_t tacho_val[3];
uint8_t resp_data[NO_OF_BYTES] = {0x00,0x00, 0x00, 0x00};

uint8_t mac_id[6];
uint8_t temp_macid_1;
uint8_t temp_macid_2;
uint8_t temp_macid_3;
uint8_t temp_macid_4;
uint8_t temp_macid_5;
uint8_t temp_macid_6;
uint8_t mac_count = 0;

typedef struct eol_serv_status
{
    WIFI0STS_ENUM wifi_status;
    uint16_t        iaq_value;
    uint8_t       num_ble_dev;
}EOL_SERV_STATUS;

static EOL_SERV_STATUS eol_serv_status;
static batt_reg batt_currstat;


/* Structure for Connector eol*/
CONNECTOR connector_eol =
{
	.name = "Connector EOL",
	.initialize = initialize_connector_eol,
};

/* DDM Parameter table for connector eol*/
static conn_eol_parameter_t conn_eol_param_db[] =
{
    {.ddm_parameter = IVEOL0REQ,                            .type = DDM2_TYPE_INT32_T, .pub = 0, .sub = 1, .i32Value =                0, .cb_func =      handle_eol_req},
    {.ddm_parameter = IVEOL0RESP,                           .type = DDM2_TYPE_INT32_T, .pub = 1, .sub = 0, .i32Value =                0, .cb_func =                 NULL},
    {.ddm_parameter = MTR0TACHO|DDM2_PARAMETER_INSTANCE(0), .type = DDM2_TYPE_INT32_T, .pub = 0, .sub = 1, .i32Value =                0, .cb_func =    handle_tacho_data},
    {.ddm_parameter = MTR0TACHO|DDM2_PARAMETER_INSTANCE(1), .type = DDM2_TYPE_INT32_T, .pub = 0, .sub = 1, .i32Value =                0, .cb_func =    handle_tacho_data},
    {.ddm_parameter = MTR0TACHO|DDM2_PARAMETER_INSTANCE(2), .type = DDM2_TYPE_INT32_T, .pub = 0, .sub = 1, .i32Value =                0, .cb_func =    handle_tacho_data},
    {.ddm_parameter = SBMEB0IAQ,                            .type = DDM2_TYPE_INT32_T, .pub = 0, .sub = 1, .i32Value =                0, .cb_func =      handle_eol_data},
    {.ddm_parameter = BT0SCAN,                              .type = DDM2_TYPE_OTHER,   .pub = 0, .sub = 1, .i32Value =                0, .cb_func =                 NULL},
    {.ddm_parameter = WIFI0STS,                             .type = DDM2_TYPE_INT32_T, .pub = 0, .sub = 1, .i32Value = WIFI0STS_UNKNOWN, .cb_func =      handle_eol_data},
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
#if CONN_EOL_DEBUG_LOG
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
#if CONN_EOL_DEBUG_LOG        
        LOG(E, "Queue error ret = %d", ret);
#endif    
    }
}

#ifdef INVENTPCBA_TEST 
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
    error_type result;
    

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

                    //parse_eol_data(eol_ctrl, &eol_data_frame);START_TESTID
                    //switch ( eol_data_frame.data_id & CONVERT_DATAID )
                    switch ( eol_data_frame.data_id - START_TESTID )
                    {
                        case TEST_START:
                            LOG(W, "TEST_START");
                            int32_t i32Value = 1;

                            //EN_IONIZER(1);

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
                            EN_IONIZER(1);
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
//#if 0

                        case TEST_BATTERY:
                            LOG(W, "TEST_BATTERY");
#ifdef DEVICE_BQ25672
                            //REG48_PART_INFO_REG bqic_part_info_new;
#endif
#ifdef DEVICE_BQ25792
                            REG48_PART_INFO_REG_BQ25672 bqic_part_info_new;

                            //result = bq25792_read_reg(PART_INFORMATION_REG48H, &bqic_part_info.byte, ONE_BYTE);
                            result = bq25792_read_reg(PART_INFORMATION_REG48H, &bqic_part_info_new.byte, ONE_BYTE);
                            if ( RES_PASS == result  )
                            {
                                LOG(W,"Battery BQ25672 PART NUMBER %d",bqic_part_info_new.byte);

                                if ( bqic_part_info_new.PART_NUMBER == BQ25672_PART_NUMBER) //BQ25792_PART_NUMBER )
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
#ifdef DEVICE_BQ25672
                            //REG48_PART_INFO_REG bqic_part_info_new;
                            REG48_PART_INFO_REG_BQ25672 bqic_part_info_new;

                            result = bq25792_read_reg(PART_INFORMATION_REG48H, &bqic_part_info_new.byte, ONE_BYTE);
                            //result = bq25672_read_reg(PART_INFORMATION_REG48H, &bqic_part_info_new.byte, ONE_BYTE);
                            if ( RES_PASS == result  )
                            {
                                LOG(W,"Battery BQ25672 PART NUMBER %d",bqic_part_info_new.byte);

                                if ( bqic_part_info_new.PART_NUMBER == BQ25672_PART_NUMBER )
                                {
                                    resp_data[2] = 0;
                                    resp_data[3] = PCBA_EOL_NO_ERR;
                                }
                                else
                                {
                                    resp_data[2] = 1;
                                    resp_data[3] = PCBA_EOL_ERR_DATAMISMATCH;
                                }
                            }
                            else
                            {
                                resp_data[2] = 2;
                                resp_data[3] = PCBA_EOL_ERR_COMM;
                            }
#endif
#if 0
                            REG48_PART_INFO_REG         bqic_part_info;

                            result = bq25792_read_reg(PART_INFORMATION_REG48H, &bqic_part_info.byte, ONE_BYTE);
                            if ( RES_PASS == result  )
                            {
                                //LOG(W,"Battery BQ25792 PART NUMBER %d",bqic_part_info.byte);

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
//#endif

                        case TEST_MACID:
                            get_macid();
                            LOG(W,"mac_count %d", eol_data_frame.data );
                            mac_count = (eol_data_frame.data >> 8);
                            if(mac_count == 1)
                            {
                                resp_data[1] = temp_macid_1;
                                resp_data[2] = temp_macid_2;
                                resp_data[3] = PCBA_EOL_NO_ERR;
                                // mac_count++;
                                LOG(W, "RESP DATA = %d ,%d, %d", resp_data[1], resp_data[2],resp_data[3]);
                                LOG(W,"mac_count %d", mac_count);
                            }
                            else if (mac_count == 2)
                            {
                                resp_data[1] = temp_macid_3;
                                resp_data[2] = temp_macid_4;
                                resp_data[3] = PCBA_EOL_NO_ERR;
                                // mac_count++;
                                LOG(W, "RESP DATA = %d ,%d, %d", resp_data[1], resp_data[2],resp_data[3]);
                                LOG(W,"mac_count %d", mac_count);
                            }
                            else if (mac_count == 3)
                            {
                                resp_data[1] = temp_macid_5;
                                resp_data[2] = temp_macid_6;
                                resp_data[3] = PCBA_EOL_NO_ERR;
                                // mac_count  = 0;
                                LOG(W, "RESP DATA = %d ,%d, %d", resp_data[1], resp_data[2],resp_data[3]);
                            }
                            else
                            {
                                mac_count  = 0;
                            }
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

#ifdef INVENTEOL_TEST
/**
  * @brief  Task to control eol functionalities
  * @param  pvParameter.
  * @retval none.
  */
static void conn_eol_ctrl_task(void *pvParameter)
{
    EOL_DATA_FRAME eol_data_frame;
    // uint8_t resp_data[NO_OF_BYTES] = {0x00,0x00, 0x00, 0x00};
    // uint32_t u32data = 0;
    int8_t rssi = 0;
    uint8_t index = 0;
    uint32_t dispbright_resol = 0;
    uint32_t led_strip_resol = 0;
    uint8_t ionizer_stat;
    int16_t pwr_arr[8];
    error_type result;
    int32_t dp_data;

    while (1)
    {
        if ( pdPASS == xQueueReceive((osal_queue_handle_t)pvParameter, (void *)&eol_data_frame, portMAX_DELAY) )
		{
#if CONN_EOL_DEBUG_LOG
            LOG(I, "EOL Test ID = %d EOL TEST DATA = %d", eol_data_frame.data_id, eol_data_frame.data);
#endif
            dp_data = 0x0003;
            //command to send DP sensor data            
            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SUBSCRIBE, SNODE0MDL, (const void*)&dp_data, sizeof(int32_t), \
                connector_eol.connector_id, portMAX_DELAY));

            if ( eol_data_frame.eol_ack == PCBA_EOL_NO_ERR )
            {
                if ( ( eol_data_frame.data_id >= START_TESTID ) && ( eol_data_frame.data_id <= END_TESTID ) )
                {
                    resp_data[0] = eol_data_frame.data_id;
                    resp_data[1] = 0x00;
                    resp_data[2] = 0x00;

                    //batt_currstat = get_battval();

                    //parse_eol_data(eol_ctrl, &eol_data_frame);
                    // switch ( eol_data_frame.data_id & CONVERT_DATAID )
                    switch ( eol_data_frame.data_id - START_TESTID)
                    {
                        case TEST_START:
#if CONN_EOL_DEBUG_LOG
                            LOG(W, "TEST_START");
#endif
                            int32_t i32Value = 1;

                            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, BT0SCAN, &i32Value, sizeof(int32_t), \
                                                        connector_eol.connector_id, portMAX_DELAY)); //BLE_SCAN
                            
                            update_and_send_value_to_broker(IV0MODE, IV0MODE_AUTO);
                            update_and_send_value_to_broker(IV0PWRON, IV0PWRON_ON);

                            i32Value = IVPMGR0STATE_ACTIVE;
                            /* Frame and send the publish request */
                            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, IVPMGR0STATE, &i32Value, \
                            sizeof(int32_t), connector_eol.connector_id, portMAX_DELAY));

                            set_fan_motor_rpm(DEV_FAN2_AIR_OUT, EOL_FAN_MIN_RPM);
                            set_fan_motor_rpm(DEV_FAN1_AIR_IN, EOL_FAN_MIN_RPM);
                            set_fan_motor_rpm(DEV_MOTOR, EOL_FAN_MIN_RPM);

                            EN_IONIZER(0);                                                                                  //Turn off the Ionizer

                            resp_data[3] = PCBA_EOL_NO_ERR;
                            break;

                        case TEST_SWVER:
#if CONN_EOL_DEBUG_LOG
                            LOG(W, "TEST_SWVER");
#endif
                            resp_data[1] = FIRMWARE_MAJOR;
                            resp_data[2] = FIRMWARE_MINOR;
                            resp_data[3] = PCBA_EOL_NO_ERR;                                                 //no error
                            break;

                        case TEST_HWVER:
#if CONN_EOL_DEBUG_LOG
                            LOG(W, "TEST_HWVER");
#endif
                            resp_data[1] = HARDWARE_MAJOR;
                            resp_data[2] = HARDWARE_MINOR;
                            resp_data[3] = PCBA_EOL_NO_ERR;                                                 //no error
                            break;

                        case TEST_DISP_SEG:         //!< 0xD4
#if CONN_EOL_DEBUG_LOG
                            LOG(W, "TEST_DISP");
#endif
                            resp_data[3] = display_seg_test(&eol_data_frame);
                            if (eol_data_frame.data == SEG_ALL_ON)
                            {
                                resp_data[1] = eol_data_frame.data & 0xFF;
                                resp_data[2] = (eol_data_frame.data >> 8 ) & 0xFF;
                            }
                            if (eol_data_frame.data == SEG_ALL_OFF)
                            {
                                resp_data[1] = eol_data_frame.data & 0xFF;
                                resp_data[2] = (eol_data_frame.data >> 8 ) & 0xFF;
                            }
                            //resp_data[3] = eol_data_frame.eol_ack;    
#if 0
                            resp_data[3] = display_seg_test(&eol_data_frame);
#endif
                            break;

                        case TEST_DISP_BRGHT:
#if CONN_EOL_DEBUG_LOG
                            LOG(W,"disp_bright, data_3 %d\n", eol_data_frame.data );
#endif
                            if(eol_data_frame.data > 100)
                            {
                                resp_data[2] = eol_data_frame.data;
                                resp_data[3] = PCBA_EOL_OUT_OF_RANGE;
                            }
                            dispbright_resol = calc_linear_interpolation(HMI_BACKLIGHT_MIN_DUTY_CYCLE, HMI_BACKLIGHT_MAX_DUTY_CYCLE, LEDC_PWM_MIN_DUTY_CYCLE, LEDC_PWM_MAX_DUTY_CYCLE, eol_data_frame.data );
                            hmi_backlight_set_duty(dispbright_resol);
                            resp_data[2] = eol_data_frame.data;
                            resp_data[3] = PCBA_EOL_NO_ERR;
                            break;

                        case TEST_FAN1:             //!< 0xD5
#if CONN_EOL_DEBUG_LOG
                            LOG(W, "TEST_FAN1RPM");
#endif
                            set_fan_motor_rpm(DEV_FAN1_AIR_IN, eol_data_frame.data);
                            resp_data[3] = eol_data_frame.eol_ack;                            

                            /* Update the response */
                            resp_data[3] = PCBA_EOL_NO_ERR;
                            break;

                        case TEST_FAN2:             //!< 0xD6
#if CONN_EOL_DEBUG_LOG
                            LOG(W, "TEST_FAN2RPM");
#endif
                            set_fan_motor_rpm(DEV_FAN2_AIR_OUT, eol_data_frame.data);
                            resp_data[3] = eol_data_frame.eol_ack;                            

                            /* Update the response */
                            resp_data[3] = PCBA_EOL_NO_ERR;
                            break;

                        case TEST_MOTOR:             //!< 0xD7
#if CONN_EOL_DEBUG_LOG
                            LOG(W, "TEST_MTRRPM");
#endif
                            set_fan_motor_rpm(DEV_MOTOR, eol_data_frame.data);
                            resp_data[3] = eol_data_frame.eol_ack;                            

                            /* Update the response */
                            resp_data[3] = PCBA_EOL_NO_ERR;
                            break;

                        case TEST_TACHO:            //!< 0xD8
                            LOG(W, "TEST_TACHO");
#if INVENT_PCBA
                            for (index = 0; index < MAX_NUM_DEVICE; index++)
                            {
                                validate_fan_mtr_rpm(&eol_data_frame, &fan_rpm_eol[index]);
                                resp_data[2] = eol_data_frame.data;
                                resp_data[3] = eol_data_frame.eol_ack; 
                            }
#endif
                            break;

                        case TEST_LEDSTRIP:          //!< 0xD9
#if CONN_EOL_DEBUG_LOG
                            LOG(W, "TEST_LEDSTRIP");
#endif
                            if(eol_data_frame.data > 100)
                            {
                                resp_data[2] = eol_data_frame.data;
                                resp_data[3] = PCBA_EOL_OUT_OF_RANGE;
                            }
                            else
                            {
                                led_strip_resol = calc_linear_interpolation(HMI_BACKLIGHT_MIN_DUTY_CYCLE, HMI_BACKLIGHT_MAX_DUTY_CYCLE, LEDC_PWM_MIN_DUTY_CYCLE, LEDC_PWM_MAX_DUTY_CYCLE, eol_data_frame.data );
                                light_dimmer_set_duty(led_strip_resol);

                                resp_data[2] = eol_data_frame.data;
#ifdef POWER_CONSUMPTION                            
                                //pwr_arr[CURR_LEDSTRIP] = power_consumption_eol(CURR_LEDSTRIP);   //power_consumption(CURR_LEDSTRIP);
                                LOG(W,"Power consumption of LED strip: %d", *pwr_arr[CURR_LEDSTRIP]);
#endif
                                resp_data[3] = PCBA_EOL_NO_ERR;

                            }
                            LOG(W,"led strip brightness, data_3 %d\n", eol_data_frame.data);

                            break;

                        case TEST_VOC:              //!< 0xDA
#if CONN_EOL_DEBUG_LOG
                            LOG(W, "TEST_VOC");
#endif
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
#if CONN_EOL_DEBUG_LOG
                                LOG(W, "BME680 Comm stat = %d  IAQ %d", bme68x_status, eol_serv_status.iaq_value);
#endif
                                resp_data[1] = (eol_serv_status.iaq_value >> 8) & 0xFF;
                                resp_data[2] = eol_serv_status.iaq_value & 0xFF;//bme68x_status;
                                resp_data[3] = PCBA_EOL_NO_ERR;
                            }
                            break;

//#if 0
                        case TEST_DIFFSEN:
                            LOG(W, "TEST_DIFFSEN");
                            break;
//#endif                        
                        case TEST_IONIZ:
#if CONN_EOL_DEBUG_LOG
                            LOG(W, "TEST_IONIZER");
#endif
                            //ionizer_stat = ( eol_data_frame.data >> 8 ) & 0xFF;
                            ionizer_stat = eol_data_frame.data;
                            EN_IONIZER(ionizer_stat);
                            resp_data[1] = 0;
                            resp_data[2] = ionizer_stat;
#ifdef POWER_CONSUMPTION  
                            //pwr_arr[CURR_IONIZ] = power_consumption_eol(CURR_IONIZ);
                            LOG(W,"Power consumption of Ionizer: %d", *pwr_arr[CURR_IONIZ]);
#endif
                            resp_data[3] = PCBA_EOL_NO_ERR;

                            break;

                        case TEST_BLE:
                            eol_serv_status.num_ble_dev = get_ble_adv_dev_count();
#if CONN_EOL_DEBUG_LOG
                            LOG(W, "TEST_BLE BLE device count = %d", eol_serv_status.num_ble_dev);
#endif
                            //resp_data[1] = eol_serv_status.num_ble_dev;
                            resp_data[2] = eol_serv_status.num_ble_dev;

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
#if CONN_EOL_DEBUG_LOG
                            LOG(W, "TEST_WIFI");
#endif
                            resp_data[1] = check_wifi(&rssi);
                            resp_data[2] = rssi;
                            //resp_data[2] = 0x00;
                            resp_data[3] = PCBA_EOL_NO_ERR;
                            break;
                     
                        case TEST_CAN:
#if CONN_EOL_DEBUG_LOG
                            LOG(W, "TEST_CAN");
#endif                            
                            can_call();
                            resp_data[3] = PCBA_EOL_NO_ERR;
                            break;

                        case TEST_LIN:
#if CONN_EOL_DEBUG_LOG                        
                            LOG(W, "TEST_LIN");
#endif                            
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

                        case TEST_CHGVOLT:
                            batt_currstat = get_battval();
                            resp_data[1] = batt_currstat.bat_volt>> 8;
                            resp_data[2] = batt_currstat.bat_volt; 
                            resp_data[3] = PCBA_EOL_NO_ERR;
#if CONN_EOL_DEBUG_LOG                            
                            LOG(W, "EOL Charging Voltage = %d %d %d",batt_currstat.bat_volt, resp_data[1],  resp_data[2]);
#endif                            
                            break;

                        case TEST_CHGCURR:
                            batt_currstat = get_battval();
                            resp_data[1] = batt_currstat.bat_curr >> 8;
                            resp_data[2]  = batt_currstat.bat_curr;
                            resp_data[3]  = PCBA_EOL_NO_ERR;
#if CONN_EOL_DEBUG_LOG                            
                            LOG(W, "EOL Charging Current = %d %d %d",batt_currstat.bat_curr, resp_data[1],  resp_data[2]);
#endif                            
                            break;

                        case TEST_CHGMODE:
                            if (batt_currstat.chrg_stat <= CHARGING_TERMINATION_DONE)
                            {
                                resp_data[2] = batt_currstat.chrg_stat;
                                resp_data[3]  = PCBA_EOL_NO_ERR;
                            }
                            break;

                        case TEST_12VSRCVOLT:
                            batt_currstat = get_battval();
                            resp_data[1] = batt_currstat.veh_batt_volt >> 8;
                            resp_data[2]  = batt_currstat.veh_batt_volt;
                            resp_data[3]  = PCBA_EOL_NO_ERR;
#if CONN_EOL_DEBUG_LOG                        
                            LOG(W, "EOL vehicle batt voltage = %d %d %d",batt_currstat.veh_batt_volt, resp_data[1],  resp_data[2]);
#endif                            
                            
                            break;

                        case TEST_SOLARSRCVOLT:
                            batt_currstat = get_battval();
                            resp_data[1] = batt_currstat.solar_volt >> 8;
                            resp_data[2] = batt_currstat.solar_volt;
                            resp_data[3] = PCBA_EOL_NO_ERR;
#if CONN_EOL_DEBUG_LOG                            
                            LOG(W, "EOL Solar Voltage = %d %d %d",batt_currstat.solar_volt, resp_data[1], resp_data[2]);
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
// #if 0
//#ifdef POWER_CONSUMPTION  
                        case TEST_PWRCONSUMPTION:
                            pwr_arr[1] = eol_power_consumption(&eol_data_frame);
                            resp_data[1] = pwr_arr[1] >> 8;
                            resp_data[2] = pwr_arr[1];
                            resp_data[3] = PCBA_EOL_NO_ERR;

                            LOG(W, "RESP DATA = %d ,%d, %d", resp_data[1], resp_data[2],resp_data[3]);
                            LOG(W,"pwr_arr %d", pwr_arr[1]);
                            break;
//#endif
// #endif
                        case TEST_MACID:
                            get_macid();
                            LOG(W,"mac_count %d", eol_data_frame.data );
                            mac_count = (eol_data_frame.data >> 8);
                            if(mac_count == 1)
                            {
                                resp_data[1] = temp_macid_1;
                                resp_data[2] = temp_macid_2;
                                resp_data[3] = PCBA_EOL_NO_ERR;
                                // mac_count++;
                                LOG(W, "RESP DATA = %d ,%d, %d", resp_data[1], resp_data[2],resp_data[3]);
                                LOG(W,"mac_count %d", mac_count);
                            }
                            else if (mac_count == 2)
                            {
                                resp_data[1] = temp_macid_3;
                                resp_data[2] = temp_macid_4;
                                resp_data[3] = PCBA_EOL_NO_ERR;
                                // mac_count++;
                                LOG(W, "RESP DATA = %d ,%d, %d", resp_data[1], resp_data[2],resp_data[3]);
                                LOG(W,"mac_count %d", mac_count);
                            }
                            else if (mac_count == 3)
                            {
                                resp_data[1] = temp_macid_5;
                                resp_data[2] = temp_macid_6;
                                resp_data[3] = PCBA_EOL_NO_ERR;
                                // mac_count  = 0;
                                LOG(W, "RESP DATA = %d ,%d, %d", resp_data[1], resp_data[2],resp_data[3]);
                            }
                            else
                            {
                                mac_count  = 0;
                            }
                            break;


                        case TEST_END:
#if CONN_EOL_DEBUG_LOG                        
                            LOG(W, "TEST_END");
#endif                            
                            resp_data[3] = PCBA_EOL_NO_ERR;
                            break;

                        case TEST_STOP:
#if CONN_EOL_DEBUG_LOG                        
                            LOG(W, "TEST_STOP");
#endif                            
                            stop_eol_test();
                            resp_data[3] = PCBA_EOL_NO_ERR;
                            break;
                        
                        default:
                            resp_data[3] = PCBA_EOL_ERR_DATAMISMATCH; 
#if CONN_EOL_DEBUG_LOG                            
                            LOG(E, "EOL CONNECT RESPONSE ERR = %d", resp_data[3]);
#endif                            
                            break;
                        }

                    //switch case data id-> send response data to the broker
                    send_datato_broker(resp_data);

                    // u32data = ((resp_data[0] << 24)) | ((resp_data[1]) << 16) | ((resp_data[2] << 8)) | (resp_data[3]); 
                    // TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, IVEOL0RESP, &u32data,  sizeof(int32_t), connector_eol.connector_id, portMAX_DELAY));
                    // LOG(W, "DATA send to broker, %d", u32data);
                }
            }
            else
            {
#if CONN_EOL_DEBUG_LOG                
                LOG(E,"Queue data unhandled");
#endif                
            }  //eol_status_flag
        } 
    }
}
#endif

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
#if INVENT_PCBA
    eol_data_frame.eol_ack  =  u32data & 0xFF;
    eol_data_frame.data     =  ( u32data >> 8  ) & 0xFF;
    eol_data_frame.data_id  =  ( u32data >> 24 ) & 0xFFFF;
#endif
    eol_data_frame.eol_ack  =  u32data & 0xFF;//a0
    eol_data_frame.data     =  ( u32data >> 8  ) & 0xFFFF;//data
    eol_data_frame.data_id  =  ( u32data >> 24 ) & 0xFFFF;//d1

#if CONN_EOL_DEBUG_LOG
    LOG(I, "handle ddm_parameter = 0x%x value = %d", ddm_param, data);
    LOG(I, " dev_ack = 0x%x", eol_data_frame.eol_ack  );
    LOG(I, " Data = 0x%x", eol_data_frame.data);
    LOG(I, " data_id = 0x%x", eol_data_frame.data_id);
    
    LOG(I, "ddm_parameter = 0x%x u32data = 0x%x dev_ack = 0x%x data = %d data_id = %d", ddm_param, u32data, eol_data_frame.eol_ack , eol_data_frame.data, eol_data_frame.data_id);
#endif
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
#if CONN_EOL_DEBUG_LOG
        LOG(W, "IAQ = %d", eol_serv_status.iaq_value);
#endif        
        break;

    case WIFI0STS:
        eol_serv_status.wifi_status = data;
#if CONN_EOL_DEBUG_LOG        
        LOG(W, "WIFI0STS = %d", eol_serv_status.wifi_status);
#endif
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
#if CONN_EOL_DEBUG_LOG
    LOG(I, "Received BT0SCAN list");
	LOG(W, "\tmfg=%04x node_type=%02x node_id=%02x\n", ble_enum->manufacturer, ble_enum->node_id, ble_enum->node_type);
	LOG(W, "\tble_id=%u:" MACSTR "\n", ble_enum->ble_address_type , MAC2STR(ble_enum->ble_address));
	LOG(W, "\trssi=%d\n", ble_enum->rssi);
	LOG(W, "\tname=%.16s\n", ble_enum->name);

    LOG(W, "BLE Device count = %d", eol_serv_status.num_ble_dev);
#endif
}

static void get_macid()
{
    esp_efuse_mac_get_default(mac_id);

    temp_macid_1 = mac_id[0];
    temp_macid_2 = mac_id[1];
    temp_macid_3 = mac_id[2];
    temp_macid_4 = mac_id[3];
    temp_macid_5 = mac_id[4];
    temp_macid_6 = mac_id[5];

    LOG(C,"The mac id: %02x:%02x:%02x:%02x:%02x:%02x", mac_id[0], mac_id[1], mac_id[2], mac_id[3], mac_id[4], mac_id[5]);
    LOG(C,"Temp mac id: %02x %02x %02x %02x %02x %02x", temp_macid_1, temp_macid_2, temp_macid_3, temp_macid_4, temp_macid_5, temp_macid_6);

}

static WIFI0STS_ENUM check_wifi(int8_t *rssi_value)
{
    WIFI0STS_ENUM wifi_stat = eol_serv_status.wifi_status;
    wifi_ap_record_t ap;

    if ( WIFI0STS_CONNECTED == eol_serv_status.wifi_status )
    {
        esp_wifi_sta_get_ap_info(&ap);
#if CONN_EOL_DEBUG_LOG
        LOG(W,"[Rssi %d ssid %s Mac 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x]", \
            ap.rssi, ap.ssid, ap.bssid[0],ap.bssid[1],ap.bssid[2],ap.bssid[3],ap.bssid[4],ap.bssid[5]);
#endif
        *rssi_value = ap.rssi;

#if CONN_EOL_DEBUG_LOG        
        LOG(W,"[WIFI Strength: %d ...... %d]",ap.rssi, *rssi_value);
#endif        
    }
    else
    {
#if CONN_EOL_DEBUG_LOG        
        LOG(E, "Wifi Not connected");
#endif        
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
#ifdef INVENTEOL_TEST
    //hmi_backlight_set_duty(LED_DIMMER_OFF_DUTY_CYCLE);
#endif
    
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
#if CONN_EOL_DEBUG_LOG
    LOG(I,"The set tacho data: %d", tacho_val[dev_id]); 
#endif
    tacho_val[dev_id] = data;
}

/**
 * @brief Function to get and set the fan motor RPM
 * 
 * @param eol_data_frame data, data_id, eol_acknowledge
 * @return EOL_ERROR_FRAME 
 */
EOL_ERROR_FRAME validate_fan_mtr_rpm(EOL_DATA_FRAME* ptr_eol_frame, uint32_t* fan_rpm)
{
    uint8_t index = 0;
    uint8_t tacho_data = 0;

    for ( index = 0; index < MAX_NUM_DEVICE; index++ )
    {
#if CONN_EOL_DEBUG_LOG        
        LOG(W, "Dev %d : SET RPM %d | TACHO %d", index, fan_rpm[index], tacho_val[index]);
#endif
        if ( tacho_val[index] != 0u )
        {
#if CONN_EOL_DEBUG_LOG            
            LOG(W, "Tacho feedback success for devid %d", index);
#endif            
            tacho_data &= ~(1 << index);
        }
        else 
        {
#if CONN_EOL_DEBUG_LOG            
            LOG(W, "Tacho feedback failed for devid %d", index);
#endif            
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
    twai_message_t msg = {0};

	msg.identifier += 1;
    msg.data_length_code = 8;

	for (int i = 0;i < msg.data_length_code; i++)
	{
		msg.data[i] = i + 1;
	}

	twai_transmit(&msg, 0);
}

/**
  * @brief  One shot timer callback function
  * @param  none.
  * @retval none.
  */
static void bt_scan_tmr_expired_cb( TimerHandle_t xTimer )
{
#if CONN_EOL_DEBUG_LOG    
    LOG(W, "Timer Expired BLE dev count = %d", eol_serv_status.num_ble_dev);
#endif
    if ( eol_serv_status.num_ble_dev == 0 )
    {
        int32_t i32Value = 1;
        osal_ubase_type_t ret;

        TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, BT0SCAN, &i32Value, sizeof(int32_t), \
                                                        connector_eol.connector_id, portMAX_DELAY));

        ret = xTimerStart( one_shot_tmr_hdle, 0 );

        if ( ret != pdPASS )
        {
#if CONN_EOL_DEBUG_LOG
            LOG(E, "Timer start failed");
#endif            
        }
    }

}

#ifdef INVENTPCBA_TEST
/**
 * @brief Function to test the display segments and backlight
 * 
 * @param eol_data_frame 
 */
EOL_ERROR_FRAME displaytest(EOL_DATA_FRAME* eol_data_frame)
{
    uint8_t read_data = 0; 
    error_type result_rd = uc1510c_read_reg(TP_TIME_REG_ADDR, (uint8_t *)&read_data, UC1510C_REGISTER_SIZE_IN_BYTES);

    if ( result_rd != RES_PASS )
    {
#if CONN_EOL_DEBUG_LOG        
        LOG(E, "LCD Display COMM Error");
#endif        
        eol_data_frame->eol_ack = PCBA_EOL_ERR_COMM;
    }
    else if ( read_data == TP_TIME_REG_VAL )
    {
#if CONN_EOL_DEBUG_LOG        
        LOG(W, "LCD Display Communication success");
#endif        
        eol_data_frame->eol_ack = PCBA_EOL_NO_ERR;
    }
    else
    {
#if CONN_EOL_DEBUG_LOG        
        LOG(E, "LCD Display read/write failed");
#endif        
        eol_data_frame->eol_ack = PCBA_EOL_ERR_DATAMISMATCH;
    }

    return eol_data_frame->eol_ack;
}
#endif

#ifdef INVENTEOL_TEST
/** Prototype definition ********************************************/
/**
 * @brief Function to test the display segments and backlight
 * 
 * @param eol_data_frame 
 */
EOL_ERROR_FRAME display_seg_test(EOL_DATA_FRAME* eol_data_frame)
{
    DISP_CTRL disp_ctrl;
    error_type result_wr ;
    ONBOARDHMI_SEGMENT eol_seg;

    disp_ctrl.seg_num = (eol_data_frame->data >> 8) & 0xFF;
    LOG(W, "disp_num ->data_3 %d\n", disp_ctrl.seg_num);

    disp_ctrl.seg_status = eol_data_frame->data & 0xFF;
    LOG(W,"disp_status -> data_2%d\n", disp_ctrl.seg_status);

    if (eol_data_frame->data == SEG_ALL_ON)
    {
        //onboard_hmi_update_segments(ENABLE_ALL_SEGEMENT);
        for ( eol_seg = SEG_AIR_QUALITY_LEVEL_1_LOW; eol_seg < ONBOARD_HMI_MAX_SEGEMENT; eol_seg++ )
        {
            result_wr = uc1510c_set_segment(eol_seg, SEG_ON);
        }
    }
    else if (eol_data_frame->data == SEG_ALL_OFF)
    {
        //onboard_hmi_update_segments(DISABLE_ALL_SEGEMENT);
        for ( eol_seg = SEG_AIR_QUALITY_LEVEL_1_LOW; eol_seg < ONBOARD_HMI_MAX_SEGEMENT; eol_seg++ )
        {
            result_wr = uc1510c_set_segment(eol_seg, SEG_OFF);
        }
    }
    else
    {
        result_wr = uc1510c_set_segment(disp_ctrl.seg_num, disp_ctrl.seg_status);
    }

    if ( result_wr != RES_PASS)
    {
        LOG(E, "LCD Display COMM Error");
        eol_data_frame->eol_ack = PCBA_EOL_ERR_COMM;
    }
    else
    {
        LOG(W, "LCD Display Communication success");
        eol_data_frame->eol_ack = PCBA_EOL_NO_ERR;
    }

    return eol_data_frame->eol_ack;

}
#endif

void send_datato_broker(uint8_t resp_data[])
{
    uint32_t u32data = 0;

    u32data = ((resp_data[0] << 24)) | ((resp_data[1]) << 16) | ((resp_data[2] << 8)) | (resp_data[3]); 
    TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, IVEOL0RESP, &u32data,  sizeof(int32_t), connector_eol.connector_id, portMAX_DELAY));
    LOG(W, "DATA send to broker, %d", u32data);
}

//#ifdef POWER_CONSUMPTION  

/**
 * @brief Construct a new eol power consumption object
 * 
 * @param case_val :variable input to swicth case to do action
 */
static int16_t eol_power_consumption(EOL_DATA_FRAME* eol_data_frame)
{
    int16_t isys_curr = 0;
    PWR_CTRL pwrctrl;

    pwrctrl.pwr_stat = (eol_data_frame->data >> 8  ) & 0xFF; //data

    LOG(C,"pwr_stat %d", pwrctrl.pwr_stat );

    switch(pwrctrl.pwr_stat)
    {
        case NOLOAD:
            stop_eol_test();
            break;

        case GET_CURRENT:
            isys_curr = power_consumption();
            LOG(W,"ISYS %d", isys_curr);
            stop_eol_test();
            break;

        case SET_FAN1_RPM:
            set_fan_motor_rpm(DEV_FAN1_AIR_IN, EOL_FAN_MAX_RPM);
            LOG(W,"Curr Fan 1 %d", resp_data[1]);
            LOG(W,"Curr FAN 1 %d", resp_data[2]);
            break; 

        case SET_FAN2_RPM:
            set_fan_motor_rpm(DEV_FAN2_AIR_OUT, EOL_FAN_MAX_RPM);
            LOG(W,"Curr Fan 2 %d", resp_data[1]);
            LOG(W,"Curr FAN 2 %d", resp_data[2]);
            break;

        case SET_MOTOR_RPM:
            /*Set Fan1, Fan2, Motor RPM*/ 
            set_fan_motor_rpm(DEV_MOTOR, EOL_FAN_MAX_RPM);   
            LOG(W,"Curr Motor %d", resp_data[1]);
            LOG(W,"Curr Motor %d", resp_data[2]);
            break;

        case SET_LED_BRIGNTNESS:
            /* Set LED Brightness*/
            light_dimmer_set_duty(LED_DIMMER_ON_DUTY_CYCLE);
            break;

        default:
            /*Do Nothing*/
            break;

    }
    
    return isys_curr;
}


#endif /*CONNECTOR_EOL*/ 
