/*! \file connector_voc_sensor.c
	\brief Connector for VOC sensor
	\Author Sundaramoorthy-LTTS
 */

/** Includes ******************************************************************/
#include "configuration.h"

#ifdef CONNECTOR_VOC_SENSOR
#include <stdint.h>
#include "osal.h"
#include "connector_voc_sensor.h"

#if ( CONFIG_DICM_SUPPORT_INTEGRATED_BSEC_LIB_1_X == 1 )
#include "bsec_integration.h"
#endif

#if ( CONFIG_DICM_SUPPORT_INTEGRATED_BSEC_LIB_2_X == 1 )
#include "bsec_integration_2_x.h"
extern const BME6X_BSEC_LIB_INTERFACE g_bsec_lib_intf;
#endif

//#include "bme680_defs.h"
//#include "seq/bme680_seq.h"
#include "hal_i2c_master.h"
#include "sorted_list.h"

#define CONN_VOC_SENS_SUB_DEPTH		                ((uint8_t)    20u)
#define VOC_SENSOR_READ_TASK_DELAY_SEC              ((uint32_t)  1000)
#define CONN_VOC_ENABLE_LOGS                        0
#define DDMP_UNAVAILABLE                            ((uint8_t)  0xFFu)
#define VOC_SENSOR_QUEUE_LENGTH			            ((osal_base_type_t) 10)
#define VOC_SENSOR_QUEUE_ITEM_SIZE   	            ((osal_base_type_t) sizeof(uint32_t))

/* Static function declarations */
static int initialize_connector_voc_sensor(void);
static int add_subscription(DDMP2_FRAME *pframe);
static void conn_voc_process_task(void *pvParameter);
static void conn_voc_read_task(void *pvParameter);
static void handle_subscribe(uint32_t ddm_param);
static void install_parameters(void);

static void publish_data_to_broker(uint32_t ddm_parameter, int32_t i32Value);
static void process_set_and_publish_request(uint32_t ddm_param, int32_t i32value, DDMP2_CONTROL_ENUM req_type);
static void start_subscribe(void);
static void start_publish(void);
static uint8_t get_ddm_index_from_db(uint32_t ddm_param);
static void pwr_state_callback(uint8_t table_index, int32_t i32Value);

static osal_queue_handle_t voc_sens_queue;

/* Structure for Connector VOC Sensor */
CONNECTOR connector_voc_sensor =
{
	.name       = "VOC Sensor Connector",
	.initialize = initialize_connector_voc_sensor,
};

/* DDM Parameter table for connector voc sensor */
static conn_voc_sensor_param_t conn_voc_sensor_param_db[] =
{
   //   ddm_parameter        type           pub  sub                         i32Value               cb_func
   {     SBMEB0IAQ, 	DDM2_TYPE_INT32_T,   1,   0,								0,     			 NULL},
   {    SBMEB0TEMP, 	DDM2_TYPE_INT32_T,   1,   0,								0,     			 NULL},
   {     SBMEB0PRS, 	DDM2_TYPE_INT32_T,   1,   0,								0,     			 NULL},
   {     SBMEB0HUM, 	DDM2_TYPE_INT32_T,   1,   0,								0,     			 NULL},
   {     SBMEB0GAS, 	DDM2_TYPE_INT32_T,   1,   0,								0,     			 NULL},
   {     SBMEB0CO2, 	DDM2_TYPE_INT32_T,   1,   0,								0,     			 NULL},
   {     SBMEB0VOC, 	DDM2_TYPE_INT32_T,   1,   0,								0,     			 NULL},
   {     SBMEB0SST, 	DDM2_TYPE_INT32_T,   1,   0,								0,     			 NULL},
   {     SBMEB0RIS, 	DDM2_TYPE_INT32_T,   1,   0,								0,     			 NULL},
   {     SBMEB0AQR, 	DDM2_TYPE_INT32_T,   1,   0,								0,     			 NULL},
   {       IV0AQST, 	DDM2_TYPE_INT32_T,   1,   0,      IV0AQST_AIR_QUALITY_UNKNOWN,     			 NULL},
   {  IVPMGR0STATE, 	DDM2_TYPE_INT32_T,   0,   1,							    0, pwr_state_callback},
   {    IV0STORAGE,     DDM2_TYPE_INT32_T, 	 0,   1, 	                            0,               NULL},
};

DECLARE_SORTED_LIST_EXTRAM(conn_voc_sens_table, CONN_VOC_SENS_SUB_DEPTH);       //!< \~ Subscription table storage

/* Calculate the connector voc database table num elements */
static const uint32_t conn_voc_db_elements = ELEMENTS(conn_voc_sensor_param_db);

/**
  * @brief  Initialize the connector for air quality
  * @param  none.
  * @retval none.
  */
static int initialize_connector_voc_sensor(void)
{
    /* Initialize the BSEC library interface with all the required function callbacks */
    init_bsec_lib_interface();
    
    /* Create queue for invent control task */
    voc_sens_queue = osal_queue_create(VOC_SENSOR_QUEUE_LENGTH, VOC_SENSOR_QUEUE_ITEM_SIZE);

    if ( NULL != voc_sens_queue )
    {
        LOG(I, "Queue creation done for voc sensor");
    }

	TRUE_CHECK(osal_task_create(conn_voc_process_task, CONNECTOR_VOC_SENSOR_PROCESS_TASK_NAME, CONNECTOR_VOC_PROCESS_TASK_STACK_DEPTH, NULL, CONNECTOR_VOC_PROCESS_TASK_PRIO, NULL));
	TRUE_CHECK(osal_task_create(conn_voc_read_task, CONNECTOR_VOC_SENSOR_CONTROL_TASK_NAME, CONNECTOR_VOC_I2C_RD_SERVICE_TASK_DEPTH, NULL, CONNECTOR_VOC_I2C_RD_SERVICE_TASK_PRIO, NULL));

	install_parameters();
    start_subscribe();
    start_publish();

	return 1;
}

/**
  * @brief  Task for connector VOC sensor to proces ddmp request
  * @param  pvParameter.
  * @retval none.
  */
static void conn_voc_process_task(void *pvParameter)
{
	DDMP2_FRAME *pframe;
	size_t frame_size;
    int32_t available = 1;

	while (1)
	{
		TRUE_CHECK ( pframe = xRingbufferReceive(connector_voc_sensor.to_connector, &frame_size, portMAX_DELAY) );

		switch (pframe->frame.control)
		{
		case DDMP2_CONTROL_PUBLISH:/*Send the changed data to broker*/
		case DDMP2_CONTROL_SET:
            process_set_and_publish_request(pframe->frame.publish.parameter, pframe->frame.publish.value.int32, pframe->frame.control);
			break;

		case DDMP2_CONTROL_SUBSCRIBE:/*Send the request for the data*/
			add_subscription(pframe);//Check if this can be done during initializtion
			handle_subscribe(pframe->frame.subscribe.parameter);
			break;

        case DDMP2_CONTROL_REG:
#if CONN_PWM_DEBUG_LOG
            LOG(I, "Received DDMP2_CONTROL_REG device_class = 0x%x", pframe->frame.reg.device_class);
#endif
            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, pframe->frame.reg.device_class, &available, 
                        sizeof(int32_t), connector_voc_sensor.connector_id, portMAX_DELAY));
            break;

		default:
			LOG(E, "Connector VOC sensor received UNHANDLED frame %02x from broker!",pframe->frame.control);
			break;
		}

		vRingbufferReturnItem(connector_voc_sensor.to_connector, pframe);
    }
}

/**
  * @brief  Task for read VOC data
  * @param  pvParameter.
  * @retval none.
  */
static void conn_voc_read_task(void *pvParameter)
{
    while (1)
	{      
        /* State macine to read VOC sensor */
#if ( CONFIG_DICM_SUPPORT_INTEGRATED_BSEC_LIB_1_X == 1 )
        /* Continous loop function to read and process the sensor data and calc IAQ */
        bsec_processing_loop(voc_sens_queue);
#endif

#if ( CONFIG_DICM_SUPPORT_INTEGRATED_BSEC_LIB_2_X == 1 )
        /* Continous loop function to read and process the sensor data and calc IAQ */
        bsec_processing_loop(voc_sens_queue);
#endif 
    }
}

/**
  * @brief  Function to publish Inventilate available to the broker
  * @param  none.
  * @retval none.
  */
static void install_parameters(void)
{
	int32_t available = 1;

    TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, SBMEB0AVL, &available, sizeof(int32_t), \
               connector_voc_sensor.connector_id, portMAX_DELAY));
}

/**
  * @brief  Function to subscribe the DDMP parameters needed for connector fan and motor
  * @param  none.
  * @retval none.
  */
static void start_subscribe(void)
{
	conn_voc_sensor_param_t *ptr_param_db;
	uint8_t db_idx;

	for ( db_idx = 0; db_idx < conn_voc_db_elements; db_idx++ )
	{
		ptr_param_db = &conn_voc_sensor_param_db[db_idx];
        
        /* Check the DDM parameter need subscribtion */
		if ( ptr_param_db->sub )
		{
            LOG(I, "Subscribed DDMP for %s is 0x%x", connector_voc_sensor.name, ptr_param_db->ddm_parameter);
            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SUBSCRIBE, ptr_param_db->ddm_parameter, &ptr_param_db->i32Value, sizeof(int32_t), connector_voc_sensor.connector_id, portMAX_DELAY));
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
    conn_voc_sensor_param_t *ptr_param_db;
    uint16_t db_idx;
    
    for ( db_idx = 0; db_idx < conn_voc_db_elements; db_idx++ )
    {
        ptr_param_db = &conn_voc_sensor_param_db[db_idx];

        /* Check the DDM parameter need to publish */
        if ( ptr_param_db->pub ) 
        {
            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, ptr_param_db->ddm_parameter, &ptr_param_db->i32Value, sizeof(int32_t), connector_voc_sensor.connector_id, portMAX_DELAY));
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

    return sorted_list_single_add(&conn_voc_sens_table, key, value);
}

/**
  * @brief  Function handle subscribed DDM parameter received from broker
  * @param  DDM parameter.
  * @retval none.
  */
static void handle_subscribe(uint32_t ddm_parameter)
{
	uint16_t db_idx;
    int index;
    int factor = 0;
    bool sent = false;
    int32_t value = 0;
	conn_voc_sensor_param_t* param_db;
    uint32_t list_value = 0;
    SORTED_LIST_RETURN_VALUE ret = sorted_list_unique_get(&list_value, &conn_voc_sens_table, ddm_parameter, 0);

    if ( SORTED_LIST_FAIL != ret )
	{
		for ( db_idx = 0; ( db_idx < ELEMENTS(conn_voc_sensor_param_db) && ( sent == false ) ); db_idx++ )
 		{
			param_db = &conn_voc_sensor_param_db[db_idx];

		 	/* validate the DDM parameter received */
		 	if ( param_db->ddm_parameter == ddm_parameter )
	  	 	{
				index = ddm2_parameter_list_lookup(DDM2_PARAMETER_BASE_INSTANCE(ddm_parameter));

                if ( -1 != index )
	            {
                    /* Get the factor form list corresponding to the DDM Parameter */
                    factor = Ddm2_unit_factor_list[Ddm2_parameter_list_data[index].out_unit];

                    if ( factor == 0 )
                    {
                        factor = 1;
                    }
      
                    /* Multiply with the factor */
                    value = param_db->i32Value * factor;
#if CONN_VOC_ENABLE_LOGS                    
                    LOG(I, "After factored i32value = %d", value);
#endif
                    /* Frame and send the publish request */
		            publish_data_to_broker(ddm_parameter, value);
                    /* Update flag */
                    sent = true;
                }
                else
                {
                    LOG(E, "DDMP 0x%x not found in ddm2_parameter_list_lookup", ddm_parameter);
                }
			}
	    }
	}
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
	conn_voc_sensor_param_t* param_db;

#if CONN_PWM_DEBUG_LOG
    LOG(I, "Received ddm_param = 0x%x i32value = %d", ddm_param, i32value);
#endif

	/* Validate the DDM parameter received */
	db_idx = get_ddm_index_from_db(ddm_param);
 
	if ( DDMP_UNAVAILABLE != db_idx )
	{
        if ( req_type == DDMP2_CONTROL_SET )
        {
            /* Frame and send the publish request */
            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, ddm_param, &pub_value, sizeof(int32_t), connector_voc_sensor.connector_id, portMAX_DELAY));
        }

		param_db = &conn_voc_sensor_param_db[db_idx];

#if CONN_PWM_DEBUG_LOG		
		LOG(I, "Valid DDMP parameter");
#endif
        i32Index = ddm2_parameter_list_lookup(DDM2_PARAMETER_BASE_INSTANCE(ddm_param));

        if ( -1 != i32Index )
        {

#if CONN_PWM_DEBUG_LOG
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
                
#if CONN_PWM_DEBUG_LOG
            LOG(I, "After factored factor_value = %d", factor_value);
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
  * @brief  Publish the value to broker
  * @param  ddm_parameter.
  * @param  i32Value.
  * @retval none.
  */
static void publish_data_to_broker(uint32_t ddm_parameter, int32_t i32Value)
{
	connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, ddm_parameter, &i32Value, sizeof(i32Value), connector_voc_sensor.connector_id, (TickType_t)portMAX_DELAY);
}

/**
  * @brief  Update the newly received value in the data base and publish to the broker
  * @param  ddm_parameter
  * @param  i32Value
  * @retval Return the Updated status.
  */
bool update_and_publish_to_broker(uint32_t ddm_parameter, int32_t i32Value)
{
    uint8_t db_idx;
    int index;
    int32_t value;
    bool updated = false;
    int factor = 0;
    conn_voc_sensor_param_t* param_db = &conn_voc_sensor_param_db[0u];
    
    for ( db_idx = 0u; ( ( db_idx < ELEMENTS(conn_voc_sensor_param_db ) ) && ( updated == false ) ); db_idx++ )
    {
        if ( param_db->ddm_parameter == ddm_parameter )
        {
            index = ddm2_parameter_list_lookup(DDM2_PARAMETER_BASE_INSTANCE(ddm_parameter));

            if ( -1 != index )
	        {
                /* Get the factor form list corresponding to the DDM Parameter */
                factor = Ddm2_unit_factor_list[Ddm2_parameter_list_data[index].out_unit];

                if ( factor == 0 )
                {
                    factor = 1;
                }

                /* Update the value in database */
                param_db->i32Value = i32Value;
        
                /* Multiply with the factor */
                value = param_db->i32Value * factor;
#if CONN_VOC_ENABLE_LOGS
                LOG(I, "After factored i32value = %d", value);
#endif 
                /* Frame and send the publish request */
		        publish_data_to_broker(ddm_parameter, value);

                /* Update the flag */
    	        updated = true;
            }
            else
            {
                LOG(E, "DDMP 0x%x not found in ddm2_parameter_list_lookup", ddm_parameter);
            }
        }

        /* Increment the pointer */
        param_db++;
    }
    
    return updated;
}

/**
  * @brief  Function to get ddm index from database table
  * @param  DDMP Parameter.
  * @retval none.
  */
static uint8_t get_ddm_index_from_db(uint32_t ddm_param)
{
	conn_voc_sensor_param_t* param_db;
	uint8_t db_idx = DDMP_UNAVAILABLE; 
	uint8_t index;
	bool avail = false;

	for ( index = 0u; ( ( index < conn_voc_db_elements ) && ( avail == false ) ); index++ )
 	{
		param_db = &conn_voc_sensor_param_db[index];
      	
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
  * @param  table_index
  * @param  i32Value.
  * @retval none.
  */
static void pwr_state_callback(uint8_t table_index, int32_t i32Value)
{
    uint32_t voc_op_mode;

    if ( IVPMGR0STATE_ACTIVE == i32Value )
    {
#if ( CONFIG_DICM_SUPPORT_INTEGRATED_BSEC_LIB_1_X == 1 )
        voc_op_mode = BME680_FORCED_MODE;
#else
        voc_op_mode = BME68X_FORCED_MODE;
#endif
    }
    else
    {
#if ( CONFIG_DICM_SUPPORT_INTEGRATED_BSEC_LIB_1_X == 1 )
        voc_op_mode = BME680_SLEEP_MODE;
#else
        voc_op_mode = BME68X_SLEEP_MODE;
#endif
    }

    // push the data in the Queue
    osal_base_type_t ret = osal_queue_send(voc_sens_queue, &voc_op_mode, 0);

    if ( osal_success != ret )
    {
        LOG(E, "Queue error ret = %d", ret);
    }
}

#endif /* CONNECTOR_VOC_SENSOR */ 

