/*! \file connector_voc_sensor.c
	\brief Connector for VOC sensor
	\Author Sundaramoorthy-LTTS
 */

/** Includes ******************************************************************/
#include "configuration.h"

#ifdef CONNECTOR_VOC_SENSOR

#include <stdint.h>

#include "broker.h"
#include "osal.h"
#include "connector_voc_sensor.h"
#include "app_error_code.h"

#if ( CONFIG_DICM_SUPPORT_INTEGRATED_BSEC_LIB_2_X == 1 )
#include "bsec_integration_2_x.h"
extern const BME6X_BSEC_LIB_INTERFACE g_bsec_lib_intf;
#endif

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
static void storage_mode_cb(uint8_t table_index, int32_t i32Value);
static void inv_err_stat_callback(uint8_t table_index, int32_t i32Value);

static osal_queue_handle_t voc_sens_queue;
static uint32_t invent_error_stat = 0;

static int l_sbmeb_instance;
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
   {     SBMEB0IAQ, 	DDM2_TYPE_INT32_T,   1,   0,								0,         			 NULL},
   {    SBMEB0TEMP, 	DDM2_TYPE_INT32_T,   1,   0,								0,         			 NULL},
   {     SBMEB0PRS, 	DDM2_TYPE_INT32_T,   1,   0,								0,         			 NULL},
   {     SBMEB0HUM, 	DDM2_TYPE_INT32_T,   1,   0,								0,         			 NULL},
   {     SBMEB0GAS, 	DDM2_TYPE_INT32_T,   1,   0,								0,         			 NULL},
   {     SBMEB0CO2, 	DDM2_TYPE_INT32_T,   1,   0,								0,         			 NULL},
   {     SBMEB0VOC, 	DDM2_TYPE_INT32_T,   1,   0,								0,         			 NULL},
   {     SBMEB0SST, 	DDM2_TYPE_INT32_T,   1,   0,								0,         			 NULL},
   {     SBMEB0RIS, 	DDM2_TYPE_INT32_T,   1,   0,								0,         			 NULL},
   {     SBMEB0AQR, 	DDM2_TYPE_INT32_T,   1,   0,								0,         			 NULL},
   {       IV0AQST, 	DDM2_TYPE_INT32_T,   0,   1,      IV0AQST_AIR_QUALITY_UNKNOWN,         			 NULL},
   {  IVPMGR0STATE, 	DDM2_TYPE_INT32_T,   0,   1,							    0,     pwr_state_callback},
   {    IV0STORAGE,     DDM2_TYPE_INT32_T, 	 0,   1, 	                            0,                   NULL},
   {       IV0STGT,     DDM2_TYPE_INT32_T, 	 0,   1, 	                            0,        storage_mode_cb},
   {      IV0ERRST,     DDM2_TYPE_INT32_T, 	 0,   1, 	                            0,  inv_err_stat_callback},
};

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

    if (NULL != voc_sens_queue)
    {
        LOG(I, "Queue creation done for voc sensor");
    }
    install_parameters();
    start_subscribe();
    start_publish();

	TRUE_CHECK(osal_task_create(conn_voc_process_task, CONNECTOR_VOC_SENSOR_PROCESS_TASK_NAME, CONNECTOR_VOC_PROCESS_TASK_STACK_DEPTH, NULL, CONNECTOR_VOC_PROCESS_TASK_PRIO, NULL));
	TRUE_CHECK(osal_task_create(conn_voc_read_task, CONNECTOR_VOC_SENSOR_CONTROL_TASK_NAME, CONNECTOR_VOC_I2C_RD_SERVICE_TASK_DEPTH, NULL, CONNECTOR_VOC_I2C_RD_SERVICE_TASK_PRIO, NULL));


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
            //fallthrough
		case DDMP2_CONTROL_SET:
            process_set_and_publish_request(pframe->frame.publish.parameter, pframe->frame.publish.value.int32, pframe->frame.control);
			break;

		case DDMP2_CONTROL_SUBSCRIBE:/*Send the request for the data*/
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
#if (CONFIG_DICM_SUPPORT_INTEGRATED_BSEC_LIB_2_X == 1)
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
	uint32_t device_class = SBMEB0AVL;
	l_sbmeb_instance = broker_register_instance(&device_class, connector_voc_sensor.connector_id);
	ASSERT(l_sbmeb_instance != -1);
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

	for (db_idx = 0; db_idx < conn_voc_db_elements; db_idx++)
	{
		ptr_param_db = &conn_voc_sensor_param_db[db_idx];

        /* Check the DDM parameter need subscribtion */
		if (ptr_param_db->sub)
		{
            LOG(I, "Subscribed DDMP for %s is 0x%x", connector_voc_sensor.name, ptr_param_db->ddm_parameter);
            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SUBSCRIBE, ptr_param_db->ddm_parameter, NULL, 0, connector_voc_sensor.connector_id, portMAX_DELAY));
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

    for (db_idx = 0; db_idx < conn_voc_db_elements; db_idx++)
    {
        ptr_param_db = &conn_voc_sensor_param_db[db_idx];

        /* Check the DDM parameter need to publish */
        if (ptr_param_db->pub)
        {
            uint32_t param = ptr_param_db->ddm_parameter;
            if (DDM2_PARAMETER_CLASS(ptr_param_db->ddm_parameter) == SBMEB0)
            {
                param = DDM2_PARAMETER_BASE_INSTANCE(ptr_param_db->ddm_parameter) | DDM2_PARAMETER_INSTANCE(l_sbmeb_instance);
            }
            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, param, &ptr_param_db->i32Value, sizeof(int32_t), connector_voc_sensor.connector_id, portMAX_DELAY));
        }
    }
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
    for (db_idx = 0; db_idx < ELEMENTS(conn_voc_sensor_param_db) && (sent == false); db_idx++)
    {
        param_db = &conn_voc_sensor_param_db[db_idx];

        /* validate the DDM parameter received */
        if (param_db->ddm_parameter == ddm_parameter)
        {
            index = ddm2_parameter_list_lookup(DDM2_PARAMETER_BASE_INSTANCE(ddm_parameter));

            if (-1 != index)
            {
                /* Get the factor form list corresponding to the DDM Parameter */
                factor = Ddm2_unit_factor_list[Ddm2_parameter_list_data[index].out_unit];

                if (factor == 0)
                {
                    factor = 1;
                }

                /* Multiply with the factor */
                value = param_db->i32Value * factor;
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

	if (DDMP_UNAVAILABLE != db_idx)
	{
        if (req_type == DDMP2_CONTROL_SET)
        {
            /* Frame and send the publish request */
            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, ddm_param, &pub_value, sizeof(int32_t), connector_voc_sensor.connector_id, portMAX_DELAY));
        }

		param_db = &conn_voc_sensor_param_db[db_idx];

#if CONN_PWM_DEBUG_LOG
		LOG(I, "Valid DDMP parameter");
#endif
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
		        if  (NULL != param_db->cb_func)
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

    for (db_idx = 0u; ((db_idx < ELEMENTS(conn_voc_sensor_param_db)) && (updated == false)); db_idx++)
    {
        if (param_db->ddm_parameter == ddm_parameter)
        {
            index = ddm2_parameter_list_lookup(DDM2_PARAMETER_BASE_INSTANCE(ddm_parameter));

            if (-1 != index)
	        {
                /* Get the factor form list corresponding to the DDM Parameter */
                factor = Ddm2_unit_factor_list[Ddm2_parameter_list_data[index].out_unit];

                if (factor == 0)
                {
                    factor = 1;
                }

                /* Update the value in database */
                param_db->i32Value = i32Value;

                /* Multiply with the factor */
                value = param_db->i32Value * factor;
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
#if ( CONFIG_DICM_SUPPORT_INTEGRATED_BSEC_LIB_2_X == 1 )
        voc_op_mode = BME68X_FORCED_MODE;
#endif
    }
    else
    {
#if ( CONFIG_DICM_SUPPORT_INTEGRATED_BSEC_LIB_2_X == 1 )
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

static void storage_mode_cb(uint8_t table_index, int32_t i32Value)
{
    uint32_t voc_op_mode_sel;


    LOG(I,"[Storage_mode %d]", i32Value);

    if (IV0STGT_RUN_3HRS == i32Value )
    {
#if ( CONFIG_DICM_SUPPORT_INTEGRATED_BSEC_LIB_2_X == 1 )
        voc_op_mode_sel = BME68X_FORCED_MODE;
#endif
    }
    else if (IV0STGT_IDLE_21HRS == i32Value )
    {
#if ( CONFIG_DICM_SUPPORT_INTEGRATED_BSEC_LIB_2_X == 1 )
        voc_op_mode_sel = BME68X_SLEEP_MODE;
#endif
    }

    // push the data in the Queue
    osal_base_type_t ret = osal_queue_send(voc_sens_queue, &voc_op_mode_sel, 0);

    if ( osal_success != ret )
    {
        LOG(E, "Queue error ret = %d", ret);
    }

}

/**
  * @brief  Callback function to handle subscribed data
  * @param  table_index
  * @param  i32Value.
  * @retval none.
  */
static void inv_err_stat_callback(uint8_t table_index, int32_t i32Value)
{
    /* Update error status value */
    invent_error_stat = i32Value;
}

/**
  * @brief  Callback function to handle subscribed data
  * @param  table_index
  * @param  i32Value.
  * @retval none.
  */
void bme68x_error_code(const BME68X_ERROR error)
{
    uint32_t err_frame = invent_error_stat;

    switch (error)
    {
        case BME68X_COMM_ERROR:
            err_frame |=  (1 << VOC_SENSOR_COMMUNICATION_ERROR);
            LOG(I,"voc Err 0");
            break;

        case BME68X_DATA_PLAUSIBLE_ERROR:
            err_frame |=  (1 << VOC_SENSOR_DATA_PLAUSIBLE_ERROR);
            LOG(I,"voc Err 1");
            break;

        case BME68X_NO_ERROR:
            err_frame &= ~(1 << VOC_SENSOR_COMMUNICATION_ERROR);
            err_frame &= ~(1 << VOC_SENSOR_DATA_PLAUSIBLE_ERROR);
            LOG(I,"voc Err 3");
            break;

        default:
            LOG(E, "Unhandled error code = %d", error);
            break;
    }

    if (err_frame != invent_error_stat)
    {
        connector_send_frame_to_broker(DDMP2_CONTROL_SET, IV0ERRST, &err_frame, sizeof(err_frame), connector_voc_sensor.connector_id, (TickType_t)portMAX_DELAY);
    }
}

#endif /* CONNECTOR_VOC_SENSOR */

