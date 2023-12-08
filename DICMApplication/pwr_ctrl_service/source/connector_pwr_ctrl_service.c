
/*! \file connector_pwr_ctrl_service.c
	\brief Connector for Power Control Service
	\Author Sundaramoorthy-LTTS
 */

/** Includes ******************************************************************/
#include "configuration.h"

#ifdef CONNECTOR_POWER_CONTROL_SERVICE

#include "connector_pwr_ctrl_service.h"
#include "app_api.h"
#include "ddm2_parameter_list.h"
#include "osal.h"
#include "sorted_list.h"
#if defined(DEVICE_BQ25792)
#include "drv_bq25792.h"
#endif

#if defined(DEVICE_BQ25798) 
#include "drv_bq25798.h"
#endif

/* Macro Definitions */
#define CONN_PWR_DEBUG_LOG      1
#define EN_POOR_SOURCE_CHECK    0

#define CONN_PWR_CTRL_SUB_DEPTH           ((uint8_t)   20u)
#define DDMP_UNAVAILABLE                  ((uint8_t) 0xFFu)
#define INV_PWR_CTRL_TASK_QUE_LEN	      ((osal_base_type_t) 10)
#define INV_PWR_CTRL_TASK_QUE_ITEM_SIZE   ((osal_base_type_t) sizeof(PWR_CTRL_DATA))

#ifdef FILTER_TEST
/*Filter Test*/
#warning "Filter in Testing mode"
#define FILTER_LIFE_TIME_MIN              ((uint32_t)   3) // 3 minutes

#else
/*filter for application*/
#define FILTER_LIFE_TIME_MIN              ((uint32_t)   30000) // 500 hours = 500 * 60 = 30000 minutes

#endif // FILTER_TEST

#define SECONDS_PER_MINUTE                ((uint32_t)    60)
#define MSEC_PER_MINUTE                   ((uint32_t) 60000)
#define FILTER_TIMER_PERIOD_MIN           ((uint32_t)     1)
#define POWER_CONTROL_SEC                 ((uint16_t)      5000)        //5 seconds

#define FILTER_TIMER_TICKS                pdMS_TO_TICKS(MIN_TO_MSEC(FILTER_TIMER_PERIOD_MIN))
#define POWER_CONTROL_TIMER_TICKS         pdMS_TO_TICKS(POWER_CONTROL_SEC)

#define BACKUP_BATTERY_LOW_THRESHOLD       11.8
#define BACKUP_BATTERY_HIGH_THRESHOLD      12.3
#define VAC_MIN_VOLT                       ((uint8_t) 8)
#define EN_VAC_ERR_CODE                     0
#define POOR_SOURCE_TIMER                   3
#define POOR_SOURCE_MAX_COUNT               5
#define POOR_SOURCE_COUNT_LIMIT             (POOR_SOURCE_MAX_COUNT - 2)
#define BACKUP_BATTERY_LOW_LIMIT            12.0
#define FLAG_EN_HIZ                         ((uint8_t) 4)

#define  STATUS_BIT_SOLAR       0
#define  STATUS_BIT_IONIZER     1

typedef enum _bat_sts_
{
    INV_BATTERY_GOOD  = 0,
    INV_BATTERY_LOW   = 1

}INV_BAT_STS;

#ifdef INVENT_BATTERY_TESTING
static uint8_t poor_source_flag = 0;
static uint8_t poor_source_count = 0;
#endif // INVENT_BATTERY_TESTING

static int initialize_connector_pwrctrl_service(void);
static error_type initialize_pwr_control_module(void);
static void install_parameters(void);
static void start_subscribe(void);
static int add_subscription(DDMP2_FRAME *pframe);
static void conn_pwr_ctrl_process_task(void *pvParameter);
static void process_set_and_publish_request(uint32_t ddm_param, int32_t i32value, DDMP2_CONTROL_ENUM req_type);
static void process_subscribe_request(uint32_t ddm_param);
static void l_update_and_send_val_to_broker(uint32_t ddm_parameter, int32_t value);
static uint8_t get_ddm_index_from_db(uint32_t ddm_param);
static void conn_pwr_ctrl_manager_task(void *pvParameter);
#ifdef INVENT_BATTERY_TESTING
#ifdef DEVICE_BQ25792
static void conn_pwr_ctrl_bms_task_bq25792(void *pvParameter);
#endif

#ifdef DEVICE_BQ25798
static void conn_pwr_ctrl_bms_task_bq25798(void *pvParameter);
#endif 

float bq_read_vbat(void);
#endif // INVENT_BATTERY_TESTING
static void init_pwr_ctrl_sm(PWR_CTRL_SM* pwr_ctrl_sm);
static void handle_pwr_ctrl_sub_data(uint32_t ddm_param, int32_t data);
static void parse_pwr_ctrl_frame(PWR_CTRL_DATA* pwr_ctrl_data_frame);
static void start_publish(void);
static void pwr_ctrl_timer_cb_func ( TimerHandle_t xTimer );
static void start_power_control_timer(void);
static void stop_power_control_timer(void);
static void fil_timer_cb_func(TimerHandle_t xTimer );
static void start_filter_timer(void);
static void stop_filter_timer(PWR_CTRL_SM* ptr_pwr_ctrl_sm);
static void change_ivpmgr_state(IVPMGR0STATE_ENUM  inv_pwr_ctrl_state);
static FILTER_RESET_STATUS validate_filter_time(PWR_CTRL_SM* ptr_pwr_ctrl_sm);
#if defined (INVENT_BATTERY_TESTING) || defined (EMC_TEST_CASE_SOLAR)
static void update_active_power_source(void);
static void push_pwrsrc_status_in_queue(IV0PWRSRC_ENUM curr_active_source);
#endif
#if defined(DEVICE_BQ25792)  || defined(DEVICE_BQ25798) 

#ifdef INVENT_BATTERY_TESTING
static void pwr_ctrl_error_code(const PWR_CTRL_ERR_CODES error);
//! Error code handle variable
static uint32_t invent_pwr_ctrl_error_stat = 0;
#endif
static uint8_t batt_ch_intr_stat = BATT_CH_STATE_IC_INIT;


static REG26_FAULT_FLAG_0_REG     fault_flag0_reg     = {0};
static REG27_FAULT_FLAG_1_REG     fault_flag1_reg     = {0};
static REG20_FAULT_STATUS_0_REG   fault_stat0_reg     = {0};
static REG21_FAULT_STATUS_1_REG   fault_stat1_reg     = {0};
static REG1B_CHARGER_STATUS_0_REG chr_status0_reg     = {0};
static REG1D_CHARGER_STATUS_2_REG chr_status2_reg     = {0};
static REG22_CHARGER_FLAG_0_REG   chr_flag0_reg       = {0};
static REG23_CHARGER_FLAG_1_REG   chr_flag1_reg       = {0};
#if defined (INVENT_BATTERY_TESTING) || defined (EMC_TEST_CASE_SOLAR)
static REG13_CHARGER_CTRL_4       chg_ctrl_reg4;
static REG29_CHARGER_1_MASK       chg_1_mask_reg29;
static float vac1 = 0.0f;
static float vac2 = 0.0f;
#endif // defined (INVENT_BATTERY_TESTING) || defined (EMC_TEST_CASE_SOLAR)

static uint8_t update_active_source_flag = 0;
static uint8_t update_active_source_timer = 0;
static uint8_t display_bms_info_timer  = 0;
static uint8_t reg22_byte_prev = 0;

extern int32_t press_sim_value;
REG1C_CHARGER_STATUS_1_REG ch_stat_1;

IV0PWRSRC_ENUM curr_active_src = IV0PWRSRC_BACKUP_BATTERY + 1;
IV0PWRSRC_ENUM prev_active_src = IV0PWRSRC_BACKUP_BATTERY + 1;
#endif // defined(DEVICE_BQ25792)  || defined(DEVICE_BQ25798)
#ifdef INVENT_BATTERY_TESTING
static inv_bat_status_flags BATTERY_STATUS;
#endif
/* Structure for Connector fan motor */
CONNECTOR connector_pwr_ctrl_service =
{
	.name = "Connector Power Ctrl Service",
	.initialize = initialize_connector_pwrctrl_service,
};

/* DDM Parameter table for connector pwr control service */
static conn_pwr_ctrl_parameter_t conn_pwr_ctrl_param_db[] =
{
	{.ddm_parameter = IV0PWRON     ,  .type = DDM2_TYPE_INT32_T, .pub = 0, .sub = 1, .i32Value = 0                     ,  .cb_func = handle_pwr_ctrl_sub_data},
    {.ddm_parameter = IVPMGR0STATE ,  .type = DDM2_TYPE_INT32_T, .pub = 1, .sub = 0, .i32Value = IVPMGR0STATE_STANDBY  ,  .cb_func =                     NULL},
    {.ddm_parameter = IV0FILST     ,  .type = DDM2_TYPE_INT32_T, .pub = 0, .sub = 1, .i32Value = 0                     ,  .cb_func = handle_pwr_ctrl_sub_data},
    {.ddm_parameter = IV0STORAGE   ,  .type = DDM2_TYPE_INT32_T, .pub =	0, .sub = 1, .i32Value = IV0STORAGE_DEACTIVATE ,  .cb_func = handle_pwr_ctrl_sub_data},
    {.ddm_parameter = IV0SETCHRGCRNT ,.type = DDM2_TYPE_INT32_T, .pub =	0, .sub = 1, .i32Value = 0                      , .cb_func = handle_pwr_ctrl_sub_data},
    {.ddm_parameter = IV0SETT ,.type = DDM2_TYPE_INT32_T, .pub =	0, .sub = 1, .i32Value = 0                          , .cb_func = handle_pwr_ctrl_sub_data}
};

/* Calculate the connector power control database table num elements */
static const uint32_t conn_pwrctrl_db_elements = ELEMENTS(conn_pwr_ctrl_param_db);

DECLARE_SORTED_LIST_EXTRAM(conn_pwrctrl_table, CONN_PWR_CTRL_SUB_DEPTH);       //!< \~ Subscription table storage
 
static osal_queue_handle_t inv_pwr_ctrl_que_hdle;
static PWR_CTRL_SM pwr_ctrl_sm;

//! Timer handle for filter handling
static TimerHandle_t xFilterTimer;
static TimerHandle_t xPowerControl_Timer;

#ifdef INVENT_BATTERY_TESTING
//! Battery status for  idle mode
static uint8_t inv_bkup_bat_sts = INV_BATTERY_GOOD;
#endif
/*
battery condition   status
good                0
bad                 1 

*/

/**
  * @brief  Initialize the connector fan motor
  * @param  none.
  * @retval none.
  */
static int initialize_connector_pwrctrl_service(void)
{
    error_type res = 0;
    LOG(I, "initialize_connector_pwrctrl_service");

    init_pwr_ctrl_sm(&pwr_ctrl_sm);

    /* Create queue for invent control task */
    inv_pwr_ctrl_que_hdle = osal_queue_create(INV_PWR_CTRL_TASK_QUE_LEN, INV_PWR_CTRL_TASK_QUE_ITEM_SIZE);

    if ( NULL != inv_pwr_ctrl_que_hdle )
    {
        LOG(I, "Queue creation done for inventilate power control task");
    }

    /* Create the filter timer, software timer for inventilate control logic, storing the handle in xFilterTimer. */
    xFilterTimer = xTimerCreate("Filtertimer", FILTER_TIMER_TICKS, pdTRUE, 0, fil_timer_cb_func);

    /*Create Power control timer*/
    xPowerControl_Timer = xTimerCreate("PwrCtrlTimer", POWER_CONTROL_TIMER_TICKS, pdTRUE, 0, pwr_ctrl_timer_cb_func);
	/* Initialize the power control module */
    res = initialize_pwr_control_module();

    /* Task for connector fan and motor control */
	TRUE_CHECK(osal_task_create(conn_pwr_ctrl_process_task, CONNECTOR_PWR_CTRL_SERV_PROCESS_TASK_NAME, CONNECTOR_PWR_CTRL_PROCESS_TASK_DEPTH, NULL, CONNECTOR_PWR_CTRL_SERV_TASK_PRIORITY, NULL));
    TRUE_CHECK(osal_task_create(conn_pwr_ctrl_manager_task, CONNECTOR_PWR_CTRL_MANAGER_TASK_NAME, CONNECTOR_PWR_CTRL_MNGR_TASK_DEPTH, inv_pwr_ctrl_que_hdle, CONNECTOR_PWR_CTRL_MNGR_TASK_PRIORITY, NULL));
    if ( RES_PASS == res )
    {
#ifdef INVENT_BATTERY_TESTING
#ifdef DEVICE_BQ25792      
        TRUE_CHECK(osal_task_create(conn_pwr_ctrl_bms_task_bq25792, CONNECTOR_PWR_CTRL_BMS_TASK_NAME, CONNECTOR_PWR_CTRL_BMS_TASK_STACK_DEPTH, NULL, CONNECTOR_PWR_CTRL_BMS_TASK_PRIORITY, NULL));
#endif

#ifdef DEVICE_BQ25798       
        TRUE_CHECK(osal_task_create(conn_pwr_ctrl_bms_task_bq25798, CONNECTOR_PWR_CTRL_BMS_TASK_NAME, CONNECTOR_PWR_CTRL_BMS_TASK_STACK_DEPTH, NULL, CONNECTOR_PWR_CTRL_BMS_TASK_PRIORITY, NULL));
#endif

#endif
    }

    pwr_ctrl_sm.filter_min_counter  = filter_data.filter_min;
    LOG(I,"[Filter Timer NVS filter_status: %d min: %d]",filter_data.filter_status, filter_data.filter_min);
    LOG(I,"[Filter Timer sec: %d min: %d]",pwr_ctrl_sm.filter_sec_counter, pwr_ctrl_sm.filter_min_counter);
	/* Install parameters in the Inventory of broker */
	install_parameters();

	/* Subscribe DDMP parameters */
	start_subscribe();
    start_publish();

    start_power_control_timer();
	return 1;
}

/**
  * @brief  Function to publish the DDMP parameters
  * @param  none.
  * @retval none.
  */
static void start_publish(void)
{
    conn_pwr_ctrl_parameter_t *ptr_param_db;
    uint16_t db_idx;
    uint8_t num_elements = ELEMENTS(conn_pwr_ctrl_param_db);
    
    for ( db_idx = 0; db_idx < num_elements; db_idx++ )
    {
        ptr_param_db = &conn_pwr_ctrl_param_db[db_idx];

        /* Check the DDM parameter need to publish */
        if ( ptr_param_db->pub ) 
        {
            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, ptr_param_db->ddm_parameter, &ptr_param_db->i32Value, \
                        sizeof(int32_t), connector_pwr_ctrl_service.connector_id, portMAX_DELAY));
        }
    }
}

/**
  * @brief  Function to publish Connector Power control Service AVL to broker
  * @param  none.
  * @retval none.
  */
static void install_parameters(void)
{
	int32_t available = 1;
    
    TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, IVPMGR0AVL, &available, sizeof(int32_t), \
                connector_pwr_ctrl_service.connector_id, portMAX_DELAY));
}

/**
  * @brief  Function to subscribe the DDMP parameters needed for connector fan and motor
  * @param  none.
  * @retval none.
  */
static void start_subscribe(void)
{
	conn_pwr_ctrl_parameter_t *ptr_param_db;
	uint8_t db_idx;

	for ( db_idx = 0; db_idx < conn_pwrctrl_db_elements; db_idx++ )
	{
		ptr_param_db = &conn_pwr_ctrl_param_db[db_idx];
        /* Check the DDM parameter need subscribtion */
		if ( ptr_param_db->sub )
		{
			/* Create and send the DDMP frame to subscribe for the ddmp paremeter IAQ index to broker */
            TRUE_CHECK(connector_send_frame_to_connector(
                DDMP2_CONTROL_SUBSCRIBE,
                ptr_param_db->ddm_parameter,
                NULL,
                0,
                connector_pwr_ctrl_service.connector_id,
                (TickType_t)portMAX_DELAY))
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

    return sorted_list_single_add(&conn_pwrctrl_table, key, value);
}

/**
  * @brief  Task for connector power control service for processing the DDMP request received from broker
  * @param  pvParameter.
  * @retval none.
  */
static void conn_pwr_ctrl_process_task(void *pvParameter)
{
	DDMP2_FRAME *pframe;
	size_t frame_size;

	while (1)
	{
		TRUE_CHECK (pframe = xRingbufferReceive(connector_pwr_ctrl_service.to_connector, &frame_size, portMAX_DELAY));

		switch (pframe->frame.control)
		{
		case DDMP2_CONTROL_PUBLISH:
#if CONN_PWR_DEBUG_LOG
		    LOG(I, "Received DDMP2_CONTROL_PUBLISH");
#endif  
            process_set_and_publish_request(pframe->frame.publish.parameter, pframe->frame.publish.value.int32, pframe->frame.control);
			break;

		case DDMP2_CONTROL_SET:
#if CONN_PWR_DEBUG_LOG
		    LOG(I, "Received DDMP2_CONTROL_SET");
#endif
		    process_set_and_publish_request(pframe->frame.set.parameter, pframe->frame.set.value.int32, pframe->frame.control);
			break;

		case DDMP2_CONTROL_SUBSCRIBE:
#if CONN_PWR_DEBUG_LOG
		    LOG(I, "Received DDMP2_CONTROL_SUBSCRIBE");
#endif
			add_subscription(pframe);
			process_subscribe_request(pframe->frame.subscribe.parameter);
			break;

		default:
			LOG(E, "Fan Motor connector received UNHANDLED frame %02x from broker!",pframe->frame.control);
			break;
		}

		vRingbufferReturnItem(connector_pwr_ctrl_service.to_connector, pframe);
    }
}

/**
  * @brief  Task for connector pwr control
  *         Todo : Need to design the state machine generic way to handle for all other projects
  *         DDMP will be different for different projects
  * @param  pvParameter.
  * @retval none.           
  */
static void conn_pwr_ctrl_manager_task(void *pvParameter)
{
    osal_queue_handle_t queue_handle = (osal_queue_handle_t)pvParameter;
    PWR_CTRL_DATA pwr_ctrl_data_frame;

    while (1)
    {
        /* Queue will be in blocked state untill data recevied */
        if ( osal_success == osal_queue_receive(queue_handle, (void *)&pwr_ctrl_data_frame, portMAX_DELAY) )
        {
#if CONN_PWR_DEBUG_LOG
            LOG(I, "Data received from queue data = %d, data_id = %d", pwr_ctrl_data_frame.data, pwr_ctrl_data_frame.data_id);
#endif
            /* Parse the pwr ctrl data frame received from the queue */
            parse_pwr_ctrl_frame(&pwr_ctrl_data_frame);

            switch (pwr_ctrl_sm.inv_pwr_ctrl_state)
            {
                case IVPMGR0STATE_STANDBY:
                    /* Powered OFF state - Device functionality not available 
                       At system startup the device will enter into standby mode
                    */
#if CONN_PWR_DEBUG_LOG
                    LOG(I, "invent_pwr_mode = %d, veh_running_status = %d", pwr_ctrl_sm.invent_pwr_mode, pwr_ctrl_sm.veh_running_status);
#endif
                    if ( ( IV0PWRON_ON           == pwr_ctrl_sm.invent_pwr_mode    ) && 
                         ( VEHICLE_STATUS_HALTED == pwr_ctrl_sm.veh_running_status ) )
                    {
                        start_filter_timer();
                        /* Change the filter status */
                        pwr_ctrl_sm.filter_cur_status = FILTER_STARTED;
                        /* Change the state to ACTIVE  */
                        pwr_ctrl_sm.inv_pwr_ctrl_state = IVPMGR0STATE_ACTIVE;
                    }
                    break;

                case IVPMGR0STATE_ACTIVE:
                    if ( ( IV0PWRON_OFF           == pwr_ctrl_sm.invent_pwr_mode    ) || 
                         ( VEHICLE_STATUS_RUNNING == pwr_ctrl_sm.veh_running_status ) )
                    {
                        stop_filter_timer(&pwr_ctrl_sm);
                        /* Change the state to STANDBY */
                        pwr_ctrl_sm.inv_pwr_ctrl_state = IVPMGR0STATE_STANDBY;
                    }
                    else if ( IV0STORAGE_ACTIVATE == pwr_ctrl_sm.storage_mode_sel )
                    {
                        ////stop_filter_timer(&pwr_ctrl_sm);
                        /* Change the filter status */
                        ////pwr_ctrl_sm.filter_cur_status  = FILTER_IDLE;
                        /* Change the state to STORAGE */
                        pwr_ctrl_sm.inv_pwr_ctrl_state = IVPMGR0STATE_STORAGE;
                        TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, IVPMGR0STATE, &pwr_ctrl_sm.inv_pwr_ctrl_state, \
                            sizeof(int32_t), connector_pwr_ctrl_service.connector_id, portMAX_DELAY));
                    }
                    break;

                case IVPMGR0STATE_STORAGE:
                    if ( ( IV0PWRON_OFF           == pwr_ctrl_sm.invent_pwr_mode    ) || 
                         ( VEHICLE_STATUS_RUNNING == pwr_ctrl_sm.veh_running_status ) )
                    {
                        /* When the device is powered OFF..Then the storage mode will be inactive */
                        pwr_ctrl_sm.storage_mode_sel = IV0STORAGE_DEACTIVATE;
                        /* Publish the storage mode status to broker */
                        TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, IV0STORAGE, &pwr_ctrl_sm.storage_mode_sel, \
                            sizeof(int32_t), connector_pwr_ctrl_service.connector_id, portMAX_DELAY));
                        /* Change the state to STANDBY */
                        pwr_ctrl_sm.inv_pwr_ctrl_state = IVPMGR0STATE_STANDBY;
                    }
                    else if ( IV0STORAGE_DEACTIVATE == pwr_ctrl_sm.storage_mode_sel )
                    {
                        pwr_ctrl_sm.inv_pwr_ctrl_state = IVPMGR0STATE_ACTIVE;
                    }
                    break;

                default:
                    break;
            }

            if ( pwr_ctrl_sm.prev_set_state != pwr_ctrl_sm.inv_pwr_ctrl_state )
            {
#if CONN_PWR_DEBUG_LOG
                LOG(I, "Inventilate state changed = %d", pwr_ctrl_sm.inv_pwr_ctrl_state);
#endif
                change_ivpmgr_state(pwr_ctrl_sm.inv_pwr_ctrl_state);

                l_update_and_send_val_to_broker(IVPMGR0STATE, pwr_ctrl_sm.inv_pwr_ctrl_state);
                /* Store the current state */
                pwr_ctrl_sm.prev_set_state = pwr_ctrl_sm.inv_pwr_ctrl_state;
            }
        }
    }
}

#ifdef INVENT_BATTERY_TESTING

#if defined(DEVICE_BQ25792)  || defined(DEVICE_BQ25798) 
char* debug_arr_ch_stat[] = {
"Not Charging",
"Trickle Charge",
"Pre-charge",
"Fast Charge(CC Mode)",
"Taper Charge(CV Mode)",
"Reserved",
"Top-off timer active charging",
"Charge termination done"
};
#endif //defined(DEVICE_BQ25792)  || defined(DEVICE_BQ25798)


#ifdef DEVICE_BQ25792
/**
  * @brief  Task for battery management service
  * @param  pvParameter.
  * @retval none.
  */
static void conn_pwr_ctrl_bms_task_bq25792(void *pvParameter)
{
    TickType_t task_frequency = (TickType_t)pdMS_TO_TICKS(5000); 
    TickType_t last_wake_time = xTaskGetTickCount();
    uint16_t src_check_count = 0;

    uint8_t data_1;
    uint16_t data_2;
    uint8_t flag_process_timer      = 0;
    uint8_t bms_flag_timer = 0;
    error_type result;
    float vbat = 0.0f;
    float vsys = 0.0f;
    float f_ibus = 0.0f;
    int16_t i16_ibus = 0; 
    float f_ibat = 0.0f;
    int16_t i16_ibat = 0;
    int16_t i16_tdie = 0;
    float ichg = 0.0f;
    float iterm = 0.0f;
    float f_ts  = 0.0f;
    float f_tdie = 0.0f;
    float f_vsysmin = 0.0f;
    float f_ch_v_lim = 0.0f;

    REG00_MINIMAL_SYS_VOLTAGE  min_sys_volt_limit;
    REG01_CHARGE_VOLTAGE_LIMIT ch_volt_lim_reg;
    REG03_CHARGE_CURRENT_LIMIT ch_cur_lim_reg;
    REG05_INPUT_VOLTAGE_LIMIT  inp_volt_lim_reg;
    REG06_INPUT_CURRENT_LIMIT  inp_cur_lim_reg;
    REG08_PRECHARGE_CONTROL_REG pre_ch_ctrl_reg;
    REG08_TERMINATION_CONTROL_REG term_ctrl_reg;

    REG0E_TIMER_CONTROL_REG   tmr_ctrl_reg;
    REG0A_RECHARGE_CTRL_REG rechg_ctrl_reg;
    REG33_IBAT_ADC_REG      ibat_adc_reg;
    REG1B_CHARGER_STATUS_0_REG ch_stat_0;
    REG1D_CHARGER_STATUS_2_REG ch_stat_2;
    REG1E_CHARGER_STATUS_3_REG ch_stat_3;
    REG1F_CHARGER_STATUS_4_REG ch_stat_4;


#if CONN_PWR_DEBUG_LOG
    LOG(I,"Battery low cut off %d mV",((MINIMUM_SYSTEM_VOLTAGE_VALUE * 250 ) + 2500));
    LOG(I,"Battery low alert %f",BACKUP_BATTERY_LOW_THRESHOLD);
    LOG(I,"Battery Charge cut off %d mV",CHARGE_VOLTAGE_LIMIT_VALUE);
#endif
    while (1)
    {
#ifdef EMC_TEST_CASE        
        ivsett_config.EN_DIS_SOLAR = true; 
#endif         
        if(ivsett_config.EN_DIS_SOLAR == true)
        {
            /* Reset the battery charger IC before it get expires */
            bq25792_wd_reset();
            if ( update_active_source_flag > 0)
            {
                update_active_source_flag = 0;
                update_active_power_source();
            }
            
            display_bms_info_timer ++;
            if(display_bms_info_timer > 0)
            {
#if CONN_PWR_DEBUG_LOG
                display_bms_info_timer = 0;
                result = bq25792_read_reg(RECHARGE_CONTROL_REG0AH, &rechg_ctrl_reg.byte, 1u);
                LOG(I, "RECHARGE_CONTROL_REG0AH 0x%x", rechg_ctrl_reg.byte);
                LOG(I, "CELL   = 0x%x", rechg_ctrl_reg.CELL);
                LOG(I, "TRECHG = 0x%x", rechg_ctrl_reg.TRECHG);
                LOG(I, "VRECHG 0x%x | VRECHG = %d mV", rechg_ctrl_reg.VRECHG, ( rechg_ctrl_reg.VRECHG * 50) + VRECHG_FIXED_OFFSET_MILLIVOLT);

                result = bq25792_read_reg(TIMER_CONTROL_REG0EH, &tmr_ctrl_reg.byte, 1u);
                LOG(I, "TIMER_CONTROL_REG0EH 0x%x", tmr_ctrl_reg.byte);

                LOG(I, "TOP_OFF_TMR   0x%x", tmr_ctrl_reg.TOPOFF_TMR);
                LOG(I, "EN_TRICHG_TMR 0x%x", tmr_ctrl_reg.EN_TRICHG_TMR);
                LOG(I, "EN_PRECHG_TMR 0x%x", tmr_ctrl_reg.EN_PRECHG_TMR);
                LOG(I, "EN_CHG_TMR    0x%x", tmr_ctrl_reg.EN_CHG_TMR);
                LOG(I, "CHG_TMR       0x%x", tmr_ctrl_reg.CHG_TMR);
                LOG(I, "TMR2X_EN      0x%x", tmr_ctrl_reg.TMR2X_EN);
#endif
                result = bq25792_read_reg(TERMINATION_CONTROL_REG09H, &data_1, 1u);
                LOG(I, "TERMINATION_CONTROL_REG09H 0x%x", data_1);

                data_1 = data_1 & 0x1F;
                iterm = (float)data_1 * 40.0f;

                LOG(I, "ITERM %f mA", iterm);

                data_1 = 0;

                result = bq25792_read_reg(CHARGE_CURRENT_LIMIT_REG03H, (uint8_t*)&data_2, 2u);
                LOG(I, "CHARGE_CURRENT_LIMIT_REG03H 0x%x", data_2);
                
                data_2 = SWAP2(data_2);
                data_2 = data_2 * 10;
                ichg = (float)data_2 / 1000.0f;

                LOG(W, "ICHG = %f Amp", ichg);
                data_2 = 0;

                result = bq25792_read_reg(CHARGE_CONTROL_0_REG0FH, &data_1, 1u);
                LOG(I, "CHARGE_CTRL_REG0 0x%x", data_1);
                data_1 = 0;

                
                result = bq25792_read_reg(CHARGE_STATUS_0_REG1BH, &ch_stat_0.byte, 1u);
#if CONN_PWR_DEBUG_LOG
                LOG(I, "VBUS_PRESENT_STAT = %d", ch_stat_0.VBUS_PRESENT_STAT);
                LOG(I, "VAC1_PRESENT_STAT = %d", ch_stat_0.VAC1_PRESENT_STAT);
                LOG(I, "VAC2_PRESENT_STAT = %d", ch_stat_0.VAC2_PRESENT_STAT);
                /*Send staus to other connectors*/
                LOG(I, "PG_STAT = %d", ch_stat_0.PG_STAT);
                LOG(I, "POORSRC_STAT = %d", ch_stat_0.POORSRC_STAT);
                LOG(I, "WD_STAT = %d", ch_stat_0.WD_STAT);
                LOG(I, "VINDPM_STAT = %d", ch_stat_0.VINDPM_STAT);
                LOG(I, "IINDPM_STAT = %d", ch_stat_0.IINDPM_STAT);
#endif
                result = bq25792_read_reg(CHARGE_STATUS_1_REG1CH, &ch_stat_1.byte, 1u);
#if CONN_PWR_DEBUG_LOG
                LOG(I, "BC1_2_DONE_STAT = %d", ch_stat_1.BC1_2_DONE_STAT);
                LOG(I, "VBUS_STAT = %d", ch_stat_1.VBUS_STAT);
                LOG(I, "CHG_STAT = %d", ch_stat_1.CHG_STAT);
#endif                
                if ( ( ch_stat_1.CHG_STAT <= CHARGING_TERMINATION_DONE ) && ( result == RES_PASS ) )
                {
                    LOG(W, "Charging Status : %s", debug_arr_ch_stat[ch_stat_1.CHG_STAT]);
                }

                result = bq25792_read_reg(CHARGE_STATUS_2_REG1DH, &ch_stat_2.byte, 1u);
#if CONN_PWR_DEBUG_LOG
                LOG(I, "VBAT_PRESENT_STAT = %d", ch_stat_2.VBAT_PRESENT_STAT);
                LOG(I, "DPDM_STAT = %d", ch_stat_2.DPDM_STAT);
                LOG(I, "TREG_STAT = %d", ch_stat_2.TREG_STAT);
                LOG(I, "ICO_STAT = %d", ch_stat_2.ICO_STAT);
#endif                
                result = bq25792_read_reg(CHARGE_STATUS_4_REG1FH, &ch_stat_4.byte, 1u);
#if CONN_PWR_DEBUG_LOG
                LOG(I, "TS_HOT_STAT = %d", ch_stat_4.TS_HOT_STAT);
                LOG(I, "TS_WARM_STAT = %d", ch_stat_4.TS_WARM_STAT);
                LOG(I, "TS_COOL_STAT = %d", ch_stat_4.TS_COOL_STAT);
                LOG(I, "TS_COLD_STAT = %d", ch_stat_4.TS_COLD_STAT);
                LOG(I, "VBATOTG_LOW_STAT = %d", ch_stat_4.VBATOTG_LOW_STAT);
#endif
                result = bq25792_read_reg(CHARGE_CONTROL_4_REG13H, &data_1, 1u);
                LOG(I, "CHARGE_CONTROL_4_REG13H 0x%x", data_1);

#if CONN_PWR_DEBUG_LOG
                data_1 = 0;

                result = bq25792_read_reg(MINIMUM_SYSTEM_VOLTAGE_REG0H, &min_sys_volt_limit.byte, 1u);
                f_vsysmin = ( ( MINIMUM_SYS_VOLT_REG_BIT_STEP_SIZE_MV * min_sys_volt_limit.byte ) + MINIMUM_SYS_VOLT_REG_OFFSET_MV )  / 1000.0f;
                LOG(I, "VSYSMIN = %f volt", f_vsysmin);

                result = bq25792_read_reg(CHARGE_VOLTAGE_LIMIT_REG01H, (uint8_t*)&ch_volt_lim_reg.byte2, 2u);
                f_ch_v_lim = (float)SWAP2(ch_volt_lim_reg.byte2);
                LOG(I, "VREG = %f volt", ((f_ch_v_lim * 10.0f ) / 1000.0f));
                data_2 = 0;

                result = bq25792_read_reg(CHARGE_CURRENT_LIMIT_REG03H, (uint8_t*)&ch_cur_lim_reg.byte2, 2u);
                data_2 = SWAP2(ch_cur_lim_reg.byte2);
                LOG(I, "ICHG = %d mA", (data_2 * 10));

                result = bq25792_read_reg(INPUT_VOLTAGE_LIMIT_REG05H, (uint8_t*)&inp_volt_lim_reg.byte, 1u);
                LOG(I, "VINDPM = %d mV", (uint16_t)((uint16_t)inp_volt_lim_reg.byte * (uint16_t)100));

                result = bq25792_read_reg(INPUT_CURRENT_LIMIT_REG06H, (uint8_t*)&inp_cur_lim_reg.byte2, 2u);
                data_2 = SWAP2(inp_cur_lim_reg.byte2);
                LOG(I, "IINDPM = %d mA", data_2 * 10);

                result = bq25792_read_reg(PRECHARGE_CONTROL_REG08H, (uint8_t*)&pre_ch_ctrl_reg.byte, 1u);
                LOG(I, "IPRECHG = %d mA VBAT_LOWV = 0x%x", (pre_ch_ctrl_reg.IPRECHG * 40), pre_ch_ctrl_reg.VBAT_LOWV);

                result = bq25792_read_reg(TERMINATION_CONTROL_REG09H, (uint8_t*)&term_ctrl_reg.byte, 1u);
                LOG(I, "ITERM = %d mA REG_RST = 0x%x", (term_ctrl_reg.ITERM * 40), term_ctrl_reg.REG_RST);
#endif
                result = bq25792_read_reg(CHARGE_STATUS_3_REG1EH, &ch_stat_3.byte, 1u);
#if CONN_PWR_DEBUG_LOG
                LOG(I, "PRECHG_TMR_STAT = %d", ch_stat_3.PRECHG_TMR_STAT);
                LOG(I, "TRICHG_TMR_STAT = %d", ch_stat_3.TRICHG_TMR_STAT);
                LOG(I, "CHG_TMR_STAT = %d", ch_stat_3.CHG_TMR_STAT);
                LOG(I, "VSYS_STAT = %d", ch_stat_3.VSYS_STAT);
                LOG(I, "ADC_DONE_STAT = %d", ch_stat_3.ADC_DONE_STAT);
                LOG(I, "ACRB1_STAT = %d", ch_stat_3.ACRB1_STAT);
                LOG(I, "ACRB2_STAT = %d", ch_stat_3.ACRB2_STAT);
#endif

                if ( ( ch_stat_3.ADC_DONE_STAT == 1 ) && ( result == RES_PASS ) )
                {
                    ch_stat_3.ADC_DONE_STAT = 0;

                    result = bq25792_read_reg(VBAT_ADC_REG3BH, (uint8_t*)&data_2, 2u);
                    LOG(I, "VBAT_ADC_REG3BH 0x%x", SWAP2(data_2));

                    data_2 = SWAP2(data_2);
                    vbat = (float)data_2 / 1000.0f;

                            
                    LOG(W, "VBAT = %f Volt",vbat);
                    data_2 = 0;

                    result = bq25792_read_reg(VAC1_ADC_REG37H, (uint8_t*)&data_2, 2u);
                    LOG(I, "VAC1_ADC_REG37H 0x%x", SWAP2(data_2));

                    data_2 = SWAP2(data_2);
                    vac1 = (float)data_2 / 1000.0f;

                    LOG(I, "VAC1 = %f Volt",vac1);

                    result = bq25792_read_reg(VAC2_ADC_REG39H, (uint8_t*)&data_2, 2u);
                    LOG(I, "VAC2_ADC_REG39H 0x%x", SWAP2(data_2));

                    data_2 = SWAP2(data_2);
                    vac2 = (float)data_2 / 1000.0f;

                    LOG(I, "VAC2 = %f Volt",vac2);

                    data_2 = 0;

                    result = bq25792_read_reg(VSYS_ADC_REG3DH, (uint8_t*)&data_2, 2u);
                    LOG(I, "VSYS_ADC_REG3DH 0x%x", SWAP2(data_2));

                    data_2 = SWAP2(data_2);
                    vsys = (float)data_2 / 1000.0f;

                    LOG(I, "VSYS = %f Volt",vsys);

                    data_2 = 0;

                    result = bq25792_read_reg(IBUS_ADC_REG31H, (uint8_t*)&data_2, 2u);
                    LOG(I, "IBUS_ADC_REG31H 0x%x", SWAP2(data_2));

                    data_2 = SWAP2(data_2);
                    i16_ibus = (int16_t)data_2;
                    f_ibus = (float)i16_ibus / 1000.0f;

                    LOG(W, "IBUS = %f Amp",f_ibus);

                    data_2 = 0;
                    
                    result = bq25792_read_reg(IBAT_ADC_REG33H, (uint8_t*)&data_2, 2u);
                    LOG(I, "IBAT_ADC_REG33H 0x%x", SWAP2(data_2));

                    data_2 = SWAP2(data_2);
                    i16_ibat = (int16_t)data_2;
                    f_ibat = (float)i16_ibat / 1000.0f;

                    LOG(W, "IBAT = %f Amp", f_ibat);

                    data_2 = 0;

                    result = bq25792_read_reg(IBAT_ADC_REG33H, (uint8_t*)&ibat_adc_reg.byte, 2u);
                    LOG(I, "IBAT_ADC_REG33H 0x%x", ibat_adc_reg.ibat_adc);

                    data_2 = 0;

                    result = bq25792_read_reg(TS_ADC_REG3FH, (uint8_t*)&data_2, 2u);
                    LOG(I, "TS_ADC_REG3FH 0x%x", SWAP2(data_2));
                    data_2 = SWAP2(data_2);
                    f_ts = (float)data_2 * TS_BIT_STEP_SIZE;

                    LOG(W, "TS = %f percent", f_ts);

                    data_2 = 0;

                    result = bq25792_read_reg(TDIE_ADC_REG41H, (uint8_t*)&data_2, 2u);
                    LOG(I, "TDIE_ADC_REG41H 0x%x", SWAP2(data_2));

                    data_2 = SWAP2(data_2);
                    i16_tdie = (int16_t)data_2;
                    f_tdie = (float)i16_tdie * TDIE_BIT_STEP_SIZE;

                    LOG(W, "TDIE = %f degC", f_tdie);
                    result = bq25792_start_adc_conversion();

                    LOG(I, "result = %d", result);
                    
                }
#if (EN_POOR_SOURCE_CHECK == 1)

                if ( ch_stat_0.POORSRC_STAT == TRUE) 
                {
                    src_check_count++;
                }
#if CONN_PWR_DEBUG_LOG
                LOG(I,"[src_cnt %d flg %d count = %d]",src_check_count, ch_stat_0.POORSRC_STAT, poor_source_count);
#endif
                if(src_check_count > POOR_SOURCE_TIMER)
                {
                    if(poor_source_count <  POOR_SOURCE_MAX_COUNT)
                    {
                        poor_source_count++;
                    }
                    src_check_count = 0;
                    if( ((ch_stat_0.POORSRC_STAT == 1) && (vbat < BACKUP_BATTERY_LOW_LIMIT) ) || (poor_source_count > = POOR_SOURCE_COUNT_LIMIT ) )
                    {
                        
                        poor_source_flag = TRUE;
#if CONN_PWR_DEBUG_LOG
                        LOG(W, "[poor_source_found]");
#endif
                    }
                    else
                    {
                    poor_source_flag = FALSE; 
                    }
                    
                    ch_stat_0.POORSRC_STAT = 0;
                    result = bq25792_read_reg(CHARGE_CONTROL_0_REG0FH, &data_1, 1u);
                    data_1 = data_1 & (~ FLAG_EN_HIZ);
                    result = bq25792_write_reg(CHARGE_CONTROL_0_REG0FH, &data_1, ONE_BYTE);

                    result = bq25792_read_reg(CHARGE_CONTROL_4_REG13H, &data_1, 1u);
#if CONN_PWR_DEBUG_LOG
                    LOG(I, "[Reset EN_HIZ Rst_CHARGE_CONTROL_4_REG13H 0x%x]", data_1);
#endif
                }
#endif

            if(ch_stat_4.TS_HOT_STAT == true)
            {
                //battery overheating
                pwr_ctrl_error_code(PWR_BAT_OVER_HEATING);
            }
            else if(ch_stat_4.TS_COOL_STAT == true)
            {
                pwr_ctrl_error_code(PWR_BAT_COOL);
            }
            
            if ( (vbat < BACKUP_BATTERY_LOW_THRESHOLD) && (inv_bkup_bat_sts != INV_BATTERY_LOW) )
            {
                update_and_send_value_to_broker(IV0MODE, IV0MODE_OFF);
                update_and_send_value_to_broker(IV0PWRON, IV0PWRON_OFF);
                update_and_send_value_to_broker(DIM0LVL, 20);
#if CONN_PWR_DEBUG_LOG
                    LOG(I,"Battery Low! mode  changed  to stand-by");
#endif
                inv_bkup_bat_sts = INV_BATTERY_LOW;
                pwr_ctrl_error_code(PWR_BACKUP_BAT_LOW);
            }
            else if ( ( vbat > BACKUP_BATTERY_HIGH_THRESHOLD) && (inv_bkup_bat_sts != INV_BATTERY_GOOD) ) 
            {
                inv_bkup_bat_sts = INV_BATTERY_GOOD;
                pwr_ctrl_error_code(PWR_BACKUP_BAT_NORMAL);
            }
                    //!< battery Charging current monitor
            if ( ( result == RES_PASS ) && ( data_2 >= 13700 ) && ( ch_stat_1.CHG_STAT == CHARGING_TERMINATION_DONE ) && \
                (BATTERY_STATUS.charging_current_exceed != true) )
            {
                /*Battery fully charged reguce charging current to avoid BMS IC heating*/
                result = bq25792_set_charging_current_limit(BQ25792_BATTERY_LIMITED_CURRENT); 
                BATTERY_STATUS.charging_current_exceed = true;
            }
            else if ( ( result == RES_PASS ) && ( ch_stat_1.CHG_STAT < CHARGING_TERMINATION_DONE ) && (BATTERY_STATUS.charging_current_exceed != false) )
            {
                result = bq25792_set_charging_current_limit(CHARGE_CURRENT_LIMIT_VALUE); 
                BATTERY_STATUS.charging_current_exceed = false;
            }       
            }
        } //Solar_enable
        vTaskDelayUntil(&last_wake_time, task_frequency);
    } //While loop
}
#endif // DEVICE_BQ25792


/*BMS control task for BQ25798*/
#ifdef DEVICE_BQ25798
/**
  * @brief  Task for battery management service
  * @param  pvParameter.
  * @retval none.
  */
static void conn_pwr_ctrl_bms_task_bq25798(void *pvParameter)
{
    TickType_t task_frequency = (TickType_t)pdMS_TO_TICKS(5000); 
    TickType_t last_wake_time = xTaskGetTickCount();
    uint16_t src_check_count = 0;

    uint8_t data_1;
    uint16_t data_2;
    uint8_t flag_process_timer      = 0;
    uint8_t bms_flag_timer = 0;
    error_type result;
    float vbat = 0.0f;
    float vsys = 0.0f;
    float f_ibus = 0.0f;
    int16_t i16_ibus = 0; 
    float f_ibat = 0.0f;
    int16_t i16_ibat = 0;
    int16_t i16_tdie = 0;
    float ichg = 0.0f;
    float iterm = 0.0f;
    float f_ts  = 0.0f;
    float f_tdie = 0.0f;
    float f_vsysmin = 0.0f;
    float f_ch_v_lim = 0.0f;

    REG00_MINIMAL_SYS_VOLTAGE  min_sys_volt_limit;
    REG01_CHARGE_VOLTAGE_LIMIT ch_volt_lim_reg;
    REG03_CHARGE_CURRENT_LIMIT ch_cur_lim_reg;
    REG05_INPUT_VOLTAGE_LIMIT  inp_volt_lim_reg;
    REG06_INPUT_CURRENT_LIMIT  inp_cur_lim_reg;
    REG08_PRECHARGE_CONTROL_REG pre_ch_ctrl_reg;
    REG08_TERMINATION_CONTROL_REG term_ctrl_reg;

    REG0E_TIMER_CONTROL_REG   tmr_ctrl_reg;
    REG0A_RECHARGE_CTRL_REG rechg_ctrl_reg;
    REG33_IBAT_ADC_REG      ibat_adc_reg;
    REG1B_CHARGER_STATUS_0_REG ch_stat_0;
    //REG1C_CHARGER_STATUS_1_REG ch_stat_1;
    REG1D_CHARGER_STATUS_2_REG ch_stat_2;
    REG1E_CHARGER_STATUS_3_REG ch_stat_3;
    REG1F_CHARGER_STATUS_4_REG ch_stat_4;

#if CONN_PWR_DEBUG_LOG
    LOG(I,"Battery low cut off %d mV",((MINIMUM_SYSTEM_VOLTAGE_VALUE * 250 ) + 2500));
    LOG(I,"Battery low alert %f",BACKUP_BATTERY_LOW_THRESHOLD);
    LOG(I,"Battery Charge cut off %d mV",CHARGE_VOLTAGE_LIMIT_VALUE);
#endif
    while (1)
    {
#ifdef EMC_TEST_CASE        
        ivsett_config.EN_DIS_SOLAR = true; 
#endif        
        if(ivsett_config.EN_DIS_SOLAR == true)
        {
           /* Reset the battery charger IC before it get expires */
            bq25798_wd_reset();
            if ( update_active_source_flag > 0)
            {
                update_active_source_flag = 0;
                update_active_power_source();
            }
            
            display_bms_info_timer ++;
            if(display_bms_info_timer > 0)
            {

                display_bms_info_timer = 0;
                result = bq25798_read_reg(RECHARGE_CONTROL_REG0AH, &rechg_ctrl_reg.byte, 1u);
#if CONN_PWR_DEBUG_LOG
                LOG(I, "RECHARGE_CONTROL_REG0AH 0x%x", rechg_ctrl_reg.byte);
                LOG(I, "CELL   = 0x%x", rechg_ctrl_reg.CELL);
                LOG(I, "TRECHG = 0x%x", rechg_ctrl_reg.TRECHG);
                LOG(I, "VRECHG 0x%x | VRECHG = %d mV", rechg_ctrl_reg.VRECHG, ( rechg_ctrl_reg.VRECHG * 50) + VRECHG_FIXED_OFFSET_MILLIVOLT);
#endif
                result = bq25798_read_reg(TIMER_CONTROL_REG0EH, &tmr_ctrl_reg.byte, 1u);
#if CONN_PWR_DEBUG_LOG                
                LOG(I, "TIMER_CONTROL_REG0EH 0x%x", tmr_ctrl_reg.byte);

                LOG(I, "TOP_OFF_TMR   0x%x", tmr_ctrl_reg.TOPOFF_TMR);
                LOG(I, "EN_TRICHG_TMR 0x%x", tmr_ctrl_reg.EN_TRICHG_TMR);
                LOG(I, "EN_PRECHG_TMR 0x%x", tmr_ctrl_reg.EN_PRECHG_TMR);
                LOG(I, "EN_CHG_TMR    0x%x", tmr_ctrl_reg.EN_CHG_TMR);
                LOG(I, "CHG_TMR       0x%x", tmr_ctrl_reg.CHG_TMR);
                LOG(I, "TMR2X_EN      0x%x", tmr_ctrl_reg.TMR2X_EN);
#endif
                result = bq25798_read_reg(TERMINATION_CONTROL_REG09H, &data_1, 1u);
#if CONN_PWR_DEBUG_LOG
                LOG(I, "TERMINATION_CONTROL_REG09H 0x%x", data_1);
#endif
                data_1 = data_1 & 0x1F;
                iterm = (float)data_1 * 40.0f;
#if CONN_PWR_DEBUG_LOG
                LOG(I, "ITERM %f mA", iterm);
#endif
                data_1 = 0;

                result = bq25798_read_reg(CHARGE_CURRENT_LIMIT_REG03H, (uint8_t*)&data_2, 2u);
#if CONN_PWR_DEBUG_LOG
                LOG(I, "CHARGE_CURRENT_LIMIT_REG03H 0x%x", data_2);
#endif                
                data_2 = SWAP2(data_2);
                data_2 = data_2 * 10;
                ichg = (float)data_2 / 1000.0f;
#if CONN_PWR_DEBUG_LOG
                LOG(W, "ICHG = %f Amp", ichg);
#endif                
                data_2 = 0;

                result = bq25798_read_reg(CHARGE_CONTROL_0_REG0FH, &data_1, 1u);
#if CONN_PWR_DEBUG_LOG
                LOG(I, "CHARGE_CTRL_REG0 0x%x", data_1);
#endif                
                data_1 = 0;

                
                result = bq25798_read_reg(CHARGE_STATUS_0_REG1BH, &ch_stat_0.byte, 1u);
#if CONN_PWR_DEBUG_LOG
                LOG(I, "VBUS_PRESENT_STAT = %d", ch_stat_0.VBUS_PRESENT_STAT);
                LOG(I, "VAC1_PRESENT_STAT = %d", ch_stat_0.VAC1_PRESENT_STAT);
                LOG(I, "VAC2_PRESENT_STAT = %d", ch_stat_0.VAC2_PRESENT_STAT);
                /*Send staus to other connectors*/

                LOG(I, "PG_STAT = %d", ch_stat_0.PG_STAT);
                LOG(I, "POORSRC_STAT = %d", ch_stat_0.POORSRC_STAT);
                LOG(I, "WD_STAT = %d", ch_stat_0.WD_STAT);
                LOG(I, "VINDPM_STAT = %d", ch_stat_0.VINDPM_STAT);
                LOG(I, "IINDPM_STAT = %d", ch_stat_0.IINDPM_STAT);
#endif
                result = bq25798_read_reg(CHARGE_STATUS_1_REG1CH, &ch_stat_1.byte, 1u);
#if CONN_PWR_DEBUG_LOG
                LOG(I, "BC1_2_DONE_STAT = %d", ch_stat_1.BC1_2_DONE_STAT);
                LOG(I, "VBUS_STAT = %d", ch_stat_1.VBUS_STAT);
                LOG(I, "CHG_STAT = %d", ch_stat_1.CHG_STAT);
#endif                
                if ( ( ch_stat_1.CHG_STAT <= CHARGING_TERMINATION_DONE ) && ( result == RES_PASS ) )
                {
                    LOG(W, "Charging Status : %s", debug_arr_ch_stat[ch_stat_1.CHG_STAT]);
                }

                result = bq25798_read_reg(CHARGE_STATUS_2_REG1DH, &ch_stat_2.byte, 1u);
#if CONN_PWR_DEBUG_LOG
                LOG(I, "VBAT_PRESENT_STAT = %d", ch_stat_2.VBAT_PRESENT_STAT);
                LOG(I, "DPDM_STAT = %d", ch_stat_2.DPDM_STAT);
                LOG(I, "TREG_STAT = %d", ch_stat_2.TREG_STAT);
                LOG(I, "ICO_STAT = %d", ch_stat_2.ICO_STAT);
#endif                
                result = bq25798_read_reg(CHARGE_STATUS_4_REG1FH, &ch_stat_4.byte, 1u);
#if CONN_PWR_DEBUG_LOG
                LOG(I, "TS_HOT_STAT = %d", ch_stat_4.TS_HOT_STAT);
                LOG(I, "TS_WARM_STAT = %d", ch_stat_4.TS_WARM_STAT);
                LOG(I, "TS_COOL_STAT = %d", ch_stat_4.TS_COOL_STAT);
                LOG(I, "TS_COLD_STAT = %d", ch_stat_4.TS_COLD_STAT);
                LOG(I, "VBATOTG_LOW_STAT = %d", ch_stat_4.VBATOTG_LOW_STAT);
#endif
                result = bq25798_read_reg(CHARGE_CONTROL_4_REG13H, &data_1, 1u);
#if CONN_PWR_DEBUG_LOG
                LOG(I, "CHARGE_CONTROL_4_REG13H 0x%x", data_1);
#endif
                data_1 = 0;

                result = bq25798_read_reg(MINIMUM_SYSTEM_VOLTAGE_REG0H, &min_sys_volt_limit.byte, 1u);
                f_vsysmin = ( ( MINIMUM_SYS_VOLT_REG_BIT_STEP_SIZE_MV * min_sys_volt_limit.byte ) + MINIMUM_SYS_VOLT_REG_OFFSET_MV )  / 1000.0f;
#if CONN_PWR_DEBUG_LOG                
                LOG(I, "VSYSMIN = %f volt", f_vsysmin);
#endif
                result = bq25798_read_reg(CHARGE_VOLTAGE_LIMIT_REG01H, (uint8_t*)&ch_volt_lim_reg.byte2, 2u);
                f_ch_v_lim = (float)SWAP2(ch_volt_lim_reg.byte2);
#if CONN_PWR_DEBUG_LOG
                LOG(I, "VREG = %f volt", ((f_ch_v_lim * 10.0f ) / 1000.0f));
#endif                
                data_2 = 0;

                result = bq25798_read_reg(CHARGE_CURRENT_LIMIT_REG03H, (uint8_t*)&ch_cur_lim_reg.byte2, 2u);
                data_2 = SWAP2(ch_cur_lim_reg.byte2);
#if CONN_PWR_DEBUG_LOG
                LOG(I, "ICHG = %d mA", (data_2 * 10));
#endif
                result = bq25798_read_reg(INPUT_VOLTAGE_LIMIT_REG05H, (uint8_t*)&inp_volt_lim_reg.byte, 1u);
#if CONN_PWR_DEBUG_LOG
                LOG(I, "VINDPM = %d mV", (uint16_t)((uint16_t)inp_volt_lim_reg.byte * (uint16_t)100));
#endif
                result = bq25798_read_reg(INPUT_CURRENT_LIMIT_REG06H, (uint8_t*)&inp_cur_lim_reg.byte2, 2u);
                data_2 = SWAP2(inp_cur_lim_reg.byte2);
#if CONN_PWR_DEBUG_LOG
                LOG(I, "IINDPM = %d mA", data_2 * 10);
#endif
                result = bq25798_read_reg(PRECHARGE_CONTROL_REG08H, (uint8_t*)&pre_ch_ctrl_reg.byte, 1u);
#if CONN_PWR_DEBUG_LOG
                LOG(I, "IPRECHG = %d mA VBAT_LOWV = 0x%x", (pre_ch_ctrl_reg.IPRECHG * 40), pre_ch_ctrl_reg.VBAT_LOWV);
#endif
                result = bq25798_read_reg(TERMINATION_CONTROL_REG09H, (uint8_t*)&term_ctrl_reg.byte, 1u);
#if CONN_PWR_DEBUG_LOG
                LOG(I, "ITERM = %d mA REG_RST = 0x%x", (term_ctrl_reg.ITERM * 40), term_ctrl_reg.REG_RST);
#endif
                result = bq25798_read_reg(CHARGE_STATUS_3_REG1EH, &ch_stat_3.byte, 1u);
#if CONN_PWR_DEBUG_LOG
                LOG(I, "PRECHG_TMR_STAT = %d", ch_stat_3.PRECHG_TMR_STAT);
                LOG(I, "TRICHG_TMR_STAT = %d", ch_stat_3.TRICHG_TMR_STAT);
                LOG(I, "CHG_TMR_STAT = %d", ch_stat_3.CHG_TMR_STAT);
                LOG(I, "VSYS_STAT = %d", ch_stat_3.VSYS_STAT);
                LOG(I, "ADC_DONE_STAT = %d", ch_stat_3.ADC_DONE_STAT);
                LOG(I, "ACRB1_STAT = %d", ch_stat_3.ACRB1_STAT);
                LOG(I, "ACRB2_STAT = %d", ch_stat_3.ACRB2_STAT);
#endif
                if ( ( ch_stat_3.ADC_DONE_STAT == 1 ) && ( result == RES_PASS ) )
                {
                    ch_stat_3.ADC_DONE_STAT = 0;

                    result = bq25798_read_reg(VBAT_ADC_REG3BH, (uint8_t*)&data_2, 2u);
                    LOG(I, "VBAT_ADC_REG3BH 0x%x", SWAP2(data_2));

                    data_2 = SWAP2(data_2);
                    vbat = (float)data_2 / 1000.0f;
                    LOG(W, "VBAT = %f Volt",vbat);
                   
                    data_2 = 0;

                    result = bq25798_read_reg(VAC1_ADC_REG37H, (uint8_t*)&data_2, 2u);
                    LOG(I, "VAC1_ADC_REG37H 0x%x", SWAP2(data_2));

                    data_2 = SWAP2(data_2);
                    vac1 = (float)data_2 / 1000.0f;
                    LOG(I, "VAC1 = %f Volt",vac1);

                    result = bq25798_read_reg(VAC2_ADC_REG39H, (uint8_t*)&data_2, 2u);
                    LOG(I, "VAC2_ADC_REG39H 0x%x", SWAP2(data_2));

                    data_2 = SWAP2(data_2);
                    vac2 = (float)data_2 / 1000.0f;
                    LOG(I, "VAC2 = %f Volt",vac2);

                    data_2 = 0;

                    result = bq25798_read_reg(VSYS_ADC_REG3DH, (uint8_t*)&data_2, 2u);
                    LOG(I, "VSYS_ADC_REG3DH 0x%x", SWAP2(data_2));

                    data_2 = SWAP2(data_2);
                    vsys = (float)data_2 / 1000.0f;
                    LOG(I, "VSYS = %f Volt",vsys);

                    data_2 = 0;

                    result = bq25798_read_reg(IBUS_ADC_REG31H, (uint8_t*)&data_2, 2u);
                    LOG(I, "IBUS_ADC_REG31H 0x%x", SWAP2(data_2));

                    data_2 = SWAP2(data_2);
                    i16_ibus = (int16_t)data_2;
                    f_ibus = (float)i16_ibus / 1000.0f;

                    LOG(W, "IBUS = %f Amp",f_ibus);

                    data_2 = 0;
                    
                    result = bq25798_read_reg(IBAT_ADC_REG33H, (uint8_t*)&data_2, 2u);
                    LOG(I, "IBAT_ADC_REG33H 0x%x", SWAP2(data_2));

                    data_2 = SWAP2(data_2);
                    i16_ibat = (int16_t)data_2;
                    f_ibat = (float)i16_ibat / 1000.0f;

                    LOG(W, "IBAT = %f Amp", f_ibat);

                    data_2 = 0;

                    result = bq25798_read_reg(IBAT_ADC_REG33H, (uint8_t*)&ibat_adc_reg.byte, 2u);
                    LOG(I, "IBAT_ADC_REG33H 0x%x", ibat_adc_reg.ibat_adc);

                    data_2 = 0;

                    result = bq25798_read_reg(TS_ADC_REG3FH, (uint8_t*)&data_2, 2u);
                    LOG(I, "TS_ADC_REG3FH 0x%x", SWAP2(data_2));
                    data_2 = SWAP2(data_2);
                    f_ts = (float)data_2 * TS_BIT_STEP_SIZE;

                    LOG(W, "TS = %f percent", f_ts);

                    data_2 = 0;

                    result = bq25798_read_reg(TDIE_ADC_REG41H, (uint8_t*)&data_2, 2u);
                    LOG(I, "TDIE_ADC_REG41H 0x%x", SWAP2(data_2));

                    data_2 = SWAP2(data_2);
                    i16_tdie = (int16_t)data_2;
                    f_tdie = (float)i16_tdie * TDIE_BIT_STEP_SIZE;

                    LOG(W, "TDIE = %f degC", f_tdie);
                    result = bq25798_start_adc_conversion();

                    LOG(I, "result = %d", result);                   
                }
            

#if (EN_POOR_SOURCE_CHECK == 1)

                if ( ch_stat_0.POORSRC_STAT == TRUE) 
                {
                    src_check_count++;
                }
#if CONN_PWR_DEBUG_LOG
                LOG(I,"[src_cnt %d flg %d count = %d]",src_check_count, ch_stat_0.POORSRC_STAT, poor_source_count);
#endif
                if(src_check_count > POOR_SOURCE_TIMER)
                {
                    if(poor_source_count <  POOR_SOURCE_MAX_COUNT)
                    {
                        poor_source_count++;
                    }
                    src_check_count = 0;
                    if( ((ch_stat_0.POORSRC_STAT == 1) && (vbat < BACKUP_BATTERY_LOW_LIMIT) ) || (poor_source_count > = POOR_SOURCE_COUNT_LIMIT ) )
                    {
                        
                        poor_source_flag = TRUE;
#if CONN_PWR_DEBUG_LOG
                        LOG(W, "[poor_source_found]");
#endif
                    }
                    else
                    {
                    poor_source_flag = FALSE; 
                    }
                    
                    ch_stat_0.POORSRC_STAT = 0;
                    result = bq25798_read_reg(CHARGE_CONTROL_0_REG0FH, &data_1, 1u);
                    data_1 = data_1 & (~ FLAG_EN_HIZ);
                    result = bq25798_write_reg(CHARGE_CONTROL_0_REG0FH, &data_1, ONE_BYTE);
                    result = bq25798_read_reg(CHARGE_CONTROL_4_REG13H, &data_1, 1u);
                }
#endif

                if(ch_stat_4.TS_HOT_STAT == true)
                {
                    //battery overheating
                    pwr_ctrl_error_code(PWR_BAT_OVER_HEATING);
                }
                else if(ch_stat_4.TS_COOL_STAT == true)
                {
                    pwr_ctrl_error_code(PWR_BAT_COOL);
                }
                
                if ( (vbat < BACKUP_BATTERY_LOW_THRESHOLD) && (inv_bkup_bat_sts != INV_BATTERY_LOW) )
                {
                    update_and_send_value_to_broker(IV0MODE, IV0MODE_OFF);
                    update_and_send_value_to_broker(IV0PWRON, IV0PWRON_OFF);
                    update_and_send_value_to_broker(DIM0LVL, 20);
#if CONN_PWR_DEBUG_LOG
                        LOG(I,"Battery Low! mode  changed  to stand-by");
#endif
                    inv_bkup_bat_sts = INV_BATTERY_LOW;
                    pwr_ctrl_error_code(PWR_BACKUP_BAT_LOW);
                }
                else if ( ( vbat > BACKUP_BATTERY_HIGH_THRESHOLD) && (inv_bkup_bat_sts != INV_BATTERY_GOOD) ) 
                {
                    inv_bkup_bat_sts = INV_BATTERY_GOOD;
                    pwr_ctrl_error_code(PWR_BACKUP_BAT_NORMAL);
                }
                        //!< battery Charging current monitor
                if ( ( result == RES_PASS ) && ( data_2 >= 13700 ) && ( ch_stat_1.CHG_STAT == CHARGING_TERMINATION_DONE ) && \
                    (BATTERY_STATUS.charging_current_exceed != true) )
                {
                    /*Battery fully charged reguce charging current to avoid BMS IC heating*/
                    result = bq25798_set_charging_current_limit(BQ25798_BATTERY_LIMITED_CURRENT); 
                    BATTERY_STATUS.charging_current_exceed = true;
                }
                else if ( ( result == RES_PASS ) && ( ch_stat_1.CHG_STAT < CHARGING_TERMINATION_DONE ) && (BATTERY_STATUS.charging_current_exceed != false) )
                {
                    result = bq25798_set_charging_current_limit(CHARGE_CURRENT_LIMIT_VALUE); 
                    BATTERY_STATUS.charging_current_exceed = false;
                } 
            }   //Display_info
        } //Solar_enable
        vTaskDelayUntil(&last_wake_time, task_frequency);
    } //While loop
}
#endif // DEVICE_BQ25798

float bq_read_vbat(void)
{
    error_type result;
    float vbat_volt = 0.0f;
    uint16_t data_2;
#ifdef DEVICE_BQ25792    
    result = bq25792_read_reg(VBAT_ADC_REG3BH, (uint8_t*)&data_2, 2u);
#endif
#ifdef DEVICE_BQ25798    
    result = bq25798_read_reg(VBAT_ADC_REG3BH, (uint8_t*)&data_2, 2u);
#endif

    LOG(I, "VBAT_ADC_REG3BH 0x%x", SWAP2(data_2));

    data_2 = SWAP2(data_2);
    vbat_volt = (float)data_2 / 1000.0f;
    return vbat_volt;
}
#endif  //INVENT_BATTERY_TESTING

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
	conn_pwr_ctrl_parameter_t* param_db;

#if CONN_PWR_DEBUG_LOG
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
                        connector_pwr_ctrl_service.connector_id, portMAX_DELAY));
        }

		param_db = &conn_pwr_ctrl_param_db[db_idx];

#if CONN_PWR_DEBUG_LOG		
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
                
#if CONN_PWR_DEBUG_LOG
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
	conn_pwr_ctrl_parameter_t* param_db;
    uint32_t list_value = 0;
    SORTED_LIST_RETURN_VALUE ret = sorted_list_unique_get(&list_value, &conn_pwrctrl_table, ddm_param, 0);

#if CONN_PWR_DEBUG_LOG
    LOG(I, "Received ddm_param = 0x%x ret = %d", ddm_param, ret);
#endif

    if ( SORTED_LIST_FAIL != ret )
	{
		/* Validate the DDM parameter received */
		db_idx = get_ddm_index_from_db(ddm_param);

		if ( DDMP_UNAVAILABLE != db_idx )
	  	{
			param_db = &conn_pwr_ctrl_param_db[db_idx];

            index = ddm2_parameter_list_lookup(DDM2_PARAMETER_BASE_INSTANCE(ddm_param));

#if CONN_PWR_DEBUG_LOG
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
#if CONN_PWR_DEBUG_LOG
                LOG(I, "After factored i32value = %d", value);
#endif
                /* Frame and send the publish request */
                TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, ddm_param, &value, sizeof(int32_t), \
                            connector_pwr_ctrl_service.connector_id, portMAX_DELAY));
            }
            else
            {
                LOG(I, "DDMP 0x%x not found in ddm2_parameter_list_lookup", ddm_param);
            }
		}
        else
        {
            LOG(I, "Invalid DDMP Request ddm_param 0x%x", ddm_param);
        }
	}
	else
	{
		LOG(I, "SORTLIST_INVALID_VALUE ddm_param 0x%x", ddm_param);
	}
}

/**
  * @brief  Update the new value in database table and publish to broker
  * @param  ddm_parameter.
  * @param  i32Value.
  * @retval none.
  */
static void l_update_and_send_val_to_broker(uint32_t ddm_parameter, int32_t value)
{
	conn_pwr_ctrl_parameter_t* param_db;
    uint8_t db_idx = get_ddm_index_from_db(ddm_parameter);
	int index;
    int32_t factor_value = 0;
    int factor;

#if CONN_PWR_DEBUG_LOG
    LOG(I, "ddm_parameter = 0x%x value = %d", ddm_parameter, value);
#endif

	if ( DDMP_UNAVAILABLE != db_idx )
   	{
		param_db = &conn_pwr_ctrl_param_db[db_idx];

		/* Update the value in db table */
		param_db->i32Value = value;
        
        index = ddm2_parameter_list_lookup(DDM2_PARAMETER_BASE_INSTANCE(ddm_parameter));

#if CONN_PWR_DEBUG_LOG
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
            factor_value = param_db->i32Value * factor;
            /* Frame and send the publish request */
            TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_PUBLISH, ddm_parameter, &factor_value, \
                       sizeof(int32_t), connector_pwr_ctrl_service.connector_id, portMAX_DELAY));
        }
        else
        {
            LOG(E, "DDMP 0x%x not found in ddm2_parameter_list_lookup", ddm_parameter);
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
	conn_pwr_ctrl_parameter_t* param_db;
	uint8_t db_idx = DDMP_UNAVAILABLE; 
	uint8_t index;

	for (index = 0u;  index < conn_pwrctrl_db_elements; index ++)
 	{
		param_db = &conn_pwr_ctrl_param_db[index];
      	
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
  * @brief  Initialize the module / hardware peripheral which control's the power 
  * @param  none.
  * @retval none.
  */
static error_type initialize_pwr_control_module(void)
{
    error_type res = RES_FAIL;

#ifdef DEVICE_BQ25792
   
    drv_bus_conf bq25792_bus_conf;

    bq25792_bus_conf.type = SEQ_BUS_CONF_TYPE_I2C;

    bq25792_bus_conf.i2c.port    = I2C_MASTER0_PORT;
    bq25792_bus_conf.i2c.sda     = I2C_MASTER0_SDA;
    bq25792_bus_conf.i2c.scl     = I2C_MASTER0_SCL;
    bq25792_bus_conf.i2c.bitrate = I2C_MASTER0_FREQ;

    res = bq25792_init(&bq25792_bus_conf);

    if ( RES_PASS != res )
    {
        LOG(E, "bq25792_init failed = %d", res);
    }
#endif

#ifdef DEVICE_BQ25798
    drv_bus_conf bq25798_bus_conf;

    bq25798_bus_conf.type = SEQ_BUS_CONF_TYPE_I2C;

    bq25798_bus_conf.i2c.port    = I2C_MASTER0_PORT;
    bq25798_bus_conf.i2c.sda     = I2C_MASTER0_SDA;
    bq25798_bus_conf.i2c.scl     = I2C_MASTER0_SCL;
    bq25798_bus_conf.i2c.bitrate = I2C_MASTER0_FREQ;

    res = bq25798_init(&bq25798_bus_conf);

    if ( RES_PASS != res )
    {
        LOG(E, "bq25798_init failed = %d", res);
    }

#endif

    return res;
}

/**
  * @brief  Function to init power control state machine structure parameters
  * @param  Pointer to the structure variable PWR_CTRL_SM.
  * @retval none.
  */
static void init_pwr_ctrl_sm(PWR_CTRL_SM* ptr_ctrl_sm)
{
    ptr_ctrl_sm->filter_cur_status  = FILTER_IDLE;
	ptr_ctrl_sm->inv_pwr_ctrl_state = IVPMGR0STATE_STANDBY;
    ptr_ctrl_sm->prev_set_state     = IVPMGR0STATE_STANDBY;
    ptr_ctrl_sm->storage_mode_sel   = IV0STORAGE_DEACTIVATE;
    ptr_ctrl_sm->veh_running_status = VEHICLE_STATUS_HALTED; /* As of now for testing purpose vechicle status set as halted */
    ptr_ctrl_sm->active_power_src   = IV0PWRSRC_BACKUP_BATTERY + 1; // Set invalid/unknown power source at initialization
}

/**
  * @brief  Function to parse the power control frame packet received through queue
  * @param  Pointer to the structure variable PWR_CTRL_DATA.
  * @retval none.
  */
static void parse_pwr_ctrl_frame(PWR_CTRL_DATA* pwr_ctrl_data_frame)
{
    error_type result;

    switch (pwr_ctrl_data_frame->data_id)
    {
        case INVENT_POWER_STATUS:
            pwr_ctrl_sm.invent_pwr_mode = pwr_ctrl_data_frame->data;
            break;

        case INVENT_PWR_CTRL_STATE:
            pwr_ctrl_sm.inv_pwr_ctrl_state = pwr_ctrl_data_frame->data;
            break;

        case INVENT_VEHICLE_RUN_STATUS:
            pwr_ctrl_sm.veh_running_status = pwr_ctrl_data_frame->data;
            break;

        case INVENT_FILTER_RESET_REQ:
            LOG(W, "Filter Req = %d", pwr_ctrl_data_frame->data);

            //if ( ( pwr_ctrl_data_frame->data     == IV0FILST_FILTER_RESET ) &&
            //     ( pwr_ctrl_sm.filter_cur_status == FILTER_TIME_EXPIRED   ) )
            
            if ( ( pwr_ctrl_data_frame->data     == IV0FILST_FILTER_RESET ) )
            {
                int32_t filt_stat = IV0FILST_FILTER_CHANGE_NOT_REQ;
                /* Update the filter change not requestd state */
                TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, IV0FILST, &filt_stat, sizeof(int32_t), \
                           connector_pwr_ctrl_service.connector_id, portMAX_DELAY));
                /* Reset the filter counter */
                pwr_ctrl_sm.filter_min_counter = 0;
                pwr_ctrl_sm.filter_sec_counter = 0;
                /* Start the filter */
                start_filter_timer();
                /* Change the filter status */
                pwr_ctrl_sm.filter_cur_status = FILTER_STARTED;
            }
            else
            {
                LOG(I, "Filter timer not expired = %d Filter minutes %d filt_staus  %d ", pwr_ctrl_data_frame->data, pwr_ctrl_sm.filter_min_counter, pwr_ctrl_sm.filter_cur_status);
            }
            break;

        case INV_FIL_TMR_EXP:
            /* Increment the minute counter */
            pwr_ctrl_sm.filter_min_counter++;
            update_data_in_nvm(IV_FILTER_TIMER,pwr_ctrl_sm.filter_min_counter);
            LOG(W, "filter_min_counter = %d", pwr_ctrl_sm.filter_min_counter);
            validate_filter_time(&pwr_ctrl_sm);
            break;

        case INVENT_SET_CHARGING_CURRENT:
            LOG(I,"Charge ichg  set to %d mA",pwr_ctrl_data_frame->data);
#ifdef BQ25792          
            result = bq25792_set_charging_current_limit( pwr_ctrl_data_frame->data/10 ); 
#endif
#ifdef BQ25798            
            result = bq25798_set_charging_current_limit( pwr_ctrl_data_frame->data/10 ); 
#endif


            
            break;

        case INVENT_EN_DIS_SOLAR:
            LOG(I,"En/Disable Solar mode %d solar Mode = %d old_conf",pwr_ctrl_data_frame->data, (pwr_ctrl_data_frame->data ? 1:0));         //ToDo
            result = 0;
            ivsett_config.EN_DIS_SOLAR      =   ( ( pwr_ctrl_data_frame->data & (1 << STATUS_BIT_SOLAR) )  >> STATUS_BIT_SOLAR ) ;
            ivsett_config.EN_DIS_IONIZER    =   ( ( pwr_ctrl_data_frame->data & (1 << STATUS_BIT_IONIZER)) >> STATUS_BIT_IONIZER ) ;
            update_data_in_nvm(IV_IVSETT,pwr_ctrl_data_frame->data);
            break;

        case INVENT_STORAGE_MODE_SEL:
            pwr_ctrl_sm.storage_mode_sel = pwr_ctrl_data_frame->data;
            break;

        case INVENT_POWER_SOURCE_CHANGED:
            if ( pwr_ctrl_data_frame->data != (int32_t)pwr_ctrl_sm.active_power_src )
            {
                int32_t pwr_src = pwr_ctrl_data_frame->data;
                LOG(I, "Switched pwr_src = %d", pwr_src);
                pwr_ctrl_sm.active_power_src = pwr_ctrl_data_frame->data;
                TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, IV0PWRSRC, &pwr_src, sizeof(int32_t), \
                           connector_pwr_ctrl_service.connector_id, portMAX_DELAY));
            }
            break;

        default:
            break; 
    }
}

/**
  * @brief  Callback function to handle all the subscribed parameters.
  * @param  ddm_param - DDM Parameter with instance
  * @param  data      - Value for the DDM Parameter
  * @retval none
  */
static void handle_pwr_ctrl_sub_data(uint32_t ddm_param, int32_t data)
{
    PWR_CTRL_DATA pwr_ctrl_data_frame;

    /* set the data */
    pwr_ctrl_data_frame.data = data;

    switch (ddm_param)
    {
        case IV0PWRON:
            pwr_ctrl_data_frame.data_id = INVENT_POWER_STATUS;
            break;

        case NGN0SP:
            pwr_ctrl_data_frame.data_id = INVENT_VEHICLE_RUN_STATUS;
            break;

        case IV0FILST:
            pwr_ctrl_data_frame.data_id = INVENT_FILTER_RESET_REQ;
            break;

        case IV0STORAGE:
            pwr_ctrl_data_frame.data_id = INVENT_STORAGE_MODE_SEL;
            break;

        case IV0SETCHRGCRNT:
            pwr_ctrl_data_frame.data_id = INVENT_SET_CHARGING_CURRENT;
             break; 

        case IV0SETT:
            pwr_ctrl_data_frame.data_id = INVENT_EN_DIS_SOLAR;
            break;    

        case SNODE0AVL:
            break;

        default:
            pwr_ctrl_data_frame.data_id = INVALID_DATA;
            break;
    }

    // push the data in the Queue
    osal_base_type_t ret = osal_queue_send (inv_pwr_ctrl_que_hdle, &pwr_ctrl_data_frame, 0);

    if ( osal_success != ret )
    {
        LOG(E, "Queue error ret = %d", ret);
    }
}


/**
 * @brief power control timer callback function 
 * @param  none.
 * @retval none.
 */
static void pwr_ctrl_timer_cb_func ( TimerHandle_t xTimer )
{
    if(update_active_source_timer > 0)
    {
        update_active_source_timer++;
        if(update_active_source_timer >1)
        {
            update_active_source_timer = 0;
            update_active_source_flag = 1;
        }
    }
    display_bms_info_timer++;

}

/**
 * @brief Function to start power control timer
 * @param
 * @retval
 */
static void start_power_control_timer(void)
{
    osal_ubase_type_t xPowerControlTimerStarted = xTimerStart( xPowerControl_Timer, 0 );

    if ( xPowerControlTimerStarted == pdPASS )
    {
        LOG(I, "power control Timer started");
    }
}

/**
  * @brief  Function to pause power control timer
  * @param  none.
  * @retval none.
  */
static void stop_power_control_timer(void)
{
 
    osal_ubase_type_t xPowerControlTimerStopped;
    xPowerControlTimerStopped = xTimerStop( xPowerControl_Timer, portMAX_DELAY );
	
    if ( xPowerControlTimerStopped != pdPASS )
    {
        LOG(E, "Power control Timer stop failed");
    }

}

/**
 * @brief Filter timer callback function 
 * @param  none.
 * @retval none.
 */
static void fil_timer_cb_func ( TimerHandle_t xTimer )
{
    PWR_CTRL_DATA filter_data;

    filter_data.data    = 0;
    filter_data.data_id = INV_FIL_TMR_EXP;

    /*Filter Time elapsed*/

    // Append the INV_FIL_TMR_EXP data in the Queue
    osal_base_type_t ret = osal_queue_send (inv_pwr_ctrl_que_hdle, &filter_data, 0);

    if ( osal_success != ret )
    {
        LOG(E, "Queue error ret = %d", ret);
    }
}

/**
 * @brief Function to start filter timer
 * @param
 * @retval
 */
static void start_filter_timer(void)
{
    osal_ubase_type_t xFilterTimerStarted = xTimerStart( xFilterTimer, 0 );

    if ( xFilterTimerStarted == pdPASS )
    {
        LOG(I, "Filter Timer started");
    }
}

/**
  * @brief  Function to add the inventilate state changed request in queue 
  * @param  inv_pwr_ctrl_state State to be transfer.
  * @retval none.
  */
static void change_ivpmgr_state(IVPMGR0STATE_ENUM inv_pwr_ctrl_state)
{
    PWR_CTRL_DATA pwr_ctrl_data_frame;

    pwr_ctrl_data_frame.data    = inv_pwr_ctrl_state;
    pwr_ctrl_data_frame.data_id = INVENT_PWR_CTRL_STATE;

    // Append the data in the queue
    osal_base_type_t ret = xQueueSendToFront (inv_pwr_ctrl_que_hdle, &pwr_ctrl_data_frame, portMAX_DELAY);

    if ( osal_success != ret )
    {
        LOG(E, "Queue error ret = %d", ret);
    }
}

/**
  * @brief  Function to pause filter timer
  * @param  none.
  * @retval none.
  */
static void stop_filter_timer(PWR_CTRL_SM* ptr_ctrl_sm)
{
    TickType_t rem_time_msec;
    uint32_t filter_elap_tim;
    osal_ubase_type_t xFilterTimerStopped;

    /* Calculate the time left and the elapsed time for the filter timer before the timer is stopped */
    rem_time_msec = xTimerGetExpiryTime(xFilterTimer) - xTaskGetTickCount();
    rem_time_msec = pdTICKS_TO_MS(rem_time_msec);
    LOG(I, "rem_time_msec = %d", rem_time_msec);

    /* Add the elapsed time to the filter second counter */
    if ( rem_time_msec < MSEC_PER_MINUTE )
    {
        filter_elap_tim = MSEC_PER_MINUTE - rem_time_msec;             // Elapsed time in millisecond
        filter_elap_tim = filter_elap_tim / NUM_MILL_SEC_PER_SECOND;   // Elapsed time in seconds 
        LOG(I, "The elapsed time of filter = %d", filter_elap_tim);
        ptr_ctrl_sm->filter_sec_counter += filter_elap_tim;

        /* If the filter second counter is equal or more than one min, increment the filter min counter */
        if ( ptr_ctrl_sm->filter_sec_counter >= SECONDS_PER_MINUTE )
        {
            ptr_ctrl_sm->filter_sec_counter    = ptr_ctrl_sm->filter_sec_counter % SECONDS_PER_MINUTE;
            ptr_ctrl_sm->filter_min_counter++;
        }
    }

    xFilterTimerStopped = xTimerStop( xFilterTimer, portMAX_DELAY );
	
    if ( xFilterTimerStopped != pdPASS )
    {
        LOG(E, "Filter Timer stop failed");
    }

    /* Change the filter status */
    ptr_ctrl_sm->filter_cur_status = FILTER_IDLE;
}

/**
  * @brief  Function to validate the filter time period is expired on not
  * @param  Pointer to the structure variable PWR_CTRL_SM.
  * @retval none.
  */
static FILTER_RESET_STATUS validate_filter_time(PWR_CTRL_SM* ptr_pwr_ctrl_sm)
{
    int32_t filt_stat = ptr_pwr_ctrl_sm->filter_cur_status;

    if ( ptr_pwr_ctrl_sm->filter_min_counter >= FILTER_LIFE_TIME_MIN )
    {
        LOG(W, "Filter Expired");

        /* Stop the filter timer */
        stop_filter_timer(ptr_pwr_ctrl_sm);

        /* Set the filter status */
        ptr_pwr_ctrl_sm->filter_cur_status = FILTER_TIME_EXPIRED;

        /* Frame and send the DDMP set request */
        filt_stat = IV0FILST_FILTER_CHANGE_REQ;

        TRUE_CHECK(connector_send_frame_to_broker(DDMP2_CONTROL_SET, IV0FILST, &filt_stat, \
               sizeof(int32_t), connector_pwr_ctrl_service.connector_id, portMAX_DELAY));
    }

    return filt_stat;
}

#if defined(DEVICE_BQ25792) || defined(DEVICE_BQ25798)

#ifdef EMC_TEST_CASE_SOLAR
/**
  * @brief  Function to update the active power source selection
  *         Solar is selected as primary source for EMC testing
  * @param  void
  * @retval void.
  */
static void update_active_power_source(void)
{
    error_type result;
    uint8_t data_ccreg;;

    LOG(W, "Charger flag status changed = 0x%x", chr_flag0_reg.byte);
    LOG(I, "S_UAPS_VAC1 = %f Volt  VAC2 = %f",vac1, vac2);
#ifdef DEVICE_BQ25792 
    /* Read the STATUS REG1B to get the status of VAC1 and VAC2 */
    result  = bq25792_read_reg(CHARGE_STATUS_0_REG1BH, &chr_status0_reg.byte, 1u);

    /* Read the STATUS REG1D to get the status of VBAT */
    result |= bq25792_read_reg(CHARGE_STATUS_2_REG1DH, &chr_status2_reg.byte, 1u);
#endif

#ifdef DEVICE_BQ25798 
    /* Read the STATUS REG1B to get the status of VAC1 and VAC2 */
    result  = bq25798_read_reg(CHARGE_STATUS_0_REG1BH, &chr_status0_reg.byte, 1u);

    /* Read the STATUS REG1D to get the status of VBAT */
    result |= bq25798_read_reg(CHARGE_STATUS_2_REG1DH, &chr_status2_reg.byte, 1u);
#endif


    if ( result != RES_FAIL )
    {
        LOG(W, "CHARGE_STATUS_0_REG1BH 0x%x", chr_status0_reg.byte);
        LOG(W, "CHARGE_STATUS_2_REG1DH 0x%x", chr_status2_reg.byte);
        
#ifdef DEVICE_BQ25792        
        result = bq25792_read_reg(CHARGE_CONTROL_0_REG0FH, &data_ccreg, 1u);
        //if ( data_ccreg & 0x04 )
        {
            data_ccreg = data_ccreg & (~ 0x04);
            LOG(I,"R:CCREG0F 0x%x",data_ccreg);
            result = bq25792_write_reg(CHARGE_CONTROL_0_REG0FH, &data_ccreg, ONE_BYTE);
        }
#endif


#ifdef DEVICE_BQ25798        
        result = bq25798_read_reg(CHARGE_CONTROL_0_REG0FH, &data_ccreg, 1u);
        //if ( data_ccreg & 0x04 )
        {
            data_ccreg = data_ccreg & (~ 0x04);
            LOG(I,"R:CCREG0F 0x%x",data_ccreg);
            result = bq25798_write_reg(CHARGE_CONTROL_0_REG0FH, &data_ccreg, ONE_BYTE);
        }
#endif

       
        if ( IS_VAC1_AVAIL(chr_status0_reg.byte) &&                             // VAC1 solar power is not avilable
            ( curr_active_src != IV0PWRSRC_SOLAR_POWER_INPUT ) )
        {
            /* As per the system requirment/design solarpower(VAC1) will be primary source 
               So whenever the valid solar power is available having the capable of driving the system 
               Source shall be switch from the CAR battery source (VAC2) to solar power (VAC1) */
			
			LOG(W, "VAC1 solar power is available");
            
            if  ( IS_VALID_BAT_PWR_AVAIL(chr_status2_reg.byte) )                      //Chk the solar power for good range
            {
                /* Source switching should be done only when the battery power is avilable..
                   Otherwise the system reset could occur */

                LOG(W, "Valid battery source is available");

                chg_ctrl_reg4.B6_EN_ACDRV1 = 1; // Select VAC1  Solar
                chg_ctrl_reg4.B7_EN_ACDRV2 = 0; // Disable  VAC2 12V CAR Battery
#ifdef DEVICE_BQ25792  
                result = bq25792_write_reg(CHARGE_CONTROL_4_REG13H, &chg_ctrl_reg4.byte, ONE_BYTE);
#endif
#ifdef DEVICE_BQ25798  
                result = bq25798_write_reg(CHARGE_CONTROL_4_REG13H, &chg_ctrl_reg4.byte, ONE_BYTE);
#endif

                if ( result != RES_FAIL )
                {
                    /* Update the current active source as VAC1 solar */
                    curr_active_src = IV0PWRSRC_SOLAR_POWER_INPUT;
                    LOG(W, "Active power source is VAC1 solar");
                    
                }

                
            }
            else
            {
                LOG(E, "Battery backup is not available..Source switching to VAC1 is not possible");
            }
        }
        /*CAR battery detected and Solar not detected*/
        else if ( ( IS_VAC2_AVAIL(chr_status0_reg.byte)                )   &&        // VAC2 car battery is avilable
                  ( IS_VAC1_AVAIL(chr_status0_reg.byte) == 0           )   &&   // VAC1 Solar is not avilable
                  ( curr_active_src != IV0PWRSRC_12V_CAR_BATTERY_INPUT ) )          // Current active source in not a CAR battery
        {
            /* VAC2 Car battery is available */
            LOG(W, "VAC2 Car battery is available ");
            if ( ( IS_VALID_BAT_PWR_AVAIL(chr_status2_reg.byte) ) )
            {
                /* Source switching should be done only when the battery power is avilable,
                Otherwise the system reset could occur */
                LOG(W, "Valid battery source is available");
                chg_ctrl_reg4.B6_EN_ACDRV1 = 0; // Disable VAC1 Solar
                chg_ctrl_reg4.B7_EN_ACDRV2 = 1; // Select VAC2 12V

#ifdef DEVICE_BQ25792               
                result = bq25792_write_reg(CHARGE_CONTROL_4_REG13H, &chg_ctrl_reg4.byte, ONE_BYTE);
#endif
#ifdef DEVICE_BQ25798               
                result = bq25798_disable_mppt();
                result = bq25798_write_reg(CHARGE_CONTROL_4_REG13H, &chg_ctrl_reg4.byte, ONE_BYTE);
#endif
                if ( result != RES_FAIL )
                {
                    /* Update the current active source as VAC2 car battery */
                    curr_active_src = IV0PWRSRC_12V_CAR_BATTERY_INPUT;
                    LOG(W, "Active power source is VAC2 car battery");
                }
            }
            else
            {
                LOG(E, "Battery backup is not available..Source switching to VAC2 is not possible");
            }
        }
        else if ( ( IS_VAC1_AVAIL(chr_status0_reg.byte) == 0     ) && // VAC1 not available
                  ( IS_VAC2_AVAIL(chr_status0_reg.byte) == 0     ) && // VAC2 not available
                  ( IS_VALID_BAT_PWR_AVAIL(chr_status2_reg.byte) ) )  // Valid battery voltage available
        {
            /* Both VAC1 and VAC2 is not available and the system runs on battery backup */
            LOG(W, "Active power source is backup battery");
            curr_active_src = IV0PWRSRC_BACKUP_BATTERY;
        }
        else
        {
            LOG(W, "No need switch power src current src %d",curr_active_src);
        }
    }
    else
    {
        LOG(E, "Error read CHARGE_STATUS_0_REG1BH %d", result);
    }

    if ( curr_active_src != prev_active_src )
    {
        push_pwrsrc_status_in_queue(curr_active_src);
    }
}

#else // EMC_TEST_CASE_SOLAR

/**
  * @brief  Function to update the active power source selection
  * @param  void
  * @retval void.
  */
#ifdef INVENT_BATTERY_TESTING
static void update_active_power_source(void)
{
    error_type result;
    uint8_t data_ccreg;;

    LOG(W, "Charger flag status changed = 0x%x", chr_flag0_reg.byte);
    LOG(I, "UAPS_VAC1 = %f Volt  VAC2 = %f",vac1, vac2);

    /* Read the STATUS REG1B to get the status of VAC1 and VAC2 */
#ifdef DEVICE_BQ25792    
    result  = bq25792_read_reg(CHARGE_STATUS_0_REG1BH, &chr_status0_reg.byte, 1u);
    /* Read the STATUS REG1D to get the status of VBAT */
    result |= bq25792_read_reg(CHARGE_STATUS_2_REG1DH, &chr_status2_reg.byte, 1u);
#endif

#ifdef DEVICE_BQ25798  
    result  = bq25798_read_reg(CHARGE_STATUS_0_REG1BH, &chr_status0_reg.byte, 1u);
    /* Read the STATUS REG1D to get the status of VBAT */
    result |= bq25798_read_reg(CHARGE_STATUS_2_REG1DH, &chr_status2_reg.byte, 1u);
#endif


    if ( result != RES_FAIL )
    {
        LOG(W, "CHARGE_STATUS_0_REG1BH 0x%x", chr_status0_reg.byte);
        LOG(W, "CHARGE_STATUS_2_REG1DH 0x%x", chr_status2_reg.byte);
        
#ifdef DEVICE_BQ25792          
        result = bq25792_read_reg(CHARGE_CONTROL_0_REG0FH, &data_ccreg, 1u);
        //if ( data_ccreg & 0x04 )
        {
            data_ccreg = data_ccreg & (~ 0x04);
            LOG(I,"R:CCREG0F 0x%x",data_ccreg);
            result = bq25792_write_reg(CHARGE_CONTROL_0_REG0FH, &data_ccreg, ONE_BYTE);
        }
#endif
#ifdef DEVICE_BQ25798          
        result = bq25798_read_reg(CHARGE_CONTROL_0_REG0FH, &data_ccreg, 1u);
        //if ( data_ccreg & 0x04 )
        {
            data_ccreg = data_ccreg & (~ 0x04);
            LOG(I,"R:CCREG0F 0x%x",data_ccreg);
            result = bq25798_write_reg(CHARGE_CONTROL_0_REG0FH, &data_ccreg, ONE_BYTE);
        }
#endif



       if ( ( IS_VAC2_AVAIL(chr_status0_reg.byte)                )  &&  // VAC2 car battery is avilable
                  ( curr_active_src != IV0PWRSRC_12V_CAR_BATTERY_INPUT ) )    // Current active source in not a CAR battery
        {
            
            
            /* VAC2 Car battery is available */
            LOG(W, "VAC2 Car battery is available ");
            if ( ( IS_VALID_BAT_PWR_AVAIL(chr_status2_reg.byte) ) )
            {
                /* Source switching should be done only when the battery power is avilable,
                Otherwise the system reset could occur */

                LOG(W, "Valid battery source is available");

                chg_ctrl_reg4.B6_EN_ACDRV1 = 0; // Disable VAC1 Solar
                chg_ctrl_reg4.B7_EN_ACDRV2 = 1; // Select VAC2 12V

#ifdef DEVICE_BQ25792          
                result = bq25792_write_reg(CHARGE_CONTROL_4_REG13H, &chg_ctrl_reg4.byte, ONE_BYTE);
#endif

#ifdef DEVICE_BQ25798          
                result = bq25798_disable_mppt();
                result = bq25798_write_reg(CHARGE_CONTROL_4_REG13H, &chg_ctrl_reg4.byte, ONE_BYTE);
#endif

                if ( result != RES_FAIL )
                {
                    /* Update the current active source as VAC2 car battery */
                    curr_active_src = IV0PWRSRC_12V_CAR_BATTERY_INPUT;
                    LOG(W, "Active power source is VAC2 car battery");
                }
            }
            else
            {
                LOG(E, "Battery backup is not available..Source switching to VAC2 is not possible");
            }
        }
        else if ( IS_VAC1_AVAIL(chr_status0_reg.byte) &&                             // VAC1 solar power is not avilable
                    ( IS_VAC2_AVAIL(chr_status0_reg.byte) == 0                )  &&  // VAC2 car battery is not avilable
                    ( curr_active_src != IV0PWRSRC_SOLAR_POWER_INPUT ) )
        {
            /* As per the system requirment/design solarpower(VAC1) will be primary source 
               So whenever the valid solar power is available having the capable of driving the system 
               Source shall be switch from the CAR battery source (VAC2) to solar power (VAC1) */
			
			LOG(W, "VAC1 solar power is available");
            
            if  ( IS_VALID_BAT_PWR_AVAIL(chr_status2_reg.byte) )                      //Chk the solar power for good range
            {
                /* Source switching should be done only when the battery power is avilable..
                   Otherwise the system reset could occur */

                LOG(W, "Valid battery source is available");

                chg_ctrl_reg4.B6_EN_ACDRV1 = 1; // Select VAC1  Solar
                chg_ctrl_reg4.B7_EN_ACDRV2 = 0; // Disable  VAC2 12V CAR Battery

#ifdef DEVICE_BQ25792          
                result = bq25792_write_reg(CHARGE_CONTROL_4_REG13H, &chg_ctrl_reg4.byte, ONE_BYTE);
#endif
#ifdef DEVICE_BQ25798          
                result = bq25798_enable_mppt();
                result = bq25798_write_reg(CHARGE_CONTROL_4_REG13H, &chg_ctrl_reg4.byte, ONE_BYTE);
#endif


                if ( result != RES_FAIL )
                {
                    /* Update the current active source as VAC1 solar */
                    curr_active_src = IV0PWRSRC_SOLAR_POWER_INPUT;
                    LOG(W, "Active power source is VAC1 solar");
                    
                }

                
            }
            else
            {
                LOG(E, "Battery backup is not available..Source switching to VAC1 is not possible");
            }
        }
        else if ( ( IS_VAC1_AVAIL(chr_status0_reg.byte) == 0     ) && // VAC1 not available
                  ( IS_VAC2_AVAIL(chr_status0_reg.byte) == 0     ) && // VAC2 not available
                  ( IS_VALID_BAT_PWR_AVAIL(chr_status2_reg.byte) ) )  // Valid battery voltage available
        {
            /* Both VAC1 and VAC2 is not available and the system runs on battery backup */
            LOG(W, "Active power source is backup battery");
            curr_active_src = IV0PWRSRC_BACKUP_BATTERY;
        }
        else
        {
            LOG(W, "No need switch power src current src %d",curr_active_src);
        }
    }
    else
    {
        LOG(E, "Error read CHARGE_STATUS_0_REG1BH %d", result);
    }

    if ( curr_active_src != prev_active_src )
    {
        push_pwrsrc_status_in_queue(curr_active_src);
    }
}
#endif // INVENT_BATTERY_TESTING
#endif // EMC_TEST_CASE_SOLAR

#ifdef DEVICE_BQ25792
/**
  * @brief  Callback function to handle the interrupt data forward from the GPIO task
  * @param  device
  * @param  port
  * @param  pin
  * @retval void.
  */
void battery_ic_interrupt_cb(int device, int port, int pin)
{
    error_type result;

    switch (batt_ch_intr_stat)
    {
        case BATT_CH_STATE_IC_INIT:
            
            if ( BQ25792_PART_NUMBER == bq25792_get_chip_id() )
            {
                /* Update the active power source detected at startup */
                if(ivsett_config.EN_DIS_SOLAR == true)
                {
                    update_active_source_timer = 1;   /*update_active_power_source */
                }
                /* Change the state to init done */
                batt_ch_intr_stat = BATT_CH_STATE_IC_INIT_DONE;
            }


            break;

        case BATT_CH_STATE_IC_INIT_DONE:
            {
                result = bq25792_read_reg(FAULT_STATUS_0_REG20H, (uint8_t*)&fault_stat0_reg.byte, 1u);

                if ( ( result != RES_FAIL ) && ( fault_stat0_reg.byte != 0 ) )
                {
                    LOG(W, "FAULT_STATUS_0_REG20H 0x%x", fault_stat0_reg.byte);
                }
                
                result = bq25792_read_reg(FAULT_FLAG_0_REG26H, (uint8_t*)&fault_flag0_reg.byte, 1u);

                if ( ( result != RES_FAIL ) && ( fault_flag0_reg.byte != 0 ) )
                {
                    LOG(W, "FAULT_FLAG_0_REG26H 0x%x", fault_flag0_reg.byte);
                }

                result = bq25792_read_reg(FAULT_STATUS_1_REG21H, (uint8_t*)&fault_stat1_reg.byte, 1u);

                if ( ( result != RES_FAIL ) && ( fault_stat1_reg.byte != 0 ) )
                {
                    LOG(W, "FAULT_STATUS_1_REG21H 0x%x", fault_stat1_reg.byte);
                }

                result = bq25792_read_reg(FAULT_FLAG_1_REG27H, (uint8_t*)&fault_flag1_reg.byte, 1u);

                if ( ( result != RES_FAIL ) && ( fault_flag1_reg.byte != 0 ) )
                {
                    LOG(W, "FAULT_FLAG_1_REG27H 0x%x", fault_flag1_reg.byte);
                }

                /* Read the FLAG0 register to validate the change in status of power source */
                result = bq25792_read_reg(CHARGER_FLAG_0_REG22H, &chr_flag0_reg.byte, 1u);
                
                if (reg22_byte_prev != chr_flag0_reg.byte )
                {
                    reg22_byte_prev = chr_flag0_reg.byte;
                    update_active_source_timer = 1;         /*update_active_power_source */
                }

                

                result = bq25792_read_reg(CHARGER_FLAG_1_REG23H, &chr_flag1_reg.byte, 1u);

                if ( ( result != RES_FAIL ) && ( chr_flag1_reg.byte != 0 ) )
                {
                    
                    LOG(W, "CHARGER_FLAG_1_REG23H 0x%x", chr_flag1_reg.byte);                   
                }

                if ( ( chr_status0_reg.IINDPM_STAT == 1 ) || ( chr_flag0_reg.IINDPM_FLAG == 1 ) )
                {
                    LOG(E, "IINDPM Regulation");
                }

                if ( ( chr_status0_reg.VINDPM_STAT == 1 ) || ( chr_flag0_reg.VINDPM_FLAG == 1 ) )
                {
                    LOG(E, "VINDPM Regulation");
                }

                if ( ( chr_status2_reg.TREG_STAT == 1 ) || ( chr_flag1_reg.TREG_FLAG == 1 ) )
                {
                    LOG(E, "THERMAL Regulation");
                }

                if ( ( chr_status2_reg.ICO_STAT == 1 ) || ( chr_flag1_reg.ICO_FLAG == 1 ) )
                {
                    LOG(E, "ICO Algo running");
                }

                if ( (  VAC1_FAULT_STAT_DEV_IN_OVP == fault_stat0_reg.VAC1_OVP_STAT ) ||
                     (  FAULT_FLAG0_VAC1_ENTER_OVP == fault_flag0_reg.VAC1_OVP_FLAG ) )
                {
                    LOG(E, "VAC1 OVP");
                }

                if ( (  VAC2_FAULT_STAT_DEV_IN_OVP == fault_stat0_reg.VAC2_OVP_STAT ) ||
                     (  FAULT_FLAG0_VAC2_ENTER_OVP == fault_flag0_reg.VAC2_OVP_FLAG ) )
                {
                    LOG(E, "VAC2 OVP");
                }

                if ( (  CONV_FAULT_STAT_DEV_IN_OCP == fault_stat0_reg.CONV_OCP_STAT ) ||
                     ( FAULT_FLAG0_CONV_ENTER_OCP  == fault_flag0_reg.CONV_OCP_FLAG ) )
                {
                    LOG(E, "CONV OCP");
                }

                if ( (  IBAT_FAULT_STAT_DEV_IN_OCP           == fault_stat0_reg.IBAT_OCP_STAT ) ||
                     ( FAULT_FLAG0_IBAT_ENTER_DISCHARGED_OCP == fault_flag0_reg.IBAT_OCP_FLAG ) )
                {
                    LOG(E, "IBAT OCP");
                }

                if ( ( IBUS_FAULT_STAT_DEV_IN_OCP == fault_stat0_reg.IBUS_OCP_STAT ) ||
                     ( FAULT_FLAG0_IBUS_ENTER_OCP == fault_flag0_reg.IBUS_OCP_FLAG ) )
                {
                    LOG(E, "IBUS OCP");
                }

                if ( ( VBAT_FAULT_STAT_DEV_IN_OVP == fault_stat0_reg.VBAT_OVP_STAT ) ||
                     ( FAULT_FLAG0_VBAT_ENTER_OVP == fault_flag0_reg.VBAT_OVP_FLAG ) )
                {
                    LOG(E, "VBAT OVP");
                }

                if ( ( VBUS_FAULT_STAT_DEV_IN_OVP == fault_stat0_reg.VBUS_OVP_STAT ) ||
                     ( FAULT_FLAG0_VBUS_ENTER_OVP == fault_flag0_reg.VBUS_OVP_FLAG ) )
                {
                    LOG(E, "VBUS OVP");
                }

                if ( ( IBAT_FAULT_STAT_DEV_IN_BAT_DISCHARG_CR   == fault_stat0_reg.IBAT_REG_STAT ) ||
                    ( FAULT_FLAG0_IBAT_ENTER_OR_EXIT_REGULATION == fault_flag0_reg.IBAT_REG_FLAG ) )
                {
                    LOG(E, "IBAT DISCHARGE CURRENT REGULATION");
                }

                if ( ( TSHUT_FAULT_STAT_DEV_IN_THER_SHUTDOWN_PROTEC == fault_stat1_reg.TSHUT_STAT ) ||
                     ( FAULT_FLAG1_TSHUT                            == fault_flag1_reg.TSHUT_FLAG ) )
                {
                    LOG(E, "THERMAL SHUTDOWN");
                }

                if ( ( OTG_FAULT_STAT_DEV_OTG_UNDER_VOLTAGE           == fault_stat1_reg.OTG_UVP_STAT ) ||
                     ( FAULT_FLAG1_STOP_OTG_DUE_TO_VBUS_UNDER_VOLTAGE == fault_flag1_reg.OTG_UVP_FLAG ) )
                {
                    LOG(E, "STOP OTG : VBUS UNDER VOLTAGE");
                }

                if ( ( OTG_FAULT_STAT_DEV_OTG_OVER_VOLTAGE            == fault_stat1_reg.OTG_OVP_STAT ) ||
                     ( FAULT_FLAG1_STOP_OTG_DUE_TO_VBUS_UNDER_VOLTAGE == fault_flag1_reg.OTG_OVP_FLAG ) )
                {
                    LOG(E, "STOP OTG : VBUS OVER VOLTAGE");
                }

                if ( ( VSYS_FAULT_STAT_DEV_IN_SYS_OVP                        == fault_stat1_reg.VSYS_OVP_STAT ) ||
                     ( FAULT_FLAG1_VSYS_STOP_SWITCH_DUE_TO_SYS_OVER_VOLTAGE  == fault_flag1_reg.VSYS_OVP_FLAG ) )
                {
                    LOG(E, "VSYS OVP");
                }

                if ( ( VSYS_FAULT_STAT_DEV_IN_SYS_SHORT_CIRCUIT_PROTEC       == fault_stat1_reg.VSYS_SHORT_STAT ) ||
                     ( FAULT_FLAG1_STOP_SWITCH_DUE_TO_SYS_SHORT              == fault_flag1_reg.VSYS_SHORT_FLAG ) )
                {
                    LOG(E, "VSYS SHORT CIRCUIT PROTECTION");
                }
            }
            break;

        default:
            break;
    }
}
#endif // DEVICE_BQ25792

/*BQ25798 interrupt handling*/
#ifdef DEVICE_BQ25798
/**
  * @brief  Callback function to handle the interrupt data forward from the GPIO task
  * @param  device
  * @param  port
  * @param  pin
  * @retval void.
  */
void battery_ic_interrupt_cb(int device, int port, int pin)
{
    error_type result;

    switch (batt_ch_intr_stat)
    {
        case BATT_CH_STATE_IC_INIT:
            
            if ( BQ25798_PART_NUMBER == bq25798_get_chip_id() )
            {
                /* Update the active power source detected at startup */
                if(ivsett_config.EN_DIS_SOLAR == true)
                {
                    update_active_source_timer = 1;     /*update_active_power_source*/
                }
                else if( BQ25798_DEV_NUMBER == bq25798_get_chip_id())
                {
                        /*BQ25672 detected*/
                        LOG(W,"[init_bq25798_IC]")
                        /* Update the active power source detected at startup */
                        update_active_source_timer = 1;     /*update_active_power_source*/
                }

                /* Change the state to init done */
                batt_ch_intr_stat = BATT_CH_STATE_IC_INIT_DONE;
            }


            break;

        case BATT_CH_STATE_IC_INIT_DONE:
            {
                result = bq25798_read_reg(FAULT_STATUS_0_REG20H, (uint8_t*)&fault_stat0_reg.byte, 1u);

                if ( ( result != RES_FAIL ) && ( fault_stat0_reg.byte != 0 ) )
                {
                    LOG(W, "FAULT_STATUS_0_REG20H 0x%x", fault_stat0_reg.byte);
                }
                
                result = bq25798_read_reg(FAULT_FLAG_0_REG26H, (uint8_t*)&fault_flag0_reg.byte, 1u);

                if ( ( result != RES_FAIL ) && ( fault_flag0_reg.byte != 0 ) )
                {
                    LOG(W, "FAULT_FLAG_0_REG26H 0x%x", fault_flag0_reg.byte);
                }

                result = bq25798_read_reg(FAULT_STATUS_1_REG21H, (uint8_t*)&fault_stat1_reg.byte, 1u);

                if ( ( result != RES_FAIL ) && ( fault_stat1_reg.byte != 0 ) )
                {
                    LOG(W, "FAULT_STATUS_1_REG21H 0x%x", fault_stat1_reg.byte);
                }

                result = bq25798_read_reg(FAULT_FLAG_1_REG27H, (uint8_t*)&fault_flag1_reg.byte, 1u);

                if ( ( result != RES_FAIL ) && ( fault_flag1_reg.byte != 0 ) )
                {
                    LOG(W, "FAULT_FLAG_1_REG27H 0x%x", fault_flag1_reg.byte);
                }

                /* Read the FLAG0 register to validate the change in status of power source */
                result = bq25798_read_reg(CHARGER_FLAG_0_REG22H, &chr_flag0_reg.byte, 1u);
                
                if (reg22_byte_prev != chr_flag0_reg.byte )
                {
                    reg22_byte_prev = chr_flag0_reg.byte;
                    update_active_source_timer = 1;     /*udate_active_power_source*/
                }

                

                result = bq25798_read_reg(CHARGER_FLAG_1_REG23H, &chr_flag1_reg.byte, 1u);

                if ( ( result != RES_FAIL ) && ( chr_flag1_reg.byte != 0 ) )
                {
                    
                    LOG(W, "CHARGER_FLAG_1_REG23H 0x%x", chr_flag1_reg.byte);                   
                }

                if ( ( chr_status0_reg.IINDPM_STAT == 1 ) || ( chr_flag0_reg.IINDPM_FLAG == 1 ) )
                {
                    LOG(E, "IINDPM Regulation");
                }

                if ( ( chr_status0_reg.VINDPM_STAT == 1 ) || ( chr_flag0_reg.VINDPM_FLAG == 1 ) )
                {
                    LOG(E, "VINDPM Regulation");
                }

                if ( ( chr_status2_reg.TREG_STAT == 1 ) || ( chr_flag1_reg.TREG_FLAG == 1 ) )
                {
                    LOG(E, "THERMAL Regulation");
                }

                if ( ( chr_status2_reg.ICO_STAT == 1 ) || ( chr_flag1_reg.ICO_FLAG == 1 ) )
                {
                    LOG(E, "ICO Algo running");
                }

                if ( (  VAC1_FAULT_STAT_DEV_IN_OVP == fault_stat0_reg.VAC1_OVP_STAT ) ||
                     (  FAULT_FLAG0_VAC1_ENTER_OVP == fault_flag0_reg.VAC1_OVP_FLAG ) )
                {
                    LOG(E, "VAC1 OVP");
                }

                if ( (  VAC2_FAULT_STAT_DEV_IN_OVP == fault_stat0_reg.VAC2_OVP_STAT ) ||
                     (  FAULT_FLAG0_VAC2_ENTER_OVP == fault_flag0_reg.VAC2_OVP_FLAG ) )
                {
                    LOG(E, "VAC2 OVP");
                }

                if ( (  CONV_FAULT_STAT_DEV_IN_OCP == fault_stat0_reg.CONV_OCP_STAT ) ||
                     ( FAULT_FLAG0_CONV_ENTER_OCP  == fault_flag0_reg.CONV_OCP_FLAG ) )
                {
                    LOG(E, "CONV OCP");
                }

                if ( (  IBAT_FAULT_STAT_DEV_IN_OCP           == fault_stat0_reg.IBAT_OCP_STAT ) ||
                     ( FAULT_FLAG0_IBAT_ENTER_DISCHARGED_OCP == fault_flag0_reg.IBAT_OCP_FLAG ) )
                {
                    LOG(E, "IBAT OCP");
                }

                if ( ( IBUS_FAULT_STAT_DEV_IN_OCP == fault_stat0_reg.IBUS_OCP_STAT ) ||
                     ( FAULT_FLAG0_IBUS_ENTER_OCP == fault_flag0_reg.IBUS_OCP_FLAG ) )
                {
                    LOG(E, "IBUS OCP");
                }

                if ( ( VBAT_FAULT_STAT_DEV_IN_OVP == fault_stat0_reg.VBAT_OVP_STAT ) ||
                     ( FAULT_FLAG0_VBAT_ENTER_OVP == fault_flag0_reg.VBAT_OVP_FLAG ) )
                {
                    LOG(E, "VBAT OVP");
                }

                if ( ( VBUS_FAULT_STAT_DEV_IN_OVP == fault_stat0_reg.VBUS_OVP_STAT ) ||
                     ( FAULT_FLAG0_VBUS_ENTER_OVP == fault_flag0_reg.VBUS_OVP_FLAG ) )
                {
                    LOG(E, "VBUS OVP");
                }

                if ( ( IBAT_FAULT_STAT_DEV_IN_BAT_DISCHARG_CR   == fault_stat0_reg.IBAT_REG_STAT ) ||
                    ( FAULT_FLAG0_IBAT_ENTER_OR_EXIT_REGULATION == fault_flag0_reg.IBAT_REG_FLAG ) )
                {
                    LOG(E, "IBAT DISCHARGE CURRENT REGULATION");
                }

                if ( ( TSHUT_FAULT_STAT_DEV_IN_THER_SHUTDOWN_PROTEC == fault_stat1_reg.TSHUT_STAT ) ||
                     ( FAULT_FLAG1_TSHUT                            == fault_flag1_reg.TSHUT_FLAG ) )
                {
                    LOG(E, "THERMAL SHUTDOWN");
                }

                if ( ( OTG_FAULT_STAT_DEV_OTG_UNDER_VOLTAGE           == fault_stat1_reg.OTG_UVP_STAT ) ||
                     ( FAULT_FLAG1_STOP_OTG_DUE_TO_VBUS_UNDER_VOLTAGE == fault_flag1_reg.OTG_UVP_FLAG ) )
                {
                    LOG(E, "STOP OTG : VBUS UNDER VOLTAGE");
                }

                if ( ( OTG_FAULT_STAT_DEV_OTG_OVER_VOLTAGE            == fault_stat1_reg.OTG_OVP_STAT ) ||
                     ( FAULT_FLAG1_STOP_OTG_DUE_TO_VBUS_UNDER_VOLTAGE == fault_flag1_reg.OTG_OVP_FLAG ) )
                {
                    LOG(E, "STOP OTG : VBUS OVER VOLTAGE");
                }

                if ( ( VSYS_FAULT_STAT_DEV_IN_SYS_OVP                        == fault_stat1_reg.VSYS_OVP_STAT ) ||
                     ( FAULT_FLAG1_VSYS_STOP_SWITCH_DUE_TO_SYS_OVER_VOLTAGE  == fault_flag1_reg.VSYS_OVP_FLAG ) )
                {
                    LOG(E, "VSYS OVP");
                }

                if ( ( VSYS_FAULT_STAT_DEV_IN_SYS_SHORT_CIRCUIT_PROTEC       == fault_stat1_reg.VSYS_SHORT_STAT ) ||
                     ( FAULT_FLAG1_STOP_SWITCH_DUE_TO_SYS_SHORT              == fault_flag1_reg.VSYS_SHORT_FLAG ) )
                {
                    LOG(E, "VSYS SHORT CIRCUIT PROTECTION");
                }
            }
            break;

        default:
            break;
    }
}
#endif  // DEVICE_BQ25798


#if defined (INVENT_BATTERY_TESTING) || defined (EMC_TEST_CASE_SOLAR)
/**
  * @brief  Functionto push the active power source in the queue
  * @param  curr_active_source Current active power soure refer enum IV0PWRSRC_ENUM
  * @retval void.
  */
static void push_pwrsrc_status_in_queue(IV0PWRSRC_ENUM curr_active_source)
{
    PWR_CTRL_DATA pwr_ctrl_que_data;

    prev_active_src           = curr_active_source;
    pwr_ctrl_que_data.data    = curr_active_source;
    pwr_ctrl_que_data.data_id = INVENT_POWER_SOURCE_CHANGED;

    osal_base_type_t ret = osal_queue_send (inv_pwr_ctrl_que_hdle, &pwr_ctrl_que_data, 0);

    if ( osal_fail == ret )
    {
        LOG(E, "Error Queuw %d", ret);
    }
}
#endif // defined (INVENT_BATTERY_TESTING) || defined (EMC_TEST_CASE_SOLAR)

#ifdef INVENT_BATTERY_TESTING
#if defined(DEVICE_BQ25792)  || defined(DEVICE_BQ25798)
/**
 * @brief  Callback function to handle subscribed data
 * @param  table_index
 * @param  i32Value.
 * @retval none.
 */
static void pwr_ctrl_error_code(const PWR_CTRL_ERR_CODES error)
{
    uint32_t err_frame = invent_pwr_ctrl_error_stat;

    switch (error)
    {
        case PWR_BACKUP_BAT_LOW:
            err_frame |=  (1 << BACKUP_BATTERY_LOW);
            #if CONN_PWR_DEBUG_LOG
                LOG(I,"pwr_ctrl Err: Bat low");
            #endif
            break;

        case PWR_COMM_ERROR_WITH_BAT_IC:
            err_frame |=  (1 << COMM_ERROR_WITH_BATTERY_IC);
            #if CONN_PWR_DEBUG_LOG
            LOG(I,"pwr_ctrl Err: BMS_COM_ERR");
            #endif
            break;
        case PWR_BAT_OVER_HEATING:
            err_frame |=  (1 << BATTERY_OVER_HEATING);
            LOG(I,"pwr_ctrl Err: Bat Heating");
            break;
        case PWR_BAT_COOL:
            err_frame &=  ~(1 << BATTERY_OVER_HEATING);
            LOG(I,"pwr_ctrl Err: Bat Cool");
            break;
        case PWR_BAT_EXPIRED:
            err_frame |=  (1 << BATTERY_EXPIRED);
            #if CONN_PWR_DEBUG_LOG
            LOG(I,"pwr_ctrl Err: Bat expired");
            #endif
            break;

        case PWR_BACKUP_BAT_NORMAL:
            err_frame &=  ~(1 << PWR_BACKUP_BAT_LOW);
            #if CONN_PWR_DEBUG_LOG
            LOG(I,"pwr_ctrl Err: Bat norm");
            #endif
            break;
        default:
            LOG(E, "Unhandled pwr_ctrl error code = %d", error);
            break;
    }

    if ( err_frame != invent_pwr_ctrl_error_stat )
    {
        invent_pwr_ctrl_error_stat = err_frame;
        connector_send_frame_to_broker(DDMP2_CONTROL_SET, IV0ERRST, &err_frame, sizeof(err_frame), connector_pwr_ctrl_service.connector_id, (TickType_t)portMAX_DELAY);
    }
}
#endif // defined(DEVICE_BQ25792)  || defined(DEVICE_BQ25798)
#endif // INVENT_BATTERY_TESTING

#endif // defined(DEVICE_BQ25792) || defined(DEVICE_BQ25798)

#endif /*CONNECTOR_POWER_CONTROL_SERVICE*/ 