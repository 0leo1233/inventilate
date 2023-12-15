/*! \file app_voc_sensor.c
	\brief This file contains implementation related to application VOC sensor
 */

#include "dicm_framework_config.h"

#ifdef APP_VOC_SENSOR
#include "iGeneralDefinitions.h"
#include "freertos/FreeRTOS.h"
#include "ddm2_parameter_list.h"
#include "driver/i2c.h"
#include "app_voc_sensor.h"
#include "connector_voc_sensor_api.h"
#include "osal.h"
#include "hal_nvs.h"
#include "hal_i2c_master.h"
#include "hal_cpu.h"
#include <string.h>

#if ( CONFIG_DICM_SUPPORT_INTEGRATED_BSEC_LIB_2_X == 1 )

#include "bsec_interface.h"
#include "bsec_integration_2_x.h"
#include "bme68x_defs.h"

#include "bsec_serialized_configurations_selectivity.h"

#if ( BME68X_CHIP_TYPE == CHIP_TYPE_INTERNAL_BME688 )
#define BME68X_I2C_ADDR BME68X_I2C_ADDR_LOW
#elif ( BME68X_CHIP_TYPE == CHIP_TYPE_EXTERNAL_BME688 )
#define BME68X_I2C_ADDR BME68X_I2C_ADDR_HIGH
#else
#define BME68X_I2C_ADDR BME68X_I2C_ADDR_LOW
#endif

#endif

#define APP_VOC_DEBUG_LOG 0
#define APP_VOC_LOG         0
#define BSEC_FLASH_KEY (const char *)"bsec"

/* Struct instance for read_service */
read_voc_data_service inst_read_service;

uint32_t bsec_critical_error = 0;


static uint32_t state_load_func(uint8_t *state_buffer, uint32_t n_buffer);
static uint32_t config_load_func(uint8_t *config_buffer, uint32_t n_buffer);
static int64_t get_timestamp_us(void);
static void state_save_func(const uint8_t *state_buffer, uint32_t length);

#if ( CONFIG_DICM_SUPPORT_INTEGRATED_BSEC_LIB_2_X == 1 )
static void bme68x_output_ready_cb(bme68x_bsec_output *bsec_out);
static BME68X_INTF_RET_TYPE bme68x_write_i2c(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
static BME68X_INTF_RET_TYPE bme68x_read_i2c(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
static void bsec_delay_us(uint32_t period, void *intf_ptr);
static void update_voc_sens_to_broker(bme68x_bsec_output *bsec_out);
#endif

#if ( CONFIG_DICM_SUPPORT_INTEGRATED_BSEC_LIB_2_X == 1 )

/*! \brief  This function provides write functionality to the BME680 sensor.
 *
 *
 *  \param reg_addr	Register address to write the value to.
 *  \param reg_data	Register data to write to the reg_addr.
 *  \param len      Length of bytes to be written.
 *  \param dev_id   Device ID
 *
 *  \return 0 if successful. Any other value otherwise.
 */
static BME68X_INTF_RET_TYPE bme68x_write_i2c(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    uint8_t txbuf[20] = { 0 };

    if ( len > ( sizeof(txbuf) / sizeof( txbuf[0] ) - 1 ) )
    {
        return true;   // Data to long for transfer buffer.
    }
    
    txbuf[0] = reg_addr;
    memcpy(&txbuf[1], data, len);

    return hal_i2c_master_write(I2C_NUM_0, BME68X_I2C_ADDR, txbuf, (size_t)len + 1);
}

/*! \brief  This function provides read functionality to the BME680 sensor.
 *
 *
 *  \param reg_addr	Register address to read the value from.
 *  \param reg_data	pointer to the data which has been read from sensor.
 *  \param len      Length of bytes to be read.
 *  \param dev_id   Device ID
 *
 *  \return 0 if successful. Any other value otherwise.
 */
static BME68X_INTF_RET_TYPE bme68x_read_i2c(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{   
    return hal_i2c_master_writeread(I2C_NUM_0, BME68X_I2C_ADDR, &reg_addr, 1, data, (size_t)len);
}

/**
  * @brief  Function to generate delay in microseconds
  * @param  period   - Delay in Microseconds.
  * @param  intf_ptr - Void pointer that can enable the linking of descriptors for interface related callbacks.
  * @retval void.
  */
static void bsec_delay_us(uint32_t period, void *intf_ptr)
{
	//osal_task_delay(period/1000);
	hal_cpu_wait_us(period);
}


/**
  * @brief  Callback Function to get the output from BSEC BME680 sensor
  * @param  Pointer to the struct type bme680_bsec_output.
  * @retval void.
  */
static void bme68x_output_ready_cb(bme68x_bsec_output *bsec_out)
{
    if ( bsec_out != NULL )
    { 
        
#if APP_VOC_DEBUG_LOG

#ifndef CONFIG_DICM_SUPPORT_INVENT_SERIAL_BME68X_BSEC_LOGS

    #ifdef BME68X_USE_FPU
        LOG(I, "IAQ:[%f]", bsec_out->iaq.data);
        LOG(I, "Temperature:[%f]", bsec_out->raw_temperature.data);
        LOG(I, "Pressure:[%f]", bsec_out->raw_pressure.data);
        LOG(I, "Humidity:[%f]", bsec_out->raw_humidity.data);
        LOG(I, "GasResistance:[%f]", bsec_out->raw_gas.data);
        LOG(I, "CO2_equivalent:[%f]", bsec_out->co2_equivalent.data);
        LOG(I, "Breath_voc_eq:[%f]", bsec_out->breath_voc_eq.data);
        LOG(I, "Accuracy:[%d]", bsec_out->iaq.accuracy);
        LOG(I, "Stabilization_status:[%f]", bsec_out->stabilization_status.data);
        LOG(I, "run_in_status:[%f]", bsec_out->run_in_status.data);
#else   //BME68X_USE_FPU       
        LOG(I, "IAQ:[%d]",                  bsec_out->iaq.data);
        LOG(I, "Temperature:[%d]",          bsec_out->raw_temperature.data);
        LOG(I, "Pressure:[%d]",             bsec_out->raw_pressure.data);
        LOG(I, "Humidity:[%d]",             bsec_out->raw_humidity.data);
        LOG(I, "GasResistance:[%d]",        bsec_out->raw_gas.data);
        LOG(I, "CO2_equivalent:[%d]",       bsec_out->co2_equivalent.data);
        LOG(I, "Breath_voc_eq:[%d]",        bsec_out->breath_voc_eq.data);
        LOG(I, "Accuracy:[%d]",             bsec_out->iaq.accuracy);
        LOG(I, "Stabilization_status:[%d]", bsec_out->stabilization_status.data);
        LOG(I, "run_in_status:[%d]",        bsec_out->run_in_status.data);
#endif  //BME68X_USE_FPU

#else   // CONFIG_DICM_SUPPORT_INVENT_SERIAL_BME68X_BSEC_LOGS
            
        LOG_TEST(I, "IAQ:[%d]", bsec_out->iaq.data);
        LOG_TEST(I, "Temperature:[%d]", bsec_out->raw_temperature.data);
        LOG_TEST(I, "Pressure:[%d]", bsec_out->raw_pressure.data);
        LOG_TEST(I, "Humidity:[%d]", bsec_out->raw_humidity.data);
        LOG_TEST(I, "GasResistance:[%d]", bsec_out->raw_gas.data);
        LOG_TEST(I, "CO2_equivalent:[%d]", bsec_out->co2_equivalent.data);
        LOG_TEST(I, "Breath_voc_eq:[%d]", bsec_out->breath_voc_eq.data);
        LOG_TEST(I, "Accuracy:[%d]", bsec_out->iaq.accuracy);
        LOG_TEST(I, "Stabilization_status:[%d]", bsec_out->stabilization_status.data);
        LOG_TEST(I, "run_in_status:[%d]", bsec_out->run_in_status.data);
            
#endif  // CONFIG_DICM_SUPPORT_INVENT_SERIAL_BME68X_BSEC_LOGS

#endif  //APP_VOC_DEBUG_LOG

        /*Send IAQ data to broker whenever data changes*/
        if ( inst_read_service.iaq_index != bsec_out->iaq.data )
        {
            /* store the new IAQ index value */
            inst_read_service.iaq_index = bsec_out->iaq.data;
            /* Publish the value to broker */
			update_voc_sens_to_broker(bsec_out);
        }

         /*Send accuracy  to broker when accuracy is updated*/
        if (inst_read_service.aqrc_lvl != bsec_out->iaq.accuracy)
        {
            /* store the new Accuracy index value */
            inst_read_service.aqrc_lvl = bsec_out->iaq.accuracy;
            /* Publish the value to broker */
			update_voc_sens_to_broker(bsec_out);
        }
    }
    else
    {
        LOG(I,"[Bme68x_output_ready_error]");
    }
}

/**
  * @brief  Function to update the BSEC data output to broker
  * @param  Pointer to the struct type bme68x_bsec_output.
  * @retval void.
  */
void update_voc_sens_to_broker(bme68x_bsec_output *bsec_out)
{
	/* Update the newly calculated value of IAQ index in data base and publish to broker */
	update_and_publish_to_broker(  SBMEB0IAQ, bsec_out->iaq.data);
	update_and_publish_to_broker( SBMEB0TEMP, bsec_out->raw_temperature.data);
	update_and_publish_to_broker(  SBMEB0PRS, bsec_out->raw_pressure.data);
	update_and_publish_to_broker(  SBMEB0HUM, bsec_out->raw_humidity.data);
	update_and_publish_to_broker(  SBMEB0GAS, bsec_out->raw_gas.data);
	update_and_publish_to_broker(  SBMEB0CO2, bsec_out->co2_equivalent.data);
	update_and_publish_to_broker(  SBMEB0VOC, bsec_out->breath_voc_eq.data);
	update_and_publish_to_broker(  SBMEB0SST, bsec_out->stabilization_status.data);
	update_and_publish_to_broker(  SBMEB0RIS, bsec_out->run_in_status.data);
	update_and_publish_to_broker(  SBMEB0AQR, bsec_out->iaq.accuracy);       // DDMP need to be update after changed in Master DDMP
}

/**
  * @brief  Callback function to load bme68x configuration
  * @param  config_buffer Pointer to config buffer.
  * @param  n_buffer Buffer size
  * @retval buffer size.
  */
static uint32_t config_load_func(uint8_t *config_buffer, uint32_t n_buffer)
{
    uint32_t buffer_size = sizeof(bsec_config_selectivity);

	if ( config_buffer != NULL )
	{
		// ...
		// Load a library config from non-volatile memory, if available.
		//config_buffer
		// Return zero if loading was unsuccessful or no config was available,
		// otherwise return length of loaded config string.
		// ...
		memcpy((void*)config_buffer, (const void *)bsec_config_selectivity, (size_t)buffer_size);
	}

    return buffer_size;
}

/**
  * @brief  Const structure initilazed for bme688 sensor
  */
const BME6X_BSEC_LIB_INTERFACE g_bsec_lib_intf = 
{
    .bus_read           = bme68x_read_i2c,
    .bus_write          = bme68x_write_i2c,
    .delay_us           = bsec_delay_us,
    .get_time_stamp_us  = get_timestamp_us,
    .output_ready       = bme68x_output_ready_cb,
    .config_load        = config_load_func,
    .state_load         = state_load_func,
    .state_save         = state_save_func,
    .temperature_offset = BME68X_TEMPERATURE_OFFSET,
    .sample_rate        = BSEC_SAMPLE_RATE_LP,
};

#endif

/**
  * @brief  Callback function to save the BSEC state in the NVS
  * @param  state_buffer Pointer to state buffer.
  * @param  length Buffer size
  * @retval Void.
  */
static void state_save_func(const uint8_t *state_buffer, uint32_t length)
{
    if ( state_buffer != NULL )
    {
        hal_nvs_write(BSEC_FLASH_KEY, HAL_NVS_DATA_TYPE_BLOB, (const void *)state_buffer, length);
    }
}

/**
  * @brief  Callback function to load the BSEC state from the NVS
  * @param  state_buffer Pointer to state buffer.
  * @param  length Buffer size
  * @retval Size of BSEC state buffer.
  */
static uint32_t state_load_func(uint8_t *state_buffer, uint32_t n_buffer)
{
	uint32_t size = 0;
    
    if ( state_buffer != NULL )
    {
        hal_nvs_read(BSEC_FLASH_KEY, HAL_NVS_DATA_TYPE_BLOB, (void*)state_buffer, n_buffer);      
        size = n_buffer;
    }
    
    return size;
}

/**
  * @brief  Function to get time in us
  * @param  void.
  * @retval time stamp in nano second.
  */
int64_t get_timestamp_us(void)
{ 
   volatile int64_t time_stamp_us;
   volatile TickType_t time_ms;
   volatile TickType_t cur_tick = xTaskGetTickCount();

   time_ms = pdTICKS_TO_MS(cur_tick);

   time_stamp_us =  (int64_t)((int64_t)time_ms * (int64_t)1000);
   
   return time_stamp_us;
}

#endif
