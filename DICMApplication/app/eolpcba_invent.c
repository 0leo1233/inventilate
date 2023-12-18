/*! \file eol_invent.c
	\brief eol for Inventilate
 */
#if 0
/** Includes ******************************************************************/
#include <stdio.h>
#include <string.h>
#include "dicm_framework_config.h"
#include "iGeneralDefinitions.h"

#include "eolpcba_invent.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "linear_interpolation.h"
#include "app_onboard_hmi_ctrl.h"
#include "app_fan_motor_ctrl.h"
#include "app_voc_sensor.h"
#ifdef DEVICE_UC1510C
#include "drv_uc1510c.h"
#endif

#include "hal_ledc.h"
#include "hal_pwm.h"

/** Variable declarations ********************************************/
uint8_t error_can_status_old = 0;
uint8_t loop_light;

//static uint8_t wifi_status = WIFI0STS_UNKNOWN;

/** Prototype definition ********************************************/

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
        LOG(E, "LCD Display COMM Error");
        eol_data_frame->eol_ack = PCBA_EOL_ERR_COMM;
    }
    else if ( read_data == TP_TIME_REG_VAL )
    {
        LOG(W, "LCD Display Communication success");
        eol_data_frame->eol_ack = PCBA_EOL_NO_ERR;
    }
    else
    {
        LOG(E, "LCD Display read/write failed");
        eol_data_frame->eol_ack = PCBA_EOL_ERR_DATAMISMATCH;
    }

    return eol_data_frame->eol_ack;
}
#endif


/*......................................................................................................................................*/
