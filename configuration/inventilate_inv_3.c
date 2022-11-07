/*! \file inventilate_inv_3.c
	\brief Additional code for custom configuration
*/

#include "configuration.h"


#include "driver/adc.h"
#ifdef HAL_GPIO

#include "hal_i2c_master.h"
#include "hal_ledc.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "bluetooth_le.h"
#include "hal_gpio.h"
#include "drv_tca9554a.h"

TimerHandle_t longpress_timer, release_timer;

/*! \brief Master GPIO pin list, used for startup initialization
	
    Creates an array of all the post that can be iterated by the initialization routines
    \sa HAL_GPIO_PINLIST
*/
#define GPIO_PIN(name, device, port, pin, pinmode, pinlevel, intrmode, cb) {device,port,pin,pinmode,pinlevel,intrmode,cb},
const HAL_GPIO_PINLIST Gpio_pinlist[]=
{
    GPIO_PINS
};
#undef GPIO_PIN

const int Gpio_pinlist_size=sizeof(Gpio_pinlist)/sizeof(Gpio_pinlist[0]);   //!< \~ Exported size of GPIO pin list

#endif

#ifdef HAL_LEDC_PWM

#define LEDC_PWM(name, gpio_num, duty_resolution, freq_hz, speed_mode, timer_num, clk_cfg, channel, duty, hpoint) {gpio_num, duty_resolution, freq_hz, speed_mode, timer_num, clk_cfg, channel, duty, hpoint},
const HAL_LEDC_PWM_CONFIG ledc_pwmconfig[]=
{
    LEDC_CONFIGURATION
};
#undef LEDC_PWM

const int ledc_pwm_config_size = sizeof(ledc_pwmconfig)/sizeof(ledc_pwmconfig[0]);   //!< \~ Exported size of ledc PWM config size

#endif

#ifndef DEVICE_BQ25792  // Temporary workaround for build issue..Once the testing is done this code will get removed
void battery_ic_interrupt_cb(int device, int port, int pin)
{
    LOG(E, "Interrupt triggered from Battery IC not handled");
}
#endif

#ifndef CONNECTOR_ONBOARD_HMI
void onboard_hmi_interrupt_cb(int device, int port, int pin)
{


}
#endif

#ifdef BOARD_INITIALIZATION
//! \~ Optional board initalization routine invoked at the end of startup, does not need to return
void board_initialization(void)
{
    adc_power_acquire();
}
#endif //BOARD_INITIALIZATION
