/*! \file inventilate.c
	\brief Additional code for Inventilate configuration
*/

#include "dicm_framework_config.h"

#include "driver/adc.h"
#include "hal_i2c_master.h"
#include "hal_ledc.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#ifdef HAL_GPIO
#include "hal_gpio.h"
#include "drv_tca9554a.h"



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

#endif // HAL_LEDC_PWM

#ifndef DEVICE_BQ25792  // Temporary workaround for build issue..Once the testing is done this code will get removed
void battery_ic_interrupt_cb(int device, int port, int pin)
{
    LOG(E, "Interrupt triggered from Battery IC not handled");
}
#endif


__attribute__((weak)) void onboard_hmi_interrupt_cb(int device, int port, int pin)
{


}

#ifdef BOARD_INITIALIZATION
//! \~ Optional board initalization routine invoked at the end of startup, does not need to return
void board_initialization(void)
{
    // Workaround for spurious interrupts on GPIO_36 and GPIO_39. See adc1_get_raw()
/*       @note ESP32:
 *       When the power switch of SARADC1, SARADC2, HALL sensor and AMP sensor is turned on,
 *       the input of GPIO36 and GPIO39 will be pulled down for about 80ns.
 *       When enabling power for any of these peripherals, ignore input from GPIO36 and GPIO39.
 *       Please refer to section 3.11 of 'ECO_and_Workarounds_for_Bugs_in_ESP32' for the description of this issue.
 *       As a workaround, call adc_power_acquire() in the app. This will result in higher power consumption (by ~1mA),
 *       but will remove the glitches on GPIO36 and GPIO39.
 */

    adc_power_acquire();
}
#endif //BOARD_INITIALIZATION
