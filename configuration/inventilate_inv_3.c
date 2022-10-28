/*! \file inventilate_inv_3.c
	\brief Additional code for custom configuration
*/

#include "configuration.h"


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
#ifdef HAL_GPIO
    TRUE_CHECK(longpress_timer=xTimerCreate(NULL,pdMS_TO_TICKS(LONG_PRESS),pdFALSE,NULL,long_press));
    TRUE_CHECK(release_timer=xTimerCreate(NULL,pdMS_TO_TICKS(RELEASED_PRESS),pdFALSE,NULL,short_press));
#endif
    uint8_t ntag0[17]={0,0xaa,0,0,0,0,0,0,0,0,0,0,0,0xe1,0x10,0x6d,0x00};

    uint8_t *m=(uint8_t*)device_information.id;

    uint8_t ntag1[17]={1,0x03,0x2c,0xd2,0x20,0x09,'a' ,'p' ,'p' ,'l' ,'i', 'c' ,'a' ,'t' ,'i' ,'o' ,'n' ,};
    uint8_t ntag2[17]={2,'/' ,'v' ,'n' ,'d' ,'.' ,'b' ,'l' ,'u' ,'e' ,'t' ,'o', 'o' ,'t' ,'h' ,'.' ,'l' ,};
    uint8_t ntag3[17]={3,'e' ,'.' ,'o' ,'o' ,'b' ,0x08,0x1b,m[5]|2,m[4],m[3],m[2],m[1],m[0],0x00,0xfe};


    vTaskDelay(1);
    ZERO_CHECK(hal_i2c_master_write(0, 0x55, ntag0, sizeof(ntag0)));
    vTaskDelay(1);
    ZERO_CHECK(hal_i2c_master_write(0, 0x55, ntag1, sizeof(ntag1)));
    vTaskDelay(1);
    ZERO_CHECK(hal_i2c_master_write(0, 0x55, ntag2, sizeof(ntag2)));
    vTaskDelay(1);
    ZERO_CHECK(hal_i2c_master_write(0, 0x55, ntag3, sizeof(ntag3)));
}
#endif //BOARD_INITIALIZATION




