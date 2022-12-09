/*! \file inventilate_inv_3.h
	\brief General configuration for inventilate board
*/

#ifndef INVENTILATE_INV_3_H_
#define INVENTILATE_INV_3_H_

// Global Application defines
#define NUM_SECONDS_PER_MINUTE                    ((uint32_t)   60u)
#define NUM_MILL_SEC_PER_SECOND                   ((uint32_t) 1000u)
#define MIN_TO_MSEC(x)                            (NUM_MILL_SEC_PER_SECOND * NUM_SECONDS_PER_MINUTE * x)

//#define INVENT_EOL_TESTING

extern void onboard_hmi_interrupt_cb(int device, int port, int pin);
extern void battery_ic_interrupt_cb(int device, int port, int pin);

#define HW_VERSION_1_0               0   // POC Board
#define HW_VERSION_2_0               1
#define HW_VERSION_4_1               2

#define HARDWARE_MAJOR               4
#define HARDWARE_MINOR               1

#define STOCK_KEEPING_UNIT "1235424665"

#ifdef INVENT_EOL_TESTING

#define FIRMWARE_MAJOR				 0
#define FIRMWARE_MINOR				 5
#define FIRMWARE_BUILD_ID			"INV"
#define FIRMWARE_STRING				"DICM Inventilate"

#define HARDWARE_STRING				"Dometic Inventilate Rev 4.1"
#define DEFAULT_DEVICE_NAME_PREFIX	"INV_"
#define INVENT_HARWARE_VERSION      HW_VERSION_4_1

#else

#define FIRMWARE_MAJOR				 0
#define FIRMWARE_MINOR				 5
#define FIRMWARE_BUILD_ID			"INV"
#define FIRMWARE_STRING				"DICM Inventilate"
#define FIRMWARE_VERSION			"1.0.5-rc.1"
#define HARDWARE_STRING				"Dometic Inventilate Rev 4.1"
#define DEFAULT_DEVICE_NAME_PREFIX	"INV_"
#define INVENT_HARWARE_VERSION      HW_VERSION_4_1

#endif

#define COMM_BLE                   0
#define COMM_I2C                   1
#define SDP3X_SENS_BOARD_COMM      COMM_BLE

#define CHIP_TYPE_INTERNAL_BME680  0
#define CHIP_TYPE_EXTERNAL_BME680  1
#define CHIP_TYPE_INTERNAL_BME688  2
#define CHIP_TYPE_EXTERNAL_BME688  3
#define BME68X_CHIP_TYPE           CHIP_TYPE_EXTERNAL_BME688

//#define INVENT_BATTERY_TESTING

#define CONNECTOR_PWM_FAN_MOTOR_CAPTURE_UNIT (0)	// Connected to MCPWM_UNIT_0
#define HAL_PWM_TIMER_0
#define HAL_PWM_TIMER_1
#define INVENTILATE_FAN_PULSE_CNT_PER_REVOL       ((uint8_t) 2u)
#define INVENTILATE_MOTOR_ENG_PULSE_CNT_PER_REVOL ((uint8_t) 6u)
#define CONNECTOR_PWM_FAN_MOTOR_CAPTURE_CHANNEL_0_0 INVENTILATE_FAN_PULSE_CNT_PER_REVOL
#define CONNECTOR_PWM_FAN_MOTOR_CAPTURE_CHANNEL_0_1 INVENTILATE_FAN_PULSE_CNT_PER_REVOL
#define CONNECTOR_PWM_FAN_MOTOR_CAPTURE_CHANNEL_0_2 INVENTILATE_MOTOR_ENG_PULSE_CNT_PER_REVOL


#define INVERTED_PWM              ((uint32_t)     0u)
#define NON_INVERTED_PWM          ((uint32_t)     1u)

#define DEV_FAN1_MIN_RPM          ((uint32_t)     0u)
#define DEV_FAN1_MAX_RPM          ((uint32_t)  2800u)
#define DEV_FAN1_AIR_OUT_PWM_MODE INVERTED_PWM

#define DEV_FAN2_MIN_RPM          ((uint32_t)     0u)
#define DEV_FAN2_MAX_RPM          ((uint32_t)  2800u)
#define DEV_FAN2_AIR_IN_PWM_MODE  INVERTED_PWM

#define DEV_MOTOR_MIN_RPM         ((uint32_t)     0u)
#define DEV_MOTOR_MAX_RPM         ((uint32_t)  2200u)
#define DEV_MOTOR_PWM_MODE        NON_INVERTED_PWM

//#define BUILD_STATION_SSID			"AP-ESP32"
//#define BUILD_STATION_PASSWORD		"987654321"

#define WIFI_NETWORK
#define WIFI_NETWORK_STA
//#define WIFI_NETWORK_AP
#define MULTICAST_DNS
#define MULTICAST_DNS_SERVICE_NAME   "Dometic Inventilate"

/* Connectors for Inventilate */
#define CONNECTOR_WIFI
#define CONNECTOR_BLE
#define CONNECTOR_BLE_PERIPHERAL_GATT
#define CONNECTOR_BLE_CENTRAL_GATT
#define CONNECTOR_BLE_CENTRAL_GAP
#define CONNECTOR_BLE_PERIPHERAL_NO_BOND_REQUIRED
#define CONNECTOR_VOC_SENSOR
#define CONNECTOR_ONBOARD_HMI
#define CONNECTOR_LIGHT
#define CONNECTOR_PWM_FAN_MOTOR
#define CONNECTOR_POWER_CONTROL_SERVICE
#define CONNECTOR_USM
#define CONNECTOR_US_DICM
#ifndef INVENT_EOL_TESTING
#define CONNECTOR_DP_SENS_SERVICE
#endif

#ifdef INVENT_EOL_TESTING
#define CONNECTOR_EOL_SERVICE
#define USE_LIN
#endif

/* Application modules for Inventilate */
#define APP_VOC_SENSOR
#define APP_FAN_MOTOR_CONTROL
#define APP_ONBOARD_HMI_CONTROL
#define APP_POWER_CONTROL_SERVICE
#define APP_DIFF_PRESSURE_SENSOR
#define APP_LIGHT_CONTROL

#ifdef CONNECTOR_EOL_SERVICE

#define CONNECTOR_RVC
#define CONNECTOR_RVC_CAN_RX		GPIO_NUM_22 
#define CONNECTOR_RVC_CAN_TX		GPIO_NUM_21 
#define CONNECTOR_LIN_COMM_TEST

#if 1
/* UART for communication with EOL/PCBA Windows Application */
#define CONNECTOR_UART
#define CONNECTOR_UART_RX			GPIO_NUM_3
#define CONNECTOR_UART_TX			GPIO_NUM_1
#define CONNECTOR_UART_NUM			UART_NUM_1
#define CONNECTOR_UART_BUF_SIZE		256
#endif

#endif /* CONNECTOR_EOL_SERVICE */

#define BLE_GATT
#define BLE_GAP
#define DOMETIC_BLE_ID 0x0845

/* HAL layers */
#define HAL_GPIO
#define HAL_LEDC_PWM
#define HAL_PWM
#define HAL_I2C_MASTER

#include <math.h>

#define LEDC_PWM_DUTY_BIT_RESOLUTION   LEDC_TIMER_7_BIT  // LEDC_TIMER_13_BIT for the frequency of 5000 Hz

#define LEDC_PWM_MIN_DUTY_CYCLE     ((uint32_t)     0u)
#define LEDC_PWM_MAX_DUTY_CYCLE     ((uint32_t)  ((pow(2, LEDC_PWM_DUTY_BIT_RESOLUTION)) - 1))

#define LED_DIMMER_ON_DUTY_CYCLE      LEDC_PWM_MAX_DUTY_CYCLE
#define LED_DIMMER_OFF_DUTY_CYCLE     LEDC_PWM_MIN_DUTY_CYCLE

/* Temporarily min duty cycle set as 100 
  later based on testing this will set to a value that user can easily control the LCD during power saving mode */
#define ONBOARD_HMI_PWM_MIN_DUTY_CYCLE  LEDC_PWM_MAX_DUTY_CYCLE //2048

#define ONBOARD_HMI_MAX_DUTY_CYCLE           LEDC_PWM_MAX_DUTY_CYCLE //127 // 100% duty cycle  
#define ONBOARD_HMI_MIN_DUTY_CYCLE            38 // 30%  duty cycle
#define ONBOARD_HMI_OFF_DUTY_CYCLE             LEDC_PWM_MIN_DUTY_CYCLE //  0%  duty cycle


#define HAL_PWM_CAP_COUNTER_U32_MAX     (4294967295uL)

#define LCD_DRIVER_IC_VERSION_NEW
//#define LCD_DRIVER_IC_VERSION_OLD

#define DEVICE_UC1510C
#ifdef LCD_DRIVER_IC_VERSION_OLD
#define UC1510C_DRIVER_IC_SECONDARY_ADDR    ((uint8_t) 0x38u)
#else
#define UC1510C_DRIVER_IC_SECONDARY_ADDR    ((uint8_t) 0x39u)
#endif

#define FAIL              								((uint8_t) 1u)
#define SUCCESS                                        	((uint8_t) 0u)

#ifndef CONNECTOR_EOL_SERVICE

#define CONNECTOR_ONBOARD_HMI_PROCESS_STACK_DEPTH     	((unsigned short) 4096)
#define CONNECTOR_ONBOARD_HMI_CTRL_TASK_STACK_DEPTH		((unsigned short) 4096)
#define CONNECTOR_LIGHT_PROCESS_TASK_STACK_DEPTH		((unsigned short) 2048)
#define CONNECTOR_PWR_CTRL_BMS_TASK_STACK_DEPTH			((unsigned short) 4096)
#define CONNECTOR_FAN_MOTOR_PROCESS_STACK_DEPTH			((unsigned short) 4096)
#define CONNECTOR_FAN_MOTOR_TACHO_READ_STACK_DEPTH		((unsigned short) 4096)
#define CONNECTOR_VOC_PROCESS_TASK_STACK_DEPTH			((unsigned short) 4096)
#define CONNECTOR_VOC_I2C_RD_SERVICE_TASK_DEPTH			((unsigned short) 4096)
#define CONNECTOR_PWR_CTRL_PROCESS_TASK_DEPTH 			((unsigned short) 2048)
#define CONNECTOR_PWR_CTRL_MNGR_TASK_DEPTH				((unsigned short) 4096)
#define CONNECTOR_DIFFPRESS_PROCESS_STACK_DEPTH			((unsigned short) 2048)
#define CONNECTOR_DIFFPRESS_READ_STACK_DEPTH			((unsigned short) 4096)
#define CONNECTOR_EOL_PROCESS_STACK_DEPTH				((unsigned short) 4096)
#define CONNECTOR_EOL_CONTROL_STACK_DEPTH				((unsigned short) 4096)

#else

#define CONNECTOR_ONBOARD_HMI_PROCESS_STACK_DEPTH		((unsigned short) 2048)
#define CONNECTOR_ONBOARD_HMI_CTRL_TASK_STACK_DEPTH 	((unsigned short) 4096)
#define CONNECTOR_LIGHT_PROCESS_TASK_STACK_DEPTH   		((unsigned short) 2048)
#define CONNECTOR_PWR_CTRL_BMS_TASK_STACK_DEPTH			((unsigned short) 4096)
#define CONNECTOR_FAN_MOTOR_PROCESS_STACK_DEPTH 		((unsigned short) 2048)
#define CONNECTOR_FAN_MOTOR_TACHO_READ_STACK_DEPTH		((unsigned short) 2048)
#define CONNECTOR_VOC_PROCESS_TASK_STACK_DEPTH			((unsigned short) 2048)
#define CONNECTOR_VOC_I2C_RD_SERVICE_TASK_DEPTH			((unsigned short) 4096)
#define CONNECTOR_PWR_CTRL_PROCESS_TASK_DEPTH			((unsigned short) 2048)
#define CONNECTOR_PWR_CTRL_MNGR_TASK_DEPTH				((unsigned short) 4096)
#define CONNECTOR_DIFFPRESS_PROCESS_STACK_DEPTH			((unsigned short) 2048)
#define CONNECTOR_DIFFPRESS_READ_STACK_DEPTH			((unsigned short) 4096)
#define CONNECTOR_EOL_PROCESS_STACK_DEPTH				((unsigned short) 2048)
#define CONNECTOR_EOL_CONTROL_STACK_DEPTH				((unsigned short) 4096)

#endif

#define CONNECTOR_EOL_CTRL_TASK_PRIORITY				((unsigned short)    5u)
#define CONNECTOR_EOL_PROCESS_TASK_PRIORITY				((unsigned short)    6u)
#define CONNECTOR_FAN_MOTOR_PROCESS_TASK_PRIORITY		((unsigned short)    7u)
#define CONNECTOR_FAN_MOTOR_TACHO_READ_TASK_PRIORITY	((unsigned short)    8u)
#define CONNECTOR_LIGHT_PROCESS_TASK_PRIORITY			((unsigned short)   10u)
#define CONNECTOR_ONBOARD_HMI_PROCESS_PRIORITY			((unsigned short)   11u)
#define CONNECTOR_VOC_PROCESS_TASK_PRIO					((unsigned short)   12u) 
#define CONNECTOR_PWR_CTRL_SERV_TASK_PRIORITY			((unsigned short)   13u)
#define CONNECTOR_PWR_CTRL_MNGR_TASK_PRIORITY			((unsigned short)   14u)
#define CONNECTOR_PWR_CTRL_BMS_TASK_PRIORITY			((unsigned short)   15u)
#define CONNECTOR_ONBOARD_HMI_CTRL_TASK_PRIORITY		((unsigned short)   16u)
#define CONNECTOR_DIFFPRESS_PROCESS_TASK_PRIORITY		((unsigned short)   17u)
#define CONNECTOR_DIFFPRESS_READ_TASK_PRIORITY			((unsigned short)   18u)
#define CONNECTOR_FAN_MOTOR_CTRL_TASK_PRIORITY			((unsigned short)   19u)
#define CONNECTOR_VOC_I2C_RD_SERVICE_TASK_PRIO			((unsigned short)   20u)

#define CONNECTOR_ONBOARDHMI_PROCESS_TASK_NAME			((const char* const) "con_ohmi_pro_tk")
#define CONNECTOR_ONBOARDHMI_PWR_CTRL_TASK_NAME			((const char* const) "con_ohmi_pwc_tk")
#define CONNECTOR_LIGHT_PROCESS_TASK_NAME             	((const char* const) "con_ligh_pro_tk")
#define CONNECTOR_PWR_CTRL_BMS_TASK_NAME				((const char* const) "con_batt_mgm_tk")
#define CONNECTOR_FAN_MOTOR_PROCESS_TASK_NAME			((const char* const) "con_fanm_pro_tk")
#define CONNECTOR_FAN_MOTOR_TACHO_READ_TASK_NAME        ((const char* const) "con_tacho_rd_tk")
#define CONNECTOR_FAN_MOTOR_CONTROL_TASK_NAME			((const char* const) "con_fanm_ctr_tk")
#define CONNECTOR_VOC_SENSOR_PROCESS_TASK_NAME			((const char* const) "con_voc_proc_tk")
#define CONNECTOR_VOC_SENSOR_CONTROL_TASK_NAME  		((const char* const) "con_voc_ctrl_tk")
#define CONNECTOR_PWR_CTRL_SERV_PROCESS_TASK_NAME		((const char* const) "con_pwr_proc_tk")
#define CONNECTOR_PWR_CTRL_MANAGER_TASK_NAME     		((const char* const) "con_pwr_mngr_tk")
#define CONNECTOR_DIFF_PRESS_PROCESS_TASK_NAME			((const char* const) "con_dif_proc_tk")
#define CONNECTOR_DIFF_PRESS_READ_TASK_NAME				((const char* const) "con_dif_read_tk")
#define CONNECTOR_EOL_PROCESS_TASK_NAME					((const char* const) "conn_eol_pro_tk")
#define CONNECTOR_EOL_CONTROL_TASK_NAME					((const char* const) "conn_eol_ctr_tk")

#define I2C_MASTER0
#define I2C_MASTER0_PORT         	0
#define I2C_MASTER0_SDA				GPIO_NUM_4
#define I2C_MASTER0_SCL			    GPIO_NUM_0
#define I2C_MASTER0_FREQ			100000

//#define CAN_TESTING_INVENT
#define CAN1_EN(x) 
#define DEVICE_TWAI                 // Enable support for internal can
#define DEVICE_TWAI_EN(x)           CAN1_EN((x) ? 0 : 1)
#define DEVICE_TWAI_RX				GPIO_NUM_22
#define DEVICE_TWAI_TX				GPIO_NUM_21
#define DEVICE_TWAI_EN_PIN			GPIO_NUM_23



#ifndef USE_RS485
#define DE_VAL 0
#else
#define CONNECTOR_RS485
#define CONNECTOR_RS485_RX         GPIO_NUM_5
#define CONNECTOR_RS485_TX         GPIO_NUM_18
#define CONNECTOR_RS485_NUM        UART_NUM_2
#define CONNECTOR_RS485_BUF_SIZE   256
#define DE_VAL                     1
#endif

#ifndef USE_LIN
#define LIN_SLEEP_N                     0
#else

#define CONNECTOR_LIN_UART_RX			GPIO_NUM_5
#define CONNECTOR_LIN_UART_TX			GPIO_NUM_18
#define CONNECTOR_LIN_UART_NUM			UART_NUM_2
#define CONNECTOR_LIN_UART_BUF_SIZE		256
#define CONNECTOR_LINDEV_UART_RX		CONNECTOR_LIN_UART_RX
#define CONNECTOR_LINDEV_UART_TX		CONNECTOR_LIN_UART_TX
#define CONNECTOR_LINDEV_UART_NUM		CONNECTOR_LIN_UART_NUM
#define CONNECTOR_LINDEV_UART_DEV		UART2
#define LIN_SLEEP_N                     1
#endif

#define DEVICE_TCA9554A
#define DEVICE_TCA9554A_ADDRESS		        0x3F

#define DEVICE_TCA9554A_I2C_PORT	        I2C_MASTER0_PORT
#define DEVICE_TCA9554A_INT_PIN		        GPIO_NUM_36     // PIN SENSOR_VP
#define DEVICE_UC1510C_INT_PIN              GPIO_NUM_2      // LCD Driver IC interrupt pin

/* Battery charger IC */
#define DEVICE_BQ25792

#define ENABLE_BAT_SHIP                     1
#define DISABLE_BAT_SHIP                    0

#ifdef DEVICE_BQ25792
#define BAT_SHIP_VALUE                      ENABLE_BAT_SHIP
#else
#define BAT_SHIP_VALUE                      DISABLE_BAT_SHIP
#endif

#ifdef HAL_LEDC_PWM 

/* PWM Configuartion for LED Strip */
#define LED_STRIP_GPIO_PIN                  GPIO_NUM_19
#define LED_STRIP_PWM_SET_FREQUENCY         ((uint32_t)  20000u)

/* PWM Configuartion for HMI Backlight */
#define HMI_BACKLIGHT_GPIO_PIN              GPIO_NUM_23          
#define HMI_BACKLIGHT_PWM_SET_FREQUENCY     ((uint32_t)  20000u)

#endif

/* PWM Configuartion for FAN1 FAN2 and Motor */
#define MCPWM_UNIT_0_GPIO_PWM0A_OUT        GPIO_NUM_25    //Set GPIO 25 as PWM0A-> FAN_1
#define MCPWM_UNIT_0_GPIO_PWM0B_OUT        GPIO_NUM_26    //Set GPIO 26 as PWM0B -> FAN_2
#define MCPWM_UNIT_0_GPIO_PWM1A_OUT        GPIO_NUM_27    //Set GPIO 27 as PWM1A -> MOTOR

/* PWM Capture Configuartion for FAN1 FAN2 and Motor */
#define MCPWM_UNIT_0_GPIO_CAP0_IN          GPIO_NUM_34   // Set GPIO 34 as  CAP0 -> FAN_1
#define MCPWM_UNIT_0_GPIO_CAP1_IN          GPIO_NUM_39   // Set GPIO 39 as  CAP1 -> FAN_2  //PIN SENSOR_VN
#define MCPWM_UNIT_0_GPIO_CAP2_IN          GPIO_NUM_35   // Set GPIO 35 as  CAP2 -> MOTOR


typedef enum {
    IO_EX_GPIO_NUM_NC  = -1,     /*!< Use to signal not connected to S/W */
    IO_EX_GPIO_NUM_0   =  0,     /*!< IO_EXP_GPIO0, input and output     */
    IO_EX_GPIO_NUM_1   =  1,     /*!< IO_EXP_GPIO1, input and output     */
    IO_EX_GPIO_NUM_2   =  2,     /*!< IO_EXP_GPIO2, input and output     */
    IO_EX_GPIO_NUM_3   =  3,     /*!< IO_EXP_GPIO3, input and output     */
    IO_EX_GPIO_NUM_4   =  4,     /*!< IO_EXP_GPIO4, input and output     */
    IO_EX_GPIO_NUM_5   =  5,     /*!< IO_EXP_GPIO5, input and output     */
    IO_EX_GPIO_NUM_6   =  6,     /*!< IO_EXP_GPIO6, input and output     */
    IO_EX_GPIO_NUM_7   =  7,     /*!< IO_EXP_GPIO7, input and output     */
    IO_EX_GPIO_NUM_8   =  9,     /*!< IO_EXP_GPIO8, input and output     */
    IO_EX_GPIO_NUM_MAX
/** @endcond */
} io_exp_sgpio_num_t;

#define BOARD_INITIALIZATION
#ifdef HAL_GPIO

/*! \brief GPIO pin definition X macro, used as pin configuration database
	\param name		 Pin name definition
	\param device	 Device where physical pin exists \sa HAL_GPIO_DEVICE_ENUM
	\param port		 GPIO port of pin on device
	\param pin		 Pin number on GPIO port, 0 if not applicable
	\param pin_mode	 Pin operation mode \sa HAL_GPIO_PINMODE_ENUM
	\param level	 Initial level to initialize pin to
	\param interrupt mode	Interrupt mode setting of pin \sa HAL_GPIO_INTRMODE_ENUM
	\param callback	 Interrupt service routine callback for this pin
*/
#define	GPIO_PINS \
GPIO_PIN( 	   IO_EXP_INT,	        HAL_GPIO_DEVICE_ESP32,		    0,	       DEVICE_TCA9554A_INT_PIN,		 HAL_GPIO_PINMODE_READ,	               0,		    HAL_GPIO_INTRMODE_NEGEDGE,   battery_ic_interrupt_cb) \
GPIO_PIN(     LCD_INT_TSO,          HAL_GPIO_DEVICE_ESP32,		    0,	        DEVICE_UC1510C_INT_PIN,      HAL_GPIO_PINMODE_READ,                0,		    HAL_GPIO_INTRMODE_NEGEDGE,	onboard_hmi_interrupt_cb) \
GPIO_PIN(  LCD_RESET_RSTB,       HAL_GPIO_DEVICE_TCA9554A,		    0,	              IO_EX_GPIO_NUM_0,     HAL_GPIO_PINMODE_WRITE,                0,		    HAL_GPIO_INTRMODE_DISABLE,	                    NULL) \
GPIO_PIN(     EN_BAT_SHIP,	     HAL_GPIO_DEVICE_TCA9554A,		    0,	              IO_EX_GPIO_NUM_1,     HAL_GPIO_PINMODE_WRITE,   BAT_SHIP_VALUE,		    HAL_GPIO_INTRMODE_DISABLE,	                    NULL) \
GPIO_PIN( 	   EN_BAT_CHG,       HAL_GPIO_DEVICE_TCA9554A,		    0,	              IO_EX_GPIO_NUM_2,     HAL_GPIO_PINMODE_WRITE,                0,		    HAL_GPIO_INTRMODE_DISABLE,	                    NULL) \
GPIO_PIN(     INT_BAT_CHG,	     HAL_GPIO_DEVICE_TCA9554A,		    0,	              IO_EX_GPIO_NUM_3,      HAL_GPIO_PINMODE_READ,                0,		    HAL_GPIO_INTRMODE_DISABLE,	                    NULL) \
GPIO_PIN(      EN_IONIZER,	     HAL_GPIO_DEVICE_TCA9554A,		    0,	              IO_EX_GPIO_NUM_4,     HAL_GPIO_PINMODE_WRITE,                0,		    HAL_GPIO_INTRMODE_DISABLE,	                    NULL) \
GPIO_PIN( 	     EN_EFUSE,       HAL_GPIO_DEVICE_TCA9554A,		    0,	              IO_EX_GPIO_NUM_5,     HAL_GPIO_PINMODE_WRITE,                0,		    HAL_GPIO_INTRMODE_DISABLE,	                    NULL) \
GPIO_PIN( 	       EN_LIN,       HAL_GPIO_DEVICE_TCA9554A,		    0,	              IO_EX_GPIO_NUM_6,     HAL_GPIO_PINMODE_WRITE,      LIN_SLEEP_N,	        HAL_GPIO_INTRMODE_DISABLE,	                    NULL) \
GPIO_PIN( 	     EN_RS485,       HAL_GPIO_DEVICE_TCA9554A,		    0,	              IO_EX_GPIO_NUM_7,     HAL_GPIO_PINMODE_WRITE,           DE_VAL,		    HAL_GPIO_INTRMODE_DISABLE,	                    NULL) \
//					 name,			       device,				   port,	              pin,	                pin_mode,	                  level,	           interrupt mode,	                 INT CB function
#endif 

#ifdef HAL_LEDC_PWM

/*! \brief LEDC PWM Module configuration
	\param name				LED Pin Name
	\param gpio_num			GPIO number to which the genrated PWM signal is going to apply \sa gpio_int_type_t
	\param duty_resolution	Resolution of generated Duty
	\param freq(Hz)		    Frequency of PWM signal to be generated in Hz
	\param speed_mode	    Speed Mode \sa ledc_mode_t
	\param timer_num	    Timer Number \sa ledc_timer_t
	\param clk_cfg          Clock Configuration \sa ledc_clk_cfg_t
	\param channel	        Channel Number \sa ledc_channel_t
	\param duty	            duty
	\param hpoint	        high point 
*/      
#define LEDC_CONFIGURATION \
LEDC_PWM(light_dimmer,          LED_STRIP_GPIO_PIN,   LEDC_PWM_DUTY_BIT_RESOLUTION,     LED_STRIP_PWM_SET_FREQUENCY,   LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, LEDC_AUTO_CLK, LEDC_CHANNEL_0,    0,       0) \
LEDC_PWM(hmi_backlight,    HMI_BACKLIGHT_GPIO_PIN,   LEDC_PWM_DUTY_BIT_RESOLUTION, HMI_BACKLIGHT_PWM_SET_FREQUENCY,   LEDC_HIGH_SPEED_MODE, LEDC_TIMER_1, LEDC_AUTO_CLK, LEDC_CHANNEL_1,    0,       0) \
//              name,           gpio_num,              duty_resolution,        freq(HZ),                         speed_mode,       timer_num,     clk_cfg,        channel,       duty,  hpoint
#endif 

#endif