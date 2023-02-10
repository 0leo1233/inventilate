# Project configuration
# Add changes compared to default config DICMFramework/FrameworkDefaults.cmake

#option(CONFIG_CONNECTOR_BLE "Enable to include BLE connector" OFF)
#option(CONFIG_CONNECTOR_WIFI "Enable to include WIFI connector" OFF)
#option(CONFIG_CONNECTOR_UART "Enable to include UART connector" OFF)
option(CONFIG_CONNECTOR_LINDEV "Enable to include LINDEV connector" OFF)
#option(CONFIG_CONNECTOR_MQTT "Enable to include MQTT connector" OFF)
#option(CONFIG_CONNECTOR_RS485 "Enable to include RS485 connector" ON)
option(CONFIG_CONNECTOR_PREM_RVC "Enable to include PREM_RVC connector" OFF)
#option(CONFIG_CONNECTOR_SYSTEM "Enable to include SYSTEM connector" ON)
#option(CONFIG_CONNECTOR_USM "Enable to include USM connector" OFF)
#option(CONFIG_CONNECTOR_US_DICM "Enable to include US_DICM connector" OFF)

# DICMApplication connectors/services
option(CONFIG_CONNECTOR_ONBOARD_HMI "Enable to include OnboardHMI connector" ON)
option(CONFIG_CONNECTOR_VOC_SENSOR "Enable VOC Sensor connector" ON)
option(CONFIG_CONNECTOR_PWM_FAN_MOTOR "Enable PWM Fan and Motor Control connector" ON)
option(CONFIG_CONNECTOR_PWR_CTRL_SERVICE "Enable PWR Control Service connector" ON)
option(CONFIG_CONNECTOR_DP_SENS_SERVICE "Enable Diff Pressure Sensor Service connector" ON)
option(CONFIG_CONNECTOR_LIGHT "Enable Light connector" ON)

# Add options that need to affect all libraries

set(USE_BSEC_2 TRUE CACHE BOOL "" FORCE)

list (APPEND EXTRA_COMPILER_DEFINITIONS "LCD_DRIVER_IC_VERSION_NEW" "UC1510C_DRIVER_IC_SECONDARY_ADDR=0x39" "DEVICE_BQ25792" "DEVICE_UC1510C" "CONFIG_DICM_SUPPORT_INTEGRATED_BSEC_LIB_2_X=1")
list (APPEND EXTRA_COMPILER_DEFINITIONS "DEVICE_LIS2DH12")