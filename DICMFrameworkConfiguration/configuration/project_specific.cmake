# Project configuration
# Add changes compared to default config DICMFramework/FrameworkDefaults.cmake

#option(CONFIG_CONNECTOR_BLE "Enable to include BLE connector" OFF)
#option(CONFIG_CONNECTOR_WIFI "Enable to include WIFI connector" OFF)
set(CONFIG_CONNECTOR_RVC OFF CACHE BOOL "Enable to include RVC connector" FORCE)
#option(CONFIG_CONNECTOR_MQTT "Enable to include MQTT connector" OFF)
#option(CONFIG_CONNECTOR_RS485 "Enable to include RS485 connector" ON)
set(CONFIG_CONNECTOR_PREM_RVC OFF CACHE BOOL "Enable to include PREM_RVC connector" FORCE)
set(CONFIG_CONNECTOR_RVC OFF CACHE BOOL "Enable to include RVC connector" FORCE)
#option(CONFIG_CONNECTOR_SYSTEM "Enable to include SYSTEM connector" ON)
#option(CONFIG_CONNECTOR_USM "Enable to include USM connector" OFF)
#option(CONFIG_CONNECTOR_US_DICM "Enable to include US_DICM connector" OFF)

# DICMApplication connectors/services
set(CONFIG_CONNECTOR_ONBOARD_HMI ON CACHE BOOL "Enable to include OnboardHMI connector" FORCE)
set(CONFIG_CONNECTOR_VOC_SENSOR ON CACHE BOOL "Enable VOC Sensor connector" FORCE)
set(CONFIG_CONNECTOR_PWM_FAN_MOTOR ON CACHE BOOL "Enable PWM Fan and Motor Control connector" FORCE)
set(CONFIG_CONNECTOR_PWR_CTRL_SERVICE ON CACHE BOOL "Enable PWR Control Service connector" FORCE)
set(CONFIG_CONNECTOR_LIGHT ON CACHE BOOL "Enable Light connector" FORCE)
set(EOL_BUILD FALSE CACHE BOOL "" FORCE)
if (EOL_BUILD)
    set(CONFIG_CONNECTOR_LINDEV OFF CACHE BOOL "Enable to include LINDEV connector" FORCE)
    set(CONFIG_CONNECTOR_UART ON CACHE BOOL "Enable to include UART connector" FORCE)
    set(CONFIG_CONNECTOR_DP_SENS_SERVICE OFF CACHE BOOL "Enable Diff Pressure Sensor Service connector" FORCE)
    set(CONFIG_CONNECTOR_EOL_SERVICE ON CACHE BOOL "Enable EOL service connector" FORCE)
else()
    set(CONFIG_CONNECTOR_LINDEV ON CACHE BOOL "Enable to include LINDEV connector" FORCE)
    set(CONFIG_CONNECTOR_DP_SENS_SERVICE ON CACHE BOOL "Enable Diff Pressure Sensor Service connector" FORCE)
    set(CONFIG_CONNECTOR_EOL_SERVICE OFF CACHE BOOL "Enable EOL service connector" FORCE)
    set(CONFIG_CONNECTOR_UART OFF CACHE BOOL "Enable to include UART connector" FORCE)
endif()

# Add options that need to affect all libraries

set(USE_BSEC_2 TRUE CACHE BOOL "" FORCE)

list(APPEND EXTRA_COMPILER_DEFINITIONS "LCD_DRIVER_IC_INIT_VERSION_200623" "UC1510C_DRIVER_IC_SECONDARY_ADDR=0x39" "DEVICE_UC1510C" "CONFIG_DICM_SUPPORT_INTEGRATED_BSEC_LIB_2_X=1")
list(APPEND EXTRA_COMPILER_DEFINITIONS "DEVICE_BQ25798")
list(APPEND EXTRA_COMPILER_DEFINITIONS "DEVICE_LIS2DH12")
if (EOL_BUILD)
    list(APPEND EXTRA_COMPILER_DEFINITIONS "INVENT_EOL_TESTING")
endif()