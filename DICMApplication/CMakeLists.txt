cmake_minimum_required(VERSION 3.5)

add_compile_definitions(${EXTRA_COMPILER_DEFINITIONS})
add_compile_options(${EXTRA_COMPILER_OPTIONS})

if (CONFIG_CONNECTOR_ONBOARD_HMI)
add_compile_definitions(CONNECTOR_ONBOARD_HMI)
list(APPEND ConnectorLIBS DICMConnectorOnboardHmi)
add_subdirectory(onboardhmi)
endif()
if (CONFIG_CONNECTOR_VOC_SENSOR)
add_compile_definitions(CONNECTOR_VOC_SENSOR)
list(APPEND ConnectorLIBS DICMConnectorVocSensor)
add_subdirectory(voc_sensor)
endif()
if (CONFIG_CONNECTOR_PWR_CTRL_SERVICE)
add_compile_definitions(CONNECTOR_POWER_CONTROL_SERVICE)
list(APPEND ConnectorLIBS DICMConnectorPwrCtrlService)
add_subdirectory(pwr_ctrl_service)
endif()
if (CONFIG_CONNECTOR_PWM_FAN_MOTOR)
add_compile_definitions(CONNECTOR_PWM_FAN_MOTOR)
list(APPEND ConnectorLIBS DICMConnectorPwmFanMotor)
add_subdirectory(pwm_fan_motor)
endif()

project(DICMApplication)

file(GLOB SRC_FILES CONFIGURE_DEPENDS "app/*.c" )

file(TOUCH ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}dummy.c)
add_library(${PROJECT_NAME} STATIC ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}dummy.c)
add_library(${PROJECT_NAME}Obj OBJECT ${SRC_FILES})

#set(CMAKE_C_COMPILE_OBJECT "${CMAKE_C_COMPILE_OBJECT} -Wa,-alh=<OBJECT>.lst")
target_include_directories(${PROJECT_NAME} 
							PUBLIC	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
							)
target_include_directories(${PROJECT_NAME}Obj 
                            PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                            )
target_link_libraries(${PROJECT_NAME}Obj PRIVATE idf::esp32 DICMFramework DICMFrameworkConfiguration)
target_link_libraries(${PROJECT_NAME}Obj PUBLIC ${ConnectorLIBS})
target_link_libraries(${PROJECT_NAME} PRIVATE idf::esp32 DICMFramework DICMFrameworkConfiguration)
target_link_libraries(${PROJECT_NAME} PUBLIC ${ConnectorLIBS} $<TARGET_OBJECTS:${PROJECT_NAME}Obj>)
