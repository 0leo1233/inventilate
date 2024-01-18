cmake_minimum_required(VERSION 3.5)

project(DICMConnectorEOLService)

message(STATUS "Adding EOL Test Service Connector (${PROJECT_NAME})")

file(GLOB SRC_FILES CONFIGURE_DEPENDS "source/*.c")

add_library(${PROJECT_NAME} STATIC ${SRC_FILES})

target_include_directories(${PROJECT_NAME} 
                            PRIVATE
                                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/source>
                                ${DICM_FRAMEWORK_EXTRA_INTERNAL_INCLUDES}
                                ${DICM_FRAMEWORKCONNECTORS_EXTRA_INTERNAL_INCLUDES}
                            PUBLIC
                                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
							)
target_link_libraries(${PROJECT_NAME} PRIVATE DICMFramework DICMFrameworkConfiguration DICMConnectorPwrCtrlService DICMConnectorApp idf::esp_wifi PUBLIC idf::freertos)
target_link_libraries(${PROJECT_NAME} INTERFACE "-u connector_eol -u connector_lin_comm_test -u connector_rvc_eol") 