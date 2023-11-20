cmake_minimum_required(VERSION 3.5)

project(DICMConnectorPwrCtrlService)

message(STATUS "Adding PWR Control Service Connector (${PROJECT_NAME})")

file(GLOB SRC_FILES CONFIGURE_DEPENDS "source/*.c")

add_library(${PROJECT_NAME} STATIC ${SRC_FILES})

target_include_directories(${PROJECT_NAME} 
                            PRIVATE
                                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/source>
                                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../app>
                            PUBLIC
                                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
							)
target_link_libraries(${PROJECT_NAME} PRIVATE DICMFramework DICMFrameworkConfiguration PUBLIC idf::freertos)
target_link_libraries(${PROJECT_NAME} INTERFACE "-u connector_pwr_ctrl_service")