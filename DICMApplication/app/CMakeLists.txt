cmake_minimum_required(VERSION 3.5)

project(DICMConnectorApp)

message(STATUS "Adding App (Inventilate) library (${PROJECT_NAME})")

file(GLOB SRC_FILES CONFIGURE_DEPENDS "*.c")

add_library(${PROJECT_NAME} STATIC ${SRC_FILES})

target_include_directories(${PROJECT_NAME} 
                            PRIVATE
                                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
                            PUBLIC
                                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
							)
target_link_libraries(${PROJECT_NAME} PRIVATE DICMFramework DICMFrameworkConfiguration DICMConnectorVocSensor PUBLIC idf::freertos)
target_link_libraries(${PROJECT_NAME} INTERFACE "-u g_bsec_lib_intf") 