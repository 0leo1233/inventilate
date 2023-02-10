cmake_minimum_required(VERSION 3.5)
cmake_minimum_required(VERSION 3.5)

project(AppConfigCode)

file(GLOB SRC_FILES CONFIGURE_DEPENDS "*.c" )

add_library(${PROJECT_NAME} STATIC ${SRC_FILES})
set(CMAKE_C_COMPILE_OBJECT "${CMAKE_C_COMPILE_OBJECT} -Wa,-alh=<OBJECT>.lst")
target_include_directories(${PROJECT_NAME} 
							PUBLIC
								$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
							)
target_link_libraries(${PROJECT_NAME} PRIVATE idf::esp32 DICMHal DICMCommonLibs DICMGeneralDefinitions DICMConnectors)
