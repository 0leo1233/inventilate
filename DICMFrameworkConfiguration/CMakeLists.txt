cmake_minimum_required(VERSION 3.5)

project(DICMFrameworkConfiguration)

file(GLOB SRC_FILES CONFIGURE_DEPENDS "source/*.c" )

file(GLOB APP_SRC_FILES CONFIGURE_DEPENDS "source/app/*.c" )
add_compile_definitions(${EXTRA_COMPILER_DEFINITIONS})
add_compile_options(${EXTRA_COMPILER_OPTIONS})

add_library(${PROJECT_NAME} STATIC ${SRC_FILES} ${APP_SRC_FILES})
#set(CMAKE_C_COMPILE_OBJECT "${CMAKE_C_COMPILE_OBJECT} -Wa,-alh=<OBJECT>.lst")
target_include_directories(${PROJECT_NAME} 
							PRIVATE
								$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/source>
							PUBLIC
								$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
								$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/source/app>
							)
target_link_libraries(${PROJECT_NAME} PRIVATE idf::esp32 idf::nvs_flash DICMOsal DICMHal DICMCommonLibs DICMGeneralDefinitions DICMConnectors)
