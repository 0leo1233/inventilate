# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)
if (CCACHE_ENABLE)
	find_program(CCACHE_FOUND ccache)
	if (CCACHE_FOUND)
		message(STATUS "ccache will be used for faster recompilation")
		set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
	else()
		message(WARNING "enabled ccache in build but ccache program not found")
	endif()
endif()

# Prepare to setup project
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/DICMFramework/tools/cmake)
include(framework_setup)

# Setup and check depending environment and cmake variables, Replace with esp32s3 for other target
setup_target()

#inventilate
project(INV C CXX ASM)

# Setup project
#Include storage partition in full binary, address see partition table
#Include hmi_data partition in full binary, address see partition table
setup_project()

# Build all
build_project()