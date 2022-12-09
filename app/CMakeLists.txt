cmake_minimum_required(VERSION 3.5)
#Need to make sure to add project specific paths where the actual data is
idf_component_register(SRC_DIRS "." 
					   INCLUDE_DIRS "." 
					   PRIV_REQUIRES main hal_fw osal drv lib
					  )
#Adding extra build define from local config
if (EXTRA_BUILD_DEFINES)
    #idf_build_set_property(COMPILE_DEFINITIONS ${EXTRA_BUILD_DEFINES_STRING} APPEND)
    component_compile_options(${EXTRA_BUILD_DEFINES_STRING})
endif()
idf_component_get_property(dir app COMPONENT_DIR)
component_compile_options(-fmacro-prefix-map=${dir}=app)
