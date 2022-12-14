cmake_minimum_required(VERSION 3.5)
#Need to make sure to add project specific paths where the actual data is
idf_component_register(SRC_DIRS "." 
					   INCLUDE_DIRS "." 
					   PRIV_REQUIRES hal_fw main drv lib
					   REQUIRES app
					  )
#Adding extra build define from local config
if (EXTRA_BUILD_DEFINES)
    component_compile_definitions(${EXTRA_BUILD_DEFINES_STRING})
endif()
set_property(TARGET ${COMPONENT_LIB} APPEND PROPERTY LINK_INTERFACE_MULTIPLICITY 5)

idf_component_get_property(dir configuration COMPONENT_DIR)
component_compile_options(-fmacro-prefix-map=${dir}=configuration)
