# Inventilate build

list(APPEND EXTRA_COMPONENT_DIRS "../../../configuration" "../../../app")
set(IDF_VERSION_CHECK TRUE)
set(IDF_VERSION_CHECK_STRING "4.4.2")
set(EXTRA_BUILD_DEFINES TRUE)
list(APPEND EXTRA_BUILD_DEFINES_STRING "-DCONFIG_DICM_SUPPORT_INTEGRATED_BSEC_LIB_2_X=1")
set(USE_BSEC_2 TRUE)
set(CONFIG_DICM_SUPPORT_INTEGRATED_BSEC_LIB_2_X TRUE)
#set(EXTRA_BUILD_DEFINES_STRING "-DCONFIG_DICM_SUPPORT_INTEGRATED_BSEC_LIB_1_X=1")
#set(USE_BSEC_1 TRUE)
#set(CONFIG_DICM_SUPPORT_INTEGRATED_BSEC_LIB_1_X TRUE)

set(PROJECT_VER "1.0.5-rc.1")
