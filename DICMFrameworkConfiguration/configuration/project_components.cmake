# Add which extra copmonents to build
set (EXTRA_IDF_COMPONENTS_NAMES "aws_iot")
set (EXTRA_IDF_COMPONENTS_PATHS "DICMFramework/components/aws_iot")
set (IOT_LIBRARY_NAME idf::aws_iot)
#option to set path of SPIFFS data image
#set (CONFIG_SPIFFS_SOURCE_IMAGE_PATH ${CMAKE_BINARY_DIR}/image)
