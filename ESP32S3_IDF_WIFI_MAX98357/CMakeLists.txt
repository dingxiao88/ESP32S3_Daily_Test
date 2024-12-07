# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# 在 project() 之前添加分区表
set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/components/spiffs)

project(ESP32S3_IDF_WIFI_MAX98357)

# 在 project() 之后添加 SPIFFS 镜像
spiffs_create_partition_image(storage data FLASH_IN_PROJECT)