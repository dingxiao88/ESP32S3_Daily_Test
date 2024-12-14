cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS
    ../../components

    # 在 project() 之前添加分区表
    # $ENV{IDF_PATH}/components/spiffs
    )

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(speech_commands_recognition)

# 在 project() 之后添加 SPIFFS 镜像
# spiffs_create_partition_image(storage data FLASH_IN_PROJECT)


