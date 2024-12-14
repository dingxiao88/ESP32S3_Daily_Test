set(srcs
    main.c
    speech_commands_action.c
    # es8388.c
    # iic.c
    # xl9555.c
    )

set(requires
    hardware_driver
    minimp3
    nvs_flash
    )

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS include
                       REQUIRES ${requires})

component_compile_options(-w)