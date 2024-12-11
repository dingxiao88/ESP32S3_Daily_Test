set(srcs
    main.c
    iic.c
    xl9555.c
    )

idf_component_register(SRCS ${srcs}
                    # INCLUDE_DIRS "."
                    INCLUDE_DIRS include
                    )

# set(COMPONENT_REQUIRES audio_pipeline audio_stream esp_peripherals)
# set(COMPONENT_PRIV_REQUIRES audio_hal esp-adf-libs)

# idf_component_register(
#     SRCS "iic.c" "xl9555.c" "main.c"
#     INCLUDE_DIRS "."
#     REQUIRES ${COMPONENT_REQUIRES}
#     PRIV_REQUIRES ${COMPONENT_PRIV_REQUIRES}
# )