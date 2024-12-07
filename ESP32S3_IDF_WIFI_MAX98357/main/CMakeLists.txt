idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS ".")

# set(COMPONENT_REQUIRES audio_pipeline audio_stream esp_peripherals)
# set(COMPONENT_PRIV_REQUIRES audio_hal esp-adf-libs)

# idf_component_register(
#     SRCS "main.c"
#     INCLUDE_DIRS "."
#     REQUIRES ${COMPONENT_REQUIRES}
#     PRIV_REQUIRES ${COMPONENT_PRIV_REQUIRES}
# )