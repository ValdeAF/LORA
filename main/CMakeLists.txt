idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_common esp_driver_gpio esp_driver_spi dernasherbrezon__sx127x
)