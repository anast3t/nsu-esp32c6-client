idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES bt esp_wifi esp_netif nvs_flash lwip led_strip esp_driver_gpio)
