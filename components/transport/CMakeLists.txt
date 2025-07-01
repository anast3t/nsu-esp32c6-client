set(TRANSPORT_SRCS)

message(STATUS ">>> CONFIG_PROTO_BT_ENABLED A = ${CONFIG_PROTO_BT_ENABLED}")

# К Wi-Fi или к BLE добавляем свой файл
if(CONFIG_PROTO_BT_ENABLED)
    list(APPEND TRANSPORT_SRCS src/transport_ble_client.c)
else()
    list(APPEND TRANSPORT_SRCS src/transport_wifi_client.c)
endif()

idf_component_register(
    SRCS
        ${TRANSPORT_SRCS}
    INCLUDE_DIRS
        include
    REQUIRES
        bt 
        led_ctrl
        esp_wifi 
        esp_netif 
        nvs_flash 
        lwip 
)