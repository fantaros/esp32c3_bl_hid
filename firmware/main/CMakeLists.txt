idf_component_register(SRCS
    "app_main.c"
    "usb_device.c"
    "hid_report.c"
    "ble_hid.c"
    "cdc_ctrl.c"
    "keymap.c"
    INCLUDE_DIRS "include"
    REQUIRES usb nimble esp_timer esp_hw_support freertos cJSON nvs_flash
)
