# ESP32-C3 BLE HID -> USB HID Bridge
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32c3_bl_hid)
