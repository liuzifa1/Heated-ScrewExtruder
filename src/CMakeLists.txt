if("$ENV{PIOENV}" STREQUAL "wifi_scan_debug")
    set(APP_SOURCE "wifi_scan_debug.c")
else()
    set(APP_SOURCE "main.c")
endif()


idf_component_register(SRCS "${APP_SOURCE}"
                       INCLUDE_DIRS ".")
