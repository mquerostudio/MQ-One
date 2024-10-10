#Add sources from ui directory
file(GLOB_RECURSE SRC_UI ${CMAKE_SOURCE_DIR} "ui/*.c")

idf_component_register(SRCS
    "main.c"
    "i2c_driver.c"
    "lv_port_disp.c"
    "lv_port_indev.c"
    "modules_driver.c"
    ${SRC_UI}
    INCLUDE_DIRS "."
    "ui")