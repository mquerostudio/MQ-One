idf_component_register(SRCS "bme280.c" "bme280_support.c"
                        PRIV_REQUIRES driver
                        INCLUDE_DIRS "include")
