idf_component_register(SRCS 
                        "bno085.c"
                        "euler.c"
                        "sh2_SensorValue.c"
                        "sh2_util.c"
                        "sh2.c"
                        "shtp.c"
                        INCLUDE_DIRS "include"
                        PRIV_REQUIRES driver 
                        esp_timer)