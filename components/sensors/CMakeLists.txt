set(srcs "bmp280.c" "gps.c" "imu.c")
idf_component_register(SRCS "${srcs}" INCLUDE_DIRS "." REQUIRES bsp LSM6DSM ahrs)
