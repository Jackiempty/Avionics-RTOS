set(srcs "LSM6DSM.c")
idf_component_register(SRCS "${srcs}" INCLUDE_DIRS "." REQUIRES driver bsp)