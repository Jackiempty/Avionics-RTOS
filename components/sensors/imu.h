#ifndef __IMU_H__
#define __IMU_H__

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#include "LSM6DSM.h"
#include "MadgwickAHRS.h"
#include "esp_system.h"

#define DEG2RAD(deg) (deg * M_PI / 180.0f)

void imu_init();
void imu_update();
imu_t* imu_fetch();

#endif
