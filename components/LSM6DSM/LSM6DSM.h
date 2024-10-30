/*
   LSM6DSM.h: Header file for LSM6DSM class

   Copyright (C) 2018 Simon D. Levy

   Adapted from https://github.com/kriswiner/LSM6DSM_LIS2MDL_LPS22HB

   This file is part of LSM6DSM.

   LSM6DSM is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   LSM6DSM is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with LSM6DSM.  If not, see <http://www.gnu.org/licenses/>.

   from https://github.com/simondlevy/LSM6DSM/blob/master/src/LSM6DSM.h
   Modified by Jackiempty
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "bsp.h"

// One ifdef needed to support delay() cross-platform
#if defined(ARDUINO)
#include <Arduino.h>

#elif defined(__arm__)
#if defined(STM32F303) || defined(STM32F405xx)
extern "C" {
void delay(uint32_t msec);
}
#else
#include <wiringPi.h>
#endif
#endif

#define ADDRESS (0x6B)
#define _ADDRESS (0xD7)

/* LSM6DSM registers
http://www.st.com/content/ccc/resource/technical/document/datasheet/76/27/cf/88/c5/03/42/6b/DM00218116.pdf/files/DM00218116.pdf/jcr:content/translations/en.DM00218116.pdf
 */
#define FUNC_CFG_ACCESS (0x01)
#define SENSOR_SYNC_TIME_FRAME (0x04)
#define SENSOR_SYNC_RES_RATIO (0x05)
#define FIFO_CTRL1 (0x06)
#define FIFO_CTRL2 (0x07)
#define FIFO_CTRL3 (0x08)
#define FIFO_CTRL4 (0x09)
#define FIFO_CTRL5 (0x0A)
#define DRDY_PULSE_CFG (0x0B)
#define INT1_CTRL (0x0D)
#define INT2_CTRL (0x0E)
#define WHO_AM_I (0x0F)
#define CTRL1_XL (0x10)
#define CTRL2_G (0x11)
#define CTRL3_C (0x12)
#define CTRL4_C (0x13)
#define CTRL5_C (0x14)
#define CTRL6_C (0x15)
#define CTRL7_G (0x16)
#define CTRL8_XL (0x17)
#define CTRL9_XL (0x18)
#define CTRL10_C (0x19)
#define MASTER_CONFIG (0x1A)
#define WAKE_UP_SRC (0x1B)
#define TAP_SRC (0x1C)
#define D6D_SRC (0x1D)
#define STATUS_REG (0x1E)
#define OUT_TEMP_L (0x20)
#define OUT_TEMP_H (0x21)
#define OUTX_L_G (0x22)
#define OUTX_H_G (0x23)
#define OUTY_L_G (0x24)
#define OUTY_H_G (0x25)
#define OUTZ_L_G (0x26)
#define OUTZ_H_G (0x27)
#define OUTX_L_XL (0x28)
#define OUTX_H_XL (0x29)
#define OUTY_L_XL (0x2A)
#define OUTY_H_XL (0x2B)
#define OUTZ_L_XL (0x2C)
#define OUTZ_H_XL (0x2D)
#define SENSORHUB1_REG (0x2E)
#define SENSORHUB2_REG (0x2F)
#define SENSORHUB3_REG (0x30)
#define SENSORHUB4_REG (0x31)
#define SENSORHUB5_REG (0x32)
#define SENSORHUB6_REG (0x33)
#define SENSORHUB7_REG (0x34)
#define SENSORHUB8_REG (0x35)
#define SENSORHUB9_REG (0x36)
#define SENSORHUB10_REG (0x37)
#define SENSORHUB11_REG (0x38)
#define SENSORHUB12_REG (0x39)
#define FIFO_STATUS1 (0x3A)
#define FIFO_STATUS2 (0x3B)
#define FIFO_STATUS3 (0x3C)
#define FIFO_STATUS4 (0x3D)
#define FIFO_DATA_OUT_L (0x3E)
#define FIFO_DATA_OUT_H (0x3F)
#define TIMESTAMP0_REG (0x40)
#define TIMESTAMP1_REG (0x41)
#define TIMESTAMP2_REG (0x42)
#define STEP_TIMESTAMP_L (0x49)
#define STEP_TIMESTAMP_H (0x4A)
#define STEP_COUNTER_L (0x4B)
#define STEP_COUNTER_H (0x4C)
#define SENSORHUB13_REG (0x4D)
#define SENSORHUB14_REG (0x4E)
#define SENSORHUB15_REG (0x4F)
#define SENSORHUB16_REG (0x50)
#define SENSORHUB17_REG (0x51)
#define SENSORHUB18_REG (0x52)
#define FUNC_SRC1 (0x53)
#define FUNC_SRC2 (0x54)
#define WRIST_TILT_IA (0x55)
#define TAP_CFG (0x58)
#define TAP_THS_6D (0x59)
#define INT_DUR2 (0x5A)
#define WAKE_UP_THS (0x5B)
#define WAKE_UP_DUR (0x5C)
#define FREE_FALL (0x5D)
#define MD1_CFG (0x5E)
#define MD2_CFG (0x5F)
#define MASTER_MODE_CODE (0x60)
#define SENS_SYNC_SPI_ERROR_CODE (0x61)
#define OUT_MAG_RAW_X_L (0x66)
#define OUT_MAG_RAW_X_H (0x67)
#define OUT_MAG_RAW_Y_L (0x68)
#define OUT_MAG_RAW_Y_H (0x69)
#define OUT_MAG_RAW_Z_L (0x6A)
#define OUT_MAG_RAW_Z_H (0x6B)
#define INT_OIS (0x6F)
#define CTRL1_OIS (0x70)
#define CTRL2_OIS (0x71)
#define CTRL3_OIS (0x72)
#define X_OFS_USR (0x73)
#define Y_OFS_USR (0x74)
#define Z_OFS_USR (0x75)

// Self-test bounds
#define ACCEL_MIN (0.09)
#define ACCEL_MAX (1.7)
#define GYRO_MIN (20.0f)
#define GYRO_MAX (80.0f)

typedef enum {

  // Note order!
  AFS_2G,
  AFS_16G,
  AFS_4G,
  AFS_8G

} Ascale_t;

typedef enum {

  GFS_245DPS,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS

} Gscale_t;

typedef enum {

  ODR_12_5Hz,
  ODR_26Hz,
  ODR_52Hz,
  ODR_104Hz,
  ODR_208Hz,
  ODR_416Hz,
  ODR_833Hz,
  ODR_1660Hz,
  ODR_3330Hz,
  ODR_6660Hz

} Rate_t;

typedef enum {

  ERROR_NONE,
  ERROR_CONNECT,
  ERROR_ID,
  ERROR_SELFTEST

} Error_t;

typedef struct {
  float x, y, z;
} vector_t;

typedef struct {
  Ascale_t _ascale;
  Gscale_t _gscale;
  Rate_t _aodr;
  Rate_t _godr;
  float _ares;
  float _gres;
  float _accelBias[3];
  float _gyroBias[3];
} calibration_t;

typedef struct {
  uint32_t freq;
  vector_t a, g;
  vector_t velocity;
  float heading, pitch, roll;
} imu_t;

// Public
Error_t lsm6dsm_init(calibration_t *);

void calibrate(calibration_t *);

void clearInterrupt(void);

bool checkNewData(void);

void readData(imu_t *, calibration_t *);

bool selfTest(calibration_t *);

void _readData(int16_t data[7]);

// Private
// Self-test helpers
bool inBounds(int16_t ptest[3], int16_t ntest[3], int16_t nom[3], float res, float minval, float maxval);
bool outOfBounds(float val, float minval, float maxval);

void writeRegister(uint8_t subAddress, uint8_t data);
uint8_t readRegister(uint8_t subAddress);
void readRegisters(uint8_t subAddress, uint8_t count, uint8_t *dest);
