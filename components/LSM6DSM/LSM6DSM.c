/*
   LSM6DSM.cpp: Implementation of LSM6DSM class

   Copyright (C) 2018 Simon D. Levy

   Adapted from https://github.com/kriswiner/LIS2MDL_LPS22HB

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

#include "LSM6DSM.h"

#include <math.h>

/*
LSM6DSM::LSM6DSM(Ascale_t ascale, Gscale_t gscale, Rate_t aodr, Rate_t godr,
                 float cal->_accelBias[3], float cal->_gyroBias[3]) {
  _ascale = ascale;
  _gscale = gscale;
  _aodr = aodr;
  _godr = godr;

  float areses[4] = {2, 16, 4, 8};
  _ares = areses[ascale] / 32768.f;

  float greses[4] = {245, 500, 1000, 2000};
  _gres = greses[gscale] / 32768.f;

  for (uint8_t k = 0; k < 3; ++k) {
    _cal->_accelBias[k] = cal->_accelBias[k];
    _cal->_gyroBias[k] = cal->_gyroBias[k];
  }
}

LSM6DSM::LSM6DSM(Ascale_t ascale, Gscale_t gscale, Rate_t aodr, Rate_t godr) {
  float cal->_accelBias[3] = {0, 0, 0};
  float cal->_gyroBias[3] = {0, 0, 0};
  LSM6DSM(ascale, gscale, aodr, godr, cal->_accelBias, cal->_gyroBias);
}
 */

Error_t lsm6dsm_init(calibration_t *cal) {
  // _i2c = cpi2c_open(ADDRESS, bus);
  float areses[4] = {2, 16, 4, 8};
  cal->_ares = areses[cal->_ascale] / 32768.f;

  float greses[4] = {245, 500, 1000, 2000};
  cal->_gres = greses[cal->_gscale] / 32768.f;

  if (readRegister(WHO_AM_I) != 0x6A) {
    printf("whoami error\n");
    return ERROR_ID;
  }

  calibrate(cal);

  // reset device
  uint8_t temp = readRegister(CTRL3_C);
  writeRegister(CTRL3_C, temp | 0x01);  // Set bit 0 to 1 to reset LSM6DSM
  vTaskDelay(pdMS_TO_TICKS(100));       // Wait for all registers to reset
  writeRegister(CTRL1_XL, cal->_aodr << 4 | cal->_ascale << 2);

  writeRegister(CTRL2_G, cal->_godr << 4 | cal->_gscale << 2);

  temp = readRegister(CTRL3_C);

  // enable block update (bit 6 = 1), auto-increment registers (bit 2 = 1)
  writeRegister(CTRL3_C, temp | 0x40 | 0x04);
  // by default, interrupts active HIGH, push pull, little endian data
  // (can be changed by writing to bits 5, 4, and 1, resp to above register)

  // enable accel LP2 (bit 7 = 1), set LP2 tp ODR/9 (bit 6 = 1), enable
  // input_composite (bit 3) for low noise
  writeRegister(CTRL8_XL, 0x80 | 0x40 | 0x08);

  // interrupt handling
  writeRegister(DRDY_PULSE_CFG, 0x80);  // latch interrupt until data read
  writeRegister(INT1_CTRL, 0x03);       // enable  data ready interrupts on INT1
  writeRegister(INT2_CTRL, 0x40);       // enable significant motion interrupts on INT2

  return selfTest(cal) ? ERROR_NONE : ERROR_SELFTEST;
}

void readData(imu_t *imu_instance, calibration_t *cal) {
  int16_t data[7];

  _readData(data);

  // Calculate the accleration value into actual g's
  imu_instance->a.x = (float)data[4] * cal->_ares - cal->_accelBias[0];  // get actual g value, this depends on scale being set
  imu_instance->a.y = (float)data[5] * cal->_ares - cal->_accelBias[1];
  imu_instance->a.z = (float)data[6] * cal->_ares - cal->_accelBias[2];

  // Calculate the gyro value into actual degrees per second
  imu_instance->g.x = (float)data[1] * cal->_gres - cal->_gyroBias[0];  // get actual gyro value, this depends on scale being set
  imu_instance->g.y = (float)data[2] * cal->_gres - cal->_gyroBias[1];
  imu_instance->g.z = (float)data[3] * cal->_gres - cal->_gyroBias[2];
}

bool checkNewData(void) {
  // use the gyro bit to check new data
  return (bool)(readRegister(STATUS_REG) & 0x02);
}

bool selfTest(calibration_t *cal) {
  int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
  int16_t accelPTest[3] = {0, 0, 0};
  int16_t accelNTest[3] = {0, 0, 0};
  int16_t gyroPTest[3] = {0, 0, 0};
  int16_t gyroNTest[3] = {0, 0, 0};
  int16_t accelNom[3] = {0, 0, 0};
  int16_t gyroNom[3] = {0, 0, 0};

  _readData(temp);
  accelNom[0] = temp[4];
  accelNom[1] = temp[5];
  accelNom[2] = temp[6];
  gyroNom[0] = temp[1];
  gyroNom[1] = temp[2];
  gyroNom[2] = temp[3];

  writeRegister(CTRL5_C, 0x01);    // positive accel self test
  vTaskDelay(pdMS_TO_TICKS(100));  // let accel respond
  _readData(temp);
  accelPTest[0] = temp[4];
  accelPTest[1] = temp[5];
  accelPTest[2] = temp[6];

  writeRegister(CTRL5_C, 0x03);    // negative accel self test
  vTaskDelay(pdMS_TO_TICKS(100));  // let accel respond
  _readData(temp);
  accelNTest[0] = temp[4];
  accelNTest[1] = temp[5];
  accelNTest[2] = temp[6];

  writeRegister(CTRL5_C, 0x04);    // positive gyro self test
  vTaskDelay(pdMS_TO_TICKS(100));  // let gyro respond
  _readData(temp);
  gyroPTest[0] = temp[1];
  gyroPTest[1] = temp[2];
  gyroPTest[2] = temp[3];

  writeRegister(CTRL5_C, 0x0C);    // negative gyro self test
  vTaskDelay(pdMS_TO_TICKS(100));  // let gyro respond
  _readData(temp);
  gyroNTest[0] = temp[1];
  gyroNTest[1] = temp[2];
  gyroNTest[2] = temp[3];

  writeRegister(CTRL5_C, 0x00);    // normal mode
  vTaskDelay(pdMS_TO_TICKS(100));  // let accel and gyro respond

  return (inBounds(accelPTest, accelNTest, accelNom, cal->_ares, ACCEL_MIN, ACCEL_MAX) &&
          inBounds(gyroPTest, gyroNTest, gyroNom, cal->_gres, GYRO_MIN, GYRO_MAX));
}

bool inBounds(int16_t ptest[3], int16_t ntest[3], int16_t nom[3], float res, float minval, float maxval) {
  for (uint8_t k = 0; k < 3; ++k) {
    if (outOfBounds((ptest[k] - nom[k]) * res, minval, maxval) || outOfBounds((ntest[k] - nom[k]) * res, minval, maxval)) {
      return false;
    }
  }

  return true;
}

bool outOfBounds(float val, float minval, float maxval) {
  val = fabs(val);
  return val < minval || val > maxval;
}

void calibrate(calibration_t *cal) {
  printf("calibrating...\n");
  int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
  int32_t sum[7] = {0, 0, 0, 0, 0, 0, 0};

  for (int ii = 0; ii < 128; ii++) {
    _readData(temp);
    sum[1] += temp[1];
    sum[2] += temp[2];
    sum[3] += temp[3];
    sum[4] += temp[4];
    sum[5] += temp[5];
    sum[6] += temp[6];
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  cal->_gyroBias[0] = sum[1] * cal->_gres / 128.0f;
  cal->_gyroBias[1] = sum[2] * cal->_gres / 128.0f;
  cal->_gyroBias[2] = sum[3] * cal->_gres / 128.0f;
  cal->_accelBias[0] = sum[4] * cal->_ares / 128.0f;
  cal->_accelBias[1] = sum[5] * cal->_ares / 128.0f;
  cal->_accelBias[2] = sum[6] * cal->_ares / 128.0f;

  if (cal->_accelBias[0] > 0.8f) {
    cal->_accelBias[0] -= 1.0f;
  }  // Remove gravity from the x-axis accelerometer bias calculation
  if (cal->_accelBias[0] < -0.8f) {
    cal->_accelBias[0] += 1.0f;
  }  // Remove gravity from the x-axis accelerometer bias calculation
  if (cal->_accelBias[1] > 0.8f) {
    cal->_accelBias[1] -= 1.0f;
  }  // Remove gravity from the y-axis accelerometer bias calculation
  if (cal->_accelBias[1] < -0.8f) {
    cal->_accelBias[1] += 1.0f;
  }  // Remove gravity from the y-axis accelerometer bias calculation
  if (cal->_accelBias[2] > 0.8f) {
    cal->_accelBias[2] -= 1.0f;
  }  // Remove gravity from the z-axis accelerometer bias calculation
  if (cal->_accelBias[2] < -0.8f) {
    cal->_accelBias[2] += 1.0f;
  }  // Remove gravity from the z-axis accelerometer bias calculation
}

void clearInterrupt(void) {
  int16_t data[7];
  _readData(data);
}

void _readData(int16_t destination[7]) {
  uint8_t rawData[14];                                       // x/y/z accel register data stored here
  readRegisters(OUT_TEMP_L, 14, &rawData[0]);                // Read the 14 raw data registers into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
  destination[3] = ((int16_t)rawData[7] << 8) | rawData[6];
  destination[4] = ((int16_t)rawData[9] << 8) | rawData[8];
  destination[5] = ((int16_t)rawData[11] << 8) | rawData[10];
  destination[6] = ((int16_t)rawData[13] << 8) | rawData[12];
}

uint8_t readRegister(uint8_t subAddress) {
  uint8_t temp;
  readRegisters(subAddress, 1, &temp);
  return temp;
}

void writeRegister(uint8_t subAddress, uint8_t data) {
  // cpi2c_writeRegister(_i2c, subAddress, data);
  uint8_t buf[2];
  buf[0] = subAddress;
  buf[1] = data;
  esp_err_t err = i2c_master_write_to_device(I2C_MASTER_NUM, ADDRESS, &buf, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
  // esp_err_t err = i2c_write_byte(I2C_MASTER_NUM, ADDRESS, subAddress, data);
  if (err != ESP_OK) {
    printf("write_byte_err: %d\n", err);
    while (1);
  }
}

void readRegisters(uint8_t subAddress, uint8_t count, uint8_t *dest) {
  // cpi2c_readRegisters(_i2c, subAddress, count, dest);
  uint8_t reg = subAddress;
  esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM, ADDRESS, &reg, 1, dest, count, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
  // esp_err_t err = i2c_read_bytes(I2C_MASTER_NUM, ADDRESS, subAddress, dest, count);
  if (err != ESP_OK) {
    printf("read_bytes_err: %d\n", err);
    while (1);
  }
}