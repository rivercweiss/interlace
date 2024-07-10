
#include <Arduino.h>
#include "MPU9250.h"

MPU9250 mpu; // You can also use MPU9255 as is

void setup_sensors(void)
{
  MPU9250Setting setting;
  setting.accel_fs_sel = ACCEL_FS_SEL::A2G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G250DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_1000HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_5HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_5HZ;

  mpu.setup(0x68); // change to your own address

  mpu.setFilterIterations(100);
}

bool init_sensors(void)
{
  if (!mpu.isConnectedMPU9250())
  {
    return false;
  }
  else
  {
    return true;
  }
}

// void setup()
// {
//     Serial.begin(9600);
//     Wire.begin(I2C_SDA, I2C_SCL);
//     delay(2000);

//     MPU9250Setting setting;
//     setting.accel_fs_sel = ACCEL_FS_SEL::A2G;
//     setting.gyro_fs_sel = GYRO_FS_SEL::G250DPS;
//     setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
//     setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_1000HZ;
//     setting.gyro_fchoice = 0x03;
//     setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_5HZ;
//     setting.accel_fchoice = 0x01;
//     setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_5HZ;

//     mpu.setup(0x68); // change to your own address

//     mpu.setFilterIterations(100);

//     // mpu.calibrateAccelGyro();
//     mpu.calibrateMag();
// }

// void loop()
// {
//     if (mpu.update())
//     {
//         // Serial.print(mpu.getYaw());
//         // Serial.print(", ");
//         // Serial.print(mpu.getPitch());
//         // Serial.print(", ");
//         // Serial.println(mpu.getRoll());

//         Serial.print(mpu.getEulerX());
//         Serial.print(", ");
//         Serial.print(mpu.getEulerY());
//         Serial.print(", ");
//         Serial.println(mpu.getEulerZ());
//     }
// }

// #include <Adafruit_LIS3MDL.h>
// Adafruit_LIS3MDL lis3mdl;

// // Can change this to be LSM6DSOX or whatever ya like
// // For (older) Feather Sense with LSM6DS33, use this:
// #include <Adafruit_LSM6DS33.h>
// Adafruit_LSM6DS33 lsm6ds;
// // For (newer) Feather Sense with LSM6DS3TR-C, use this:
// //#include <Adafruit_LSM6DS3TRC.h>
// // Adafruit_LSM6DS3TRC lsm6ds;

// bool init_sensors(void) {
//   if (!lsm6ds.begin_I2C() || !lis3mdl.begin_I2C()) {
//     return false;
//   }
//   accelerometer = lsm6ds.getAccelerometerSensor();
//   gyroscope = lsm6ds.getGyroSensor();
//   magnetometer = &lis3mdl;

//   return true;
// }

// void setup_sensors(void) {
//   // set lowest range
//   lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
//   lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
//   lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

//   // set slightly above refresh rate
//   lsm6ds.setAccelDataRate(LSM6DS_RATE_104_HZ);
//   lsm6ds.setGyroDataRate(LSM6DS_RATE_104_HZ);
//   lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
//   lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
//   lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
// }

// #include <Arduino.h>
// #include "MPU9250.h"
// #include <Wire.h>

// MPU9250 mpu; // You can also use MPU9255 as is

// #define I2C_SDA 27
// #define I2C_SCL 26

// void setup()
// {
//     Serial.begin(9600);
//     Wire.begin(I2C_SDA, I2C_SCL);
//     delay(2000);

//     MPU9250Setting setting;
//     setting.accel_fs_sel = ACCEL_FS_SEL::A2G;
//     setting.gyro_fs_sel = GYRO_FS_SEL::G250DPS;
//     setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
//     setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_1000HZ;
//     setting.gyro_fchoice = 0x03;
//     setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_5HZ;
//     setting.accel_fchoice = 0x01;
//     setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_5HZ;

//     mpu.setup(0x68); // change to your own address

//     mpu.setFilterIterations(100);

//     // mpu.calibrateAccelGyro();
//     mpu.calibrateMag();
// }

// void loop()
// {
//     if (mpu.update())
//     {
//         // Serial.print(mpu.getYaw());
//         // Serial.print(", ");
//         // Serial.print(mpu.getPitch());
//         // Serial.print(", ");
//         // Serial.println(mpu.getRoll());

//         Serial.print(mpu.getEulerX());
//         Serial.print(", ");
//         Serial.print(mpu.getEulerY());
//         Serial.print(", ");
//         Serial.println(mpu.getEulerZ());
//     }
// }