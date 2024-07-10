// /***************************************************************************
//   This is an example for the Adafruit SensorLab library
//   It will look for a supported magnetometer and output
//   uTesla data as well as the hard iron calibration offsets

//   Written by Limor Fried for Adafruit Industries.
//  ***************************************************************************/

// #include <Arduino.h>
// #include <Adafruit_Sensor_Calibration.h>
// #include "MPU9250.h"

// MPU9250 mpu; 

// #define I2C_SDA 27
// #define I2C_SCL 26

// float min_x, max_x, mid_x;
// float min_y, max_y, mid_y;
// float min_z, max_z, mid_z;

// void setup()
// {
//     Serial.begin(9600);
//     Wire.begin(I2C_SDA, I2C_SCL);
//     delay(2000);

//     Serial.println("Started setup");

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

//     mpu.setMagBias(-301.47, 299.81, -67.32);
//     mpu.setMagneticDeclination(-3.93);

//     Wire.setClock(400000); // 400KHz

//     Serial.println("Finished setup");
// }

// void loop()
// {
//     if (mpu.update())
//     {
//         float x = mpu.getMagX();
//         float y = mpu.getMagY();
//         float z = mpu.getMagZ();

//         // Serial.print("Mag: (");
//         // Serial.print(x);
//         // Serial.print(", ");
//         // Serial.print(y);
//         // Serial.print(", ");
//         // Serial.print(z);
//         // Serial.print(")");

//         min_x = min(min_x, x);
//         min_y = min(min_y, y);
//         min_z = min(min_z, z);

//         max_x = max(max_x, x);
//         max_y = max(max_y, y);
//         max_z = max(max_z, z);

//         mid_x = (max_x + min_x) / 2;
//         mid_y = (max_y + min_y) / 2;
//         mid_z = (max_z + min_z) / 2;
//         Serial.print(" Hard offset: (");
//         Serial.print(mid_x);
//         Serial.print(", ");
//         Serial.print(mid_y);
//         Serial.print(", ");
//         Serial.print(mid_z);
//         Serial.print(")");

//         Serial.print(" Field: (");
//         Serial.print((max_x - min_x) / 2);
//         Serial.print(", ");
//         Serial.print((max_y - min_y) / 2);
//         Serial.print(", ");
//         Serial.print((max_z - min_z) / 2);
//         Serial.println(")");
//         delay(10);
//     }
// }