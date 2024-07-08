// // Full orientation sensing using NXP/Madgwick/Mahony and a range of 9-DoF
// // sensor sets.
// // You *must* perform a magnetic calibration before this code will work.
// //
// // To view this data, use the Arduino Serial Monitor to watch the
// // scrolling angles, or run the OrientationVisualiser example in Processing.
// // Based on  https://github.com/PaulStoffregen/NXPMotionSense with adjustments
// // to Adafruit Unified Sensor interface

// #include <Adafruit_Sensor_Calibration.h>
// #include <Adafruit_AHRS.h>
// #include "MPU9250.h"

// #define I2C_SDA 27
// #define I2C_SCL 26

// // uncomment one combo 9-DoF!
// // #include "LSM6DS_LIS3MDL.h"  // see the the LSM6DS_LIS3MDL file in this project to change board to LSM6DS33, LSM6DS3U, LSM6DSOX, etc
// // #include "LSM9DS.h"           // LSM9DS1 or LSM9DS0
// // #include "NXP_FXOS_FXAS.h"  // NXP 9-DoF breakout

// // pick your filter! slower == better quality output
// // Adafruit_NXPSensorFusion filter; // slowest
// // Adafruit_Madgwick filter;  // faster than NXP
// Adafruit_Mahony filter; // fastest/smalleset

// #define FILTER_UPDATE_RATE_HZ 100
// #define PRINT_EVERY_N_UPDATES 50
// // #define AHRS_DEBUG_OUTPUT

// uint32_t timestamp;

// MPU9250 mpu;

// void setup()
// {
//     Serial.begin(9600);
//     Wire.begin(I2C_SDA, I2C_SCL);
//     Wire.setClock(400000); // 400KHz
//     delay(2000);

//     Serial.println("Started setup");

//     MPU9250Setting setting;
//     setting.accel_fs_sel = ACCEL_FS_SEL::A2G;
//     setting.gyro_fs_sel = GYRO_FS_SEL::G250DPS;
//     setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
//     setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_1000HZ;
//     setting.gyro_fchoice = 0x03;
//     setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_184HZ;
//     setting.accel_fchoice = 0x01;
//     setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_218HZ_0;

//     mpu.setup(0x68, setting); // required setup

//     mpu.setMagBias(-271.49, 286.55, -66.47);
//     mpu.setMagScale(0.97, 1.10, 0.94);
//     mpu.setMagneticDeclination(-3.93);
//     // mpu.setFilterIterations(1);

//     Serial.println("Finished setup");

//     filter.begin(FILTER_UPDATE_RATE_HZ);
//     timestamp = millis();
// }

// void loop()
// {
//     float roll, pitch, heading;
//     float gx, gy, gz;
//     static uint8_t counter = 0;

//     mpu.update();

//     if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ))
//     {
//         return;
//     }

//     timestamp = millis();

//     // Update the SensorFusion filter
//     filter.update(mpu.getGyroX(), mpu.getGyroY(), mpu.getGyroZ(),
//                   mpu.getAccX(), mpu.getAccY(), mpu.getAccZ(),
//                   mpu.getMagX(), mpu.getMagY(), mpu.getMagZ());

// #if defined(AHRS_DEBUG_OUTPUT)
//     Serial.print("Update took ");
//     Serial.print(millis() - timestamp);
//     Serial.println(" ms");
// #endif

//     // only print the calculated output once in a while
//     if (counter++ <= PRINT_EVERY_N_UPDATES)
//     {
//         return;
//     }
//     // reset the counter
//     counter = 0;

// #if defined(AHRS_DEBUG_OUTPUT)
//     Serial.print("Raw: ");
//     Serial.print(mpu.getAccX(), 4);
//     Serial.print(", ");
//     Serial.print(mpu.getAccY(), 4);
//     Serial.print(", ");
//     Serial.print(mpu.getAccZ(), 4);
//     Serial.print(", ");
//     Serial.print(mpu.getGyroX(), 4);
//     Serial.print(", ");
//     Serial.print(mpu.getGyroY(), 4);
//     Serial.print(", ");
//     Serial.print(mpu.getGyroZ(), 4);
//     Serial.print(", ");
//     Serial.print(mpu.getMagX(), 4);
//     Serial.print(", ");
//     Serial.print(mpu.getMagY(), 4);
//     Serial.print(", ");
//     Serial.print(mpu.getMagZ(), 4);
//     Serial.println("");
// #endif

//     // print the heading, pitch and roll
//     roll = filter.getRoll();
//     pitch = filter.getPitch();
//     heading = filter.getYaw();
//     Serial.print("Orientation: ");
//     Serial.print(heading);
//     Serial.print(", ");
//     Serial.print(pitch);
//     Serial.print(", ");
//     Serial.println(roll);

//     // float qw, qx, qy, qz;
//     // filter.getQuaternion(&qw, &qx, &qy, &qz);
//     // Serial.print("Quaternion: ");
//     // Serial.print(qw, 4);
//     // Serial.print(", ");
//     // Serial.print(qx, 4);
//     // Serial.print(", ");
//     // Serial.print(qy, 4);
//     // Serial.print(", ");
//     // Serial.println(qz, 4);

// #if defined(AHRS_DEBUG_OUTPUT)
//     Serial.print("Took ");
//     Serial.print(millis() - timestamp);
//     Serial.println(" ms");
// #endif
// }