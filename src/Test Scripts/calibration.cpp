// /***************************************************************************
//   This is an example for the Adafruit AHRS library
//   It will look for a supported magnetometer and output
//   PJRC Motion Sensor Calibration Tool-compatible serial data

//   PJRC & Adafruit invest time and resources providing this open source code,
//   please support PJRC and open-source hardware by purchasing products
//   from PJRC!

//   Written by PJRC, adapted by Limor Fried for Adafruit Industries.
//  ***************************************************************************/

// #include <Adafruit_Sensor_Calibration.h>
// #include <Arduino.h>
// #include "MPU9250.h"

// #define I2C_SDA 27
// #define I2C_SCL 26

// Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

// // uncomment one combo 9-DoF!
// // #include "setup_MPU9250.h" // see the the LSM6DS_LIS3MDL file in this project to change board to LSM6DS33, LSM6DS3U, LSM6DSOX, etc
// // #include "LSM6DS_LIS3MDL.h"  // see the the LSM6DS_LIS3MDL file in this project to change board to LSM6DS33, LSM6DS3U, LSM6DSOX, etc
// // #include "LSM9DS.h"           // LSM9DS1 or LSM9DS0
// // #include "NXP_FXOS_FXAS.h"  // NXP 9-DoF breakout

// // select either EEPROM or SPI FLASH storage:
// #ifdef ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM
// Adafruit_Sensor_Calibration_EEPROM cal;
// #else
// Adafruit_Sensor_Calibration_SDFat cal;
// #endif

// // sensors_event_t mag_event, gyro_event, accel_event;

// int loopcount = 0;

// /********************************************************/

// byte caldata[68]; // buffer to receive magnetic calibration data
// byte calcount = 0;

// uint16_t crc16_update(uint16_t crc, uint8_t a)
// {
//     int i;
//     crc ^= a;
//     for (i = 0; i < 8; i++)
//     {
//         if (crc & 1)
//         {
//             crc = (crc >> 1) ^ 0xA001;
//         }
//         else
//         {
//             crc = (crc >> 1);
//         }
//     }
//     return crc;
// }

// void receiveCalibration()
// {
//     uint16_t crc;
//     byte b, i;

//     while (Serial.available())
//     {
//         b = Serial.read();
//         if (calcount == 0 && b != 117)
//         {
//             // first byte must be 117
//             return;
//         }
//         if (calcount == 1 && b != 84)
//         {
//             // second byte must be 84
//             calcount = 0;
//             return;
//         }
//         // store this byte
//         caldata[calcount++] = b;
//         if (calcount < 68)
//         {
//             // full calibration message is 68 bytes
//             return;
//         }
//         // verify the crc16 check
//         crc = 0xFFFF;
//         for (i = 0; i < 68; i++)
//         {
//             crc = crc16_update(crc, caldata[i]);
//         }
//         if (crc == 0)
//         {
//             // data looks good, use it
//             float offsets[16];
//             memcpy(offsets, caldata + 2, 16 * 4);
//             cal.accel_zerog[0] = offsets[0];
//             cal.accel_zerog[1] = offsets[1];
//             cal.accel_zerog[2] = offsets[2];

//             cal.gyro_zerorate[0] = offsets[3];
//             cal.gyro_zerorate[1] = offsets[4];
//             cal.gyro_zerorate[2] = offsets[5];

//             cal.mag_hardiron[0] = offsets[6];
//             cal.mag_hardiron[1] = offsets[7];
//             cal.mag_hardiron[2] = offsets[8];

//             cal.mag_field = offsets[9];

//             cal.mag_softiron[0] = offsets[10];
//             cal.mag_softiron[1] = offsets[13];
//             cal.mag_softiron[2] = offsets[14];
//             cal.mag_softiron[3] = offsets[13];
//             cal.mag_softiron[4] = offsets[11];
//             cal.mag_softiron[5] = offsets[15];
//             cal.mag_softiron[6] = offsets[14];
//             cal.mag_softiron[7] = offsets[15];
//             cal.mag_softiron[8] = offsets[12];

//             if (!cal.saveCalibration())
//             {
//                 Serial.println("**WARNING** Couldn't save calibration");
//             }
//             else
//             {
//                 Serial.println("Wrote calibration");
//             }
//             cal.printSavedCalibration();
//             calcount = 0;
//             return;
//         }
//         // look for the 117,84 in the data, before discarding
//         for (i = 2; i < 67; i++)
//         {
//             if (caldata[i] == 117 && caldata[i + 1] == 84)
//             {
//                 // found possible start within data
//                 calcount = 68 - i;
//                 memmove(caldata, caldata + i, calcount);
//                 return;
//             }
//         }
//         // look for 117 in last byte
//         if (caldata[67] == 117)
//         {
//             caldata[0] = 117;
//             calcount = 1;
//         }
//         else
//         {
//             calcount = 0;
//         }
//     }
// }


// MPU9250 mpu; // You can also use MPU9255 as is

// void setup()
// {
//   Serial.begin(9600);
//   Wire.begin(I2C_SDA, I2C_SCL);
//   delay(2000);

//   // if (!cal.begin()) {
//   //   Serial.println("Failed to initialize calibration helper");
//   // } else if (! cal.loadCalibration()) {
//   //   Serial.println("No calibration loaded/found");
//   // }

//   Serial.println("Started setup");

//   MPU9250Setting setting;
//   setting.accel_fs_sel = ACCEL_FS_SEL::A2G;
//   setting.gyro_fs_sel = GYRO_FS_SEL::G250DPS;
//   setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
//   setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_1000HZ;
//   setting.gyro_fchoice = 0x03;
//   setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_5HZ;
//   setting.accel_fchoice = 0x01;
//   setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_5HZ;
//   mpu.setup(0x68); // change to your own address
//   // mpu.setFilterIterations(100);

//   Wire.setClock(400000); // 400KHz

//   Serial.println("Finished setup");
// }

// void loop()
// {
//     if (mpu.update())
//     {
//         // 'Raw' values to match expectation of MotionCal
//         Serial.print("Raw:");
//         Serial.print(int(mpu.getAccX() * 8192 / 9.8));
//         Serial.print(",");
//         Serial.print(int(mpu.getAccY() * 8192 / 9.8));
//         Serial.print(",");
//         Serial.print(int(mpu.getAccZ() * 8192 / 9.8));
//         Serial.print(",");
//         Serial.print(int(mpu.getGyroX() * 16));
//         Serial.print(",");
//         Serial.print(int(mpu.getGyroY() * 16));
//         Serial.print(",");
//         Serial.print(int(mpu.getGyroZ() * 16));
//         Serial.print(",");
//         Serial.print(int(mpu.getMagX() * 10));
//         Serial.print(",");
//         Serial.print(int(mpu.getMagY() * 10));
//         Serial.print(",");
//         Serial.print(int(mpu.getMagZ() * 10));
//         Serial.println("");

//         // unified data
//         Serial.print("Uni:");
//         Serial.print(mpu.getAccX());
//         Serial.print(",");
//         Serial.print(mpu.getAccY());
//         Serial.print(",");
//         Serial.print(mpu.getAccZ());
//         Serial.print(",");
//         Serial.print(mpu.getGyroX(), 4);
//         Serial.print(",");
//         Serial.print(mpu.getGyroY(), 4);
//         Serial.print(",");
//         Serial.print(mpu.getGyroZ(), 4);
//         Serial.print(",");
//         Serial.print(mpu.getMagX());
//         Serial.print(",");
//         Serial.print(mpu.getMagY());
//         Serial.print(",");
//         Serial.print(mpu.getMagZ());
//         Serial.println("");
//         loopcount++;
//         receiveCalibration();

//         // occasionally print calibration
//         if (loopcount == 50 || loopcount > 100)
//         {
//             Serial.print("Cal1:");
//             for (int i = 0; i < 3; i++)
//             {
//                 Serial.print(cal.accel_zerog[i], 3);
//                 Serial.print(",");
//             }
//             for (int i = 0; i < 3; i++)
//             {
//                 Serial.print(cal.gyro_zerorate[i], 3);
//                 Serial.print(",");
//             }
//             for (int i = 0; i < 3; i++)
//             {
//                 Serial.print(cal.mag_hardiron[i], 3);
//                 Serial.print(",");
//             }
//             Serial.println(cal.mag_field, 3);
//             loopcount++;
//         }
//         if (loopcount >= 100)
//         {
//             Serial.print("Cal2:");
//             for (int i = 0; i < 9; i++)
//             {
//                 Serial.print(cal.mag_softiron[i], 4);
//                 if (i < 8)
//                     Serial.print(',');
//             }
//             Serial.println();
//             loopcount = 0;
//         }

//         delay(10);
//     }
// }
