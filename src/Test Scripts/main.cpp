// #include <Arduino.h>
// #include "FastIMU.h"
// #include <Wire.h>
// #include <math.h>
// #define IMU_ADDRESS 0x68 // MPU9250
// // #define IMU_ADDRESS 0x1E // LSM303

// #define I2C_SDA 27
// #define I2C_SCL 26
// // #define PERFORM_CALIBRATION //Comment to disable startup calibration
// MPU9250 IMU; // Change to the name of any supported IMU!
// // LSM303 IMU; // Change to the name of any supported IMU!

// calData calib = {0}; // Calibration data
// AccelData accelData; // Sensor data
// GyroData gyroData;
// MagData magData;

// void setup()
// {
//   Wire.begin(I2C_SDA, I2C_SCL);
//   // Wire.setClock(400000); // 400khz clock
//   Serial.begin(9600);

//   int err = IMU.init(calib, IMU_ADDRESS);
//   if (err != 0)
//   {
//     Serial.print("Error initializing IMU: ");
//     Serial.println(err);
//   }
// }

// void loop()
// {
//   // IMU.update();
//   // IMU.getMag(&magData);
//   // Serial.print(magData.magX);
//   // Serial.print(",");
//   // Serial.print(magData.magY);
//   // Serial.print(",");
//   // Serial.print(magData.magZ);
//   // Serial.print("\n");
//   // delay(125);

//   IMU.update();
//   IMU.getMag(&magData);
//   Serial.print(sqrt(pow(magData.magX,2) + pow(magData.magY,2)));
//   Serial.print("\n");
//   delay(10);

//   // unsigned long start_time = millis();
//   // static int average_size = 8;
//   // float average_x = 0;
//   // float average_y = 0;
//   // for (int i = 0; i < average_size; i++)
//   // {
//   //   IMU.update();
//   //   IMU.getMag(&magData);
//   //   average_x = magData.magX + calibration_x;
//   //   average_y = magData.magY + calibration_y;
//   //   delay(125);
//   // }
//   // average_x = average_x/average_size;
//   // average_y = average_y/average_size;
//   // unsigned long end_time = millis();
//   // Serial.println(end_time-start_time);

//   // IMU.update();
//   // IMU.getMag(&magData);
//   // Serial.println(180 * atan2(magData.magY, magData.magX) / 3.1415);
//   // unsigned long end_time = millis();
//   // Serial.println(end_time-start_time);
//   // Serial.println(180 * atan2(average_y, average_x) / 3.1415);
//   // Serial.print(sqrt(magData.magY * magData.magY + magData.magX * magData.magX));
//   // Serial.print(",");
//   // Serial.print(magData.magX + calibration_x);
//   // Serial.print(",");
//   // Serial.print(magData.magY + calibration_y);
//   // Serial.print(",");
//   // Serial.println(magData.magZ);
//   // Serial.print("\n");
//   // if (IMU.hasTemperature()) {
//   //   Serial.print("\t");
//   //   Serial.println(IMU.getTemp());
//   // }
//   // delay(100);
//   // }
// }
