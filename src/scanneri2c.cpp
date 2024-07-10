// #include <Arduino.h>
// #include <Wire.h>

// #include <Adafruit_Sensor_Calibration.h>
// #include <Adafruit_AHRS.h>

// #define I2C_SDA 27
// #define I2C_SCL 26

// void setup()
// {
//     // Wire.setPins(I2C_SDA, I2C_SCL);
//     // Wire.begin();
//     Wire.begin(I2C_SDA, I2C_SCL);
//     Serial.begin(9600);
//     Serial.println("\nI2C Scanner");
// }

// void loop()
// {
//     byte error, address;
//     int nDevices;
//     Serial.println("Scanning...");
//     nDevices = 0;
//     for (address = 1; address < 127; address++)
//     {
//         Wire.beginTransmission(address);
//         error = Wire.endTransmission();
//         if (error == 0)
//         {
//             Serial.print("I2C device found at address 0x");
//             if (address < 16)
//             {
//                 Serial.print("0");
//             }
//             Serial.println(address, HEX);
//             nDevices++;
//         }
//         else if (error == 4)
//         {
//             Serial.print("Unknow error at address 0x");
//             if (address < 16)
//             {
//                 Serial.print("0");
//             }
//             Serial.println(address, HEX);
//         }
//     }
//     if (nDevices == 0)
//     {
//         Serial.println("No I2C devices found\n");
//     }
//     else
//     {
//         Serial.println("done\n");
//     }
//     delay(5000);
// }
