// Full orientation sensing using NXP/Madgwick/Mahony and a range of 9-DoF
// sensor sets.
// You *must* perform a magnetic calibration before this code will work.
//
// To view this data, use the Arduino Serial Monitor to watch the
// scrolling angles, or run the OrientationVisualiser example in Processing.
// Based on  https://github.com/PaulStoffregen/NXPMotionSense with adjustments
// to Adafruit Unified Sensor interface

#include <Adafruit_AHRS.h>

#define I2C_SDA 27
#define I2C_SCL 26

#include "LSM6DS_LIS3MDL.h"

// pick your filter! slower == better quality output
// Adafruit_NXPSensorFusion filter; // slowest
Adafruit_Madgwick filter; // faster than NXP
// Adafruit_Mahony filter;  // fastest/smalleset

#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 10
// #define AHRS_CALIBRATION

#define WINDOW_SIZE 250

int INDEX = 0;
float VALUE = 0;
double SUM = 0;
float READINGS[WINDOW_SIZE];
float AVERAGED = 0;

uint32_t timestamp;

void setup()
{
  Serial.begin(9600);
  Wire.begin(I2C_SDA, I2C_SCL);

  if (!init_sensors())
  {
    Serial.println("Failed to find sensors");
    while (1)
      delay(10);
  }

  magnetometer->printSensorDetails();

  setup_sensors();
  filter.begin(FILTER_UPDATE_RATE_HZ);
  timestamp = millis();
  Wire.setClock(400000); // 400KHz
}

void loop()
{

  // if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ))
  // {
  //   return;
  // }
  // timestamp = millis();

  static sensors_event_t mag;
  static uint8_t counter = 0;

  if (magnetometer->getEvent(&mag))
  {
#if defined(AHRS_CALIBRATION)
    Serial.print(mag.magnetic.x, 4);
    Serial.print(", ");
    Serial.print(mag.magnetic.y, 4);
    Serial.print(", ");
    Serial.print(mag.magnetic.z, 4);
    Serial.println("");
    return;
#endif

    // a, b, c, d, e = 0.9392358475850507 -0.3432725194523897 1.1342217466868487 -4.744984964639308 -1.2260721569674102
    static float a = 0.9392358475850507;
    static float b = -0.3432725194523897;
    static float c = 1.1342217466868487;
    static float d = -19.83709436463931;
    static float e = 1.1180974430325896;

    float mag_x_corr = (mag.magnetic.x - d) * a - (mag.magnetic.y - e) * b;
    float mag_y_corr = ((mag.magnetic.x - d) * b + (mag.magnetic.y - e) * a) * c;

    // Calculate angle for heading, assuming board is parallel to
    // the ground and  Y points toward heading.
    static float heading = 0;

    heading = -1 * (atan2(mag_x_corr, mag_y_corr) * 180) / M_PI + 180;
    // heading = -1 * (atan2(mpu.getMagX(), mpu.getMagY()) * 180) / M_PI;
    // heading = heading + 180;

    // // Update the SensorFusion filter
    // filter.update(lsm6ds.readFloatGyroX(), lsm6ds.readFloatGyroY(), lsm6ds.readFloatGyroZ(),
    //               lsm6ds.readFloatAccelX(), lsm6ds.readFloatAccelY(), lsm6ds.readFloatAccelZ(),
    //               mag_x_corr, mag_y_corr, mag.magnetic.z);

    // VALUE = filter.getYaw(); // Read the next sensor value
    VALUE = heading;                   // Read the next sensor value
    SUM = SUM - READINGS[INDEX];       // Remove the oldest entry from the sum
    READINGS[INDEX] = VALUE;           // Add the newest reading to the window
    SUM = SUM + VALUE;                 // Add the newest reading to the sum
    INDEX = (INDEX + 1) % WINDOW_SIZE; // Increment the index, and wrap to 0 if it exceeds the window size

    AVERAGED = SUM / (float)WINDOW_SIZE; // Divide the sum of the window by the window size for the result

    // only print the calculated output once in a while
    if (counter++ <= PRINT_EVERY_N_UPDATES)
    {
      return;
    }
    // reset the counter
    counter = 0;

    Serial.print("Avg: ");
    Serial.print(AVERAGED, 3);
    Serial.print(", Raw: ");
    Serial.println(VALUE, 3);
  }

  // //Get all parameters
  // Serial.print("\nAccelerometer:\n");
  // Serial.print(" X = ");
  // Serial.println(lsm6ds.readFloatAccelX(), 3);
  // Serial.print(" Y = ");
  // Serial.println(lsm6ds.readFloatAccelY(), 3);
  // Serial.print(" Z = ");
  // Serial.println(lsm6ds.readFloatAccelZ(), 3);

  // Serial.print("\nGyroscope:\n");
  // Serial.print(" X = ");
  // Serial.println(lsm6ds.readFloatGyroX(), 3);
  // Serial.print(" Y = ");
  // Serial.println(lsm6ds.readFloatGyroY(), 3);
  // Serial.print(" Z = ");
  // Serial.println(lsm6ds.readFloatGyroZ(), 3);

  //   float roll, pitch, heading;
  //   float gx, gy, gz;
  //   static uint8_t counter = 0;

  //   if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
  //     return;
  //   }
  //   timestamp = millis();
  //   // Read the motion sensors
  //   sensors_event_t accel, gyro, mag;
  //   accelerometer->getEvent(&accel);
  //   gyroscope->getEvent(&gyro);
  //   magnetometer->getEvent(&mag);
  // #if defined(AHRS_DEBUG_OUTPUT)
  //   Serial.print("I2C took "); Serial.print(millis()-timestamp); Serial.println(" ms");
  // #endif

  //   // cal.calibrate(mag);
  //   // cal.calibrate(accel);
  //   // cal.calibrate(gyro);
  //   // Gyroscope needs to be converted from Rad/s to Degree/s
  //   // the rest are not unit-important
  //   gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
  //   gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
  //   gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

  //   // Update the SensorFusion filter
  //   filter.update(gx, gy, gz,
  //                 accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
  //                 mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);
  // #if defined(AHRS_DEBUG_OUTPUT)
  //   Serial.print("Update took "); Serial.print(millis()-timestamp); Serial.println(" ms");
  // #endif

  //   // only print the calculated output once in a while
  //   if (counter++ <= PRINT_EVERY_N_UPDATES) {
  //     return;
  //   }
  //   // reset the counter
  //   counter = 0;

  // #if defined(AHRS_DEBUG_OUTPUT)
  //   Serial.print("Raw: ");
  //   Serial.print(accel.acceleration.x, 4); Serial.print(", ");
  //   Serial.print(accel.acceleration.y, 4); Serial.print(", ");
  //   Serial.print(accel.acceleration.z, 4); Serial.print(", ");
  //   Serial.print(gx, 4); Serial.print(", ");
  //   Serial.print(gy, 4); Serial.print(", ");
  //   Serial.print(gz, 4); Serial.print(", ");
  //   Serial.print(mag.magnetic.x, 4); Serial.print(", ");
  //   Serial.print(mag.magnetic.y, 4); Serial.print(", ");
  //   Serial.print(mag.magnetic.z, 4); Serial.println("");
  // #endif

  //   // print the heading, pitch and roll
  //   roll = filter.getRoll();
  //   pitch = filter.getPitch();
  //   heading = filter.getYaw();
  //   Serial.print("Orientation: ");
  //   Serial.print(heading);
  //   Serial.print(", ");
  //   Serial.print(pitch);
  //   Serial.print(", ");
  //   Serial.println(roll);

  //   float qw, qx, qy, qz;
  //   filter.getQuaternion(&qw, &qx, &qy, &qz);
  //   Serial.print("Quaternion: ");
  //   Serial.print(qw, 4);
  //   Serial.print(", ");
  //   Serial.print(qx, 4);
  //   Serial.print(", ");
  //   Serial.print(qy, 4);
  //   Serial.print(", ");
  //   Serial.println(qz, 4);

  // #if defined(AHRS_DEBUG_OUTPUT)
  //   Serial.print("Took "); Serial.print(millis()-timestamp); Serial.println(" ms");
  // #endif
}