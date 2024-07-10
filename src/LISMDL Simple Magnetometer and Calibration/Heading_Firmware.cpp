/*
Notes:
- Needs to be calibrated in final setup
- Need to tune filter for movement in final setup
- Should be oriented so we have continuous outputs in range of movement (no -180 to 0 for example)

Main Calibration:
https://www.mirkosertic.de/blog/2023/01/magnetometer-calibration-ellipsoid/
*/

#include "LIS3MDL.h"

#define I2C_SDA 27
#define I2C_SCL 26

#define PRINT_EVERY_N_UPDATES 10
// #define AHRS_CALIBRATION

#define WINDOW_SIZE 250
int INDEX = 0;
float VALUE = 0;
double SUM = 0;
float READINGS[WINDOW_SIZE];
float AVERAGED = 0;

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

  Wire.setClock(400000); // 400KHz
}

void loop()
{
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

    // Copy and paste calibration data below
    // a, b, c, d, e = 0.8997438084310719 0.4364184679753487 1.1317375086031287 -8.298640083858855 1.3939266028550261
    static float a = 0.8997438084310719;
    static float b = 0.4364184679753487;
    static float c = 1.1317375086031287;
    static float d = -21.590995883858856;
    static float e = 9.817197302855027;

    float mag_x_corr = (mag.magnetic.x - d) * a - (mag.magnetic.y - e) * b;
    float mag_y_corr = ((mag.magnetic.x - d) * b + (mag.magnetic.y - e) * a) * c;

    // Calculate angle for heading, assuming board is parallel to
    // the ground and  Y points toward heading.
    static float heading = 0;
    heading = -1 * (atan2(mag_x_corr, mag_y_corr) * 180) / M_PI + 180;

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
}