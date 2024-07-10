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

unsigned long startMicros; // some global variables available anywhere in the program
unsigned long currentMicros;
const unsigned long period = 1000; // number of micros - 1000 Hz

const int WINDOW_SIZE = 2000; // last 2 seconds of data averaged

#define PRINT_EVERY_N_UPDATES 100 // print every 10 Hz
// #define AHRS_CALIBRATION

unsigned long startMillis; // some global variables available anywhere in the program
unsigned long currentMillis;

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

  setup_sensors();

  Wire.setClock(400000); // 400KHz
}

float getHeading()
{
  // Copy and paste calibration data below
  // a, b, c, d, e = 0.8997438084310719 0.4364184679753487 1.1317375086031287 -8.298640083858855 1.3939266028550261
  const float a = 0.8997438084310719;
  const float b = 0.4364184679753487;
  const float c = 1.1317375086031287;
  const float d = -21.590995883858856;
  const float e = 9.817197302855027;

  lis3mdl.read();

  float mag_x_corr = ((lis3mdl.x_gauss * 100) - d) * a - ((lis3mdl.y_gauss * 100) - e) * b;
  float mag_y_corr = (((lis3mdl.x_gauss * 100) - d) * b + ((lis3mdl.y_gauss * 100) - e) * a) * c;

  // Calculate angle for heading, assuming board is parallel to
  // the ground and  Y points toward heading.
  float heading = -1 * (atan2(mag_x_corr, mag_y_corr) * 180) / M_PI + 180;

  return heading;
}

float getHeadingAverage(float VALUE)
{
  static int INDEX = 0;
  static double SUM = 0;
  static float READINGS[WINDOW_SIZE];
  static float AVERAGED = 0;

  SUM = SUM - READINGS[INDEX];       // Remove the oldest entry from the sum
  READINGS[INDEX] = VALUE;           // Add the newest reading to the window
  SUM = SUM + VALUE;                 // Add the newest reading to the sum
  INDEX = (INDEX + 1) % WINDOW_SIZE; // Increment the index, and wrap to 0 if it exceeds the window size

  AVERAGED = SUM / (float)WINDOW_SIZE; // Divide the sum of the window by the window size for the result

  return AVERAGED;
}

void loop()
{
#if defined(AHRS_CALIBRATION)
  lis3mdl.read();
  Serial.print((lis3mdl.x_gauss * 100), 4);
  Serial.print(", ");
  Serial.print((lis3mdl.y_gauss * 100), 4);
  Serial.print(", ");
  Serial.print((lis3mdl.z_gauss * 100), 4);
  Serial.println("");
  return;
#endif

  static float heading = 0;

  currentMicros = micros();
  if ((currentMicros - startMicros) >= period)
  {
    heading = getHeadingAverage(getHeading());
    startMicros = currentMicros;

    static int counter = 0;
    // only print the calculated output once in a while
    if (counter++ >= PRINT_EVERY_N_UPDATES)
    {
      counter = 0;
      currentMillis = millis();

      Serial.print("H: ");
      Serial.print(heading, 3);
      Serial.print(" M: ");
      Serial.println(currentMillis - startMillis);
      startMillis = currentMillis;
    }
  }
}