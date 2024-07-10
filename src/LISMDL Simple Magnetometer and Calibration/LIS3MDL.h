#include <Adafruit_LIS3MDL.h>
Adafruit_LIS3MDL lis3mdl;

Adafruit_Sensor *magnetometer;

bool init_sensors(void) {
  if (!lis3mdl.begin_I2C()) {
    return false;
  }
  
  magnetometer = &lis3mdl;

  return true;
}

void setup_sensors(void) {
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_560_HZ);
  lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
}
