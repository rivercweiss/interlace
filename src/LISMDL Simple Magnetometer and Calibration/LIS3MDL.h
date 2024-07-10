#include <Adafruit_LIS3MDL.h>
Adafruit_LIS3MDL lis3mdl;

bool init_sensors(void) {
  if (!lis3mdl.begin_I2C()) {
    return false;
  }
  
  return true;
}

void setup_sensors(void) {
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
  lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
}
