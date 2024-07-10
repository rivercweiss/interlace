#include <Adafruit_LIS3MDL.h>
Adafruit_LIS3MDL lis3mdl;

// Can change this to be LSM6DSOX or whatever ya like
// #include <SparkFunLSM6DSO.h>
// LSM6DSO lsm6ds; //Default constructor is I2C, addr 0x6B

Adafruit_Sensor *magnetometer;

bool init_sensors(void) {
  // if (!lsm6ds.begin()) {
  //   return false;
  // }

  if (!lis3mdl.begin_I2C()) {
    return false;
  }
  
  magnetometer = &lis3mdl;

  return true;
}

void setup_sensors(void) {

  // lsm6ds.setAccelRange(uint8_t) ;
  // lsm6ds.setAccelDataRate(uint16_t) ;
  // lsm6ds.setGyroDataRate(uint16_t);
  // lsm6ds.setGyroRange(uint16_t) ;

  // lsm6ds.imuSettings.gyroEnabled = true;  //Can be 0 or 1
	// lsm6ds.imuSettings.gyroRange = 125;   //Max deg/s.  Can be: 125, 250, 500, 1000, 2000
	// lsm6ds.imuSettings.gyroSampleRate = 208;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
	// lsm6ds.imuSettings.gyroBandWidth = 200;  //Hz.  Can be: 50, 100, 200, 400;
	// lsm6ds.imuSettings.gyroFifoEnabled = 1;  //Set to include gyro in FIFO
	// lsm6ds.imuSettings.gyroAccelDecimation = 1;  //Set to include gyro in FIFO
	// lsm6ds.imuSettings.accelEnabled = true;
	// lsm6ds.imuSettings.accelRange = 2;      //Max G force readable.  Can be: 2, 4, 8, 16
	// lsm6ds.imuSettings.accelSampleRate = 208;  //Hz.  Can be: 1.6 (16), 12.5 (125), 26, 52, 104, 208, 416, 833, 1660, 3330, 6660
	// lsm6ds.imuSettings.accelFifoEnabled = 1;  //Set to include accelerometer in the FIFO
  // lsm6ds.imuSettings.fifoEnabled = true;
	// lsm6ds.imuSettings.fifoThreshold = 3000;  //Can be 0 to 4096 (16 bit bytes)
	// lsm6ds.imuSettings.fifoSampleRate = 416; 
	// lsm6ds.imuSettings.fifoModeWord = 0;  //Default off

  // lsm6ds.beginSettings();

  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_560_HZ);
  lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
}
