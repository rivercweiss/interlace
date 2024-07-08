/*
Notes:
- Needs to be calibrated in final setup
- Should be oriented so we have continuous outputs in range of movement (no -180 to 0 for example)
- Will try with other sensor
- Can tune filter for movement

Main Calibration:
https://www.mirkosertic.de/blog/2023/01/magnetometer-calibration-ellipsoid/

Other Resources:
https://www.digikey.com/en/maker/projects/how-to-calibrate-a-magnetometer/50f6bc8f36454a03b664dca30cf33a8b
https://www.nxp.com/docs/en/application-note/AN4247.pdf
https://www.nxp.com/docs/en/application-note/AN4246.pdf
https://teslabs.com/articles/magnetometer-calibration/
https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration
*/

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include "MPU9250.h"

MPU9250 mpu;

#define I2C_SDA 27
#define I2C_SCL 26

#define WINDOW_SIZE 50

int INDEX = 0;
float VALUE = 0;
double SUM = 0;
float READINGS[WINDOW_SIZE];
float AVERAGED = 0;

void setup()
{
    Serial.begin(9600);
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000); // 400KHz
    delay(2000);

    Serial.println("Started setup");

    MPU9250Setting setting;
    setting.accel_fs_sel = ACCEL_FS_SEL::A2G;
    setting.gyro_fs_sel = GYRO_FS_SEL::G250DPS;
    setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_125HZ;
    setting.gyro_fchoice = 0x03;
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_5HZ;
    setting.accel_fchoice = 0x01;
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_5HZ;

    mpu.setup(0x68, setting); // required setup

    // mpu.setMagBias(-301.47, 299.81, -67.32);
    // mpu.setMagBias(-271.49, 286.55, -66.47);
    // mpu.setMagBias(-306.75, 300.70, -73.29);
    // mpu.setMagScale(0.97, 1.10, 0.94);
    // mpu.setMagScale(1.08, 1.04, 0.90);
    mpu.setMagneticDeclination(-3.93);
    mpu.setFilterIterations(20);

    Serial.println("Finished setup");

    // calibrate anytime you want to
    // Serial.println("Accel Gyro calibration will start in 1sec.");
    // Serial.println("Please leave the device still on the flat plane.");
    // mpu.verbose(true);
    // delay(1000);
    // mpu.calibrateAccelGyro();

    // Serial.println("Mag calibration will start in 1sec.");
    // delay(1000);
    // mpu.calibrateMag();

    // print_calibration();
    // mpu.verbose(false);
}

void loop()
{
    if (mpu.update())
    {
        // Serial.print(mpu.getMagX());
        // Serial.print(", ");
        // Serial.print(mpu.getMagY());
        // Serial.print(", ");
        // Serial.print(mpu.getMagZ());
        // Serial.print("\n");

        // static float heading = 0;

        // a, b, c, d, e = 0.992274109564494 -0.12406486806501901 1.1706813299483279 -234.50803305124708 278.5065614770521
        // float a = 0.992274109564494;
        // float b = -0.12406486806501901;
        // float c = 1.1706813299483279;
        // float d = -234.50803305124708;
        // float e = 278.5065614770521;

        // float X = mpu.getMagX();
        // float Y = mpu.getMagY();

        // float mag_x_corr = (X - d) * a - (Y - e) * b;
        // float mag_y_corr = ((X - d) * b + (Y - e) * a) * c;

        // Calculate angle for heading, assuming board is parallel to
        // the ground and  Y points toward heading.
        // heading = -1 * (atan2(mag_x_corr, mag_y_corr) * 180) / M_PI;
        // heading = -1 * (atan2(mpu.getMagX(), mpu.getMagY()) * 180) / M_PI;
        // heading = heading + 180;

        VALUE = mpu.getYaw();                   // Read the next sensor value
        SUM = SUM - READINGS[INDEX];       // Remove the oldest entry from the sum
        READINGS[INDEX] = VALUE;           // Add the newest reading to the window
        SUM = SUM + VALUE;                 // Add the newest reading to the sum
        INDEX = (INDEX+1) % WINDOW_SIZE;   // Increment the index, and wrap to 0 if it exceeds the window size

        AVERAGED = SUM / (float)WINDOW_SIZE;      // Divide the sum of the window by the window size for the result

        Serial.print("Avg: ");
        Serial.print(AVERAGED, 3);        
        Serial.print(", Raw: ");
        Serial.println(VALUE, 3);
    }
}