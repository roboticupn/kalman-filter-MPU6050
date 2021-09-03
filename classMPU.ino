#include<Wire.h>
#include<Kalman.h>
#include<MPU6050.h>
#include<I2Cdev.h>
#include<Adafruit_BMP085.h>


//=====================
// dibawah ini adalah list register
// yang digunakan untuk pembacaan
// method readSingleAxis()
//=====================
#define imuRegs 0x68
#define powerRegs 0x6B
#define accRegsX 0x3B
#define accRegsY 0x3D
#define accRegsZ 0x3F
#define gyrRegsX 0x43
#define gyrRegsY 0x45
#define gyrRegsZ 0x47
#define accConfig 0x1B
#define gyrConfig 0x1C

Adafruit_BMP085 bmp;

class angleKalman {
  public:
    //================ Variables ===============
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    Kalman KalmanX;
    Kalman KalmanY;
    double kalAngleX, kalAngleY;
    double roll, pitch;
    double gyroXrate, gyroYrate;
    double dt;
    uint32_t timer;
    MPU6050 imuSens;
    int bufferSize;
    long mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;

    //================= Constructor =============
    angleKalman() {

    }

    void setUpSensor() {
      Serial.println("Sensor setting up");
      Wire.beginTransmission(imuRegs);
      Wire.write(powerRegs);
      Wire.write(0x00);
      Wire.endTransmission();

      Wire.beginTransmission(imuRegs);
      Wire.write(accConfig);
      Wire.write(0x00);
      Wire.endTransmission();

      Wire.beginTransmission(imuRegs);
      Wire.write(gyrConfig);
      Wire.write(0x00);
      Wire.endTransmission();
    }

    float readSingleAxis(unsigned long int reg) {
      int16_t val;
      Wire.beginTransmission(imuRegs);
      Wire.write(reg);
      Wire.endTransmission(false);
      Wire.requestFrom(imuRegs, 2, true);
      val = Wire.read() << 8 | Wire.read();
      return val;
    }

    void readSensor() {
      ax = readSingleAxis(accRegsX);
      ay = readSingleAxis(accRegsY);
      az = readSingleAxis(accRegsZ);
      gx = readSingleAxis(gyrRegsX);
      gy = readSingleAxis(gyrRegsY);
      gz = readSingleAxis(gyrRegsZ);
    }

    void calculateRollPitch() {
      roll = atan2(ay, az) * RAD_TO_DEG;
      pitch = atan((double) - ax / sqrt((double)ay * (double)ay + (double)az * (double)az)) * RAD_TO_DEG;
    }

    void setStartingAngle() {
      calculateRollPitch();
      KalmanX.setAngle(roll);
      KalmanY.setAngle(pitch);
    }

    uint32_t calculateDeltaTime(double timer) {
      dt = (double)(micros() - timer) / 1000000;
      return dt;
    }

    void calculateGyroRate() {
      gyroXrate = gx / 131.0;
      gyroYrate = gy / 131.0;
    }

    void calculateKalman() {
      if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
        KalmanX.setAngle(roll);
        kalAngleX = roll;
      } else {
        kalAngleX = KalmanX.getAngle(roll, gyroXrate, dt);
      }

      if (abs(kalAngleX) > 90) {
        gyroYrate = -gyroYrate;
      }
      kalAngleY = KalmanY.getAngle(pitch, gyroYrate, dt);
    }

    void calibrateSensor(double ax_offset, double ay_offset, double az_offset,
                         double gx_offset, double gy_offset, double gz_offset) {
      imuSens.setXAccelOffset(ax_offset);
      imuSens.setYAccelOffset(ay_offset);
      imuSens.setZAccelOffset(az_offset);
      imuSens.setXGyroOffset(gx_offset);
      imuSens.setYGyroOffset(gy_offset);
      imuSens.setZGyroOffset(gz_offset);
      Serial.println("Done Calibrating");
    }
    void startBmp() {
      if (!bmp.begin()) {
        Serial.println("Couldnt find sensor, check wiring!");
      }
      while (1) {}
    }

    void readBmp() {
      Serial.print("Pressure : ");
      Serial.print(bmp.readPressure());
      Serial.println(" Pa");

      Serial.print("Altitude : ");
      Serial.print(bmp.readAltitude());
      Serial.println(" Meters");
    }
};
