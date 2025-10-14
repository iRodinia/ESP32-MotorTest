#ifndef __GY87_MANAGER__
#define __GY87_MANAGER__

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_HMC5883_U.h>

// Driver for GY-87 module, a combination of 3 sensors: 
// MPU6050 for acceleration and angular speed, HMC5883L for gesture and BMP085 for barometer and temperature

// GY-87 connect with ESP32-Wroom-DA: 
// Vcc-5V, 3.3V-NC (3.3V output pin), GND-GND, SCL-GPIO22, SDA-GPIO21, Fsync-NC (input for frames sync), Inta-NC (output for MPU6050 interrupt), Drdy-NC (output for HMC5883L data ready)
// Must connect to the default hardware I2C pins: Wire1
// Software I2C is not allowed

class MyIMU {
public:
  bool init();
  int readTemperatureRaw(float& temp);  // in Celsius
  int readPressureRaw(float& pre);  // in Pa
  int readAltitudeRaw(float& alt);  // in meters
  int readAccelerationRaw(float& x, float& y, float& z);  // in m/s^2
  int readGyroRaw(float& x, float& y, float& z);  // in rad/s
  int readMagnetRaw(float& x, float& y, float& z);  // in μT
  int getHeadingAngleDegRaw(float& ang_deg);  // in deg

  int calibrateMPU6050();  // must be called in the setup() function
  int setDeclinationAngle(float dec_rad);  // find from http://www.magnetic-declination.com/, if get -x(W), then dec=x here
  bool checkInitStatus();

private:
  Adafruit_BMP085 _my_barometer;
  Adafruit_MPU6050 _my_accelerometer;
  Adafruit_HMC5883_Unified _my_magnentometer;

  bool _imu_initialized = false;
  float _dec_angle_rad = 0.057;  // -3°16'(W) at HKUSTGZ, need to be added to the heading direction calculated
  float _acc_x_bias, _acc_y_bias, _acc_z_bias, _gyr_x_bias, _gyr_y_bias, _gyr_z_bias;
};

bool MyIMU::init(){
  _imu_initialized = false;
  bool bmp_flag = _my_barometer.begin();
  if(!bmp_flag){
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  }
  else{
    Serial.println("Successfully loaded BMP085.");
    delay(50);
  }
  bool motion_flag = _my_accelerometer.begin();
  if(!motion_flag){
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
  }
  else{
    // _my_accelerometer.setMotionDetectionThreshold(1);  // default is 0, which disables the motion detection. 1 means 1LSB = 2mg (at ±2g scale)
    // _my_accelerometer.setMotionDetectionDuration(20);  // default is 0, which disables the motion detection. 20 means the motion duration >= 20ms
    _my_accelerometer.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    _my_accelerometer.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
    // _my_accelerometer.setMotionInterrupt(true);  // turn on the motion detection, so motionInteruputStatus can be True
    Serial.println("Successfully loaded MPU6050.");
    delay(50);
  }
  bool mag_flag = _my_magnentometer.begin();
  if(!mag_flag){
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
  }
  else{
    _my_magnentometer.setMagGain(HMC5883_MAGGAIN_1_3);  // check if enough to measure the magnetic field
    Serial.println("Successfully loaded HMC5883L.");
    delay(50);
  }
  _imu_initialized = bmp_flag && motion_flag && mag_flag;
  if(!_imu_initialized){
    Serial.println("IMU module initialization failed.");
    return false;
  }
  setDeclinationAngle(0.057);
  _acc_x_bias = 0;
  _acc_y_bias = 0;
  _acc_z_bias = 0;
  _gyr_x_bias = 0;
  _gyr_y_bias = 0;
  _gyr_z_bias = 0;
  return true;
}

int MyIMU::setDeclinationAngle(float dec_rad){
  _dec_angle_rad = dec_rad;
  return 0;
}

int MyIMU::calibrateMPU6050(){
  if(!_imu_initialized){
    return -1;
  }
  const int samples = 100;
  long sumaX = 0, sumaY = 0, sumaZ = 0, sumgX = 0, sumgY = 0, sumgZ = 0;
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    _my_accelerometer.getEvent(&a, &g, &temp);
    sumaX += a.acceleration.x;
    sumaY += a.acceleration.y;
    sumaZ += a.acceleration.z;
    sumgX += g.gyro.x;
    sumgY += g.gyro.y;
    sumgZ += g.gyro.z;
    delay(10);
  }
  _acc_x_bias = sumaX / samples;
  _acc_y_bias = sumaY / samples;
  _acc_z_bias = sumaZ / samples;
  _gyr_x_bias = sumgX / samples;
  _gyr_y_bias = sumgY / samples;
  _gyr_z_bias = sumgZ / samples;
  return 0;
}

bool MyIMU::checkInitStatus(){
  return _imu_initialized;
}

int MyIMU::readTemperatureRaw(float& temp){
  if(!_imu_initialized){
    return -1;
  }
  temp = _my_barometer.readTemperature();
  return 0;
}

int MyIMU::readPressureRaw(float& pre){
  if(!_imu_initialized){
    return -1;
  }
  pre = _my_barometer.readPressure();
  return 0;
}

int MyIMU::readAltitudeRaw(float& alt){
  if(!_imu_initialized){
    return -1;
  }
  alt = _my_barometer.readAltitude();
  return 0;
}

int MyIMU::readAccelerationRaw(float& x, float& y, float& z){
  if(!_imu_initialized){
    return -1;
  }
  sensors_event_t a, g, temp;
  _my_accelerometer.getEvent(&a, &g, &temp);
  x = a.acceleration.x - _acc_x_bias;
  y = a.acceleration.y - _acc_y_bias;
  z = a.acceleration.z - _acc_z_bias;
  return 0;
}

int MyIMU::readGyroRaw(float& x, float& y, float& z){
  if(!_imu_initialized){
    return -1;
  }
  sensors_event_t a, g, temp;
  _my_accelerometer.getEvent(&a, &g, &temp);
  x = g.gyro.x - _gyr_x_bias;
  y = g.gyro.y - _gyr_y_bias;
  z = g.gyro.z - _gyr_z_bias;
  return 0;
}

int MyIMU::readMagnetRaw(float& x, float& y, float& z){
  if(!_imu_initialized){
    return -1;
  }
  sensors_event_t event; 
  _my_magnentometer.getEvent(&event);
  x = event.magnetic.x;
  y = event.magnetic.y;
  z = event.magnetic.z;
  return 0;
}

int MyIMU::getHeadingAngleDegRaw(float& ang_deg){
  if(!_imu_initialized){
    return -1;
  }
  sensors_event_t event; 
  _my_magnentometer.getEvent(&event);
  float heading = atan2(event.magnetic.y, event.magnetic.x) + _dec_angle_rad;
  if(heading < 0)
    heading += 2*PI;
  if(heading > 2*PI)
    heading -= 2*PI;
  ang_deg = heading * 180/M_PI;
  return 0;
}

#endif