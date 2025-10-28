#ifndef __GY85_MANAGER__
#define __GY85_MANAGER__

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <ITG3200.h>
#include <Adafruit_HMC5883_U.h>

// Driver for GY-85 module, a combination of 3 sensors:
// ITG3205 for angular speed, ADXL345 for acceleration and HMC5883L for magnetometer

// GY-85 connect with ESP32-Wroom-DA:
// Vcc-5V, 3.3V-NC (3.3V output pin), GND-GND, SCL-GPIO22, SDA-GPIO21
// Must connect to the default hardware I2C pins: Wire1
// Software I2C is not allowed

class MyIMU_GY85 {
public:
  String init();
  int readAccelerationRaw(float& x, float& y, float& z);  // in m/s^2
  int readGyroRaw(float& x, float& y, float& z);  // in rad/s
  int readMagnetRaw(float& x, float& y, float& z);  // in µT
  int readTemperatureRaw(float& deg);  // in degree centigrade
  int getHeadingAngleDegRaw(float& ang_deg);  // in deg
  void setAccelBias(float dax, float day, float daz);

  int calibrateAccel();  // must be called in the setup() function
  int calibrateGyro();  // must be called in the setup() function
  int setDeclinationAngle(float dec_rad);  // find from http://www.magnetic-declination.com/, if get -x(W), then dec=x here
  bool checkInitStatus();

private:
  Adafruit_ADXL345_Unified _my_accelerometer;
  ITG3200 _my_gyro;
  Adafruit_HMC5883_Unified _my_magnetometer;

  bool _imu_initialized = false;
  float _dec_angle_rad = 0.057;  // -3°16'(W) at HKUSTGZ, need to be added to the heading direction calculated
  float _acc_x_bias, _acc_y_bias, _acc_z_bias;
};

String MyIMU_GY85::init(){
  int initNum = 0;
  // Initialize ADXL345 accelerometer
  while(!_my_accelerometer.begin() && initNum < 10){
    initNum++;
    delay(50);
  }
  if(initNum >= 10){
    _imu_initialized = false;
    return "Could not find a valid ADXL345 sensor, check wiring!";
  }
  _my_accelerometer.setRange(ADXL345_RANGE_4_G);
  Serial.println("Successfully loaded ADXL345.");
  delay(50);

  // Initialize ITG3205 gyroscope
  _my_gyro.init();
  _my_gyro.zeroCalibrate(128, 10);  // 128 samples, 10ms between samples
  Serial.println("Successfully loaded ITG3205.");
  delay(50);
  
  // Initialize HMC5883L magnetometer
  initNum = 0;
  while(!_my_magnetometer.begin() && initNum < 10){
    initNum++;
    delay(50);
  }
  if(initNum >= 10){
    _imu_initialized = false;
    return "Could not find a valid HMC5883L sensor, check wiring!";
  }
  _my_magnetometer.setMagGain(HMC5883_MAGGAIN_1_3);  // check if enough to measure the magnetic field
  Serial.println("Successfully loaded HMC5883L.");
  delay(50);
  
  _imu_initialized = true;
  setDeclinationAngle(0.057);
  setAccelBias(0, 0, 0);
  return "";
}

void MyIMU_GY85::setAccelBias(float dax, float day, float daz)
{
  _acc_x_bias = dax;
  _acc_y_bias = day;
  _acc_z_bias = daz;
}

int MyIMU_GY85::setDeclinationAngle(float dec_rad){
  _dec_angle_rad = dec_rad;
  return 0;
}

int MyIMU_GY85::calibrateAccel(){
  if(!_imu_initialized){
    return -1;
  }
  const int samples = 100;
  float sumaX = 0, sumaY = 0, sumaZ = 0;
  for (int i = 0; i < samples; i++) {
    sensors_event_t event;
    _my_accelerometer.getEvent(&event);
    sumaX += event.acceleration.x;
    sumaY += event.acceleration.y;
    sumaZ += event.acceleration.z;
    delay(10);
  }
  setAccelBias(sumaX / samples, sumaY / samples, sumaZ / samples);
  Serial.println("ADXL345 calibration result:");
  Serial.printf("dAccx:%.2f, dAccy:%.2f, dAccz:%.2f\n",
    _acc_x_bias, _acc_y_bias, _acc_z_bias);
  return 0;
}

int MyIMU_GY85::calibrateGyro(){
  if(!_imu_initialized){
    return -1;
  }
  _my_gyro.zeroCalibrate(128, 10);  // samples number, sampling time interval (in ms)
  Serial.println("ITG3205 gyroscope calibrated.");
  return 0;
}

bool MyIMU_GY85::checkInitStatus(){
  return _imu_initialized;
}

int MyIMU_GY85::readAccelerationRaw(float& x, float& y, float& z){
  if(!_imu_initialized){
    return -1;
  }
  sensors_event_t event;
  _my_accelerometer.getEvent(&event);
  // Convert from m/s^2 (Adafruit output) to m/s^2 with bias correction
  x = event.acceleration.x - _acc_x_bias;
  y = event.acceleration.y - _acc_y_bias;
  z = event.acceleration.z - _acc_z_bias;
  return 0;
}

int MyIMU_GY85::readGyroRaw(float& x, float& y, float& z){
  if(!_imu_initialized){
    return -1;
  }
  // Read raw gyro values from ITG3205 (in rad/s)
  _my_gyro.getAngularVelocity(&x, &y, &z);
  return 0;
}

int MyIMU_GY85::readMagnetRaw(float& x, float& y, float& z){
  if(!_imu_initialized){
    return -1;
  }
  sensors_event_t event;
  _my_magnetometer.getEvent(&event);
  x = event.magnetic.x;
  y = event.magnetic.y;
  z = event.magnetic.z;
  return 0;
}

int MyIMU_GY85::readTemperatureRaw(float& deg){
  if(!_imu_initialized){
    return -1;
  }
  deg = _my_gyro.getTemperature();
  return 0;
}

int MyIMU_GY85::getHeadingAngleDegRaw(float& ang_deg){
  if(!_imu_initialized){
    return -1;
  }
  sensors_event_t event;
  _my_magnetometer.getEvent(&event);
  float heading = atan2(event.magnetic.y, event.magnetic.x) + _dec_angle_rad;
  while(heading < 0)
    heading += 2*PI;
  while(heading > 2*PI)
    heading -= 2*PI;
  ang_deg = heading * 180/M_PI;
  return 0;
}

#endif