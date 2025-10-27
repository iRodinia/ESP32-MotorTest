#ifndef __FORCE_SENSOR__
#define __FORCE_SENSOR__

#include <Arduino.h>
#include "AD770X.h"

class MyForceSensor {
public:
  MyForceSensor();
  ~MyForceSensor();
  int init(float max_force = 10 * 9.81f);
  void calibrate();
  float getZeroForce();
  int getForceRaw(float& force);
  int getForceCalibrated(float& force);
private:
  AD770X* _my_sensor;
  bool _inited = false;
  float _max_volt = 3.3;
  float _max_force = 10 * 9.81f;
  float _zero_volt = 0; 
};

MyForceSensor::MyForceSensor(){
  _inited = false;
  _max_volt = 3.3;
  _zero_volt = 0;
}

MyForceSensor::~MyForceSensor(){
  if(_my_sensor){
    delete(_my_sensor);
  }
}

int MyForceSensor::init(float max_force){
  if(max_force > 0){
    _max_force = max_force;
  }
  else{
    Serial.println("Force sensor maximum range set error. Reset to default.");
    _max_force = 10 * 9.81f;
  }
  _my_sensor = new AD770X(_max_volt);
  _my_sensor->reset();
  _my_sensor->init(AD770X::CHN_AIN1, AD770X::CLK_DIV_1, AD770X::BIPOLAR, AD770X::GAIN_1, AD770X::UPDATE_RATE_50);
  _inited = true;
}

void MyForceSensor::calibrate(){
  float sum_volt = 0.0f;
  for(int i=0; i<200; i++){
    sum_volt += _my_sensor->readADResult(AD770X::CHN_AIN1);
    delay(10);
  }
  _zero_volt = sum_volt / 200.0f;
}

float MyForceSensor::getZeroForce(){
  return _zero_volt;
}

int MyForceSensor::getForceRaw(float& force){
  if(_inited){
    float volt_read = _my_sensor->readADResult(AD770X::CHN_AIN1);
    force = volt_read / _max_volt * _max_force;
    return 0;
  }
  return -1;
}

int MyForceSensor::getForceCalibrated(float& force){
  if(_inited){
    float volt_read = _my_sensor->readADResult(AD770X::CHN_AIN1, _zero_volt);
    force = volt_read / _max_volt * _max_force;
    return 0;
  }
  return -1;
}

#endif