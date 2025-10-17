#include <Arduino.h>
#include <Wire.h>  // Must include Wire here, otherwise all .h files won't include Wire
#include <TimeLib.h>

#include "SD_manager.h"
#include "12864_display.h"
#include "IMU_GY87_manager.h"
#include "IMU_Kalman_filter.h"
#include "Serial_down_lib.h"

/*
MCU (down) Functionality:
Measure the Z-axis acceleration; Estimate the Z-axis velocity;
Record the data in SD card by 20Hz;
Synchronize time through UART;
Communicate with MCU (up) using UART, receive the start command and send the data;
Display the real-time data on the screen.
*/

MyDisplay myscreen;  // scl-33, sda-32
MyIMU_GY87 mysensor;  // scl-22, sda-21
SDCard mysd(18, 19, 23, 5);  // sck = 18; miso = 19; mosi = 23; cs = 5;
GY87_KalmanFilter myfilter;
// Serial2： rx-16, tx-17

hw_timer_t *timer = NULL;
unsigned long start_record_local_time = 0;

void IRAM_ATTR onTimer() {
  if(timeStatus() != timeNotSet){
    myscreen.set_Line1("T:"+getCurrentHmsTime());
  }
  else{
    if(data_sending){
      unsigned long _cur_local_time_ms = millis() - start_record_local_time;
      float _cur_time = _cur_local_time_ms / 1000;
      myscreen.set_Line1("T:"+String(_cur_time, 2));
    }
    else{
      myscreen.set_Line1("T:0");
    }
  }
  myscreen.OLED_UpdateRam();
  myscreen.OLED_Refresh();
}

void setup(){
  Serial.begin(115200);  // usb
  while (!Serial)
    delay(10);

  Init_Data_Serial();  // Rx-16, Tx-17

  Serial.printf("\n");
  Serial.println("##### Start the Data Recording Board (Down) #####");

  Wire.setClock(100000);
  //Serial.println("Set the I2C port to be 400kHz (fast mode).");
  
  myscreen.init();
  delay(50);
  int init_count = 0;
  while(!mysd.checkCardStatus() && init_count < 20){
    mysd.init();
    delay(200);
    init_count += 1;
  }
  init_count = 0;
  while(!mysensor.checkInitStatus() && init_count < 20){
    mysensor.init();
    delay(200);
    init_count += 1;
  }
  if(!(mysd.checkCardStatus()&&mysensor.checkInitStatus())){
    Serial.println("Device initialization failed!");
    while(1);
  }
  Serial.println("All modules initialized.");
  myscreen.set_Line4(" ");
  myscreen.set_Checkbox(false);
  delay(20);

  timer = timerBegin(1000000);
  if(timer == NULL) {
    Serial.println("OLED Timer initialization failed!");
    while(1);
  }
  timerAlarm(timer, 250000, true, 0);
  timerAttachInterrupt(timer, &onTimer);
  // timerStart(timer);
  delay(20);

  myfilter.init(0.0);

  myscreen.set_Line4("Pending...");
  Serial.println("Data Recording Board (Down) Initialization Done.");
}

static unsigned long lastRcdLocalTime = 0;
uint32_t lastImuFastUpdate = 0;
uint32_t lastImuSlowUpdate = 0;
float lastAlt = 0;
float lastMx = 0;
float lastMy = 0;
float lastMz = 0;

void loop(){
  if (millis() - lastRcdLocalTime > 100) {
    lastRcdLocalTime = millis();
    float vx, vy, vz, ax, ay, az;
    myfilter.getVelocity(vx, vy, vz);
    myfilter.getAcceleration(ax, ay, az);
    if(file_created){
      String _glb_t = (timeStatus()==timeNotSet)? "N/A" : getCurrentHmsTime();
      String _lca_t = String(millis()-start_record_local_time);
      mysd.record(_glb_t + " " + _lca_t + " " + String(-az, 3) + " " + String(-vz, 3));  // -az and -vz is for the up direction
    }
    if(data_sending){
      Serial2.printf("z-vel:%.3f,z-acc:%.3f\n", -vz, -az);
      myscreen.set_Checkbox(true);
    }
    myscreen.set_Line2("Acc:" + String(-az, 3));
    myscreen.set_Line3("Vel:" + String(-vz, 3));


    Serial.printf("Mx:%.2f, My:%.2f, Mz:%.2f \n", lastMx, lastMy, lastMz);
    Serial.printf("Alt:%.2f \n", lastAlt);
  }

  if(millis() - lastImuSlowUpdate > 25) {
    lastImuSlowUpdate = millis();
    mysensor.readAltitudeRaw(lastAlt);
    mysensor.readMagnetRaw(lastMx, lastMy, lastMz);
  }

  uint32_t _dt = millis() - lastImuFastUpdate;
  if(_dt > 10) {
    lastImuFastUpdate = millis();
    float _dt_s = _dt / 1000.0f;
    float ax = 0; float ay = 0; float az = 0;
    float gx = 0; float gy = 0; float gz = 0;
    mysensor.readAccelerationRaw(ax, ay, az);
    mysensor.readGyroRaw(gx, gy, gz);
    myfilter.update(ax, ay, az, gx, gy, gz,
      lastMx, lastMy, lastMz, lastAlt, _dt_s);
  }

  if (Serial2.available()) {
    Serial2Event();
  }
  if(cmd_received){
    String _command = up_cmd;
    up_cmd = "";
    cmd_received = false;
    Process_Command(_command);
  }
}
