#include <Arduino.h>
#include <Wire.h>  // Must include Wire here, otherwise all .h files won't include Wire
#include <TimeLib.h>

#include "SD_manager.h"
#include "12864_display.h"
#include "IMU_manager.h"
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
MyIMU mysensor;  // scl-22, sda-21
SDCard mysd(18, 19, 23, 5);

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
  timerStart(timer);

  myscreen.set_Line2("Calibrating");
  myscreen.set_Line3("Hold Still!");
  myscreen.set_Line4(" ");

  mysensor.calibrateMPU6050();

  myscreen.set_Line4("Done.");
  delay(300);

  myscreen.set_Line2("Acc: 0.00");
  myscreen.set_Line3("Vel: 0.00");
  myscreen.set_Line4("Pending...");

  Send_Ready_Sgn();
}

static unsigned long last_loop_local_time = 0;
float temp_vel = 0;

void loop(){
  unsigned long _dt = millis() - last_loop_local_time;
  if (_dt > 50) {
    last_loop_local_time = millis();
    float ax, ay, az;
    if(mysensor.readAccelerationRaw(ax, ay, az) == 0){
      temp_vel += az * _dt/1000;  // Vel 估计现在还有很大的问题
      if(file_created){
        String _glb_t = (timeStatus()==timeNotSet)? "N/A" : getCurrentHmsTime();
        String _lca_t = String(millis()-start_record_local_time);
        mysd.record(_glb_t + " " + _lca_t + " " + String(az, 3) + " " + String(temp_vel, 3));
      }
      if(data_sending){
        Serial2.printf("z-vel:%.3f,z-acc:%.3f\n", temp_vel, az);
        myscreen.set_Checkbox(true);
      }
      myscreen.set_Line2("Acc:" + String(az, 3));
      myscreen.set_Line3("Vel:" + String(temp_vel, 3));
    }
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
