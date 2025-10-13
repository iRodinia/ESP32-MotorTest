#include <Arduino.h>
#include <Wire.h>  // Must include Wire here, otherwise all .h files won't include Wire
#include <TimeLib.h>

#include "SD_manager.h"
#include "12864_display.h"
#include "IMU_manager.h"
#include "Serial_sender.h"

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

uint32_t start_record_time = 0;
uint32_t last_loop_time = 0;
String curr_time_str = "";

void setup(){
  Serial.begin(115200);  // usb
  while (!Serial)
    delay(10);

  Serial2.begin(115200);  // Rx-16, Tx-17
  while(!Serial2)
    delay(10);
  
  Serial.printf("\n");
  Serial.println("##### Start the Data Recording Board (Down) #####");
  myscreen.init();
  mysensor.init();
  mysd.init();
  delay(300);
  if(!(timer.getStatus()&&mysd.checkCardStatus()&&mysensor.checkInitStatus()&&myuart.getStatus())){
    Serial.println("Device initialization failed!");
    while(1);
  }
  Serial.println("All modules initialized.");

  Serial.println("Syncronizing time with MCU (up) through UART.");
  myscreen.set_Line1("T:N/A");
  myscreen.set_Line2("Syncing");
  myscreen.set_Line3("Time...");
  myscreen.set_Line4(" ");
  myscreen.set_Checkbox(false);
  myscreen.OLED_UpdateRam();
  myscreen.OLED_Refresh();

  setSyncProvider(serial2RequestSync);
  setSyncInterval(60);
  while(timeStatus() == timeNotSet){
    while(!Serial2.avaliable())
      delay(10);
    unsigned long pctime;
    const unsigned long DEFAULT_TIME = 1357041600;  // Jan 1 2013
    if(Serial2.find("T")) {
      pctime = Serial2.parseInt();
      if(pctime >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
        setTime(pctime); // Sync Arduino clock to the time received on the serial port
      }
    }
  }
  
  Serial.println("Time synchronized.");
  myscreen.set_Line1("T:"+getCurrentHmsTime());
  myscreen.set_Line4("Done.");
  myscreen.OLED_UpdateRam();
  myscreen.OLED_Refresh();
  delay(300);

  myscreen.set_Line1("T:"+getCurrentHmsTime());
  myscreen.set_Line2("Creating");
  myscreen.set_Line3("Log File...");
  myscreen.set_Line4(" ");
  myscreen.OLED_UpdateRam();
  myscreen.OLED_Refresh();

  String data_headline = "t vel_z acc_z";
  mysd.set_folder_name(getCurrentDate());
  int file_flag = mysd.create_file("MTest_"+getCurrentHmsTime(), data_headline);

  if(file_flag < 0){
    myscreen.set_Line1("T:"+getCurrentHmsTime());
    myscreen.set_Line4("Failed!");
    myscreen.OLED_UpdateRam();
    myscreen.OLED_Refresh();
    while(1);
  }
  else{
    myscreen.set_Line1("T:"+getCurrentHmsTime());
    myscreen.set_Line4("Succeeded!");
    myscreen.OLED_UpdateRam();
    myscreen.OLED_Refresh();
  }
  delay(300);

  myscreen.set_Line1("T:"+getCurrentHmsTime());
  myscreen.set_Line2("Calibrating");
  myscreen.set_Line3("Hold Still!");
  myscreen.set_Line4(" ");
  myscreen.OLED_UpdateRam();
  myscreen.OLED_Refresh();

  mysensor.calibrateMPU6050();

  myscreen.set_Line1("T:"+getCurrentHmsTime());
  myscreen.set_Line4("Done.");
  myscreen.OLED_UpdateRam();
  myscreen.OLED_Refresh();
  delay(300);

  myscreen.set_Line1("T:"+getCurrentHmsTime());
  myscreen.set_Line2("Acc: 0.00");
  myscreen.set_Line3("Vel: 0.00");
  myscreen.set_Line4("Pending...");
  myscreen.OLED_UpdateRam();
  myscreen.OLED_Refresh();
}

void loop(){

}

time_t serial2RequestSync()
{
  Serial2.write("T_sync_req");  
  return 0;
}

String getCurrentHmsTime(){
  if(timeStatus() != timeNotSet){
    char timeStr[8];
    snprintf(timeStr, 8, "%02d:%02d:%02d", hour(), minute(), second());
    return String(timeStr);
  }
  return "";
}

String getCurrentDate(){
  if(timeStatus() != timeNotSet){
    char timeStr[10];
    snprintf(timeStr, 10, "%04d-%02d-%02d", year(), month(), day());
    return String(timeStr);
  }
  return "";
}