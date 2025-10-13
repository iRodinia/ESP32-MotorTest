#include <Arduino.h>
#include <Wire.h>  // Must include Wire here, otherwise all .h files won't include Wire

#include "SD_manager.h"
#include "12864_display.h"
#include "IMU_manager.h"
#include "Serial_sender.h"

/*
MCU (down) Functionality:
Measure the Z-axis acceleration; Estimate the Z-axis velocity;
Record the data in SD card by 20Hz;
Communicate with MCU (up) using UART, receive the start command and send the data;
Display the real-time data on the screen.
*/

const char* ssid = "BioInBot_Lab";
const char* password = "11223344";
WiFiUDP espUDP;
uint32_t loop_time = 0;

void setup(){
  Serial.begin(115200);
  while (!Serial)
    delay(10);
  
  Serial.printf("\n");
  Serial.println("##### Start the Motor Test Recording Board (Down) #####");
  Serial.print("Connecting to WiFi.");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nWiFi connected.\n");

  //这里不对，要单独写init函数，把定义写在外面！！！！！！！！！
  MyTimer timer(espUDP);
  SDCard mysd(18, 19, 23, 5);
  MyDisplay myscreen();  // scl-33, sda-32
  myIMU mysensor();  // scl-22, sda-21
  mySender myuart(26, 25, 115200);
  ///////////////////////////////////////////////////////////
  delay(500);
  if(!(timer.getStatus()&&mysd.checkCardStatus()&&mysensor.checkInitStatus()&&myuart.getStatus())){
    Serial.println("Device initialization failed!");
    while(1);
  }

  myscreen.set_Line1("IP:" + WiFi.localIP().toString());
  myscreen.set_Line2("Creating");
  myscreen.set_Line3("Log File");
  myscreen.set_Line4(" ");
  myscreen.set_Checkbox(false);
  myscreen.OLED_UpdateRam();
  myscreen.OLED_Refresh();

  String current_time = timer.getCurrentDateTime();
  String current_date = current_time.substring(0, 10);
  String current_sec = current_time.substring(10);
  String data_headline = "t vel_z acc_z";
  mysd.set_folder_name(current_date);
  int file_flag = mysd.create_file("MotorTest"+current_sec, data_headline);

  if(file_flag < 0){
    myscreen.set_Line4("Failed!");
    myscreen.OLED_UpdateRam();
    myscreen.OLED_Refresh();
    while(1);
  }
  else{
    myscreen.set_Line4("Succeeded!");
    myscreen.OLED_UpdateRam();
    myscreen.OLED_Refresh();
  }
  delay(500);

  myscreen.set_Line2("Calibrating");
  myscreen.set_Line3("Hold Still!");
  myscreen.set_Line4(" ");
  myscreen.OLED_UpdateRam();
  myscreen.OLED_Refresh();

  myIMU.calibrateMPU6050();

  myscreen.set_Line4("Done.");
  myscreen.OLED_UpdateRam();
  myscreen.OLED_Refresh();
  delay(500);

  myscreen.set_Line2("Acc: 0.00");
  myscreen.set_Line3("Vel: 0.00");
  myscreen.set_Line4("Pending...");
  myscreen.OLED_UpdateRam();
  myscreen.OLED_Refresh();
  delay(500);

  
}

void loop(){

}