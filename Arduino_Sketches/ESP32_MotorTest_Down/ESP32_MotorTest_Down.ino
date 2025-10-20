#include <Arduino.h>
#include <Wire.h>  // Must include Wire here, otherwise all .h files won't include Wire
#include <WiFi.h>
#include <WiFiUdp.h>
#include <TimeLib.h>

#include "SD_manager.h"
#include "12864_display.h"
#include "IMU_GY85_manager.h"
#include "helper_functions.h"


#define LED_PIN 2  // LED on means initializaiton done, LED blink means data sending
#define LED_TOGGLE() digitalWrite(LED_PIN, digitalRead(LED_PIN) ^ 1)

/*
MCU (down) Functionality:
Measure the acceleration, gyro and gesture in the world frame (ENU), and measure angular velocity in the body frame (FLU);
Record and send the data by 10Hz;
Synchronize time through UART;
Send data through WiFi if available;
Communicate with MCU (up) using UART, receive commands and send the data;
Display the real-time data on the screen.
*/

const char* wifi_ssid = "BioInBot_Lab";
const char* wifi_pswd = "11223344";
WiFiUDP udp;
String udpAddress = "192.168.31.240";  // receiver IP
int udpPort = 8888;  // receiver port

MyDisplay myscreen;  // scl-33, sda-32
MyIMU_GY85 mysensor;  // scl-22, sda-21
SDCard mysd(18, 23, 19, 5);  // sck = 18; miso = 23; mosi = 19; cs = 5;
// Serial2： rx-16, tx-17

hw_timer_t *timer = NULL;
bool cmd_received = false;
bool start_log = false;  // data sending status
bool start_wifi_broadcast = false;  // wifi data sending status
uint32_t DEFAULT_TIME = 1357041600;  // Jan 1 2013
uint32_t screen_fresh_cnt = 0;
uint32_t startRecordLT = 0;  // start recording local time
uint32_t lastDataLoopT = 0;
uint32_t lastSensorFastUpdate = 0;
uint32_t lastSensorSlowUpdate = 0;
float lastAx = 0; float lastAy = 0; float lastAz = 0;  // in m/s^2
float lastGx = 0; float lastGy = 0; float lastGz = 0;  // in rad/s
float lastMx = 0; float lastMy = 0; float lastMz = 0;  // in μT
float lastTmp = 0;  // in degree centigrade
String up_cmd = "";  // store the received cmd from MCU (up)

void onTimer() {
  if (screen_fresh_cnt < 800) {
    myscreen.set_Line1("T:" + getCurrentTime());
    myscreen.set_Line2("acc:"+String(lastAx,1)+","+String(lastAy,1)+","+String(lastAz,1));
    myscreen.set_Line3("gyo:"+String(lastGx,1)+","+String(lastGy,1)+","+String(lastGz,1));
    myscreen.set_Line4("mag:"+String(lastMx,1)+","+String(lastMy,1)+","+String(lastMz,1));
    myscreen.set_Line5("tmp:"+String(lastTmp,2));
    myscreen.OLED_UpdateRam();
    myscreen.OLED_Refresh();
    screen_fresh_cnt++;
  }
  else {
    myscreen.OLED_Reset_Display();
    screen_fresh_cnt = 0;
  }
}

void setup(){
  Serial.begin(115200);  // usb
  while (!Serial)
    delay(10);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.printf("\n");
  Serial.println("##### Start the Data Recording Board (Down) #####");

  Serial.println("Starting the data serial (Serial 2: Rx-16, Tx-17).");
  Serial2.begin(115200);  // Rx-16, Tx-17
  while(!Serial2)
    delay(10);
  Serial.println("Initialized data serial.");
  up_cmd.reserve(200);

  Serial.println("Starting the OLED screen.");
  myscreen.init();
  timer = timerBegin(1000000);
  if(timer == NULL) {
    Serial.println("OLED Timer initialization failed!");
    while(1);
  }
  timerAlarm(timer, 400000, true, 0);
  timerAttachInterrupt(timer, &onTimer);
  Serial.println("Screen initialized.");
  delay(50);

  Serial.print("Trying to conncet to Lab WiFi.");
  WiFi.mode(WIFI_STA);
  uint8_t wifi_connect_tms = 0;
  WiFi.begin(wifi_ssid, wifi_pswd);
  while(WiFi.status() != WL_CONNECTED && wifi_connect_tms < 5){
    Serial.print(".");
    wifi_connect_tms += 1;
    delay(200);
  }
  if(WiFi.status() == WL_CONNECTED){
    Serial.println("\nConnected to the Lab WiFi.");
    Serial.print("Local MCU (down) IP: ");
    Serial.println(WiFi.localIP());
    udp.begin(0);
  }
  else{
    Serial.println("\nWiFi not connected.");
  }

  Serial.println("Set I2C to fast mode (400kHz).");
  Wire.setClock(400000);
  
  Serial.println("Starting the SD card and sensor.");
  int init_count = 0;
  while(!mysd.checkCardStatus() && init_count < 10){
    mysd.init();
    delay(200);
    init_count += 1;
  }
  init_count = 0;
  while(!mysensor.checkInitStatus() && init_count < 10){
    mysensor.init();
    delay(200);
    init_count += 1;
  }
  if(!(mysd.checkCardStatus() && mysensor.checkInitStatus())){
    Serial.println("Device initialization failed!");
    while(1);
  }
  Serial.println("All modules initialized.");
  Serial.println("Data Recording Board (Down) Initialization Done.");
  LED_TOGGLE();
  delay(50);
}


void loop(){
  ////////////// MCU (up) command /////////////////
  if (Serial2.available()) {
    Serial2Event();
  }
  if(cmd_received){
    String _command = up_cmd;
    up_cmd = "";
    cmd_received = false;
    Process_Command(_command);
  }
  /////////////////////////////////////////////////

  if (millis() - lastDataLoopT > 100) {
    lastDataLoopT = millis();
    char resultStr[200];
    sprintf(resultStr, "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f",
          lastAx, lastAy, lastAz, lastGx, lastGy, lastGz, lastMx, lastMy, lastMz, lastTmp);

    if(start_log){
      String _glb_t = (timeStatus() == timeNotSet)? "N/A" : getCurrentHmsTime();
      String _lca_t = String((millis() - startRecordLT)/1000.0f, 2);
      if(mysd.checkFileStatus()){
        mysd.record(_glb_t + " " + _lca_t + " " + String(resultStr));
      }
      Serial2.println(String(resultStr));
      LED_TOGGLE();
    }

    // Serial.println(resultStr);

    if(start_wifi_broadcast && WiFi.status() == WL_CONNECTED){
      udp.beginPacket(udpAddress.c_str(), udpPort);
      udp.print(resultStr);
      udp.endPacket();
    }
  }

  if(millis() - lastSensorSlowUpdate > 100) {
    lastSensorSlowUpdate = millis();
    float _mx, _my, _mz;
    mysensor.readMagnetRaw(_mx, _my, _mz);
    lastMx = _mx; lastMy = -_my; lastMz = -_mz;
    mysensor.readTemperatureRaw(lastTmp);
  }

  uint32_t _dt = millis() - lastSensorFastUpdate;
  if(_dt > 50) {
    lastSensorFastUpdate = millis();
    float _ax, _ay, _az, _gx, _gy, _gz;
    mysensor.readAccelerationRaw(_ax, _ay, _az);
    lastAx = _ax; lastAy = -_ay; lastAz = -_az - 9.81f;
    mysensor.readGyroRaw(_gx, _gy, _gz);
    lastGx = _gx; lastGy = -_gy; lastGz = -_gz;
  }
}

void Process_Command(String command) {
  command.trim();
  if (command == "Start_rcd") {
    start_log = true;
    startRecordLT = millis();
    Serial.println("Data recording started.");
  }
  else if (command == "Stop_rcd") {
    start_log = false;
    Serial.println("Data recording stopped.");
    digitalWrite(LED_PIN, HIGH);
  }
  else if (command == "Create_f"){
    String data_headline = "global_t local_t acc_x acc_y acc_z omega_x omega_y omega_z mag_x mag_y mag_z temp";
    int file_flag;
    if(timeStatus() != timeNotSet){
      mysd.set_folder_name(getCurrentDate());
      file_flag = mysd.create_file(data_headline, "MTest_"+getCurrentHmsTime());
    }
    else{
      file_flag = mysd.create_file(data_headline);
    }
    if(file_flag < 0){
      Serial.println("SD file creation failed.");
    }
    else{
      Serial.println("SD file creation succeeded.");
    }
  }
  else if (command.startsWith("T:")) {
    String valueStr = command.substring(2);
    unsigned long time_interger = valueStr.toInt();
    if(time_interger >= DEFAULT_TIME){
      setTime(time_interger);
      Serial.println("Global time updated.");
      Serial2.println("T_ack");
    }
  }
  else if (command == "Get_ip") {
    if(WiFi.status() == WL_CONNECTED){
      WiFi.begin(wifi_ssid, wifi_pswd);
      udp.begin(0);
      Serial.println("WiFi not connected, retrying.");
      Serial2.println("IP:N/A");
    }
    else{
      Serial2.print("IP:");
      Serial2.println(WiFi.localIP());
    }
  }
  else if (command.startsWith("IP:")) {  // set udp target IP address
    String valueStr = command.substring(3);
    if(isValidIP(valueStr)){
      udpAddress = valueStr;
      myscreen.set_Checkbox(false);
      Serial.println("UDP target IP updated: " + valueStr);
      Serial2.println("IP_ack");
    }
  }
  else if (command.startsWith("P:")) {  // set udp target port number
    String valueStr = command.substring(2);
    int port_interger = valueStr.toInt();
    if(port_interger >= 1024){
      udpPort = port_interger;
      myscreen.set_Checkbox(false);
      Serial.println("UDP target port updated: " + valueStr);
      Serial2.println("P_ack");
    }
  }
  else if (command == "Start_brd") {
    start_wifi_broadcast = true;
    myscreen.set_Checkbox(true);
    Serial.println("Data broadcasting started.");
  }
  else if (command == "Stop_brd") {
    start_wifi_broadcast = false;
    myscreen.set_Checkbox(false);
    Serial.println("Data broadcasting stopped.");
  }
  else if (command.length() > 0) {
    Serial.print("Unknown command: ");
    Serial.println(command);
  }
}
