#include <Arduino.h>
#include <Wire.h>
#include <DShotRMT.h>

#include "Serial_manager.h"
#include "12864_display.h"
#include "RPMObserver.h"
#include "My_ads1115_sensor.h"

#define MOTOR_CMD_PIN 27
#define SCREEN_SCL_PIN 22
#define SCREEN_SDA_PIN 21

float motorVoltCritical = 21.5;
uint16_t motorDShotVal = DSHOT_CMD_MOTOR_STOP;
uint16_t motorMaxDShot = 500;  // a value between 1 to 1000, full throttle range is 1000

bool esc_connected = false;
float volt_reading = 0.0;
float force_reading = 0.0;
float rpm_estm = 0.0;

uint16_t low_power_cnt = 0;
bool low_power_flag = false;

uint32_t startLT = 0;
uint32_t lastADCUpdate = 0;
uint32_t lastPrint = 0;

DShotRMT myMotor(MOTOR_CMD_PIN, DSHOT300);
RpmObserver motorObserver(7, 0.2f, 0.01f);
MyADS1115Sensor myADC;
SingleScreen myScreen(SCREEN_SCL_PIN, SCREEN_SDA_PIN);
TaskHandle_t MotorControlTaskHandle;
TaskHandle_t SensorTaskHandle;

void MotorControlTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1); 

    for(;;) {
        if (motorDShotVal == 0 || low_power_flag) {
            myMotor.sendCommand(DSHOT_CMD_MOTOR_STOP);
        }
        else {
            myMotor.sendThrottle(motorDShotVal);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void SensorTask(void *pvParameters) {
    for (;;) {
        serial0CmdEvent();
        serial1DataEvent();
        serial2CmdEvent();

        uint32_t time_now = millis();
        esc_connected = time_now - myEscData.alive_ts < 1000;

        if (esc_connected) {
            float restVolt = myEscData.voltage + myEscData.current * 0.03;
            if (restVolt < motorVoltCritical) {
                if (low_power_cnt < 100) {
                    low_power_cnt++;
                }
                else{
                    low_power_flag = true;
                }
            }
            else{
                low_power_cnt = 0;
            }

            motorObserver.update(myEscData.erpm);
            rpm_estm = motorObserver.getRpm();
            volt_reading = myEscData.voltage;
        }

        if (lastADCUpdate - time_now > 100) {
            lastADCUpdate = time_now;
            myADC.readForce(force_reading);
        }

        vTaskDelay(pdMS_TO_TICKS(5)); 
    }
}


void setup()
{
    String init_message = initAllSerials();

    Wire.begin();
    delay(50);
    Wire.setClock(400000);  // fast mode
    delay(50);

    myMotor.begin();
    init_message += myADC.init();
    myScreen.setChannel(0);
    myScreen.setHeadLine("Motor Control");
    myScreen.setLineLabels("Volt:", "CMD :", "RPM :", "Forc:");
    myScreen.init();

    xTaskCreatePinnedToCore(
        MotorControlTask,            /* 任务函数 */
        "MotorControlTask",          /* 任务名称 */
        2048,                        /* 任务栈大小 (字节) */
        NULL,                        /* 传递给任务的参数 */
        configMAX_PRIORITIES-1,      /* 任务优先级 (1 为默认优先级，比 loop 的 1 优先级低一点以防卡死，也可设为 1) */
        &MotorControlTaskHandle,     /* 任务句柄 */
        0                            /* 核心编号 (0 = PRO_CPU) */
    );

    xTaskCreatePinnedToCore(
        SensorTask,
        "SensorTask",
        4096,
        NULL,
        1,
        &SensorTaskHandle,
        0
    );

    if(init_message.length() >= 2) {
        Serial.println(init_message);
        Serial.println("Halting.");
        while(1);
    }

    startLT = millis();
}

void loop()
{
    myScreen.updateData(volt_reading, motorDShotVal, rpm_estm, force_reading);
    myScreen.refresh();

    static float currentT = 0.0;
    uint32_t localT = millis();
    if (localT - lastPrint >= 80) {
        lastPrint = localT;
        currentT = (localT - startLT) / 1000.0;
        if (low_power_flag) {
            Serial.println("Low Power.");
        }
        else {
            Serial.printf("%.2f,%.2f,%d,%.2f,%.2f\n", currentT, volt_reading, motorDShotVal, rpm_estm, force_reading);
            Serial2.printf("%.2f,%.2f,%d,%.2f,%.2f\n", currentT, volt_reading, motorDShotVal, rpm_estm, force_reading);
        }
    }
}


void parseSerial0Cmd(String command) {
    int target_speed = command.toInt();
    if (abs(target_speed) > motorMaxDShot) return;

    if (target_speed == 0) motorDShotVal = 0;
    else if (target_speed > 0) motorDShotVal = map(target_speed, 1, 1000, 1049, 2047);
    else motorDShotVal = map(target_speed, -1, -1000, 48, 1047);

    Serial.printf("Set throttle command to %d (/±1000)\n", motorDShotVal);
}