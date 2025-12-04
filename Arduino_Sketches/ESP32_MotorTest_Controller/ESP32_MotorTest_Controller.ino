#include <Arduino.h>
#include <DShotRMT.h>

#include "Serial_manager.h"

#define MOTOR_CMD_PIN 27

uint16_t motorThrottle = DSHOT_CMD_MOTOR_STOP;
uint32_t lastEscPrint = 0;

DShotRMT myMotor(MOTOR_CMD_PIN, DSHOT300);

void setup()
{
    Serial.begin(115200);
    delay(50);
    Serial1.begin(115200, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX);
    delay(50);

    myMotor.begin();
}

void loop()
{
    serial0CmdEvent();
    serial1DataEvent();

    if (motorThrottle == 0)
    {
        myMotor.sendCommand(DSHOT_CMD_MOTOR_STOP);
    }
    else {
        myMotor.sendThrottle(motorThrottle);
    }

    uint32_t localT = millis();
    if (localT - lastEscPrint >= 200) {
        lastEscPrint = localT;
        Serial.printf("Temp: %.2f, Vol: %.2f, Cur: %.2f, Pwr_cost: %.2f, Erpm: %d \n", 
            myEscData.temperature, myEscData.voltage, myEscData.current, myEscData.consumption,
            myEscData.erpm);
    }
}


void parseSerial0Cmd(String command) {
    int throttle_value = command.toInt();
    if (throttle_value <= 0) motorThrottle = 0;
    else if (throttle_value <= 100) motorThrottle = uint16_t(DSHOT_THROTTLE_MIN + (DSHOT_THROTTLE_MAX-DSHOT_THROTTLE_MIN) * (throttle_value/100.0));
    Serial.printf("Set throttle to %d \n", motorThrottle);
}