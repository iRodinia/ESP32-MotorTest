#ifndef SERIAL_MANAGER_DOWN
#define SERIAL_MANAGER_DOWN

char serial2Buffer[256];
uint16_t serial2BufferIndex = 0;
unsigned long serial2LastReceiveTime = 0;

bool read_serial2_data(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &mx, float &my, float &mz, float &temp) {
  if (Serial2.available() <= 0) {
    if (serial2BufferIndex > 0 && (millis() - serial2LastReceiveTime > 800)) {
      serial2BufferIndex = 0;
      serial2Buffer[0] = '\0';
    }
    return false;
  }

  while (Serial2.available() > 0) {
    char c = Serial2.read();
    serial2LastReceiveTime = millis();

    if (serial2BufferIndex >= 255) {
      serial2BufferIndex = 0;
      serial2Buffer[0] = '\0';
      continue;
    }
    
    serial2Buffer[serial2BufferIndex] = c;
    serial2BufferIndex++;
    if (c == '\n') {
      serial2Buffer[serial2BufferIndex] = '\0';
      int parsedCount = sscanf(serial2Buffer, 
        "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
        &ax, &ay, &az,
        &gx, &gy, &gz,
        &mx, &my, &mz,
        &temp
      );
      if (parsedCount == 10) {
        serial2BufferIndex = 0;
        serial2Buffer[0] = '\0';
        return true;
      }
      else {
        serial2BufferIndex = 0;
        serial2Buffer[0] = '\0';
      }
    }
  }
  return false;
}

void flush_serial2_buffer() {
  while (Serial2.available() > 0) {
      Serial2.read();
  }
  serial2BufferIndex = 0;
  serial2Buffer[0] = '\0';
}



#endif