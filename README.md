
# ESP32-MotorTest

A Repository for BLDC Test and Data Collection with ESP32 Controller

## Flash BL-32 ESC Firmware (with Arduino Nano)

0. Prepare a Arduino Nano board to serve as the connection interface between PC and ESC:

![arduino-nano-flash-esc](.\\README_resources\\arduino-nano-flash-esc.jpg "Picture of the Arduino Nano")

    Pay attention to the port definitions.

1. Connect the Arduino board to the computer with the USB cable, then open the "BLHeliSuit32Test" program.

    You can find a copy of the program in this repository, under the "Tools" folder.

2. Program the Arduino to make the interface:

![make_interface](.\\README_resources\\make_interface.png "Picture of the Make interface session")

    Select "Nano w/ATmega328(old bootloader)", then make the 4way-interface. Make sure you have selected the right COM Port at the bottom of this page.

3. Connect the ESC with Arduino:

    As shown in the first figure, connect the signal input of the ESC to Arduino's "D3" port, and the "GND" to any of the "GND" port on the arduino.

    If your Arduino's port configuration is correct and ESC's input port is a 3-pin wire, you can directly plug the ESC to Arduino's "D3-D2-GND" ports without re-wiring.

4. Select the interface in the "BLHeliSuit32Test" program:

![select_interface](.\\README_resources\\select_interface.png "Picture of the interface selection")

    Choose the interface as "4way-if", then click "Connect". Make sure you have selected the correct COM Port.

5. Read the settings and Adjust:

![read_setup](.\\README_resources\\read_setup.png "Picture of the ESC setup")

    Click "Read Setup" and wait for the configurations to be loaded. Then you can make adjustments and click "Write Setup" to refresh the ESC. The descriptions of these setups can be found at "BLHeli_32_Info->manual.pdf".

    **Important**: If you want to collect the voltage, current, temperature or rotation speed from the ESC, make sure to turn "Auto Telemetry" to "On"! (Then the ESC will send back the data once every 32ms) The data is sent through the "Tx" port of the ESC.

    If multiple ESCs are used, all the "Tx"s can be connected together to one "Rx" port on the Autopilot or the Microcontroller.

## ESP32 Settings

Overview of the ESP32-Wroom-DA module:

![ESP32WROOMDA_pins](.\\README_resources\\ESP32WROOMDA_pins.png "Picture of the ESP32 Pins assignments")

- Devices Connection

  1. **SD card reader**: CS-5, SCK-18, MOSI-23, MISO-19
  2. **OLED Screen**: SCL-22, SDA-21
  3. **BLHeli32 ESC**: Tx-16, Dshot-13

- Arduino Libraries Needed
  
  1. **SD** by Arduino;
  2. **NTPClient** by Fabrice Weinberg; **Time** by Michael Margolis;
  3. **Adafruit MPU6050** by Adafruit; **Adafruit HMC5883 Unified** by Adafruit; **Adafruit BMP085 Library** by Adafruit;
  4. **EspSoftwareSerial** by Dirk Kaar and Peter Lerup

- Arduino IDE Settings

  ![arduino_ide_settings](.\\README_resources\\arduino_ide_settings.png "Picture of the Arduino IDE settings")

  Pay attention that **Flash Mode** is set to **DIO**

  