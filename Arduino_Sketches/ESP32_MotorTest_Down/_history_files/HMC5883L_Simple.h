/**
 * Simple HMC5883L Compass Library
 *
 * Implements a simple interface to get a Heading in Degrees out of a 
 * HMC5883L based compass module.
 * 
 * Datasheet: http://goo.gl/w1criV
 * 
 * Copyright (C) 2014 James Sleeman
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a 
 * copy of this software and associated documentation files (the "Software"), 
 * to deal in the Software without restriction, including without limitation 
 * the rights to use, copy, modify, merge, publish, distribute, sublicense, 
 * and/or sell copies of the Software, and to permit persons to whom the 
 * Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
 * THE SOFTWARE.
 * 
 * @author James Sleeman, http://sparks.gogo.co.nz/
 * @license MIT License
 */

// Please note, the Arduino IDE is a bit retarded, if the below define has an
// underscore other than _h, it goes mental.  Wish it wouldn't  mess
// wif ma files!
#ifndef HMC5883LSimple_h
#define HMC5883LSimple_h

/* NOTE:
 * There are both 7- and 8-bit versions of I2C addresses.
 * 7 bits identify the device, and the eighth bit determines
 * if it's being written to or read from. The Wire library uses
 * 7 bit addresses throughout. If you have a datasheet or sample code 
 * that uses 8 bit address, you'll want to drop the low bit
 * (i.e. shift the value one bit to the right), yielding an 
 * address between 0 and 127.
 * 
 * The HMC datasheet says...
 * " The default (factory) HMC5883L 7-bit slave address is 0x3C for write operations, or 0x3D for read operations. "
 * which is of course silly, what they mean is that these are 8=bit addresses and thus we want to shift right 
 * one of them to get the 7-bit address that Wire wants
 * 
 * 0x3C = 111100
 * >> 1 = 11110
 *      = 0x1E
 * 
 * 0x3D = 111101
 * >> 1 = 11110
 *      = 0x1E
 *   
 */

#define COMPASS_I2C_ADDRESS  0x3C >> 1

#define COMPASS_CONFIG_REGISTER_A 0x00
#define COMPASS_CONFIG_REGISTER_B 0x01
#define COMPASS_MODE_REGISTER     0x02
#define COMPASS_DATA_REGISTER     0x03

// We use 16 bits for storing various configs
//  xxxxxxxxxxxxxxMM
//  MODE:

#define COMPASS_CONTINUOUS 0x00
#define COMPASS_SINGLE     0x01
#define COMPASS_IDLE       0x02

//  xxxxxxxxxxxSSSxx
//  SCALE:
//   A lower value indicates a higher precision
//   but "noisier", magentic noise may necessitate
//   you to choose a higher scale.

#define COMPASS_SCALE_088  0x00 << 2
#define COMPASS_SCALE_130  0x01 << 2
#define COMPASS_SCALE_190  0x02 << 2
#define COMPASS_SCALE_250  0x03 << 2
#define COMPASS_SCALE_400  0x04 << 2
#define COMPASS_SCALE_470  0x05 << 2
#define COMPASS_SCALE_560  0x06 << 2
#define COMPASS_SCALE_810  0x07 << 2

//  xxXXXYYYZZZxxxxx
//  ORIENTATION: 
#define COMPASS_NORTH 0x00 
#define COMPASS_SOUTH 0x01
#define COMPASS_WEST  0x02
#define COMPASS_EAST  0x03
#define COMPASS_UP    0x04
#define COMPASS_DOWN  0x05

// When "pointing" north, define the direction of each of the silkscreen'd arrows
// (imagine the Z arrow points out of the top of the device) only N/S/E/W are allowed
#define COMPASS_HORIZONTAL_X_NORTH  ( (COMPASS_NORTH << 6)  | (COMPASS_WEST  << 3)  | COMPASS_UP    ) << 5
#define COMPASS_HORIZONTAL_Y_NORTH  ( (COMPASS_EAST  << 6)  | (COMPASS_NORTH << 3)  | COMPASS_UP    ) << 5
#define COMPASS_VERTICAL_X_EAST     ( (COMPASS_EAST  << 6)  | (COMPASS_UP    << 3)  | COMPASS_SOUTH ) << 5
#define COMPASS_VERTICAL_Y_WEST     ( (COMPASS_UP    << 6)  | (COMPASS_WEST  << 3)  | COMPASS_SOUTH ) << 5

class HMC5883L_Simple
{
public:
  HMC5883L_Simple();
  // The scale can be adjusted to one of several levels, you can probably leave it at the default.
  // Essentially this controls how sensitive the device is.
  //   Options are 088, 130 (default), 190, 250, 400, 470, 560, 810
  // Specify the option as COMPASS_SCALE_xxx
  // Lower values are more sensitive, higher values are less sensitive.
  // The default is probably just fine, it works for me.  If it seems very noisy
  // (jumping around), incrase the scale to a higher one.
  // Compass.SetScale(COMPASS_SCALE_130);
  void SetScale( uint16_t sampling_mode );
  // The compass has 3 axes, but two of them must be close to parallel to the earth's surface to read it, 
  // (we do not compensate for tilt, that's a complicated thing) - just like a real compass has a floating 
  // needle you can imagine the digital compass does too.
  //
  // To allow you to mount the compass in different ways you can specify the orientation:
  //   COMPASS_HORIZONTAL_X_NORTH (default), the compass is oriented horizontally, top-side up. when pointing North the X silkscreen arrow will point North
  //   COMPASS_HORIZONTAL_Y_NORTH, top-side up, Y is the needle,when pointing North the Y silkscreen arrow will point North
  //   COMPASS_VERTICAL_X_EAST,    vertically mounted (tall) looking at the top side, when facing North the X silkscreen arrow will point East
  //   COMPASS_VERTICAL_Y_WEST,    vertically mounted (wide) looking at the top side, when facing North the Y silkscreen arrow will point West
  // Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);
  void SetOrientation( uint16_t sampling_mode );
  // Magnetic Declination is the correction applied according to your present location
  // in order to get True North from Magnetic North, it varies from place to place.
  // The declination for your area can be obtained from http://www.magnetic-declination.com/
  // Take the "Magnetic Declination" line that it gives you in the information, 
  // Examples:
  //   Christchurch, 23° 35' EAST
  //   Wellington  , 22° 14' EAST
  //   Dunedin     , 25° 8'  EAST
  //   Auckland    , 19° 30' EAST  
  // Compass.SetDeclination(23, 35, 'E');
  void SetDeclination( int declination_degs , int declination_mins, char declination_dir );
  // The device can operate in SINGLE (default) or CONTINUOUS mode
  //   SINGLE simply means that it takes a reading when you request one
  //   CONTINUOUS means that it is always taking readings
  // for most purposes, SINGLE is what you want.
  // Compass.SetSamplingMode(COMPASS_SINGLE);
  void SetSamplingMode( uint16_t sampling_mode );
  // Get a heading in degrees
  float GetHeadingDegrees();
  float GetGain();
  void GetMagneticFieldRaw(float& mx, float& my, float& mz);
  
protected:
  void     Write(uint8_t address, uint8_t byte);
  uint8_t  Read(uint8_t register_address, uint8_t buffer[], uint8_t length);
  
  struct MagnetometerSample
  {
    int X;
    int Y;
    int Z;
  };
  
  MagnetometerSample ReadAxes();
      
  uint16_t mode;
  float declination_offset_radians;
  uint16_t current_scale;
  
private:
  uint8_t i2c_address;
};

#endif