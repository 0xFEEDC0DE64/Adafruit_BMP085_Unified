/*!
 * @file Adafruit_BMP085_U.cpp
 *
 * @mainpage Adafruit BMP085 Pressure Sensor
 *
 * @section intro_sec Introduction
 *
 * This is a library for the BMP085 pressure sensor
 *
 * Designed specifically to work with the Adafruit BMP085 or BMP180 Breakout
 * ----> http://www.adafruit.com/products/391
 * ----> http://www.adafruit.com/products/1603
 *
 * These displays use I2C to communicate, 2 pins are required to interface.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit andopen-source hardware by purchasing products
 * from Adafruit!
 *
 * @section author Author
 *
 * Written by Kevin Townsend for Adafruit Industries.
 *
 * @section license License
 * BSD license, all text above must be included in any redistribution
 */

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#ifdef __AVR_ATtiny85__
#include "TinyWireM.h"
#define Wire TinyWireM
#else
#include <Wire.h>
#endif

//#define DEBUG_OUTPUT
#ifdef DEBUG_OUTPUT
#include <esp_log.h>
#define DBGPRNT(format, ...) ESP_LOGW("BMP085", format, ##__VA_ARGS__)
#else
#define DBGPRNT(format, ...)
#endif

#include <limits.h>
#include <math.h>

#include <espchrono.h>

#include "Adafruit_BMP085_U.h"

static bmp085_calib_data
    _bmp085_coeffs; // Last read accelerometer data will be available here
static uint8_t _bmp085Mode;

#define BMP085_USE_DATASHEET_VALS                                              \
  (0) //!< Set to 1 for sanity check, when true, will use values from datasheet

#if ARDUINO >= 100
#define WIRE_WRITE Wire.write
#define WIRE_READ Wire.read
#else
#define WIRE_WRITE Wire.send
#define WIRE_READ Wire.receive
#endif

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C
*/
/**************************************************************************/
static bool writeCommand(byte reg, byte value) {
  bool succ{true};

  Wire.beginTransmission((uint8_t)BMP085_ADDRESS);
  if (WIRE_WRITE((uint8_t)reg) != 1) { DBGPRNT("fail"); succ = false; }
  if (WIRE_WRITE((uint8_t)value) != 1) { DBGPRNT("fail"); succ = false; }
  if (const auto result = Wire.endTransmission(); result != I2C_ERROR_OK) { DBGPRNT("fail %hhu", result); succ = false; }

  return succ;
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/
static bool read8(byte reg, uint8_t *value) {
  bool succ{true};

  Wire.beginTransmission((uint8_t)BMP085_ADDRESS);
  if (WIRE_WRITE((uint8_t)reg) != 1) { DBGPRNT("fail"); succ = false; }
  if (const auto result = Wire.endTransmission(); result != I2C_ERROR_OK) { DBGPRNT("fail %hhu", result); succ = false; }

  if (const auto result = Wire.requestFrom((uint8_t)BMP085_ADDRESS, (byte)1); result != 1) { DBGPRNT("fail %hhu", result); succ = false; }

  if (const auto result = WIRE_READ(); result == -1) { DBGPRNT("fail %i", result); succ = false; }
  else if (succ) *value = result;

  //if (const auto result = Wire.endTransmission(); result != I2C_ERROR_OK) { DBGPRNT("fail %hhu", result); succ = false; }
  Wire.endTransmission();

  return succ;
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit value over I2C
*/
/**************************************************************************/
static bool read16(byte reg, uint16_t *value) {
  bool succ{true};

  Wire.beginTransmission((uint8_t)BMP085_ADDRESS);
  if (WIRE_WRITE((uint8_t)reg) != 1) { DBGPRNT("fail"); succ = false; }
  if (const auto result = Wire.endTransmission(); result != I2C_ERROR_OK) { DBGPRNT("fail %hhu", result); succ = false; }

  if (const auto result = Wire.requestFrom((uint8_t)BMP085_ADDRESS, (byte)2); result != 2) { DBGPRNT("fail %hhu", result); succ = false; }
  const auto result0 = WIRE_READ();
  const auto result1 = WIRE_READ();
  if (result0 == -1) { DBGPRNT("fail %i", result0); succ = false; }
  else if (result1 == -1) { DBGPRNT("fail %i", result1); succ = false; }

  if (succ) { *value = (result0 << 8) | result1; }

  //if (const auto result = Wire.endTransmission(); result != I2C_ERROR_OK) { DBGPRNT("fail %hhu", result); succ = false; }
  Wire.endTransmission();

  return succ;
}

/**************************************************************************/
/*!
    @brief  Reads a signed 16 bit value over I2C
*/
/**************************************************************************/
static bool readS16(byte reg, int16_t *value) {
  bool succ{true};
  uint16_t i;
  if (!read16(reg, &i)) { DBGPRNT("fail"); succ = false; }
  if (succ) *value = (int16_t)i;
  return succ;
}

/**************************************************************************/
/*!
    @brief  Reads the factory-set coefficients
*/
/**************************************************************************/
static bool readCoefficients(void) {
  bool succ{true};
#if BMP085_USE_DATASHEET_VALS
  _bmp085_coeffs.ac1 = 408;
  _bmp085_coeffs.ac2 = -72;
  _bmp085_coeffs.ac3 = -14383;
  _bmp085_coeffs.ac4 = 32741;
  _bmp085_coeffs.ac5 = 32757;
  _bmp085_coeffs.ac6 = 23153;
  _bmp085_coeffs.b1 = 6190;
  _bmp085_coeffs.b2 = 4;
  _bmp085_coeffs.mb = -32768;
  _bmp085_coeffs.mc = -8711;
  _bmp085_coeffs.md = 2868;
  _bmp085Mode = 0;
#else
  if (!readS16(BMP085_REGISTER_CAL_AC1, &_bmp085_coeffs.ac1)) { DBGPRNT("fail"); succ = false; }
  if (!readS16(BMP085_REGISTER_CAL_AC2, &_bmp085_coeffs.ac2)) { DBGPRNT("fail"); succ = false; }
  if (!readS16(BMP085_REGISTER_CAL_AC3, &_bmp085_coeffs.ac3)) { DBGPRNT("fail"); succ = false; }
  if (!read16(BMP085_REGISTER_CAL_AC4, &_bmp085_coeffs.ac4)) { DBGPRNT("fail"); succ = false; }
  if (!read16(BMP085_REGISTER_CAL_AC5, &_bmp085_coeffs.ac5)) { DBGPRNT("fail"); succ = false; }
  if (!read16(BMP085_REGISTER_CAL_AC6, &_bmp085_coeffs.ac6)) { DBGPRNT("fail"); succ = false; }
  if (!readS16(BMP085_REGISTER_CAL_B1, &_bmp085_coeffs.b1)) { DBGPRNT("fail"); succ = false; }
  if (!readS16(BMP085_REGISTER_CAL_B2, &_bmp085_coeffs.b2)) { DBGPRNT("fail"); succ = false; }
  if (!readS16(BMP085_REGISTER_CAL_MB, &_bmp085_coeffs.mb)) { DBGPRNT("fail"); succ = false; }
  if (!readS16(BMP085_REGISTER_CAL_MC, &_bmp085_coeffs.mc)) { DBGPRNT("fail"); succ = false; }
  if (!readS16(BMP085_REGISTER_CAL_MD, &_bmp085_coeffs.md)) { DBGPRNT("fail"); succ = false; }
#endif
  return succ;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
static std::optional<int32_t> readRawTemperature() {
#if BMP085_USE_DATASHEET_VALS
  return 27898;
#else
  bool succ{true};
  if (!writeCommand(BMP085_REGISTER_CONTROL, BMP085_REGISTER_READTEMPCMD)) { DBGPRNT("fail"); succ = false; }
  delay(5);
  uint16_t t;
  if (!read16(BMP085_REGISTER_TEMPDATA, &t)) { DBGPRNT("fail"); succ = false; }
  if (succ)
    return t;
  else
    return std::nullopt;
#endif
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
static std::optional<int32_t> readRawPressure() {
#if BMP085_USE_DATASHEET_VALS
  return 23843;
#else
  bool succ{true};

  if (!writeCommand(BMP085_REGISTER_CONTROL,
                    BMP085_REGISTER_READPRESSURECMD + (_bmp085Mode << 6))) { DBGPRNT("fail"); succ = false; }
  switch (_bmp085Mode) {
  case BMP085_MODE_ULTRALOWPOWER:
    delay(5);
    break;
  case BMP085_MODE_STANDARD:
    delay(8);
    break;
  case BMP085_MODE_HIGHRES:
    delay(14);
    break;
  case BMP085_MODE_ULTRAHIGHRES:
  default:
    delay(26);
    break;
  }

  int32_t p32{};

  {
      uint16_t p16;
      if (!read16(BMP085_REGISTER_PRESSUREDATA, &p16)) { DBGPRNT("fail"); succ = false; }
      else if (succ) p32 += (uint32_t)p16 << 8;
  }

  {
      uint8_t p8;
      if (!read8(BMP085_REGISTER_PRESSUREDATA + 2, &p8)) { DBGPRNT("fail"); succ = false; }
      else if (succ) p32 += p8;
  }

  if (succ) {
    p32 >>= (8 - _bmp085Mode);
    return p32;
  } else
    return std::nullopt;
#endif
}

/**************************************************************************/
/*!
    @brief  Compute B5 coefficient used in temperature & pressure calcs.
*/
/**************************************************************************/
int32_t Adafruit_BMP085_Unified::computeB5(int32_t ut) {
  int32_t X1 =
      (ut - (int32_t)_bmp085_coeffs.ac6) * ((int32_t)_bmp085_coeffs.ac5) >> 15;
  int32_t X2 =
      ((int32_t)_bmp085_coeffs.mc << 11) / (X1 + (int32_t)_bmp085_coeffs.md);
  return X1 + X2;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
bool Adafruit_BMP085_Unified::begin(bool skipWireBegin, bmp085_mode_t mode) {
  // Enable I2C
  if (!skipWireBegin)
    if (!Wire.begin())
      { DBGPRNT("fail"); return false; }

  /* Mode boundary check */
  if ((mode > BMP085_MODE_ULTRAHIGHRES) || (mode < 0)) {
    mode = BMP085_MODE_ULTRAHIGHRES;
  }

  /* Make sure we have the right device */
  uint8_t id;
  if (!read8(BMP085_REGISTER_CHIPID, &id)) { DBGPRNT("fail"); return false; }
  if (id != 0x55) { DBGPRNT("fail"); return false; }

  /* Set the mode indicator */
  _bmp085Mode = mode;

  /* Coefficients need to be read once */
  if (!readCoefficients()) { DBGPRNT("fail"); return false; }

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the compensated pressure level in kPa
*/
/**************************************************************************/
std::optional<float> Adafruit_BMP085_Unified::getPressure() {
  int32_t ut;

  /* Get the raw pressure and temperature values */
  if (const auto rawTemp = readRawTemperature())
    ut = *rawTemp;
  else
  { DBGPRNT("fail"); return std::nullopt; }

  int32_t up;

  if (const auto rawPress = readRawPressure())
    up = *rawPress;
  else
  { DBGPRNT("fail"); return std::nullopt; }

  return getPressure(ut, up);
}

/**************************************************************************/
/*!
    @brief  Gets the compensated pressure level in kPa
*/
/**************************************************************************/
float Adafruit_BMP085_Unified::getPressure(int32_t ut, int32_t up) {
  int32_t x1, x2, b5, b6, x3, b3, p;
  uint32_t b4, b7;

  /* Temperature compensation */
  b5 = computeB5(ut);

  /* Pressure compensation */
  b6 = b5 - 4000;
  x1 = (_bmp085_coeffs.b2 * ((b6 * b6) >> 12)) >> 11;
  x2 = (_bmp085_coeffs.ac2 * b6) >> 11;
  x3 = x1 + x2;
  b3 = (((((int32_t)_bmp085_coeffs.ac1) * 4 + x3) << _bmp085Mode) + 2) >> 2;
  x1 = (_bmp085_coeffs.ac3 * b6) >> 13;
  x2 = (_bmp085_coeffs.b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (_bmp085_coeffs.ac4 * (uint32_t)(x3 + 32768)) >> 15;
  b7 = ((uint32_t)(up - b3) * (50000 >> _bmp085Mode));

  if (b7 < 0x80000000) {
    p = (b7 << 1) / b4;
  } else {
    p = (b7 / b4) << 1;
  }

  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  float compp = p + ((x1 + x2 + 3791) >> 4);

  /* Assign compensated pressure value */
  return compp / 100.f;
}

std::optional<Adafruit_BMP085_Unified::TemperatureAndPressure> Adafruit_BMP085_Unified::getTemperatureAndPressure() {
  int32_t ut;

  /* Get the raw pressure and temperature values */
  if (const auto rawTemp = readRawTemperature())
    ut = *rawTemp;
  else
  { DBGPRNT("fail"); return std::nullopt; }

  int32_t up;

  if (const auto rawPress = readRawPressure())
    up = *rawPress;
  else
  { DBGPRNT("fail"); return std::nullopt; }

  return TemperatureAndPressure {
    .temperature = getTemperature(ut),
    .pressure = getPressure(ut, up)
  };
}

/**************************************************************************/
/*!
    @brief  Reads the temperatures in degrees Celsius
*/
/**************************************************************************/
std::optional<float> Adafruit_BMP085_Unified::getTemperature() {  
#if BMP085_USE_DATASHEET_VALS
  // use datasheet numbers!
  UT = 27898;
  _bmp085_coeffs.ac6 = 23153;
  _bmp085_coeffs.ac5 = 32757;
  _bmp085_coeffs.mc = -8711;
  _bmp085_coeffs.md = 2868;
#else
  int32_t UT;
  if (const auto rawTemp = readRawTemperature())
    UT = *rawTemp;
  else
  { DBGPRNT("fail"); return std::nullopt; }
#endif

  return getTemperature(UT);
}

/**************************************************************************/
/*!
    @brief  Reads the temperatures in degrees Celsius
*/
/**************************************************************************/
float Adafruit_BMP085_Unified::getTemperature(int32_t UT) {
  int32_t B5; // following ds convention
  float t;

  B5 = computeB5(UT);
  t = (B5 + 8) >> 4;
  t /= 10;

  return t;
}

/**************************************************************************/
/*!
    Calculates the altitude (in meters) from the specified atmospheric
    pressure (in hPa), and sea-level pressure (in hPa).

    @param  seaLevel      Sea-level pressure in hPa
    @param  atmospheric   Atmospheric pressure in hPa
*/
/**************************************************************************/
float Adafruit_BMP085_Unified::pressureToAltitude(float seaLevel,
                                                  float atmospheric) {
  // Equation taken from BMP180 datasheet (page 16):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude.  See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

  return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

/**************************************************************************/
/*!
    Calculates the altitude (in meters) from the specified atmospheric
    pressure (in hPa), and sea-level pressure (in hPa).  Note that this
    function just calls the overload of pressureToAltitude which takes
    seaLevel and atmospheric pressure--temperature is ignored.  The original
    implementation of this function was based on calculations from Wikipedia
    which are not accurate at higher altitudes.  To keep compatibility with
    old code this function remains with the same interface, but it calls the
    more accurate calculation.

    @param  seaLevel      Sea-level pressure in hPa
    @param  atmospheric   Atmospheric pressure in hPa
    @param  temp          Temperature in degrees Celsius
*/
/**************************************************************************/
float Adafruit_BMP085_Unified::pressureToAltitude(float seaLevel,
                                                  float atmospheric,
                                                  float temp) {
  return pressureToAltitude(seaLevel, atmospheric);
}

/**************************************************************************/
/*!
    Calculates the pressure at sea level (in hPa) from the specified altitude
    (in meters), and atmospheric pressure (in hPa).

    @param  altitude      Altitude in meters
    @param  atmospheric   Atmospheric pressure in hPa
*/
/**************************************************************************/
float Adafruit_BMP085_Unified::seaLevelForAltitude(float altitude,
                                                   float atmospheric) {
  // Equation taken from BMP180 datasheet (page 17):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude.  See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

  return atmospheric / pow(1.0 - (altitude / 44330.0), 5.255);
}

/**************************************************************************/
/*!
    Calculates the pressure at sea level (in hPa) from the specified altitude
    (in meters), and atmospheric pressure (in hPa).  Note that this
    function just calls the overload of seaLevelForAltitude which takes
    altitude and atmospheric pressure--temperature is ignored.  The original
    implementation of this function was based on calculations from Wikipedia
    which are not accurate at higher altitudes.  To keep compatibility with
    old code this function remains with the same interface, but it calls the
    more accurate calculation.

    @param  altitude      Altitude in meters
    @param  atmospheric   Atmospheric pressure in hPa
    @param  temp          Temperature in degrees Celsius
*/
/**************************************************************************/
float Adafruit_BMP085_Unified::seaLevelForAltitude(float altitude,
                                                   float atmospheric,
                                                   float temp) {
  return seaLevelForAltitude(altitude, atmospheric);
}

/**************************************************************************/
/*!
    @brief  Provides the sensor_t data for this sensor
*/
/**************************************************************************/
sensor_t Adafruit_BMP085_Unified::getSensor() {
  sensor_t sensor;

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor.name, "BMP085", sizeof(sensor.name) - 1);
  sensor.name[sizeof(sensor.name) - 1] = 0;

  sensor.version = 1;
  sensor.sensor_id = _sensorID;
  sensor.type = SENSOR_TYPE_PRESSURE;
  sensor.min_delay = 0;
  sensor.max_value = 1100.0F; // 300..1100 hPa
  sensor.min_value = 300.0F;
  sensor.resolution = 0.01F; // Datasheet states 0.01 hPa resolution
  return sensor;
}

/**************************************************************************/
/*!
    @brief  Reads the sensor and returns the data as a sensors_event_t
*/
/**************************************************************************/
std::optional<sensors_event_t> Adafruit_BMP085_Unified::getEvent() {
  const auto pressure = getPressure();
  if (!pressure)
    return std::nullopt;

  sensors_event_t event;

  event.version = sizeof(sensors_event_t);
  event.sensor_id = _sensorID;
  event.type = SENSOR_TYPE_PRESSURE;
  event.timestamp = espchrono::millis_clock::now();
  event.pressure = *pressure;

  return event;
}
