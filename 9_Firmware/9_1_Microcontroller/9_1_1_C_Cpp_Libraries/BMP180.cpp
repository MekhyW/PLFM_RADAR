/***************************************************************************************************/
/*
   This is an Arduino basic library for Bosch BMP180 & BMP085 barometric pressure &
   temperature sensor

   Power supply voltage:   1.8v - 3.6v
   Range:                  30,000Pa..110,000Pa at -40°C..+85°C 
   Typ. resolution:        1Pa     / 0.1°C
   Typ. accuracy:          ±100Pa* / ±1.0°C* at 0°C..+65°C
   Typ. relative accuracy: ±12Pa   / xx°C
   Duty cycle:             10% active & 90% inactive, to prevent self heating

                          *sensor is sensitive to direct light, which can affect
                           the accuracy of the measurement

   written by : enjoyneering79
   sourse code: https://github.com/enjoyneering/


   This chip uses I2C bus to communicate, specials pins are required to interface
   Board:                                    SDA                    SCL                    Level
   Uno, Mini, Pro, ATmega168, ATmega328..... A4                     A5                     5v
   Mega2560................................. 20                     21                     5v
   Due, SAM3X8E............................. 20                     21                     3.3v
   Leonardo, Micro, ATmega32U4.............. 2                      3                      5v
   Digistump, Trinket, ATtiny85............. 0/physical pin no.5    2/physical pin no.7    5v
   Blue Pill, STM32F103xxxx boards.......... PB7                    PB6                    3.3v/5v
   ESP8266 ESP-01........................... GPIO0/D5               GPIO2/D3               3.3v/5v
   NodeMCU 1.0, WeMos D1 Mini............... GPIO4/D2               GPIO5/D1               3.3v/5v
   ESP32.................................... GPIO21/D21             GPIO22/D22             3.3v

   NOTE:
   - EOC  pin is not used, shows the end of conversion
   - XCLR pin is not used, reset pin

   Frameworks & Libraries:
   ATtiny  Core          - https://github.com/SpenceKonde/ATTinyCore
   ESP32   Core          - https://github.com/espressif/arduino-esp32
   ESP8266 Core          - https://github.com/esp8266/Arduino
   STM32   Core          - https://github.com/rogerclarkmelbourne/Arduino_STM32

   GNU GPL license, all text above must be included in any redistribution,
   see link for details  - https://www.gnu.org/licenses/licenses.html
*/
/***************************************************************************************************/

#include "BMP180.h"


/**************************************************************************/
/*
    Constructor

    NOTE:
    - BMP180_ULTRALOWPOWER, pressure oversampled 1 time  & consumption 3μA
    - BMP180_STANDARD,      pressure oversampled 2 times & consumption 5μA
    - BMP180_HIGHRES,       pressure oversampled 4 times & consumption 7μA
    - BMP180_ULTRAHIGHRES,  pressure oversampled 8 times & consumption 12μA
*/
/**************************************************************************/
BMP180::BMP180(BMP180_RESOLUTION res_mode)
{
  _resolution = res_mode;
}


/**************************************************************************/
/*
    begin()

    Probes the BMP180 over I2C, verifies the chip ID, and reads the 11
    factory calibration coefficients into _calCoeff. MUST be called once
    after HAL_I2C is initialized and before any getTemperature/getPressure
    call — without this, _calCoeff stays at zero defaults and computeB5
    short-circuits via 0/0 to 0, producing bogus output.

    NOT done in the constructor because the constructor runs at C++
    static-initialization time, before HAL_I2C_Init has been called from
    main(). Calling I2C from a static initializer is fragile and would
    silently fail.

    Returns true on success; false on I2C failure or chip-ID mismatch.
    On failure, _calCoeff is left at its previous value (all zeros after
    construction) so the caller can treat the sensor as absent and avoid
    propagating NaN/wild values into downstream consumers.
*/
/**************************************************************************/
bool BMP180::begin(void)
{
  /* Probe + chip-ID check first — distinguishes "BMP180 not present"
   * from "BMP180 present but I2C noisy" so the caller can act differently. */
  if (readDeviceID() != 180) return false;

  return readCalibrationCoefficients();
}


/**************************************************************************/
/*
    getPressure()

    Calculates compensated pressure, in Pa

    NOTE:
    - resolutin 1Pa with accuracy ±150Pa at range 30,000Pa..110,000Pa
*/
/**************************************************************************/
int32_t BMP180::getPressure(void)
{
  int32_t  UP_signed = 0;
  int32_t  B3       = 0;
  int32_t  B5       = 0;
  int32_t  B6       = 0;
  int32_t  X1       = 0;
  int32_t  X2       = 0;
  int32_t  X3       = 0;
  int32_t  pressure = 0;
  uint32_t B4       = 0;
  uint32_t B7       = 0;

  /* AUDIT-C17: uint16_t raw_UT is widened to int32_t value-preserving;
   * uint32_t raw_UP fits in int32_t (19-bit max). */
  uint16_t raw_UT = 0;
  uint32_t raw_UP = 0;

  if (!readRawTemperature(&raw_UT)) return INT32_MIN;                   //I2C error sentinel (cannot collide with valid reading)
  if (!readRawPressure(&raw_UP))    return INT32_MIN;

  B5 = computeB5((int32_t)raw_UT);
  UP_signed = (int32_t)raw_UP;

  /* pressure calculation */
  B6 = B5 - 4000;
  X1 = ((int32_t)_calCoeff.bmpB2 * ((B6 * B6) >> 12)) >> 11;
  X2 = ((int32_t)_calCoeff.bmpAC2 * B6) >> 11;
  X3 = X1 + X2;
  B3 = ((((int32_t)_calCoeff.bmpAC1 * 4 + X3) << _resolution) + 2) / 4;

  X1 = ((int32_t)_calCoeff.bmpAC3 * B6) >> 13;
  X2 = ((int32_t)_calCoeff.bmpB1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = ((uint32_t)_calCoeff.bmpAC4 * (X3 + 32768L)) >> 15;
  B7 = (UP_signed - B3) * (50000UL >> _resolution);

  if (B4 == 0) return INT32_MIN;                                        //safety check, avoiding division by zero

  if   (B7 < 0x80000000) pressure = (B7 * 2) / B4;
  else                   pressure = (B7 / B4) * 2;

  X1 = pow((pressure >> 8), 2);
  X1 = (X1 * 3038L) >> 16;
  X2 = (-7357L * pressure) >> 16;

  return pressure = pressure + ((X1 + X2 + 3791L) >> 4);
}

/**************************************************************************/
/*
    getTemperature()

    Calculates compensated temperature, in °C

    NOTE:
    - resolution 0.1°C with accuracy ±1.0°C at range 0°C..+65°C
*/
/**************************************************************************/
float BMP180::getTemperature(void)
{
  /* AUDIT-C17: was `int16_t rawTemperature = readRawTemperature();`
   * which silently narrowed uint16_t→int16_t. Bit-patterns ≥ 0x8000 (reachable
   * across the BMP180 -40..+85 °C window) became negative int16_t and
   * sign-extended to large negative int32_t inside computeB5(), producing
   * temperature errors of order 100s of °C. Keep raw as uint16_t and widen
   * to int32_t value-preservingly. */
  uint16_t rawTemperature = 0;
  if (!readRawTemperature(&rawTemperature)) return NAN;                                          //I2C error sentinel (cannot collide with any valid float reading)

  return (float)((computeB5((int32_t)rawTemperature) + 8) >> 4) / 10;
}

/**************************************************************************/
/*
    getSeaLevelPressure()

    Converts current pressure to sea level pressure at specific true
    altitude, in Pa

    NOTE:
    - true altitude is the actual elevation above sea level, to find out
      your current true altitude do search with google earth or gps
    - see level pressure is commonly used in weather reports & forecasts
      to compensate current true altitude
    - for example, we know that a sunny day happens if the current sea
      level pressure is 250Pa above the average sea level pressure of
      101325 Pa, so by converting the current pressure to sea level &
      comparing it with an average sea level pressure we can instantly
      predict the weather conditions
*/
/**************************************************************************/
int32_t BMP180::getSeaLevelPressure(int16_t trueAltitude)
{
  int32_t pressure = getPressure();

  if (pressure == INT32_MIN) return INT32_MIN;                          //propagate I2C error sentinel
  return (pressure / pow(1.0 - (float)trueAltitude / 44330, 5.255));
}

/**************************************************************************/
/*
    softReset()

    Soft reset

    NOTE:
    - performs the same sequence as power on reset
*/
/**************************************************************************/
void BMP180::softReset(void)
{
  write8(BMP180_SOFT_RESET_REG, BMP180_SOFT_RESET_CTRL);
}

/**************************************************************************/
/*
    readFirmwareVersion()

    Reads ML & AL Version

    NOTE:
    - ML version is LSB, 4-bit..0-bit
    - AL version is MSB, 7-bit..5-bit
*/
/**************************************************************************/
uint8_t BMP180::readFirmwareVersion(void)
{
  uint8_t v = 0;
  read8(BMP180_GET_VERSION_REG, &v);  //best-effort telemetry; v stays 0 on I2C error
  return v;
}

/**************************************************************************/
/*
    readDeviceID()

    Reads chip ID
*/
/**************************************************************************/
uint8_t BMP180::readDeviceID(void)
{
  uint8_t id = 0;
  if (!read8(BMP180_GET_ID_REG, &id)) return 0;            //I2C error
  if (id == BMP180_CHIP_ID)           return 180;
  return 0;                                                //chip mismatch
}

/**************************************************************************/
/*
    readCalibrationCoefficients()

    Reads factory calibration coefficients from E2PROM

    NOTE:
    - every sensor module has individual calibration coefficients
    - before first temperature & pressure calculation master have to read
      calibration coefficients from 176-bit E2PROM
*/
/**************************************************************************/
bool BMP180::readCalibrationCoefficients()
{
  uint16_t value = 0;

  for (uint8_t reg = BMP180_CAL_AC1_REG; reg <= BMP180_CAL_MD_REG; reg++)
  {
    if (!read16(reg, &value)) return false;  //AUDIT-C17: bool out-param signals I2C error without colliding with valid uint16 cal byte (e.g. 0x00FF)

    switch (reg)
    {
      case BMP180_CAL_AC1_REG:               //used for pressure computation
        _calCoeff.bmpAC1 = value;
        break;

      case BMP180_CAL_AC2_REG:               //used for pressure computation
        _calCoeff.bmpAC2 = value;
        break;

      case BMP180_CAL_AC3_REG:               //used for pressure computation
        _calCoeff.bmpAC3 = value;
        break;

      case BMP180_CAL_AC4_REG:               //used for pressure computation
        _calCoeff.bmpAC4 = value;
        break;

      case BMP180_CAL_AC5_REG:               //used for temperature computation
        _calCoeff.bmpAC5 = value;
        break;

      case BMP180_CAL_AC6_REG:               //used for temperature computation
        _calCoeff.bmpAC6 = value;
        break;

      case BMP180_CAL_B1_REG:                //used for pressure computation
        _calCoeff.bmpB1 = value;
        break;

      case BMP180_CAL_B2_REG:                //used for pressure computation
        _calCoeff.bmpB2 = value;
        break;

      case BMP180_CAL_MB_REG:                //???
        _calCoeff.bmpMB = value;
        break;

      case BMP180_CAL_MC_REG:                //used for temperature computation
        _calCoeff.bmpMC = value;
        break;

      case BMP180_CAL_MD_REG:                //used for temperature computation
        _calCoeff.bmpMD = value;
        break;
    }
  }

  return true;
}

/**************************************************************************/
/*
    readRawTemperature()

    Reads raw/uncompensated temperature value, 16-bit
*/
/**************************************************************************/
bool BMP180::readRawTemperature(uint16_t* out)
{
  /* send temperature measurement command */
  if (!write8(BMP180_START_MEASURMENT_REG, BMP180_GET_TEMPERATURE_CTRL)) return false;             //I2C error

  /* set measurement delay */
   HAL_Delay(5);

  /* read result (msb + lsb); read16 sets *out only on success */
  return read16(BMP180_READ_ADC_MSB_REG, out);
}

/**************************************************************************/
/*
    readRawPressure()

    Reads raw/uncompensated pressure value, 19-bits
*/
/**************************************************************************/
bool BMP180::readRawPressure(uint32_t* out)
{
  uint8_t  regControl  = 0;
  uint16_t msb_lsb     = 0;
  uint8_t  xlsb        = 0;

  /* convert resolution to register control */
  switch (_resolution)
  {
    case BMP180_ULTRALOWPOWER:                    //oss0
      regControl = BMP180_GET_PRESSURE_OSS0_CTRL;
      break;

    case BMP180_STANDARD:                         //oss1
      regControl = BMP180_GET_PRESSURE_OSS1_CTRL;
      break;

    case BMP180_HIGHRES:                          //oss2
      regControl = BMP180_GET_PRESSURE_OSS2_CTRL;
      break;

    case BMP180_ULTRAHIGHRES:                     //oss3
      regControl = BMP180_GET_PRESSURE_OSS3_CTRL;
      break;
  }

  /* send pressure measurement command */
  if (!write8(BMP180_START_MEASURMENT_REG, regControl)) return false;   //I2C error

  /* set measurement delay */
  switch (_resolution)
  {
    case BMP180_ULTRALOWPOWER:
       HAL_Delay(5);
      break;

    case BMP180_STANDARD:
       HAL_Delay(8);
      break;

    case BMP180_HIGHRES:
       HAL_Delay(14);
      break;

    case BMP180_ULTRAHIGHRES:
       HAL_Delay(26);
      break;
  }

  /* read msb+lsb and xlsb separately; signal failure on either */
  if (!read16(BMP180_READ_ADC_MSB_REG, &msb_lsb)) return false;
  if (!read8(BMP180_READ_ADC_XLSB_REG, &xlsb))    return false;         //AUDIT-C17: previously OR'd in sentinel 0xFF on I2C fail, silently corrupting LSB

  uint32_t rawPressure = ((uint32_t)msb_lsb << 8) | xlsb;               //19-bits before shift
  rawPressure >>= (8 - _resolution);

  *out = rawPressure;
  return true;
}

/**************************************************************************/
/*
    computeB5()

    Computes B5 value

    NOTE:
    - to compensate raw/uncompensated temperature
    - also used for compensated pressure calculation
*/
/**************************************************************************/
int32_t BMP180::computeB5(int32_t UT)
{
  int32_t X1 = ((UT - (int32_t)_calCoeff.bmpAC6) * (int32_t)_calCoeff.bmpAC5) >> 15;
  int32_t X2 = ((int32_t)_calCoeff.bmpMC << 11) / (X1 + (int32_t)_calCoeff.bmpMD);

  return X1 + X2;
}

/**************************************************************************/
/*
    read8()

    Reads 8-bit value over I2C
*/
/**************************************************************************/
bool BMP180::read8(uint8_t reg, uint8_t* out)
{
  uint8_t data = 0;
  HAL_StatusTypeDef status;

  // Write register address
  status = HAL_I2C_Master_Transmit(&hi2c3, BMP180_ADDRESS, &reg, 1, I2C_TIMEOUT);
  if (status != HAL_OK) return false;

  // Read data from register
  status = HAL_I2C_Master_Receive(&hi2c3, BMP180_ADDRESS, &data, 1, I2C_TIMEOUT);
  if (status != HAL_OK) return false;

  *out = data;
  return true;
}

/**************************************************************************/
/*
    read16()

    Reads 16-bits value over I2C
*/
/**************************************************************************/

bool BMP180::read16(uint8_t reg, uint16_t* out)
{
  uint8_t data[2] = {0, 0};
  HAL_StatusTypeDef status;

  // Write register address
  status = HAL_I2C_Master_Transmit(&hi2c3, BMP180_ADDRESS, &reg, 1, I2C_TIMEOUT);
  if (status != HAL_OK) return false;

  // Read 2 bytes from register
  status = HAL_I2C_Master_Receive(&hi2c3, BMP180_ADDRESS, data, 2, I2C_TIMEOUT);
  if (status != HAL_OK) return false;

  // Combine bytes (MSB first)
  *out = ((uint16_t)data[0] << 8) | data[1];
  return true;
}


/**************************************************************************/
/*
    write8()

    Writes 8-bits value over I2C
*/
/**************************************************************************/

bool BMP180::write8(uint8_t reg, uint8_t control)
{
  uint8_t data[2] = {reg, control};
  HAL_StatusTypeDef status;
  
  // Write register address and data
  status = HAL_I2C_Master_Transmit(&hi2c3, BMP180_ADDRESS, data, 2, I2C_TIMEOUT);
  
  return (status == HAL_OK);
}

