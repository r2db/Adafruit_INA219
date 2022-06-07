/*!
 * @file Adafruit_INA219.cpp
 *
 * @mainpage Adafruit INA219 current/power monitor IC
 *
 * @section intro_sec Introduction
 *
 *  Driver for the INA219 current sensor
 *
 *  This is a library for the Adafruit INA219 breakout
 *  ----> https://www.adafruit.com/product/904
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing
 *  products from Adafruit!
 *
 * @section author Author
 *
 * Written by Bryan Siepert and Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "Arduino.h"

#include <Wire.h>

#include "Adafruit_INA219.h"

/*!
 *  @brief  Instantiates a new INA219 class
 *  @param addr the I2C address the device can be found on. Default is 0x40
 */
Adafruit_INA219::Adafruit_INA219(uint8_t addr) {
  ina219_i2caddr = addr;
  ina219_currentDivider_mA = 0.0f;
  ina219_powerMultiplier_mW = 0.0f;
}

/*!
 *  @brief INA219 class destructor
 */
Adafruit_INA219::~Adafruit_INA219() { delete i2c_dev; }

/*!
 *  @brief  Sets up the HW (defaults to 32V and 2A for calibration values)
 *  @param theWire the TwoWire object to use
 *  @return true: success false: Failed to start I2C
 */
bool Adafruit_INA219::begin(TwoWire *theWire) {
  if (!i2c_dev) {
    i2c_dev = new Adafruit_I2CDevice(ina219_i2caddr, theWire);
  }

  if (!i2c_dev->begin()) {
    return false;
  }
  init();
  return true;
}

/*!
 *  @brief  begin I2C and set up the hardware
 */
void Adafruit_INA219::init() {
  // Set chip to large range config values to start
  setCalibration_32V_2A();
}

/*!
 *  @brief  Gets the raw bus voltage (16-bit signed integer, so +-32767)
 *  @return the raw bus voltage reading
 */
int16_t Adafruit_INA219::getBusVoltage_raw() {
  uint16_t value;

  Adafruit_BusIO_Register bus_voltage_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_BUSVOLTAGE, 2, MSBFIRST);
  _success = bus_voltage_reg.read(&value);

  // Set flags
  _overflow = value & 0x01;        // Overflow flag is bit 0
  _conversionReady = value & 0x02; // Conversion Ready flag is bit 1
  // Shift to the right 3 to drop CNVR and OVF and multiply by 4 mV per LSB
  // For efficiency, shift to the right 1 and drop the two LSB
  return (int16_t)((value >> 1) & 0xFC);
}

/*!
 *  @brief  Gets the raw shunt voltage (16-bit signed integer, so +-32767)
 *  @return the raw shunt voltage reading
 */
int16_t Adafruit_INA219::getShuntVoltage_raw() {
  uint16_t value;
  Adafruit_BusIO_Register shunt_voltage_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_SHUNTVOLTAGE, 2, MSBFIRST);
  _success = shunt_voltage_reg.read(&value);
  return value;
}

/*!
 *  @brief  Gets the raw current value (16-bit signed integer, so +-32767)
 *  @return the raw current reading
 */
int16_t Adafruit_INA219::getCurrent_raw() {
  uint16_t value;

  // Sometimes a sharp load will reset the INA219, which will
  // reset the cal register, meaning CURRENT and POWER will
  // not be available ... avoid this by always setting a cal
  // value even if it's an unfortunate extra step
  Adafruit_BusIO_Register calibration_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CALIBRATION, 2, MSBFIRST);
  calibration_reg.write(ina219_calValue, 2);

  // Now we can safely read the CURRENT register!
  Adafruit_BusIO_Register current_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CURRENT, 2, MSBFIRST);
  _success = current_reg.read(&value);
  return value;
}

/*!
 *  @brief  Gets the raw power value (16-bit signed integer, so +-32767)
 *  @return raw power reading
 */
int16_t Adafruit_INA219::getPower_raw() {
  uint16_t value;

  // Sometimes a sharp load will reset the INA219, which will
  // reset the cal register, meaning CURRENT and POWER will
  // not be available ... avoid this by always setting a cal
  // value even if it's an unfortunate extra step
  Adafruit_BusIO_Register calibration_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CALIBRATION, 2, MSBFIRST);
  calibration_reg.write(ina219_calValue, 2);

  // Now we can safely read the POWER register!
  Adafruit_BusIO_Register power_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_POWER, 2, MSBFIRST);
  _success = power_reg.read(&value);
  return value;
}

/*!
 *  @brief  Gets the shunt voltage in mV (so +-327mV)
 *  @return the shunt voltage converted to millivolts
 */
float Adafruit_INA219::getShuntVoltage_mV() {
  int16_t value;
  value = getShuntVoltage_raw();
  return value * 0.01;
}

/*!
 *  @brief  Gets the bus voltage in volts
 *  @return the bus voltage converted to volts
 */
float Adafruit_INA219::getBusVoltage_V() {
  int16_t value = getBusVoltage_raw();
  return value * 0.001;
}

/*!
 *  @brief  Gets the current value in mA, taking into account the
 *          config settings and current LSB
 *  @return the current reading convereted to milliamps
 */
float Adafruit_INA219::getCurrent_mA() {
  float valueDec = getCurrent_raw();
  valueDec /= ina219_currentDivider_mA;
  return valueDec;
}

/*!
 *  @brief  Gets the power value in mW, taking into account the
 *          config settings and current LSB
 *  @return power reading converted to milliwatts
 */
float Adafruit_INA219::getPower_mW() {
  float valueDec = getPower_raw();
  valueDec *= ina219_powerMultiplier_mW;
  return valueDec;
}

/*!
 *  @brief  Gets the stored value of the shunt resistor
 *  @return resistor value in Ohms
 */
float Adafruit_INA219::getShuntResistor_Ohms() {
  return ina219_shuntResistor_Ohms;
}

/*!
 *  @brief  Gets the configuration register value
 *  @return the value of the 0x00 register
 */
uint16_t Adafruit_INA219::getConfig() {
  uint16_t value;
  Adafruit_BusIO_Register configuration_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CONFIG, 2, MSBFIRST);
  _success = configuration_reg.read(&value);
  return value;
}

/*!
 *  @brief  Gets the calibration register value
 *  @return the value of the 0x05 register
 */
int16_t Adafruit_INA219::getCalibration() {
  uint16_t value;
  Adafruit_BusIO_Register calibration_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CALIBRATION, 2, MSBFIRST);
  _success = calibration_reg.read(&value);
  return value;
}

/*!
 *  @brief  Configures the INA219 to user preferences
 *  @notes  - A supplied calibration register value takes precedence over
 *          a calculated value, but it may cause loss of precision.
 *          - Only shunt resistance or both shunt ampacity and milliVolt
 *          rating need to be supplied for calculation of the current
 *          register calibration. Shunt resistance takes precedence if all
 *          values are supplied.
 *          - If no calibration value or resistance are supplied and the
 *          calibration cannot be calculated with the supplied data then
 *          the current/ power calculations will return zero.
 */
void Adafruit_INA219::setConfiguration(
            INA219_CONFIG_BVOLTAGERANGE voltageRange, 
            INA219_CONFIG_GAIN currentGain,
            INA219_CONFIG_BADCRES busADCResolution,
            INA219_CONFIG_SADCRES shuntADCResolution,
            float shuntResistorOhms, float shuntRatedCurrentAmps,
            float shuntRatedMilliVolts,
            float maxExpectedCurrentAmps, uint16_t calibrationRegister,
            INA219_CONFIG_MODE operatingMode) {
  Adafruit_BusIO_Register config_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CONFIG, 2, MSBFIRST);
  uint16_t config = 0;
  Adafruit_BusIO_Register calibration_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CALIBRATION, 2, MSBFIRST);
  float cal = 0;
  float currentLSB = 0;
  uint8_t gain = 1;
  
  // Sanitize the configuration register inputs, reset the configuration,
  // and set config as requested
  currentGain &= INA219_CONFIG_GAIN_MASK;
  // Save the current gain divisor in variable gain
  while (config < currentGain) {
      gain *= 2;
      config += INA219_CONFIG_GAIN_2_80MV;
  }
  voltageRange &= INA219_CONFIG_BVOLTAGE_MASK;
  config |= voltageRange;
  busADCResolution &= INA219_CONFIG_BADCRES_MASK;
  config |= busADCResolution;
  shuntADCResolution &= INA219_CONFIG_SADCRES_MASK;
  config |= shuntADCResolution;
  operatingMode &= INA219_CONFIG_MODE_MASK;
  config |= operatingMode;
  ina219_configValue = config;
  _success = config_reg.write(INA219_CONFIG_RESET, 2);
  _success &= config_reg.write(ina219_configValue, 2);
  
  // Sanitize the supplied data (resistance, voltage, current) as these numbers
  // should never be negative numbers. The calibration register does not have a
  // sign bit.
  if (shuntResistorOhms < 0) { shuntResistorOhms *= -1; }
  if (shuntRatedCurrentAmps < 0) { shuntRatedCurrentAmps *= -1; }
  if (shuntRatedMilliVolts < 0) { shuntRatedMilliVolts *= -1; }
  if (maxExpectedCurrentAmps < 0) { maxExpectedCurrentAmps *= -1; }

  // If the shunt resistance was not supplied then attempt to calculate it.
  if (shuntResistorOhms == 0) {
      // If the shunt rated current was not supplied avoid a div/0 error
      if (shuntRatedCurrentAmps > 0) {
          shuntResistorOhms = shuntRatedMilliVolts / shuntRatedCurrent;
      }
  }
  ina219_shuntResistor_Ohms = shuntResistorOhms;

  // If the maximum expected current was not supplied then calculate it using the
  // supplied resistance and gain settings. If the maximum expected current exceeds
  // a shunt voltage of 320 mV (the sensor's maximum range) or the maximum voltage
  // at the set gain then derate the maximum expected current
  if ((maxExpectedCurrentAmps == 0 && shuntResistorOhms > 0) ||
     (maxExpectedCurrentAmps * shuntResistorOhms > 0.32 / gain)) {
      maxExpectedCurrentAmps = 0.32 / (gain * shuntResistorOhms);
  }

  // Voltage per LSB is a constant 10 uV/ count - the gain only sets the bit range
  //     (PGA=/1 = 12 bits for +/- 40 mV, PGA=/2 = 13 bits for +/- 80 mV,
  //     PGA=/4 = 14 bits for +/- 160 mV, PGA=/8 = 15 bits for +/- 320 mV)
  // CurrentLSB = Maximum Expected Current/ 32,767 or 2^15-1

  currentLSB = maxExpectedCurrentAmps / 32767;

  // Compute the calibration register
  // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
  // Cal = 4096 (0x1000) for reference board

  // If the shunt resistance could not be calculated or currentLSB is zero then skip
  // this calculation, otherwise, perform the calculation to determine calibration.
  if (shuntResistorOhms > 0 && currentLSB != 0) {
      cal = 0.04096 / (currentLSB * shuntResistorOhms);
  }

  // Calibration register sanity checks. Make sure that the calibration register
  // will give a reasonable measurement range for current (no more than 1 bit
  // of loss of precision over the full scale range). As the voltage range is clamped
  // to +/- 4000/ 8000/ 16000/ 32000 but the current register is a full scale signed
  // 16 bit integer (+32767/-32768) the voltage reading can be multiplied by up to
  // 1.024 (32,767/32000) at gain/1 without significant loss of precision.
  if (cal != 0) {
      while ((uint16_t) cal < 2048) {
          cal *= 2; // shunt voltage * 2048 / 4096 is 1 bit loss of precision
      }
      while ((uint16_t) cal > (4194 * gain)) {
          cal /= 2; // For each fewer bit in shunt voltage (change in gain) the
                    // upper limit doubles before the current calculation overflows.
      }
      // 'cal' is now either zero or between 2048 and 4194 * gain (inclusive) and
      // there is no more than our arbitrary limit of 1 bit loss of precision.
  }
  // If a value for 'calibrationRegister' was not supplied, use the calculated one
  if (calibrationRegister == 0) {
      ina219_calValue = (uint16_t) cal;
  } else {
      ina219_calValue = calibrationRegister;
  }
  ina219_calValue &= 0xFFFE; // Discard the least significant bit (read-only and zero
                             // per datasheet page 24)

  // Set Calibration register to 'cal' calculated above
  _success &= calibration_reg.write(ina219_calValue, 2);

  // Set multipliers to convert raw current/power values using the written calibration
  // register value
  ina219_currentDivider_mA = ina219_calValue * ina219_shuntResistor_Ohms / 40.96;
  ina219_powerMultiplier_mW = 20 / ina219_currentDivider; // Datasheet equation 3
}

/*!
 *  @brief  Configures the INA219 to be able to measure up to 32V and 2A
 *          of current.  Each unit of current corresponds to 100uA, and
 *          each unit of power corresponds to 2mW. Counter overflow
 *          occurs at 3.2A.
 *  @note   These calculations assume a 0.1 ohm resistor is present
 */
void Adafruit_INA219::setCalibration_32V_2A() {
  // By default we use a pretty huge range for the input voltage,
  // which probably isn't the most appropriate choice for system
  // that don't use a lot of power.  But all of the calculations
  // are shown below if you want to change the settings.  You will
  // also need to change any relevant register settings, such as
  // setting the VBUS_MAX to 16V instead of 32V, etc.

  // VBUS_MAX = 32V             (Assumes 32V, can also be set to 16V)
  // VSHUNT_MAX = 0.32          (Assumes Gain 8, 320mV, can also be 0.16, 0.08,
  // 0.04) RSHUNT = 0.1               (Resistor value in ohms)
  ina219_shuntResistor_Ohms = 0.1;

  // 1. Determine max possible current
  // MaxPossible_I = VSHUNT_MAX / RSHUNT
  // MaxPossible_I = 3.2A

  // 2. Determine max expected current
  // MaxExpected_I = 2.0A

  // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
  // MinimumLSB = MaxExpected_I/32767
  // MinimumLSB = 0.000061              (61uA per bit)
  // MaximumLSB = MaxExpected_I/4096
  // MaximumLSB = 0,000488              (488uA per bit)

  // 4. Choose an LSB between the min and max values
  //    (Preferrably a roundish number close to MinLSB)
  // CurrentLSB = 0.0001 (100uA per bit)

  // 5. Compute the calibration register
  // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
  // Cal = 4096 (0x1000)

  ina219_calValue = 4096;

  // 6. Calculate the power LSB
  // PowerLSB = 20 * CurrentLSB
  // PowerLSB = 0.002 (2mW per bit)

  // 7. Compute the maximum current and shunt voltage values before overflow
  //
  // Max_Current = Current_LSB * 32767
  // Max_Current = 3.2767A before overflow
  //
  // If Max_Current > Max_Possible_I then
  //    Max_Current_Before_Overflow = MaxPossible_I
  // Else
  //    Max_Current_Before_Overflow = Max_Current
  // End If
  //
  // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
  // Max_ShuntVoltage = 0.32V
  //
  // If Max_ShuntVoltage >= VSHUNT_MAX
  //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
  // Else
  //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
  // End If

  // 8. Compute the Maximum Power
  // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
  // MaximumPower = 3.2 * 32V
  // MaximumPower = 102.4W

  // Set multipliers to convert raw current/power values
  ina219_currentDivider_mA = 10; // Current LSB = 100uA per bit (1000/100 = 10)
  ina219_powerMultiplier_mW = 2; // Power LSB = 1mW per bit (2/1)

  // Set Calibration register to 'Cal' calculated above
  Adafruit_BusIO_Register calibration_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CALIBRATION, 2, MSBFIRST);
  calibration_reg.write(ina219_calValue, 2);

  // Set Config register to take into account the settings above
  ina219_configValue = INA219_CONFIG_BVOLTAGERANGE_32V |
                    INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
  Adafruit_BusIO_Register config_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CONFIG, 2, MSBFIRST);
  _success = config_reg.write(ina219_configValue, 2);
}

/*!
 *  @brief  Set power save mode according to parameters
 *  @param  on
 *          boolean value
 */
void Adafruit_INA219::powerSave(bool on) {
  Adafruit_BusIO_Register config_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CONFIG, 2, MSBFIRST);

  Adafruit_BusIO_RegisterBits mode_bits =
      Adafruit_BusIO_RegisterBits(&config_reg, 3, 0);
  if (on) {
    _success = mode_bits.write(INA219_CONFIG_MODE_POWERDOWN);
  } else {
    _success = mode_bits.write(INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS);
  }
}

/*!
 *  @brief  Configures the INA219 to be able to measure up to 32V and 1A
 *          of current.  Each unit of current corresponds to 40uA, and each
 *          unit of power corresponds to 800uW. Counter overflow occurs at
 *          1.3A.
 *  @note   These calculations assume a 0.1 ohm resistor is present
 */
void Adafruit_INA219::setCalibration_32V_1A() {
  // By default we use a pretty huge range for the input voltage,
  // which probably isn't the most appropriate choice for system
  // that don't use a lot of power.  But all of the calculations
  // are shown below if you want to change the settings.  You will
  // also need to change any relevant register settings, such as
  // setting the VBUS_MAX to 16V instead of 32V, etc.

  // VBUS_MAX = 32V		(Assumes 32V, can also be set to 16V)
  // VSHUNT_MAX = 0.32	(Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
  // RSHUNT = 0.1			(Resistor value in ohms)
  ina219_shuntResistor_Ohms = 0.1;

  // 1. Determine max possible current
  // MaxPossible_I = VSHUNT_MAX / RSHUNT
  // MaxPossible_I = 3.2A

  // 2. Determine max expected current
  // MaxExpected_I = 1.0A

  // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
  // MinimumLSB = MaxExpected_I/32767
  // MinimumLSB = 0.0000305             (30.5uA per bit)
  // MaximumLSB = MaxExpected_I/4096
  // MaximumLSB = 0.000244              (244uA per bit)

  // 4. Choose an LSB between the min and max values
  //    (Preferrably a roundish number close to MinLSB)
  // CurrentLSB = 0.0000400 (40uA per bit)

  // 5. Compute the calibration register
  // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
  // Cal = 10240 (0x2800)

  ina219_calValue = 10240;

  // 6. Calculate the power LSB
  // PowerLSB = 20 * CurrentLSB
  // PowerLSB = 0.0008 (800uW per bit)

  // 7. Compute the maximum current and shunt voltage values before overflow
  //
  // Max_Current = Current_LSB * 32767
  // Max_Current = 1.31068A before overflow
  //
  // If Max_Current > Max_Possible_I then
  //    Max_Current_Before_Overflow = MaxPossible_I
  // Else
  //    Max_Current_Before_Overflow = Max_Current
  // End If
  //
  // ... In this case, we're good though since Max_Current is less than
  // MaxPossible_I
  //
  // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
  // Max_ShuntVoltage = 0.131068V
  //
  // If Max_ShuntVoltage >= VSHUNT_MAX
  //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
  // Else
  //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
  // End If

  // 8. Compute the Maximum Power
  // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
  // MaximumPower = 1.31068 * 32V
  // MaximumPower = 41.94176W

  // Set multipliers to convert raw current/power values
  ina219_currentDivider_mA = 25;    // Current LSB = 40uA per bit (1000/40 = 25)
  ina219_powerMultiplier_mW = 0.8f; // Power LSB = 800uW per bit

  // Set Calibration register to 'Cal' calculated above
  Adafruit_BusIO_Register calibration_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CALIBRATION, 2, MSBFIRST);
  calibration_reg.write(ina219_calValue, 2);

  // Set Config register to take into account the settings above
  ina219_configValue = INA219_CONFIG_BVOLTAGERANGE_32V |
                    INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
  Adafruit_BusIO_Register config_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CONFIG, 2, MSBFIRST);
  _success = config_reg.write(ina219_configValue, 2);
}

/*!
 *  @brief set device to calibration which uses the highest precision for
 *     current measurement (0.1mA), at the expense of
 *     only supporting 16V at 400mA max.
 */
void Adafruit_INA219::setCalibration_16V_400mA() {

  // Calibration which uses the highest precision for
  // current measurement (0.1mA), at the expense of
  // only supporting 16V at 400mA max.

  // VBUS_MAX = 16V
  // VSHUNT_MAX = 0.04          (Assumes Gain 1, 40mV)
  // RSHUNT = 0.1               (Resistor value in ohms)
  ina219_shuntResistor_Ohms = 0.1;

  // 1. Determine max possible current
  // MaxPossible_I = VSHUNT_MAX / RSHUNT
  // MaxPossible_I = 0.4A

  // 2. Determine max expected current
  // MaxExpected_I = 0.4A

  // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
  // MinimumLSB = MaxExpected_I/32767
  // MinimumLSB = 0.0000122              (12uA per bit)
  // MaximumLSB = MaxExpected_I/4096
  // MaximumLSB = 0.0000977              (98uA per bit)

  // 4. Choose an LSB between the min and max values
  //    (Preferrably a roundish number close to MinLSB)
  // CurrentLSB = 0.00005 (50uA per bit)

  // 5. Compute the calibration register
  // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
  // Cal = 8192 (0x2000)

  ina219_calValue = 8192;

  // 6. Calculate the power LSB
  // PowerLSB = 20 * CurrentLSB
  // PowerLSB = 0.001 (1mW per bit)

  // 7. Compute the maximum current and shunt voltage values before overflow
  //
  // Max_Current = Current_LSB * 32767
  // Max_Current = 1.63835A before overflow
  //
  // If Max_Current > Max_Possible_I then
  //    Max_Current_Before_Overflow = MaxPossible_I
  // Else
  //    Max_Current_Before_Overflow = Max_Current
  // End If
  //
  // Max_Current_Before_Overflow = MaxPossible_I
  // Max_Current_Before_Overflow = 0.4
  //
  // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
  // Max_ShuntVoltage = 0.04V
  //
  // If Max_ShuntVoltage >= VSHUNT_MAX
  //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
  // Else
  //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
  // End If
  //
  // Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
  // Max_ShuntVoltage_Before_Overflow = 0.04V

  // 8. Compute the Maximum Power
  // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
  // MaximumPower = 0.4 * 16V
  // MaximumPower = 6.4W

  // Set multipliers to convert raw current/power values
  ina219_currentDivider_mA = 20;    // Current LSB = 50uA per bit (1000/50 = 20)
  ina219_powerMultiplier_mW = 1.0f; // Power LSB = 1mW per bit

  // Set Calibration register to 'Cal' calculated above
  Adafruit_BusIO_Register calibration_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CALIBRATION, 2, MSBFIRST);
  calibration_reg.write(ina219_calValue, 2);
  // Set Config register to take into account the settings above
  ina219_configValue = INA219_CONFIG_BVOLTAGERANGE_16V |
                    INA219_CONFIG_GAIN_1_40MV | INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

  Adafruit_BusIO_Register config_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CONFIG, 2, MSBFIRST);
  _success = config_reg.write(ina219_configValue, 2);
}

/*!
 *  @brief  Provides the underlying return value from the last operation
 *          called on the device.
 *  @return true: Last operation was successful false: Last operation failed
 *  @note   For function calls that have intermediary device operations,
 *          e.g. calibration before read/write, only the final operation's
 *          result is stored.
 */
bool Adafruit_INA219::success() { return _success; }

/*!
 *  @brief  Provides access to the Conversion Ready flag from the last bus
 *          voltage read on the device.
 *  @return true: Last bus read operation had shown the Conversion Ready flag.
 */
bool Adafruit_INA219::conversionReady() { return _conversionReady; }

/*!
 *  @brief  Provides access to the Overflow flag from the last bus voltage
 *          read on the device.
 *  @return true: There was an overflow when calculating the current and/ or
 *          power.
 */
bool Adafruit_INA219::overflow() { return _overflow; }
