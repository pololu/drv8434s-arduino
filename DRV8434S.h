// Copyright Pololu Corporation.  For more information, see http://www.pololu.com/

/// \file DRV8434S.h
///
/// This is the main header file for the DRV8434S library,
/// a library for controlling the DRV8434S stepper motor driver.
///
/// For more information about this library, see:
///
///   https://github.com/pololu/drv8434s-arduino
///
/// That is the main repository for this library.

#pragma once

#include <Arduino.h>
#include <SPI.h>

/// Addresses of control and status registers.
enum class DRV8434SRegAddr : uint8_t
{
  FAULT = 0x00,
  DIAG1 = 0x01,
  DIAG2 = 0x02,
  CTRL1 = 0x03,
  CTRL2 = 0x04,
  CTRL3 = 0x05,
  CTRL4 = 0x06,
  CTRL5 = 0x07,
  CTRL6 = 0x08,
  CTRL7 = 0x09,
  CTRL8 = 0x0A,
  CTRL9 = 0x0B,
};


/// This class provides low-level functions for reading and writing from the SPI
/// interface of a DRV8434S stepper motor controller IC.
///
/// Most users should use the HighPowerStepperDriver class, which provides a
/// higher-level interface, instead of this class.
class DRV8434SSPI
{
public:
  /// Configures this object to use the specified pin as a chip select pin.
  ///
  /// You must use a chip select pin; the DRV8434S requires it.
  void setChipSelectPin(uint8_t pin)
  {
    csPin = pin;
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
  }

  /// Reads the register at the given address and returns its raw value.
  uint8_t readReg(uint8_t address)
  {
    // Arduino out / DRV8434 in: First byte contains read/write bit and register
    // address; second byte is unused.
    // Arduino in / DRV8434 out: First byte contains status; second byte
    // contains data in register being read.

    selectChip();
    lastStatus = transfer((0x20 | (address & 0b11111)) << 1);
    uint8_t data = transfer(0);
    deselectChip();
    return data;
  }

  /// Reads the register at the given address and returns its raw value.
  uint16_t readReg(DRV8434SRegAddr address)
  {
    return readReg((uint8_t)address);
  }

  /// Writes the specified value to a register.
  uint8_t writeReg(uint8_t address, uint8_t value)
  {
    // Arduino out / DRV8434 in: First byte contains read/write bit and register
    // address; second byte contains data to write to register.
    // Arduino in / DRV8434 out: First byte contains status; second byte
    // contains old (existing) data in register being written to.

    selectChip();
    lastStatus = transfer((address & 0b11111) << 1);
    uint8_t oldData = transfer(value);
    // The CS line must go low after writing for the value to actually take
    // effect.
    deselectChip();
    return oldData;
  }

  /// Writes the specified value to a register.
  void writeReg(DRV8434SRegAddr address, uint8_t value)
  {
    writeReg((uint8_t)address, value);
  }

private:

  SPISettings settings = SPISettings(500000, MSBFIRST, SPI_MODE1);

  uint8_t transfer(uint8_t value)
  {
    return SPI.transfer(value);
  }

  void selectChip()
  {
    digitalWrite(csPin, LOW);
    SPI.beginTransaction(settings);
  }

  void deselectChip()
  {
   SPI.endTransaction();
   digitalWrite(csPin, HIGH);
  }

  uint8_t csPin;

public:

  /// The status reported by the driver during the last read or write.  This
  /// status is the same as that which would be returned by reading the FAULT
  /// register with DRV8434S::readFault(), except the upper two bits are always
  /// 1.
  uint8_t lastStatus = 0;
};


/// Bits that are set in the return value of readFault() to indicate warning and
/// fault conditions.
///
/// See the DRV8434S datasheet for detailed descriptions of these conditions.
enum class DRV8434SFaultBit : uint8_t
{
  /// Fault indication (0 when nFAULT pin is high, 1 when nFAULT pin is low)
  FAULT = 7,

  /// SPI protocol error (latched)
  SPI_ERROR = 6,

  /// Supply undervoltage lockout fault
  UVLO = 5,

  /// Charge pump undervoltage fault
  CPUV = 4,

  /// Overcurrent fault
  OCP = 3,

  /// Motor stall
  STL = 2,

  /// Overtemperature warning or shutdown
  TF = 1,

  /// Open load
  OL = 0,
};

/// Bits that are set in the return value of readDiag1() to indicate warning and
/// fault conditions.
///
/// See the DRV8434S datasheet for detailed descriptions of these conditions.
enum class DRV8434SDiag1Bit : uint8_t
{
  /// Overcurrent fault on low-side FET of half bridge 2 in BOUT
  OCP_LS2_B = 7,

  /// Overcurrent fault on high-side FET of half bridge 2 in BOUT
  OCP_HS2_B = 6,

  /// Overcurrent fault on low-side FET of half bridge 1 in BOUT
  OCP_LS1_B = 5,

  /// Overcurrent fault on high-side FET of half bridge 1 in BOUT
  OCP_HS1_B = 4,

  /// Overcurrent fault on low-side FET of half bridge 2 in AOUT
  OCP_LS2_A = 3,

  /// Overcurrent fault on high-side FET of half bridge 2 in AOUT
  OCP_HS2_A = 2,

  /// Overcurrent fault on low-side FET of half bridge 1 in AOUT
  OCP_LS1_A = 1,

  /// Overcurrent fault on high-side FET of half bridge 1 in AOUT
  OCP_HS1_A = 0,
};

/// Bits that are set in the return value of readDiag2() to indicate warning and
/// fault conditions.
///
/// See the DRV8434S datasheet for detailed descriptions of these conditions.
enum class DRV8434SDiag2Bit : uint8_t
{
  /// Overtemperature warning
  OTW = 6,

  /// Overtemperature shutdown
  OTS = 5,

  /// Stall detection learning successful
  STL_LRN_OK = 4,

  /// Motor stall condition
  STALL = 3,

  /// Open load on BOUT
  OL_B = 1,

  /// Open load on AOUT
  OL_A = 0,
};

/// Possible arguments to setDecayMode().
enum class DRV8434SDecayMode : uint8_t
{
  Slow                   = 0b000,
  IncSlowDecMixed30      = 0b001,
  IncSlowDecMixed60      = 0b010,
  IncSlowDecFast         = 0b011,
  Mixed30                = 0b100,
  Mixed60                = 0b101,
  SmartTuneDynamicDecay  = 0b110,
  SmartTuneRippleControl = 0b111,
};

/// Possible arguments to setStepMode().
enum class DRV8434SStepMode : uint8_t
{
  /// Full step with 100% current
  MicroStep1_100 = 0b0000,

  /// Full step with 71% current
  MicroStep1     = 0b0001,

  /// Non-circular 1/2 step
  MicroStep2_NC  = 0b0010,

  /// Circular 1/2 step
  MicroStep2     = 0b0011,

  MicroStep4     = 0b0100,
  MicroStep8     = 0b0101,
  MicroStep16    = 0b0110,
  MicroStep32    = 0b0111,
  MicroStep64    = 0b1000,
  MicroStep128   = 0b1001,
  MicroStep256   = 0b1010,
};


/// This class provides high-level functions for controlling a DRV8434S stepper
/// motor driver.
class DRV8434S
{
public:
  /// The default constructor.
  DRV8434S()
  {
    // All settings set to power-on defaults
    ctrl1 = 0x00;
    ctrl2 = 0x0F;
    ctrl3 = 0x06;
    ctrl4 = 0x30;
    ctrl5 = 0x08;
    ctrl6 = 0x03;
    ctrl7 = 0x20;
  }

  /// Configures this object to use the specified pin as a chip select pin.
  /// You must use a chip select pin; the DRV8711 requires it.
  void setChipSelectPin(uint8_t pin)
  {
    driver.setChipSelectPin(pin);
  }

  /// Changes all of the driver's settings back to their default values.
  ///
  /// It is good to call this near the beginning of your program to ensure that
  /// there are no settings left over from an earlier time that might affect the
  /// operation of the driver.
  void resetSettings()
  {
    ctrl1 = 0x00;
    ctrl2 = 0x0F;
    ctrl3 = 0x06;
    ctrl4 = 0x30;
    ctrl5 = 0x08;
    ctrl6 = 0x03;
    ctrl7 = 0x20;
    applySettings();
  }

  /// Reads back the SPI configuration registers from the device and verifies
  /// that they are equal to the cached copies stored in this class.
  ///
  /// This can be used to verify that the driver is powered on and has not lost
  /// them due to a power failure.  The STATUS register is not verified because
  /// it does not contain any driver settings.
  ///
  /// @return 1 if the settings from the device match the cached copies, 0 if
  /// they do not.
  bool verifySettings()
  {
    return driver.readReg(DRV8434SRegAddr::CTRL1) == ctrl1 &&
           driver.readReg(DRV8434SRegAddr::CTRL2) == ctrl2 &&
           driver.readReg(DRV8434SRegAddr::CTRL3) == ctrl3 &&
           driver.readReg(DRV8434SRegAddr::CTRL4) == ctrl4 &&
           driver.readReg(DRV8434SRegAddr::CTRL5) == ctrl5 &&
           driver.readReg(DRV8434SRegAddr::CTRL6) == ctrl6 &&
           driver.readReg(DRV8434SRegAddr::CTRL7) == ctrl7;
  }

  /// Re-writes the cached settings stored in this class to the device.
  ///
  /// You should not normally need to call this function because settings are
  /// written to the device whenever they are changed.  However, if
  /// verifySettings() returns false (due to a power interruption, for
  /// instance), then you could use applySettings() to get the device's settings
  /// back into the desired state.
  void applySettings()
  {
    writeCachedReg(DRV8434SRegAddr::CTRL1);
    writeCachedReg(DRV8434SRegAddr::CTRL3);
    writeCachedReg(DRV8434SRegAddr::CTRL4);
    writeCachedReg(DRV8434SRegAddr::CTRL5);
    writeCachedReg(DRV8434SRegAddr::CTRL6);
    writeCachedReg(DRV8434SRegAddr::CTRL7);

    // CTRL2 is written last because it contains the EN_OUT bit, and we want to
    // try to have all the other settings correct first.
    writeCachedReg(DRV8434SRegAddr::CTRL2);
  }

  /// Sets the driver's current scalar (TRQ_DAC), which scales the full current
  /// limit (as set by VREF) by the specified percentage. The available settings
  /// are multiples of 6.25%.
  ///
  /// This function takes an integer, and if the desired current limit is not
  /// available, it generally tries to pick the closest current limit that is
  /// lower than the desired one (although the lowest possible setting is
  /// 6.25%). However, it will round up if the next setting is no more than
  /// 0.75% higher; this allows you to specify 43.75% by passing a value of 43,
  /// for example.
  ///
  /// Example usage:
  /// ~~~{.cpp}
  /// // This sets TRQ_DAC to 37.5% (the closest setting lower than 42%):
  /// sd.setCurrentPercent(42);
  ///
  /// // This sets TRQ_DAC to 43.75% (rounding 43 up by 0.75% to 43.75%):
  /// sd.setCurrentPercent(43);
  ///
  /// // This also sets TRQ_DAC to 43.75%; even though the argument is truncated
  /// // to an integer (43), that is then rounded up by 0.75% to 43.75%:
  /// sd.setCurrentPercent(43.75);
  /// ~~~
  void setCurrentPercent(uint8_t percent)
  {
    if (percent > 100) { percent = 100; }
    if (percent < 6) { percent = 6; }

    uint8_t td = ((uint16_t)percent * 4 + 3) / 25; // convert 6-100% to 1-16, rounding up by at most 0.75%
    td = 16 - td;                                  // convert 1-16 to 15-0 (15 = 6.25%, 0 = 100%)
    ctrl1 = (ctrl1 & 0b00001111) | (td << 4);
    writeCachedReg(DRV8434SRegAddr::CTRL1);
  }

  /// Sets the driver's current scalar (TRQ_DAC) to produce the specified scaled
  /// current limit in milliamps. In order to calculate the correct value for
  /// TRQ_DAC, this function also needs to know the full current limit set by
  /// VREF (i.e. what the current limit is when the scaling is set to 100%).
  /// This is specified by the optional `fullCurrent` argument, which defaults
  /// to 2000 milliamps (2 A).
  ///
  /// If the desired current limit is not
  /// available, this function tries to pick the closest current limit that is
  /// lower than the desired one (although the lowest possible setting is 6.25%
  /// of the full current limit).
  ///
  /// Example usage:
  /// ~~~{.cpp}
  /// // This specifies that we want a scaled current limit of 1200 mA and that
  /// // VREF is set to produce a full current limit of 1500 mA. TRQ_DAC will be
  /// // set to 75%, which will produce a 1125 mA scaled current limit.
  /// sd.setCurrentMilliamps(1200, 1500);
  /// ~~~
  void setCurrentMilliamps(uint16_t current, uint16_t fullCurrent = 2000)
  {
    if (fullCurrent > 4000) { fullCurrent = 4000; }
    if (current > fullCurrent) { current = fullCurrent; }

    uint8_t td = (current * 16 / fullCurrent); // convert 0-fullCurrent to 0-16
    if (td == 0) { td = 1; }                   // restrict to 1-16
    td = 16 - td;                              // convert 1-16 to 15-0 (15 = 6.25%, 0 = 100%)
    ctrl1 = (ctrl1 & 0b00001111) | (td << 4);
    writeCachedReg(DRV8434SRegAddr::CTRL1);
  }

  /// Enables the driver (EN_OUT = 1).
  void enableDriver()
  {
    ctrl2 |= (1 << 7);
    writeCachedReg(DRV8434SRegAddr::CTRL2);
  }

  /// Disables the driver (EN_OUT = 0).
  void disableDriver()
  {
    ctrl2 &= ~(1 << 7);
    writeCachedReg(DRV8434SRegAddr::CTRL2);
  }

  /// Sets the driver's decay mode (DECAY).
  ///
  /// Example usage:
  /// ~~~{.cpp}
  /// sd.setDecayMode(DRV8434SDecayMode::SmartTuneDynamicDecay);
  /// ~~~
  void setDecayMode(DRV8434SDecayMode mode)
  {
    ctrl2 = (ctrl2 & 0b11111000) | (((uint8_t)mode & 0b111) << 8);
    writeCachedReg(DRV8434SRegAddr::CTRL2);
  }

  /// Sets the motor direction (DIR).
  ///
  /// Allowed values are 0 or 1.
  ///
  /// You must first call enableSPIDirection() to allow the direction to be
  /// controlled through SPI.  Once you have done so, you can use this command
  /// to control the direction of the stepper motor and leave the DIR pin
  /// disconnected.
  void setDirection(bool value)
  {
    if (value)
    {
      ctrl3 |= (1 << 7);
    }
    else
    {
      ctrl3 &= ~(1 << 7);
    }
    writeCachedReg(DRV8434SRegAddr::CTRL3);
  }

  /// Returns the cached value of the motor direction (DIR).
  ///
  /// This does not perform any SPI communication with the driver.
  bool getDirection()
  {
    return (ctrl3 >> 7) & 1;
  }

  /// Advances the indexer by one step (STEP = 1).
  ///
  /// You must first call enableSPIStep() to allow stepping to be controlled
  /// through SPI.  Once you have done so, you can use this command to step the
  /// motor and leave the STEP pin disconnected.
  ///
  /// The driver automatically clears the STEP bit after it is written.
  void step()
  {
    driver.writeReg(DRV8434SRegAddr::CTRL3, ctrl3 | (1 << 6));
  }

  /// Enables direction control through SPI (SPI_DIR = 1), allowing
  /// setDirection() to override the DIR pin.
  void enableSPIDirection()
  {
    ctrl3 |= (1 << 5);
    writeCachedReg(DRV8434SRegAddr::CTRL3);
  }

  /// Disables direction control through SPI (SPI_DIR = 0), making the DIR pin
  /// control direction instead.
  void disableSPIDirection()
  {
    ctrl3 &= ~(1 << 5);
    writeCachedReg(DRV8434SRegAddr::CTRL3);
  }

  /// Enables stepping through SPI (SPI_STEP = 1), allowing step() to override
  /// the STEP pin.
  void enableSPIStep()
  {
    ctrl3 |= (1 << 4);
    writeCachedReg(DRV8434SRegAddr::CTRL3);
  }

  /// Disables stepping through SPI (SPI_STEP = 0), making the STEP pin control
  /// stepping instead.
  void disableSPIStep()
  {
    ctrl3 &= ~(1 << 4);
    writeCachedReg(DRV8434SRegAddr::CTRL3);
  }

  /// Sets the driver's stepping mode (MICROSTEP_MODE).
  ///
  /// This affects many things about the performance of the motor, including how
  /// much the output moves for each step taken and how much current flows
  /// through the coils in each stepping position.
  ///
  /// If an invalid stepping mode is passed to this function, then it selects
  /// 1/16 micro-step, which is the driver's default.
  ///
  /// Example usage:
  /// ~~~{.cpp}
  /// sd.setStepMode(DRV8434SStepMode::MicroStep32);
  /// ~~~
  void setStepMode(DRV8434SStepMode mode)
  {
    if (mode > DRV8434SStepMode::MicroStep256)
    {
      // Invalid mode; pick 1/16 micro-step by default.
      mode = DRV8434SStepMode::MicroStep16;
    }

    ctrl3 = (ctrl3 & 0b11110000) | (uint8_t)mode;
    writeCachedReg(DRV8434SRegAddr::CTRL3);
  }

  /// Sets the driver's stepping mode (MICROSTEP_MODE).
  ///
  /// This version of the function allows you to express the requested
  /// microstepping ratio as a number directly.
  ///
  /// Example usage:
  /// ~~~{.cpp}
  /// sd.setStepMode(32);
  /// ~~~
  void setStepMode(uint16_t mode)
  {
    DRV8434SStepMode sm;

    switch (mode)
    {
      case 1:   sm = DRV8434SStepMode::MicroStep1;   break;
      case 2:   sm = DRV8434SStepMode::MicroStep2;   break;
      case 4:   sm = DRV8434SStepMode::MicroStep4;   break;
      case 8:   sm = DRV8434SStepMode::MicroStep8;   break;
      case 16:  sm = DRV8434SStepMode::MicroStep16;  break;
      case 32:  sm = DRV8434SStepMode::MicroStep32;  break;
      case 64:  sm = DRV8434SStepMode::MicroStep64;  break;
      case 128: sm = DRV8434SStepMode::MicroStep128; break;
      case 256: sm = DRV8434SStepMode::MicroStep256; break;

      // Invalid mode; pick 1/16 micro-step by default.
      default:  sm = DRV8434SStepMode::MicroStep16;
    }

    setStepMode(sm);
  }

  /// Reads the FAULT status register of the driver.
  ///
  /// The return value is an 8-bit unsigned integer that has one bit for each
  /// FAULT condition.  You can simply compare the return value to 0 to see if
  /// any of the bits are set, or you can use the logical AND operator (`&`) and
  /// the #DRV8434SFaultBit enum to check individual bits.
  ///
  /// Example usage:
  /// ~~~{.cpp}
  /// if (sd.readFault() & (1 << (uint8_t)DRV8434SFaultBit::UVLO))
  /// {
  ///   // Supply undervoltage lockout is active.
  /// }
  /// ~~~
  uint8_t readFault()
  {
    return driver.readReg(DRV8434SRegAddr::FAULT);
  }

  /// Reads the DIAG1 status register of the driver.
  ///
  /// The return value is an 8-bit unsigned integer that has one bit for each
  /// DIAG1 condition.  You can simply compare the return value to 0 to see if
  /// any of the bits are set, or you can use the logical AND operator (`&`) and
  /// the #DRV8434SDiag1Bit enum to check individual bits.
  uint8_t readDiag1()
  {
    return driver.readReg(DRV8434SRegAddr::DIAG1);
  }

  /// Reads the DIAG2 status register of the driver.
  ///
  /// The return value is an 8-bit unsigned integer that has one bit for each
  /// DIAG2 condition.  You can simply compare the return value to 0 to see if
  /// any of the bits are set, or you can use the logical AND operator (`&`) and
  /// the #DRV8434SDiag2Bit enum to check individual bits.
  uint8_t readDiag2()
  {
    return driver.readReg(DRV8434SRegAddr::DIAG2);
  }

  /// Clears any fault conditions that are currently latched in the driver
  /// (CLR_FLT = 1).
  ///
  /// WARNING: Calling this function clears latched faults, which might allow
  /// the motor driver outputs to reactivate.  If you do this repeatedly without
  /// fixing an abnormal condition (like a short circuit), you might damage the
  /// driver.
  ///
  /// The driver automatically clears the CLR_FLT bit after it is written.
  void clearFaults()
  {
    driver.writeReg(DRV8434SRegAddr::CTRL4, ctrl4 | (1 << 7));
  }

  /// Writes the specified value to a register after updating the cached value
  /// to match.
  ///
  /// Using this function keeps this object's cached settings consistent with
  /// the settings being written to the driver, so if you are using
  /// verifySettings(), applySettings(), and/or any of the other functions for
  /// specific settings that this library provides, you should use this function
  /// for direct register accesses instead of calling DRV8434SSPI::writeReg()
  /// directly.
  void setReg(DRV8434SRegAddr address, uint8_t value)
  {
    if (address < DRV8434SRegAddr::CTRL1) { return; }
    if (address > DRV8434SRegAddr::CTRL7) { return; }
    *cachedReg[(uint8_t)address] = value;
    driver.writeReg(address, value);
  }

protected:

  uint16_t ctrl1, ctrl2, ctrl3, ctrl4, ctrl5, ctrl6, ctrl7;

  // Lookup table for converting register address to cache variable pointer
  uint16_t * const cachedReg[12] =
  {
    nullptr, // 0x00 FAULT (not writable or cached)
    nullptr, // 0x01 DIAG1 (not writable or cached)
    nullptr, // 0x02 DIAG2 (not writable of cached)
    &ctrl1,  // 0x03
    &ctrl2,  // 0x04
    &ctrl3,  // 0x05
    &ctrl4,  // 0x06
    &ctrl5,  // 0x07
    &ctrl6,  // 0x08
    &ctrl7,  // 0x09
    nullptr, // 0x0A CTRL8 (not writable or cached)
    nullptr, // 0x0B CTRL9 (not writable or cached)
  };

  /// Writes the cached value of the given register to the device.
  void writeCachedReg(DRV8434SRegAddr address)
  {
    if (address < DRV8434SRegAddr::CTRL1) { return; }
    if (address > DRV8434SRegAddr::CTRL7) { return; }
    uint8_t value = *cachedReg[(uint8_t)address];
    driver.writeReg(address, value);
  }

public:
  /// This object handles all the communication with the DRV8711.  Generally,
  /// you should not need to use it in your code for basic usage of a
  /// High-Power Stepper Motor Driver, but you might want to use it to access
  /// more advanced settings that the HighPowerStepperDriver class does not
  /// provide functions for.
  DRV8434SSPI driver;
};
