// This example demonstrates direct access to a control register in the DRV8434S
// stepper motor driver.
//
// The DRV8434S library does not provide dedicated functions for some more
// advanced settings, such as the TOFF setting in the CTRL2 register. These
// settings can still be changed by using the library's setReg() function to
// write a value to the register directly. The setReg() function also updates
// the library object's cached settings to match the value being written to the
// device. This is important to maintain consistency when other library
// functions are later used, and it allows the driver's settings to be verified
// against the cached settings (and the cached settings to be re-applied if
// necessary).
//
// See this library's documentation for information about how to connect the
// driver: https://pololu.github.io/drv8434s-arduino

#include <SPI.h>
#include <DRV8434S.h>

const uint8_t CSPin = 4;

DRV8434S sd;

void setup()
{
  Serial.begin(115200);

  SPI.begin();
  sd.setChipSelectPin(CSPin);

  // Give the driver some time to power up.
  delay(1);

  // Reset the driver to its default settings and clear latched fault
  // conditions.
  sd.resetSettings();
  sd.clearFaults();
}

void loop()
{
  uint8_t ctrl2 = sd.getCachedReg(DRV8434SRegAddr::CTRL2);

  ctrl2 = (ctrl2 & 0b11100111) | (0b10 << 3); // TOFF = 0b10: 24 us PWM off time
  sd.setReg(DRV8434SRegAddr::CTRL2, ctrl2);
  Serial.println("TOFF set to 24 us");

  tryVerifySettings();

  delay(1000);

  ctrl2 = (ctrl2 & 0b11100111) | (0b01 << 3); // TOFF = 0b01: 16 us PWM off time
  sd.setReg(DRV8434SRegAddr::CTRL2, ctrl2);
  Serial.println("TOFF set to 16 us");

  tryVerifySettings();

  delay(1000);
}

void tryVerifySettings()
{
  if (sd.verifySettings())
  {
    Serial.println("Settings verified");
  }
  else
  {
    Serial.println("Could not verify settings");
  }
}