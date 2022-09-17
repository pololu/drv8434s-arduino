// This example shows basic use of a DRV8434S stepper motor driver.
//
// It shows how to initialize the driver, configure various settings, and enable
// the driver. It shows how to step the motor and switch directions using the
// driver's SPI interface through the library's step() and setDirection() member
// functions.
//
// Since SPI is used to trigger steps and set the direction, connecting the
// driver's STEP and DIR pins is optional for this example. However, note that
// using SPI control adds some overhead compared to using the STEP and DIR pins.
//
// Before using this example, be sure to change the setCurrentMilliamps line
// to have an appropriate current limit for your system.  Also, see this
// library's documentation for information about how to connect the driver:
//   https://pololu.github.io/drv8434s-arduino

#include <SPI.h>
#include <DRV8434S.h>

const uint8_t CSPin = 4;

// This period is the length of the delay between steps, which controls the
// stepper motor's speed.  You can increase the delay to make the stepper motor
// go slower.  If you decrease the delay, the stepper motor will go faster, but
// there is a limit to how fast it can go before it starts missing steps.
const uint16_t StepPeriodUs = 2000;

DRV8434S sd;

void setup()
{
  SPI.begin();
  sd.setChipSelectPin(CSPin);

  // Give the driver some time to power up.
  delay(1);

  // Reset the driver to its default settings and clear latched fault
  // conditions.
  sd.resetSettings();
  sd.clearFaults();

  // Optional: Change the decay mode. The default is Smart tune Ripple Control,
  // and we find that it usually works well, but you might want to experiment
  // with other decay modes.
  //sd.setDecayMode(DRV8434SDecayMode::SmartTuneDynamicDecay);

  // Set the scaled current limit. You should change the number here to an
  // appropriate value for your particular system.
  //
  // This function call assumes that VREF on the DRV8434S has been set to
  // produce a full current limit of 2000 mA. This is true if you are using the
  // "2A Max. Current Limit" version or if you have VREF set to 2.64 V on the
  // "Potentiometer for Max. Current Limit" version.
  //
  // Otherwise, you should pass your actual full current limit as the second
  // argument to this function; for example, if you want a current limit of 1 A
  // and your full current limit is set to 1.5 A, you can use:
  //
  // sd.setCurrentMilliamps(1000, 1500);
  //
  // Alternatively, you can use the setCurrentPercent() function to set the
  // scaled current limit as a percentage of the full current limit, or if you
  // are using the potentiometer version, you can set the current limit using
  // VREF alone and comment out this line to leave the current scalar at 100%.
  sd.setCurrentMilliamps(1000);

  // Set the number of microsteps that correspond to one full step.
  sd.setStepMode(DRV8434SStepMode::MicroStep32);

  // Enable direction and step control through SPI.
  sd.enableSPIDirection();
  sd.enableSPIStep();

  // Enable the motor outputs.
  sd.enableDriver();
}

void loop()
{
  // Step in the default direction 1000 times.
  sd.setDirection(0);
  for(unsigned int x = 0; x < 1000; x++)
  {
    sd.step();
    delayMicroseconds(StepPeriodUs);
  }

  // Wait for 300 ms.
  delay(300);

  // Step in the other direction 1000 times.
  sd.setDirection(1);
  for(unsigned int x = 0; x < 1000; x++)
  {
    sd.step();
    delayMicroseconds(StepPeriodUs);
  }

  // Wait for 300 ms.
  delay(300);
}
