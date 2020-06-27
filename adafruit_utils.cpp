/**
 * This code is from the Adafruit_nRF52_Arduino repo here: 
 * https://github.com/adafruit/Adafruit_nRF52_Arduino
 */
#include "adafruit_utils.h"
#include <bluefruit.h>

#define VBAT_PIN          A7
#define VBAT_MV_PER_LSB   (0.73242188F) // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define VBAT_DIVIDER      (0.71275837F)   // 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
#define VBAT_DIVIDER_COMP (1.403F)        // Compensation factor for the VBAT divider
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)

/**
 * Reads battery ADC.
 * @return battery voltage in mV
 */
int readVBAT(void) {
  int raw;

  // Set the analog reference to 3.0V (default = 3.6V) 
  analogReference(AR_INTERNAL_3_0);
  
  // Set the resolution to 12-bit (0..4095) 
  analogReadResolution(12); // Can be 8, 10, 12 or 14
  
  // Let the ADC settle
  delay(1);
  
  // Get the raw 12-bit, 0..3000mV ADC value 
  raw = analogRead(VBAT_PIN);
  
  // Set the ADC back to the default settings 
  analogReference(AR_DEFAULT); 
  analogReadResolution(10);
  
  // Convert the raw value to compensated mv, taking the resistor-
  // divider into account (providing the actual LIPO voltage)
  // ADC range is 0..3000mV and resolution is 12-bit (0..4095)
  return raw * REAL_VBAT_MV_PER_LSB;
}

/**
 * Battery voltage to percent full.
 * From Adafruit example code, don't know where the constants are from.
 * @param mvolts battery voltage in millivolts
 * @return percent charge from 1 to 100
 */
uint8_t mvToPercent(float mvolts) { 
  if (mvolts < 3300) {
    return 0;
  }

  if (mvolts < 3600) {
    mvolts -= 3300;
    return mvolts / 30;
  }

  mvolts -= 3600;
  return 10 + (mvolts * 0.15F);  // thats mvolts /6.66666666
}
