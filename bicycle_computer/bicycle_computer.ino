/* 
 *  RPM counter based on sketch by InterlinkKnight
 *  https://www.youtube.com/watch?v=u2uJMJWsfsg
 *  https://drive.google.com/file/d/1fF-b-D2hYyrRiSvu4FH45FCHrIeO1kYh/view
 *  
 *  Nokia 5110 i2c lib by Maxint R&D 2017.
 *  https://github.com/maxint-rd/I2C-PCF8574-PCD8544-Nokia-5110-LCD
 *  
 */

// --- set debug start ---
//#define DEBUG_SERIAL
// --- set debug end ---

#ifndef DEBUG_SERIAL
  // --- lcd setup start ---
  #include <PCF8574_PCD8544.h>
  // I2C to SPI via PCF8574 interface (slower updates, less pins):
  // address (LCD interface: 0x20-0x27 selectable by connecting pads, all open=0x27)
  // pcf-P7 - 5. Serial clock out (SCLK, CLK)
  // pcf-P6 - 4. Serial data out (DIN)
  // pcf-P5 - 3. Data/Command select (D/C, DC)
  // pcf-P4 - 2. LCD chip select (CS, CE), can be set to -1 if not used (tie line low)
  // pcf-P2 - 1. LCD reset (RST), can be set to -1 if not used (tie line high or to reset of MCU)
  // pcf-P3 - 7. Backlight control (LIGHT), not used in i2c display constructor
  PCF8574_PCD8544 display = PCF8574_PCD8544(0x27, 7, 6, 5, 4, 2);
  // --- lcd setup end ---
#endif


// --- set variables start ---
const float fwVersion = 0.02;               // Firmware version
const int wheelDiameterIn = 26;             // Wheel diameter in inches
const int cadPin = 2;                       // Set cadence sensor pin
const int spdPin = 3;                       // Set speed sensor pin

//-------------------------------------------------------------------------------------------
const int wheelDiameterCm = wheelDiameterIn * 2.54; // Convert wheel diameter to centimeters
const int cadInt = digitalPinToInterrupt(cadPin); // Convert pin number to interrupt number (pin2 = int0)
const int spdInt = digitalPinToInterrupt(spdPin); // Convert pin number to interrupt number (pin3 = int1)
// --- set variables end ---

//---------------------------------------------------------------------------------------------------------

// --- set update LCD without delay start ---
unsigned long CurrentMillis = 0;            // Store curret cycle start time 
unsigned long lcdPreviousUpdateMillis = 0;  // Last LCD update time
const int lcdUpdateInterval = 100;          // Update LCD interval (milliseconds)
// --- set update LCD without delay end ---


// --- RPM counter config start ---

// Cadence
volatile unsigned long cadPeriodBetweenPulses = 0; // Stores the period between pulses in microseconds.
volatile unsigned long cadLastTimeWeMeasured;      // Stores the last time we measured a pulse so we can calculate the period
unsigned long cadLastTimeCycleMeasure = 0;  // Stores the last time we measure a pulse in that cycle.
                                            // We need a variable with a value that is not going to be affected by the interrupt
                                            // because we are going to do math and functions that are going to mess up if the values
                                            // changes in the middle of the cycle.
unsigned long cadCurrentMicros = micros();// Stores the micros in that cycle.
                                          // We need a variable with a value that is not going to be affected by the interrupt
                                          // because we are going to do math and functions that are going to mess up if the values
                                          // changes in the middle of the cycle.

// We get the RPM by measuring the time between 2 or more pulses so the following will set how many pulses to
// take before calculating the RPM. 1 would be the minimum giving a result every pulse, which would feel very responsive
// even at very low speeds but also is going to be less accurate at higher speeds.
// With a value around 10 you will get a very accurate result at high speeds, but readings at lower speeds are going to be
// farther from eachother making it less "real time" at those speeds.
// There's a function that will set the value depending on the speed so this is done automatically.
unsigned int cadAmountOfReadings = 4;



volatile unsigned long cadPeriodAverage = 2000000; // Stores the period between pulses in microseconds in total, if we are taking multiple pulses.
                                                   // It has a big number so it doesn't start with 0 which would be interpreted as a high frequency.
unsigned long cadPeriodSum = 0;                    // Stores the summation of all the periods to do the average.
#ifdef DEBUG_SERIAL
  unsigned long cadFrequencyReal = 0;              // Frequency without decimals.
#endif
unsigned long cadFrequencyRaw = 0;                 // Calculated frequency, based on the period. This has a lot of extra decimals without the decimal point.

// If the period between pulses is too high, or even if the pulses stopped, then we would get stuck showing the
// last value instead of a 0. Because of this we are going to set a limit for the maximum period allowed.
// If the period is above this value, the RPM will show as 0.
// The higher the set value, the longer lag/delay will have to sense that pulses stopped, but it will allow readings
// at very low RPM.
// Setting a low value is going to allow the detection of stop situations faster, but it will prevent having low RPM readings.
// The unit is in microseconds.
const unsigned long cadZeroTimeout = 3000000;   // For reading very low RPM, a good value would be 300000
                                                // For high response time, a good value would be 100000.

// RPM calibration 
const byte cadPulsesPerRevolution = 1;         // Set how many pulses there are on each revolution. Default: 2.
unsigned long cadRPM = 0;                      // Raw RPM without any processing.
unsigned int cadPulseCounter = 1;              // Counts the amount of pulse readings we took so we can average multiple pulses before calculating the period.

// Calibration for smoothing RPM:
const byte cadNumReadings = 5;                 // Number of samples for smoothing. The higher, the more smoothing, but it's going to
                                               // react slower to changes. 1 = no smoothing. Default: 2.
                                                
// Variables for smoothing tachometer:
unsigned long cadReadings[cadNumReadings];     // The input.
unsigned long cadReadIndex;                    // The index of the current reading.
unsigned long cadTotal;                        // The running total.
unsigned long cadRPMAverage;                   // The RPM value after applying the smoothing.

//-------------------------------------------------------------------------------------------------------------------------------------

// Speed
volatile unsigned long spdPeriodBetweenPulses = 0; // Stores the period between pulses in microseconds.
volatile unsigned long spdLastTimeWeMeasured;      // Stores the last time we measured a pulse so we can calculate the period
unsigned long spdLastTimeCycleMeasure = 0;  // Stores the last time we measure a pulse in that cycle.
                                            // We need a variable with a value that is not going to be affected by the interrupt
                                            // because we are going to do math and functions that are going to mess up if the values
                                            // changes in the middle of the cycle.
unsigned long spdCurrentMicros = micros();// Stores the micros in that cycle.
                                          // We need a variable with a value that is not going to be affected by the interrupt
                                          // because we are going to do math and functions that are going to mess up if the values
                                          // changes in the middle of the cycle.

// We get the RPM by measuring the time between 2 or more pulses so the following will set how many pulses to
// take before calculating the RPM. 1 would be the minimum giving a result every pulse, which would feel very responsive
// even at very low speeds but also is going to be less accurate at higher speeds.
// With a value around 10 you will get a very accurate result at high speeds, but readings at lower speeds are going to be
// farther from eachother making it less "real time" at those speeds.
// There's a function that will set the value depending on the speed so this is done automatically.
unsigned int spdAmountOfReadings = 4;



volatile unsigned long spdPeriodAverage = 2000000; // Stores the period between pulses in microseconds in total, if we are taking multiple pulses.
                                                   // It has a big number so it doesn't start with 0 which would be interpreted as a high frequency.
unsigned long spdPeriodSum = 0;                    // Stores the summation of all the periods to do the average.
#ifdef DEBUG_SERIAL
  unsigned long spdFrequencyReal = 0;              // Frequency without decimals.
#endif
unsigned long spdFrequencyRaw = 0;                 // Calculated frequency, based on the period. This has a lot of extra decimals without the decimal point.

// If the period between pulses is too high, or even if the pulses stopped, then we would get stuck showing the
// last value instead of a 0. Because of this we are going to set a limit for the maximum period allowed.
// If the period is above this value, the RPM will show as 0.
// The higher the set value, the longer lag/delay will have to sense that pulses stopped, but it will allow readings
// at very low RPM.
// Setting a low value is going to allow the detection of stop situations faster, but it will prevent having low RPM readings.
// The unit is in microseconds.
const unsigned long spdZeroTimeout = 3000000;   // For reading very low RPM, a good value would be 300000
                                                // For high response time, a good value would be 100000.

// RPM calibration 
const byte spdPulsesPerRevolution = 1;         // Set how many pulses there are on each revolution. Default: 2.
unsigned long spdRPM = 0;                      // Raw RPM without any processing.
unsigned int spdPulseCounter = 1;              // Counts the amount of pulse readings we took so we can average multiple pulses before calculating the period.

// Calibration for smoothing RPM:
const byte spdNumReadings = 5;                 // Number of samples for smoothing. The higher, the more smoothing, but it's going to
                                                // react slower to changes. 1 = no smoothing. Default: 2.
                                                
// Variables for smoothing tachometer:
unsigned long spdReadings[cadNumReadings];     // The input.
unsigned long spdReadIndex;                    // The index of the current reading.
unsigned long spdTotal;                        // The running total.
unsigned long spdRPMAverage;                   // The RPM value after applying the smoothing.
unsigned long spdKmPh;                         // The Speed in Km/h value.
// --- RPM counter config end ---


// --- debounce interrupt readings start ---
// Cadence
unsigned long cadDebounceCurrentMillis = 0; // store current time
unsigned long cadLastDebounceTime = 0;      // the last time the output pin was toggled
unsigned long cadDebounceDelay = 120;       // the debounce time; increase if the output flickers

//---------------------------------------------------------------------------------------------------

// Speed
unsigned long spdDebounceCurrentMillis = 0; // store current time
unsigned long spdLastDebounceTime = 0;      // the last time the output pin was toggled
unsigned long spdDebounceDelay = 120;       // the debounce time; increase if the output flickers
// --- debounce interrupt readings end ---

// --- update lcd and serial start ---
#ifdef DEBUG_SERIAL
  void update_serial() {
    // Cadence
    Serial.print("CadCnt: ");
    Serial.print(cadPulseCounter);
    Serial.print("\tCadPer: ");
    Serial.print(cadPeriodBetweenPulses);
    Serial.print("\tCadPerAvg: ");
    Serial.print(cadPeriodAverage);
    Serial.print("\tCadFreq: ");
    Serial.print(cadFrequencyReal);
    Serial.print("\tCadRPM: ");
    Serial.print(cadRPM);
    Serial.print("\tCadTacho: ");
    Serial.print(cadRPMAverage);

    // Speed
    Serial.print("\tSpdCnt: ");
    Serial.print(spdPulseCounter);
    Serial.print("\tSpdPeriod: ");
    Serial.print(spdPeriodBetweenPulses);
    Serial.print("\tSpdPeriodAWG: ");
    Serial.print(spdPeriodAverage);
    Serial.print("\tSpdFreq: ");
    Serial.print(spdFrequencyReal);
    Serial.print("\tSpdRPM: ");
    Serial.print(spdRPM);
    Serial.print("\tSpdTacho: ");
    Serial.print(spdRPMAverage);
    Serial.print("\tSpdKmPh: ");
    Serial.println(spdKmPh);
  }
#else
  void update_lcd() {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(BLACK);

    // Cadence
    display.setCursor(0,0);
    display.print(cadRPM);

    // Speed
    display.setCursor(0,15);
    display.print(spdRPM);
    //display.print(spdKmPh);
  
    display.display(); 
  }
#endif
// --- update lcd and serial end ---


//--- interrupt service routine start ---
// Cadence
void cadPulse() {  // The interrupt runs this to calculate the period between pulses:

    // Debounce readings
    cadDebounceCurrentMillis = millis();
    
    if (cadDebounceCurrentMillis - cadLastDebounceTime >= cadDebounceDelay) {
      cadLastDebounceTime = cadDebounceCurrentMillis;   // Update last debounce timer

      cadPeriodBetweenPulses = micros() - cadLastTimeWeMeasured;  // Current "micros" minus the old "micros" when the last pulse happens.
                                                                  // This will result with the period (microseconds) between both pulses.
                                                                  // The way is made, the overflow of the "micros" is not going to cause any issue.

      cadLastTimeWeMeasured = micros();  // Stores the current micros so the next time we have a pulse we would have something to compare with.

      if(cadPulseCounter >= cadAmountOfReadings)  // If counter for amount of readings reach the set limit:
      {
        cadPeriodAverage = cadPeriodSum / cadAmountOfReadings;    // Calculate the final period dividing the sum of all readings by the
                                                                  // amount of readings to get the average.
        cadPulseCounter = 1;  // Reset the counter to start over. The reset value is 1 because its the minimum setting allowed (1 reading).
        cadPeriodSum = cadPeriodBetweenPulses;  // Reset PeriodSum to start a new averaging operation.
      }
    
      else
      {
        cadPulseCounter++; // Increase the counter for amount of readings by 1.
        cadPeriodSum = cadPeriodSum + cadPeriodBetweenPulses;  // Add the periods so later we can average.
      }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------

// Speed
void spdPulse() {  // The interrupt runs this to calculate the period between pulses:

    // Debounce readings
    spdDebounceCurrentMillis = millis();
    
    if (spdDebounceCurrentMillis - spdLastDebounceTime >= spdDebounceDelay) {
      spdLastDebounceTime = spdDebounceCurrentMillis;   // Update last debounce timer

      spdPeriodBetweenPulses = micros() - spdLastTimeWeMeasured;  // Current "micros" minus the old "micros" when the last pulse happens.
                                                                  // This will result with the period (microseconds) between both pulses.
                                                                  // The way is made, the overflow of the "micros" is not going to cause any issue.

      spdLastTimeWeMeasured = micros();  // Stores the current micros so the next time we have a pulse we would have something to compare with.

      if(spdPulseCounter >= spdAmountOfReadings)  // If counter for amount of readings reach the set limit:
      {
        spdPeriodAverage = spdPeriodSum / spdAmountOfReadings;    // Calculate the final period dividing the sum of all readings by the
                                                                  // amount of readings to get the average.
        spdPulseCounter = 1;  // Reset the counter to start over. The reset value is 1 because its the minimum setting allowed (1 reading).
        spdPeriodSum = spdPeriodBetweenPulses;  // Reset PeriodSum to start a new averaging operation.
      }
    
      else
      {
        spdPulseCounter++; // Increase the counter for amount of readings by 1.
        spdPeriodSum = spdPeriodSum + spdPeriodBetweenPulses;  // Add the periods so later we can average.
      }
    }
}
//--- interrupt service routine end ---


void setup() {
  #ifdef DEBUG_SERIAL
    Serial.begin(9600);   // init serial
    Serial.print("FW ver: ");
    Serial.print(fwVersion);
    Serial.println("\tStart");
    delay(50);
  #else
    // --- init LCD start ---
    display.begin();   // regular begin() using default settings and high speeed (1MHz on ESP8266, 400kHz on others)
    delay(100);        // wait LCD init
    display.setContrast(60);  // you can change the contrast around to adapt the display
                              // for the best viewing!
    display.clearDisplay();
    // --- init lcd end ---

    // --- boot logo start ---
    display.setTextSize(1);
    display.setTextColor(BLACK);
    display.setCursor(0,0);
    display.print("FW: ");
    display.print(fwVersion);
    display.display();
    delay(2000);
    display.clearDisplay();
    // --- boot logo end ---
  #endif

  // --- attach interrupts start ---
  attachInterrupt(cadInt, cadPulse, RISING);
  attachInterrupt(spdInt, spdPulse, RISING);
  // --- attach interrupts end ---
}

void loop() {

  // --- RPM start ---

  // Cadence
  
  // The following is going to store the two values that might change in the middle of the cycle.
  // We are going to do math and functions with those values and they can create glitches if they change in the
  // middle of the cycle.
  cadLastTimeCycleMeasure = cadLastTimeWeMeasured;  // Store the LastTimeWeMeasured in a variable.
  cadCurrentMicros = micros();  // Store the micros() in a variabl

  // CurrentMicros should always be higher than LastTimeWeMeasured, but in rare occasions that's not true.
  // I'm not sure why this happens, but my solution is to compare both and if CurrentMicros is lower than
  // LastTimeCycleMeasure I set it as the CurrentMicros.
  // The need of fixing this is that we later use this information to see if pulses stopped.
  if(cadCurrentMicros < cadLastTimeCycleMeasure)
  {
    cadLastTimeCycleMeasure = cadCurrentMicros;
  }

  // Calculate the frequency:
  cadFrequencyRaw = 10000000000 / cadPeriodAverage;  // Calculate the frequency using the period between pulses.

  // Detect if pulses stopped or frequency is too low, so we can show 0 Frequency:
  if(cadPeriodBetweenPulses > cadZeroTimeout || cadCurrentMicros - cadLastTimeCycleMeasure > cadZeroTimeout)
  { // If the pulses are too far apart that we reached the timeout for zero:
    cadFrequencyRaw = 0;  // Set frequency as 0.
  }

  #ifdef DEBUG_SERIAL
    cadFrequencyReal = cadFrequencyRaw / 10000;   // Get frequency without decimals.
                                                  // This is not used to calculate RPM but we remove the decimals just in case
                                                  // you want to print it.
  #endif


  // Calculate the RPM:
  cadRPM = cadFrequencyRaw / cadPulsesPerRevolution * 60;   // Frequency divided by amount of pulses per revolution multiply by
                                                            // 60 seconds to get minutes.
  cadRPM = cadRPM / 10000;  // Remove the decimals.


  // Smoothing RPM:
  cadTotal = cadTotal - cadReadings[cadReadIndex];  // Advance to the next position in the array.
  cadReadings[cadReadIndex] = cadRPM;  // Takes the value that we are going to smooth.
  cadTotal = cadTotal + cadReadings[cadReadIndex];  // Add the reading to the total.
  cadReadIndex = cadReadIndex + 1;  // Advance to the next position in the array.

  if (cadReadIndex >= cadNumReadings)  // If we're at the end of the array:
  {
    cadReadIndex = 0;  // Reset array index.
  }
  
  // Calculate the average:
  cadRPMAverage = cadTotal / cadNumReadings;  // The average value it's the smoothed result

  //--------------------------------------------------------------------------------------------------------------------------

    
  // Speed

  // The following is going to store the two values that might change in the middle of the cycle.
  // We are going to do math and functions with those values and they can create glitches if they change in the
  // middle of the cycle.
  spdLastTimeCycleMeasure = spdLastTimeWeMeasured;  // Store the LastTimeWeMeasured in a variable.
  spdCurrentMicros = micros();  // Store the micros() in a variabl

  // CurrentMicros should always be higher than LastTimeWeMeasured, but in rare occasions that's not true.
  // I'm not sure why this happens, but my solution is to compare both and if CurrentMicros is lower than
  // LastTimeCycleMeasure I set it as the CurrentMicros.
  // The need of fixing this is that we later use this information to see if pulses stopped.
  if(spdCurrentMicros < spdLastTimeCycleMeasure)
  {
    spdLastTimeCycleMeasure = spdCurrentMicros;
  }

  // Calculate the frequency:
  spdFrequencyRaw = 10000000000 / spdPeriodAverage;  // Calculate the frequency using the period between pulses.

  // Detect if pulses stopped or frequency is too low, so we can show 0 Frequency:
  if(spdPeriodBetweenPulses > spdZeroTimeout || spdCurrentMicros - spdLastTimeCycleMeasure > spdZeroTimeout)
  { // If the pulses are too far apart that we reached the timeout for zero:
    spdFrequencyRaw = 0;  // Set frequency as 0.
  }

  #ifdef DEBUG_SERIAL
    spdFrequencyReal = spdFrequencyRaw / 10000;   // Get frequency without decimals.
                                                  // This is not used to calculate RPM but we remove the decimals just in case
                                                  // you want to print it.
  #endif


  // Calculate the RPM:
  spdRPM = spdFrequencyRaw / spdPulsesPerRevolution * 60;   // Frequency divided by amount of pulses per revolution multiply by
                                                            // 60 seconds to get minutes.
  spdRPM = spdRPM / 10000;  // Remove the decimals.


  // Smoothing RPM:
  spdTotal = spdTotal - spdReadings[spdReadIndex];  // Advance to the next position in the array.
  spdReadings[spdReadIndex] = spdRPM;  // Takes the value that we are going to smooth.
  spdTotal = spdTotal + spdReadings[spdReadIndex];  // Add the reading to the total.
  spdReadIndex = spdReadIndex + 1;  // Advance to the next position in the array.

  if (spdReadIndex >= spdNumReadings)  // If we're at the end of the array:
  {
    spdReadIndex = 0;  // Reset array index.
  }
  
  // Calculate the average:
  spdRPMAverage = spdTotal / spdNumReadings;  // The average value it's the smoothed result

  // Calculate speed in Km/h
  spdKmPh = wheelDiameterCm * spdRPMAverage * 0.001885; 
                                                        // Formula: k = d × r × 0.001885
                                                        // Where,
                                                        // k = Kilometer Per Hour(km/hr)
                                                        // d = Wheel Diameter(cm)
                                                        // r = Revolution Per Minute(RPM)
                                                        // constant 0.001885

  // --- RPM end ---



  //--- update lcd start ---
  CurrentMillis = millis();

  if (CurrentMillis - lcdPreviousUpdateMillis >= lcdUpdateInterval) {
    lcdPreviousUpdateMillis = CurrentMillis;
    
    #ifdef DEBUG_SERIAL
      update_serial();
    #else
      update_lcd();
    #endif
  }
  //--- update lcd end ---

}
