/*
  ========== Change Log ==========
  - Files without a version number are taken from the library examples of the respective sensors
  - Ver 1.0: MAX30102 Pulse Oximeter sensor operational, capturing Heart Rate and SpO2 values
  - Ver 1.1: Added support to capture Temperature value from the MAX30102 sensor
  - Ver 1.2: Added support to capture Temperature value from DS18B20 Temperature Probe
  - Ver 1.2.1: Rectified the issue with blocking while loop associated with MAX30102
  - Ver 1.3: Added support for MLX90614 Contactless IR Temperature Sensor
  - Ver 1.3.1: Fixed I2C bus to use default pins (SDA: GPIO 21, SCL: GPIO 22) and MLX90614 nan issue
  - Ver 1.3.2: Optimized MAX30102 for accurate SpO2 (90-100%) and heart rate readings
  - Ver 1.3.3: Fixed MLX90614 nan issue with I2C reset and MAX30102 finger detection with lower threshold
  - Ver 1.3.4: Fixed MAX30102 invalid HR/SpO2 data with high IR values and enhanced MLX90614 stability
  - Ver 1.4: Added button-controlled sensor navigation (DS18B20, MLX90614, MAX30102, Auto Mode), with circular navigation and Select button to start readings
  - Ver 1.4.1: Added shutdown/wakeup for MAX30102 and minimized MLX90614 I2C activity to reduce bus contention and power consumption
  - Ver 1.4.2: Fixed DS18B20 not reading on first press with reinitialization, removed repeated Wire.end()/Wire.begin() for MLX90614 stability, added I2C bus recovery
  - Ver 1.4.3: Fixed MAX30102 not stopping by adding timeout to read loop, frequent button checks, and ensuring shutdown on state change
  - Ver 1.4.4: Fixed MAX30102 invalid HR/SpO2 by initializing buffers, checking finger presence early, optimizing sensor settings
  - Ver 1.4.5: Improved MAX30102 accuracy with adjusted settings, dynamic IR threshold, increased buffer refresh, and signal filtering
  - Ver 1.4.6: Fixed MAX30102 finger detection with lower dynamic threshold, increased LED brightness, reduced debug clutter
  - Ver 1.4.7: Added finger placement prompt, ensured initial samples with finger present, adjusted sensor settings for better IR signal
  - Ver 1.4.8: Implemented user flow with clear prompts, optimized settings for signal variation, fixed serial output stability
  - Ver 1.4.9: Fixed MAX30102 saturation (IR=262143), adjusted LED brightness and ADC range, enhanced finger detection stability
  - Ver 1.4.10: Fixed min() type mismatch, further reduced LED brightness, increased pulse width, added red LED debug output
  - Ver 1.4.11: Increased LED brightness, reduced sample averaging, added dynamic LED adjustment and high-pass filter, lowered signal variation threshold
  - Ver 1.4.12: Fixed getPulseAmplitudeIR() error, added currentLEDBrightness tracking, increased LED brightness and sample rate, added finger placement guidance
  - Ver 1.4.13: Increased LED brightness to 100, lowered MIN_SIGNAL_VARIATION to 500, increased SampleRate to 400 Hz, lowered thsIRFingerMin to 5000, enhanced filters, improved finger detection
  - Ver 1.4.14: Fixed max() type mismatch, increased LED brightness to 120, increased sample averaging to 8, lowered thsIRFingerMin to 3000, strengthened low-pass filter, added calibration prompt
  - Ver 1.4.15: Fixed saturation (IR=262143) by reducing LED brightness to 60, enhanced dynamic brightness adjustment, lowered MIN_SIGNAL_VARIATION to 300, increased pulse width to 2150 µs, simplified filtering, added saturation counter
  - Ver 1.4.16: Increased LED brightness to 80, lowered MIN_SIGNAL_VARIATION to 200, increased pulse width to 4110 µs, reduced low-pass filter alpha to 0.7, added low signal prompt, tightened dynamic finger threshold
*/
#include <math.h>
#include <Wire.h>
#include "MAX30105.h" // SparkFun Library for Pulse Oximeter
#include "spo2_algorithm.h" // SparkFun library for SpO2
#include <OneWire.h> // Library for DS18B20 Temp Sensor
#include <DallasTemperature.h> // Library for DS18B20 Temp Sensor
#include <Adafruit_MLX90614.h> // Library for MLX90614 IR Temp Sensor
#include <WiFi.h>
#include <HTTPClient.h>
#include "WiFiClient.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

//====== Object Creation ======
MAX30105 pulOxym; // Object for Pulse Oximeter MAX30102
Adafruit_MLX90614 senMLX = Adafruit_MLX90614(); // Object for MLX90614
#define pinDS18B20 4  // GPIO 4 for DS18B20 Temp Sensor
OneWire oWire(pinDS18B20); // Object for DS18B20 Sensor
DallasTemperature senDS18B20(&oWire);

//====== Pin Definitions ======
#define btnNext 15 
#define btnPrevious 2
#define btnSelect 0
#define ledConnect 17
#define ledActivity 5
#define pinBuzz 13

//--- General Variable Declaration
bool bolDebug = true; // Debug output flag
enum SensorState {
  SENSOR_DS18B20,   // 1: DS18B20 Temperature
  SENSOR_MLX90614,  // 2: MLX90614 IR Temperature
  SENSOR_MAX30102,  // 3: MAX30102 Pulse Oximeter
  SENSOR_AMG8833,   // 4: Reserved for AMG8833
  SENSOR_ECG,       // 5: Reserved for ECG
  SENSOR_AUTO       // 6: Auto Mode (MAX30102 + DS18B20)
};
SensorState currentState = SENSOR_DS18B20; // Start at DS18B20
bool isReading = false; // Flag to control reading state
bool initialSamplesTaken = false; // Track if initial samples are acquired
uint32_t lastButtonCheck = 0; // Last button check time
#define btnDebounce 50 // Debounce interval in ms

//--- Pulse Oximeter Variables ---
#define RespInterval 1000 // Time interval in ms for reporting
#define lenBuffer 100
#define SampleRate 400 // For better temporal resolution
#define varAvgSize 4
#define thsIRFingerMin 3000  // For finger detection
#define thsIRFingerMax 150000 // Maximum to avoid saturation
#define MAX30102_TIMEOUT 100 // Timeout for MAX30102 data availability (ms)
#define MIN_SIGNAL_VARIATION 200 // Lowered for better detection
#define FINGER_WAIT_TIMEOUT 5000 // Timeout for finger placement (ms)
#define IR_STABILITY_THRESHOLD 10000 // For robust finger detection
#define IR_SATURATION_VALUE 262143 // Max 18-bit ADC value
#define TARGET_IR_MIN 50000 // Target IR for dynamic LED adjustment
#define TARGET_IR_MAX 100000 // Target IR for dynamic LED adjustment

uint32_t irBuffer[lenBuffer];
uint32_t redBuffer[lenBuffer];
int32_t intSpO2;
int8_t intVSpO2;
int32_t intHeartRate;
int8_t intVHR;
uint32_t intLstRep = 0;
byte currentLEDBrightness = 80; // Track current LED brightness
uint8_t saturationAttempts = 0; // Track saturation retries
uint8_t lowSignalAttempts = 0; // Track low signal retries

int32_t arrBufferBPM[varAvgSize] = {0}; // Initialized to avoid undefined behavior
uint8_t intIndexBPM = 0;

//--- DS18B20 Temperature Sensor Variables ---
#define TempPrec 9
int devDS18B20;
DeviceAddress tmpDevAdd;

//--- Timing Variables ---
uint32_t lastTempRead = 0; // Track last DS18B20 and MLX90614 read time

// I2C bus recovery function
void resetI2C() {
  if (bolDebug) {
    Serial.println("Resetting I2C bus...");
    delay(10); // Stabilize Serial
  }
  Wire.end();
  pinMode(21, OUTPUT); // SDA
  pinMode(22, OUTPUT); // SCL
  for (int i = 0; i < 9; i++) { // Send 9 clock pulses to clear stuck devices
    digitalWrite(22, HIGH);
    delayMicroseconds(5);
    digitalWrite(22, LOW);
    delayMicroseconds(5);
  }
  pinMode(21, INPUT); // Restore SDA/SCL to input for I2C
  pinMode(22, INPUT);
  Wire.begin(); // Reinitialize I2C
  delay(50); // Allow bus to stabilize
  if (bolDebug) {
    Serial.println("I2C bus reset complete.");
    delay(10); // Stabilize Serial
  }
}

// Reset MAX30102 sensor
void resetMAX30102(byte ledBrightness) {
  if (bolDebug) {
    //Serial.println("Resetting MAX30102 sensor...");
    delay(10); // Stabilize Serial
  }
  pulOxym.shutDown();
  delay(100);
  pulOxym.wakeUp();
  // Reconfigure sensor
  byte irLEDBr = ledBrightness; // Use provided brightness
  byte byteSmplAvg = 4; // For responsiveness
  byte irLEDMode = 2;
  int intPulseWidth = 4110; // Max for better signal
  int intRange = 16384; // To avoid clipping
  pulOxym.setup(irLEDBr, byteSmplAvg, irLEDMode, SampleRate, intPulseWidth, intRange);
  pulOxym.setPulseAmplitudeRed(irLEDBr);
  pulOxym.setPulseAmplitudeIR(irLEDBr);
  pulOxym.setPulseAmplitudeGreen(0);
  pulOxym.enableDIETEMPRDY();
  currentLEDBrightness = irLEDBr; // Update tracked brightness
  if (bolDebug) {
   // Serial.print("MAX30102 reset complete. LED Brightness: ");
   // Serial.println(irLEDBr);
    delay(10); // Stabilize Serial
  }
}

// Dynamic LED current adjustment
byte adjustLEDBrightness(uint32_t initialIR) {
  byte irLEDBr = currentLEDBrightness;
  if (initialIR >= IR_SATURATION_VALUE && irLEDBr > 10) {
    irLEDBr = max(10, irLEDBr - 30); // Aggressively reduce brightness
    saturationAttempts++;
  } else if (initialIR < TARGET_IR_MIN && irLEDBr < 255) {
    irLEDBr = min(255, irLEDBr + 10); // Finer increase
    saturationAttempts = 0; // Reset on non-saturated reading
  } else if (initialIR > TARGET_IR_MAX && irLEDBr > 10) {
    irLEDBr = max(10, irLEDBr - 10); // Finer decrease
    saturationAttempts = 0; // Reset on non-saturated reading
  } else {
    saturationAttempts = 0; // Reset if within target range
  }
  return irLEDBr;
}

int32_t fncCalcMovAvgBPM(int32_t newBPM) {
  static uint32_t lastReset = 0;
  if (millis() - lastReset >= 10000) { // Reset every 10 seconds
    for (uint8_t i = 0; i < varAvgSize; i++) {
      arrBufferBPM[i] = 0;
    }
    intIndexBPM = 0;
    lastReset = millis();
  }
  if (newBPM >= 40 && newBPM <= 120) {
    arrBufferBPM[intIndexBPM % varAvgSize] = newBPM;
    intIndexBPM++;
    int32_t intSum = 0;
    uint8_t intCount = 0;
    for (uint8_t i = 0; i < varAvgSize; i++) {
      if (arrBufferBPM[i] > 0) {
        intSum += arrBufferBPM[i];
        intCount++;
      }
    }
    return intCount > 0 ? intSum / intCount : 0;
  }
  return 0;
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Allow serial to stabilize
  while (!Serial);
  if (bolDebug) {
    Serial.println("Initializing sensors and buttons...");
    delay(10); // Stabilize Serial
  }

  //====== Button, LED and Buzzer Initialization ======
  pinMode(btnNext, INPUT_PULLUP);
  pinMode(btnPrevious, INPUT_PULLUP);
  pinMode(btnSelect, INPUT_PULLUP);
  pinMode(ledConnect, OUTPUT);
  pinMode(ledActivity, OUTPUT);
  pinMode(pinBuzz, OUTPUT);
  digitalWrite(ledConnect, LOW);
  digitalWrite(ledActivity, LOW);

  //====== Sensors Initialization ======
  Wire.begin(); // Initialize I2C on default pins (SDA: GPIO 21, SCL: GPIO 22)
  senDS18B20.begin(); // Initialize DS18B20 sensor
  devDS18B20 = senDS18B20.getDeviceCount();
  if (bolDebug) {
    Serial.print("Found ");
    Serial.print(devDS18B20, DEC);
    Serial.println(" DS18B20 devices.");
    delay(10); // Stabilize Serial
  }
  for (int i = 0; i < devDS18B20; i++) {
    if (senDS18B20.getAddress(tmpDevAdd, i)) {
      if (bolDebug) {
        Serial.print("Device ");
        Serial.print(i, DEC);
        Serial.print(" x DS18B20 Found. Address: ");
        prnAdd(tmpDevAdd);
        Serial.println();
        Serial.print("Precision value: ");
        Serial.println(TempPrec, DEC);
        senDS18B20.setResolution(tmpDevAdd, TempPrec);
        Serial.print("Resolution: ");
        Serial.println(senDS18B20.getResolution(tmpDevAdd), DEC);
        delay(10); // Stabilize Serial
      }
    } else {
      if (bolDebug) {
        Serial.print("Device ");
        Serial.print(i, DEC);
        Serial.println(" x Ghost device with no address.");
        delay(10); // Stabilize Serial
      }
    }
  }

  if (!senMLX.begin()) { // Initialize MLX90614 IR sensor on I2C
    if (bolDebug) {
      Serial.println("Error: MLX90614 not found. Check wiring or I2C address (expected 0x5A).");
      delay(10); // Stabilize Serial
    }
    while (1);
  }
  delay(500); // Increased delay for MLX90614 stabilization
  if (bolDebug) {
    Serial.print("MLX90614 Emissivity: ");
    Serial.println(senMLX.readEmissivity());
    delay(10); // Stabilize Serial
  }
  if (!pulOxym.begin(Wire, 400000)) { // Initialize MAX30102 at 400kHz
    if (bolDebug) {
      Serial.println("Pulse Oximeter Sensor not found. Check wiring.");
      delay(10); // Stabilize Serial
    }
    while (1);
  }

  // --- MAX30102 Configuration ---
  byte irLEDBr = 80; // Increased for better signal
  byte byteSmplAvg = 4; // For responsiveness
  byte irLEDMode = 2;
  int intPulseWidth = 4110; // Max for better signal
  int intRange = 16384; // To avoid clipping

  pulOxym.setup(irLEDBr, byteSmplAvg, irLEDMode, SampleRate, intPulseWidth, intRange);
  pulOxym.setPulseAmplitudeRed(irLEDBr);
  pulOxym.setPulseAmplitudeIR(irLEDBr);
  pulOxym.setPulseAmplitudeGreen(0);
  pulOxym.enableDIETEMPRDY();
  currentLEDBrightness = irLEDBr; // Initialize tracked brightness
  pulOxym.shutDown(); // Shut down MAX30102 immediately after setup

  // Initialize buffers to zero
  for (int i = 0; i < lenBuffer; i++) {
    irBuffer[i] = 0;
    redBuffer[i] = 0;
  }

  // Initial message
  if (bolDebug) {
    Serial.println("System ready. Press Select to start DS18B20 readings.");
    delay(10); // Stabilize Serial
  }
}

void getTemp(DeviceAddress devAdd) {
  if (bolDebug) {
    //Serial.println("Reading DS18B20..."); // disabled
    delay(10); // Stabilize Serial
  }
  senDS18B20.begin(); // Reinitialize to ensure readiness
  senDS18B20.requestTemperatures();
  float valTemp = senDS18B20.getTempC(devAdd);
  if (valTemp == DEVICE_DISCONNECTED_C) {
    if (bolDebug) {
      Serial.println("Error: Could not read DS18B20 temperature data");
      delay(10); // Stabilize Serial
    }
    return;
  }
  if (bolDebug) {
    Serial.print("DS18B20\t\t ");
    Serial.print(valTemp);
    Serial.print("oC\t\t");
    Serial.print(DallasTemperature::toFahrenheit(valTemp));
    Serial.println("oF");
    delay(10); // Stabilize Serial
  }
}

void readMLX90614() {
  if (bolDebug) {
    Serial.println("Reading MLX90614...");
    delay(10); // Stabilize Serial
  }
  Wire.beginTransmission(0x5A);
  uint8_t error = Wire.endTransmission();
  if (error != 0) {
    if (bolDebug) {
      Serial.print("MLX90614 Pre-read I2C Error Code: ");
      Serial.println(error);
      Serial.println("Attempting I2C bus reset...");
      delay(10); // Stabilize Serial
    }
    resetI2C();
    Wire.beginTransmission(0x5A);
    error = Wire.endTransmission();
    if (error != 0) {
      if (bolDebug) {
        Serial.println("MLX90614 I2C error persists after reset.");
        delay(10); // Stabilize Serial
      }
      return;
    }
  }
  float ambientTempC = NAN, objectTempC = NAN;
  for (uint8_t retry = 0; retry < 3; retry++) {
    ambientTempC = senMLX.readAmbientTempC();
    objectTempC = senMLX.readObjectTempC();
    if (!isnan(ambientTempC) && !isnan(objectTempC)) break;
    delay(50);
    if (bolDebug && retry == 2) {
      Serial.println("MLX90614 read failed after retries. Attempting I2C reset...");
      delay(10); // Stabilize Serial
      resetI2C();
    }
  }
  if (isnan(ambientTempC) || isnan(objectTempC)) {
    if (bolDebug) {
      Serial.println("Error: MLX90614 failed to read temperature after retries.");
      delay(10); // Stabilize Serial
    }
  } else {
    if (bolDebug) {
      Serial.print("MLX90614\tAmbient = ");
      Serial.print(ambientTempC);
      Serial.print("oC/");
      Serial.print(senMLX.readAmbientTempF());
      Serial.print("oF\tObject = ");
      Serial.print(objectTempC);
      Serial.print("oC/");
      Serial.print(senMLX.readObjectTempF());
      Serial.println("oF");
      delay(10); // Stabilize Serial
    }
  }
}

bool waitForFinger() {
  if (bolDebug) {
    Serial.println("Place finger to acquire samples...");
    Serial.println("Ensure firm contact with moderate pressure.");
    delay(10); // Stabilize Serial
  }
  uint32_t startTime = millis();
  uint32_t lastValidIR = 0;
  uint8_t stableCount = 0;
  byte irLEDBr = currentLEDBrightness;
  while (millis() - startTime < FINGER_WAIT_TIMEOUT) {
    uint32_t initialIR = pulOxym.getIR();
    // Check for saturation
    if (initialIR >= IR_SATURATION_VALUE) {
      if (saturationAttempts >= 3) {
        if (bolDebug) {
          Serial.println("MAX30102 repeated saturation. Check sensor or ambient light.");
          delay(10); // Stabilize Serial
        }
        return false;
      }
      if (bolDebug) {
        Serial.println("MAX30102 signal saturated. Adjusting brightness and retrying...");
        delay(10); // Stabilize Serial
      }
      irLEDBr = max(10, irLEDBr - 30); // Aggressively reduce brightness
      resetMAX30102(irLEDBr);
      delay(100); // Allow stabilization
      continue; // Retry finger detection
    }
    // Dynamic LED adjustment
    irLEDBr = adjustLEDBrightness(initialIR);
    if (irLEDBr != currentLEDBrightness) {
      resetMAX30102(irLEDBr);
      delay(100); // Allow stabilization
    }
    uint32_t thsIRFinger = thsIRFingerMin;
    if (initialIR > 100000) thsIRFinger = min(initialIR / 2, (uint32_t)(thsIRFingerMax - 5000)); // Tighter dynamic threshold
    if (initialIR >= thsIRFinger && initialIR <= thsIRFingerMax) {
      if (lastValidIR > 0 && abs((int32_t)initialIR - (int32_t)lastValidIR) < IR_STABILITY_THRESHOLD) {
        stableCount++;
        if (stableCount >= 3) { // Require 3 stable readings
          if (bolDebug) {
            Serial.println("Finger detected, wait for sample acquisition...");
            delay(10); // Stabilize Serial
          }
          return true;
        }
      }
      lastValidIR = initialIR;
    } else {
      stableCount = 0; // Reset stability count if IR is invalid
      lastValidIR = 0;
      if (initialIR > 1000 && initialIR < thsIRFingerMin) {
        lowSignalAttempts++;
        if (lowSignalAttempts >= 3 && bolDebug) {
          Serial.println("Weak signal detected. Try repositioning finger.");
          delay(10); // Stabilize Serial
          lowSignalAttempts = 0; // Reset to avoid spam
        }
      }
    }
    delay(100); // Check every 100ms
    handleButtons(); // Allow stopping/switching during wait
    if (!isReading || (currentState != SENSOR_MAX30102 && currentState != SENSOR_AUTO)) {
      if (bolDebug) {
        Serial.println("MAX30102 reading cancelled during finger wait.");
        delay(10); // Stabilize Serial
      }
      return false;
    }
  }
  if (bolDebug) {
    Serial.print("Timeout waiting for finger placement. Last IR Value: ");
    Serial.println(pulOxym.getIR());
    delay(10); // Stabilize Serial
  }
  return false;
}

void readMAX30102() {
  // Reset initial samples flag when readings start
  if (!isReading) initialSamplesTaken = false;

  // Wait for finger and acquire initial samples
  if (!initialSamplesTaken) {
    saturationAttempts = 0; // Reset saturation counter
    lowSignalAttempts = 0; // Reset low signal counter
    if (!waitForFinger()) {
      return;
    }
    // Collect 100 samples with finger present
    for (byte i = 0; i < lenBuffer; i++) {
      uint32_t startTime = millis();
      while (!pulOxym.available()) {
        if (millis() - startTime > MAX30102_TIMEOUT) {
          if (bolDebug && millis() - intLstRep >= RespInterval) {
            Serial.println("MAX30102 timeout waiting for data.");
            delay(10); // Stabilize Serial
            intLstRep = millis();
          }
          return;
        }
        pulOxym.check();
        handleButtons(); // Check buttons
        if (!isReading || (currentState != SENSOR_MAX30102 && currentState != SENSOR_AUTO)) {
          if (bolDebug && millis() - intLstRep >= RespInterval) {
            Serial.println("MAX30102 reading interrupted by button press.");
            delay(10); // Stabilize Serial
            intLstRep = millis();
          }
          return;
        }
        delay(1);
      }
      irBuffer[i] = pulOxym.getIR();
      redBuffer[i] = pulOxym.getRed();
      // Check for saturation or finger removal
      if (irBuffer[i] >= IR_SATURATION_VALUE) {
        if (saturationAttempts >= 3) {
          if (bolDebug && millis() - intLstRep >= RespInterval) {
            Serial.println("MAX30102 repeated saturation during acquisition. Check sensor or ambient light.");
            delay(10); // Stabilize Serial
            intLstRep = millis();
          }
          return;
        }
        if (bolDebug && millis() - intLstRep >= RespInterval) {
          Serial.println("MAX30102 signal saturated during acquisition. Adjusting brightness and retrying...");
          delay(10); // Stabilize Serial
          intLstRep = millis();
        }
        resetMAX30102(max(10, currentLEDBrightness - 30));
        initialSamplesTaken = false;
        return;
      }
      if (irBuffer[i] < thsIRFingerMin) {
        if (bolDebug && millis() - intLstRep >= RespInterval) {
          Serial.println("Finger removed or poor contact. Resetting buffers...");
          delay(10); // Stabilize Serial
          intLstRep = millis();
        }
        for (int j = 0; j < lenBuffer; j++) {
          irBuffer[j] = 0;
          redBuffer[j] = 0;
        }
        initialSamplesTaken = false;
        return;
      }
      pulOxym.nextSample();
    }
    if (bolDebug) {
      Serial.println("Samples acquired, Thank you");
      delay(10); // Stabilize Serial
    }
    initialSamplesTaken = true;
  }

  // Collect 50 new samples for continuous readings
  for (byte i = 50; i < lenBuffer; i++) {
    uint32_t startTime = millis();
    while (!pulOxym.available()) {
      if (millis() - startTime > MAX30102_TIMEOUT) {
        if (bolDebug && millis() - intLstRep >= RespInterval) {
          Serial.println("MAX30102 timeout waiting for data.");
          delay(10); // Stabilize Serial
          intLstRep = millis();
        }
        return;
      }
      pulOxym.check();
      handleButtons(); // Check buttons
      if (!isReading || (currentState != SENSOR_MAX30102 && currentState != SENSOR_AUTO)) {
        if (bolDebug && millis() - intLstRep >= RespInterval) {
          Serial.println("MAX30102 reading interrupted by button press.");
          delay(10); // Stabilize Serial
          intLstRep = millis();
        }
        return;
      }
      delay(1);
    }
    irBuffer[i] = pulOxym.getIR();
    redBuffer[i] = pulOxym.getRed();
    // Check for saturation or finger removal
    if (irBuffer[i] >= IR_SATURATION_VALUE) {
      if (saturationAttempts >= 3) {
        if (bolDebug && millis() - intLstRep >= RespInterval) {
          Serial.println("MAX30102 repeated saturation during reading. Check sensor or ambient light.");
          delay(10); // Stabilize Serial
          intLstRep = millis();
        }
        return;
      }
      if (bolDebug && millis() - intLstRep >= RespInterval) {
        Serial.println("MAX30102 signal saturated during reading. Adjusting brightness and retrying...");
        delay(10); // Stabilize Serial
        intLstRep = millis();
      }
      resetMAX30102(max(10, currentLEDBrightness - 30));
      initialSamplesTaken = false;
      return;
    }
    if (irBuffer[i] < thsIRFingerMin) {
      if (bolDebug && millis() - intLstRep >= RespInterval) {
        Serial.println("Finger removed or poor contact. Resetting buffers...");
        delay(10); // Stabilize Serial
        intLstRep = millis();
      }
      for (int j = 0; j < lenBuffer; j++) {
        irBuffer[j] = 0;
        redBuffer[j] = 0;
      }
      initialSamplesTaken = false;
      return;
    }
    pulOxym.nextSample();
  }

  // Shift 50 samples
  for (byte i = 50; i < lenBuffer; i++) {
    redBuffer[i - 50] = redBuffer[i];
    irBuffer[i - 50] = irBuffer[i];
  }

  // High-pass and low-pass filter
  float irAC[lenBuffer], redAC[lenBuffer];
  float irAvg = 0, redAvg = 0;
  for (int i = 0; i < lenBuffer; i++) {
    irAvg += irBuffer[i];
    redAvg += redBuffer[i];
  }
  irAvg /= lenBuffer;
  redAvg /= lenBuffer;
  // High-pass filter: Subtract DC component
  for (int i = 0; i < lenBuffer; i++) {
    irAC[i] = irBuffer[i] - irAvg;
    redAC[i] = redBuffer[i] - redAvg;
  }
  // Low-pass filter: Smooth AC signal (alpha = 0.7)
  float alpha = 0.7;
  float irFiltered[lenBuffer], redFiltered[lenBuffer];
  irFiltered[0] = irAC[0];
  redFiltered[0] = redAC[0];
  for (int i = 1; i < lenBuffer; i++) {
    irFiltered[i] = alpha * irFiltered[i-1] + (1 - alpha) * irAC[i];
    redFiltered[i] = alpha * redFiltered[i-1] + (1 - alpha) * redAC[i];
  }
  // Restore DC component
  for (int i = 0; i < lenBuffer; i++) {
    irBuffer[i] = (uint32_t)max(0.0f, irFiltered[i] + irAvg);
    redBuffer[i] = (uint32_t)max(0.0f, redFiltered[i] + redAvg);
  }

  // Verify buffer signal quality
  uint32_t irMin = UINT32_MAX, irMax = 0;
  uint32_t redMin = UINT32_MAX, redMax = 0;
  for (int i = 0; i < lenBuffer; i++) {
    if (irBuffer[i] > 0) {
      if (irBuffer[i] < irMin) irMin = irBuffer[i];
      if (irBuffer[i] > irMax) irMax = irBuffer[i];
    }
    if (redBuffer[i] > 0) {
      if (redBuffer[i] < redMin) redMin = redBuffer[i];
      if (redBuffer[i] > redMax) redMax = redBuffer[i];
    }
  }
  if (irMax - irMin < MIN_SIGNAL_VARIATION) {
    lowSignalAttempts++;
    if (bolDebug && millis() - intLstRep >= RespInterval) {
      //Serial.print("MAX30102 signal too weak or noisy. IR Min: ");
      //Serial.print(irMin);
      //Serial.print(", IR Max: ");
      //Serial.print(irMax);
      //Serial.print(", Red Min: ");
      //Serial.print(redMin);
      //Serial.print(", Red Max: ");
      //Serial.println(redMax);
      //Serial.println("Try adjusting finger pressure or position.");
      if (lowSignalAttempts >= 3) {
        //Serial.println("Signal too weak after retries. Increase brightness or check sensor.");
        resetMAX30102(min(255, currentLEDBrightness + 10));
        lowSignalAttempts = 0; // Reset attempts
      }
      delay(10); // Stabilize Serial
      intLstRep = millis();
    }
    return;
  }

  // Basic signal filtering
  for (int i = 0; i < lenBuffer; i++) {
    if (irBuffer[i] < thsIRFingerMin || irBuffer[i] > thsIRFingerMax) {
      irBuffer[i] = 0;
      redBuffer[i] = 0;
    }
  }

  // Calculate HR and SpO2
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, lenBuffer,
    redBuffer,
    &intSpO2, &intVSpO2,
    &intHeartRate, &intVHR
  );

  if (millis() - intLstRep >= RespInterval) {
    int32_t intSmoBPM = fncCalcMovAvgBPM(intHeartRate);
    if (intVHR && intVSpO2 && intSmoBPM >= 50 && intSmoBPM <= 130 && intSpO2 >= 85 && intSpO2 <= 100) {
      float fltTemp = pulOxym.readTemperature();
      if (bolDebug) {

        Serial.print("MAX30102\tHeart Rate: ");
        Serial.print(intSmoBPM);
        Serial.print(" bpm\tSpO₂: ");
        Serial.print(intSpO2);
        Serial.print(" %\tTemp: ");
        Serial.print(fltTemp, 1);
        Serial.println(" °C");
//        Serial.print("MAX30102 IR Value: ");
//        Serial.print(irBuffer[lenBuffer - 1]);
//        Serial.print(", Red Value: ");
//       Serial.println(redBuffer[lenBuffer - 1]);
        delay(10); // Stabilize Serial
      }
      lowSignalAttempts = 0; // Reset on valid reading
    } else if (bolDebug) {
      Serial.print("MAX30102\tInvalid HR or SpO2 data. Raw HR: ");
      Serial.print(intHeartRate);
      Serial.print(" bpm, Raw SpO2: ");
      Serial.print(intSpO2);
      Serial.print(" %, Valid HR: ");
      Serial.print(intVHR);
      Serial.print(", Valid SpO2: ");
      Serial.print(intVSpO2);
      Serial.print(", IR Value: ");
      Serial.print(irBuffer[lenBuffer - 1]);
      Serial.print(", Red Value: ");
      Serial.println(redBuffer[lenBuffer - 1]);
      delay(10); // Stabilize Serial
    }
    intLstRep = millis();
  }
}

void readAutoMode() {
  if (bolDebug) {
    Serial.println("Reading Auto Mode...");
    delay(10); // Stabilize Serial
  }
  // Read DS18B20
  senDS18B20.begin(); // Reinitialize to ensure readiness
  senDS18B20.requestTemperatures();
  for (int i = 0; i < devDS18B20; i++) {
    if (senDS18B20.getAddress(tmpDevAdd, i)) {
      getTemp(tmpDevAdd);
    }
  }
  // Read MAX30102
  readMAX30102();
}

void manageSensorPower() {
  // Shut down MAX30102 by default
  pulOxym.shutDown();
  if (bolDebug) {
    Serial.println("MAX30102 powered off.");
    delay(10); // Stabilize Serial
  }

  // Wake up only the required sensor(s) based on state
  if (currentState == SENSOR_MAX30102 || currentState == SENSOR_AUTO) {
    pulOxym.wakeUp();
    delay(100); // Allow signal stabilization
    initialSamplesTaken = false; // Reset for new session
    if (bolDebug) {
      Serial.println("MAX30102 powered on.");
      delay(10); // Stabilize Serial
    }
  }
  // DS18B20 and MLX90614 do not require explicit power management
  if (currentState == SENSOR_DS18B20) {
    if (bolDebug) {
      Serial.println("DS18B20 active.");
      delay(10); // Stabilize Serial
    }
  }
  if (currentState == SENSOR_MLX90614) {
    if (bolDebug) {
      Serial.println("MLX90614 active.");
      delay(10); // Stabilize Serial
    }
  }
}

void handleButtons() {
  if (millis() - lastButtonCheck < btnDebounce) return;
  lastButtonCheck = millis();

  if (digitalRead(btnNext) == LOW) {
    isReading = false; // Stop readings when switching
    initialSamplesTaken = false; // Reset for new session
    currentState = (SensorState)((currentState + 1) % 6); // Circular navigation
    if (bolDebug) {
      Serial.print("Switched to state: ");
      Serial.println(currentState);
      delay(10); // Stabilize Serial
    }
    manageSensorPower(); // Manage sensor power state
    delay(200); // Debounce
  }
  if (digitalRead(btnPrevious) == LOW) {
    isReading = false; // Stop readings when switching
    initialSamplesTaken = false; // Reset for new session
    if (currentState == SENSOR_DS18B20) {
      currentState = SENSOR_AUTO; // Loop back to Auto Mode
    } else {
      currentState = (SensorState)(currentState - 1);
    }
    if (bolDebug) {
      Serial.print("Switched to state: ");
      Serial.println(currentState);
      delay(10); // Stabilize Serial
    }
    manageSensorPower(); // Manage sensor power state
    delay(200); // Debounce
  }
  if (digitalRead(btnSelect) == LOW) {
    isReading = !isReading; // Toggle reading state
    initialSamplesTaken = false; // Reset for new session
    if (bolDebug) {
      Serial.print("Readings ");
      Serial.println(isReading ? "started" : "stopped");
      delay(10); // Stabilize Serial
    }
    if (!isReading) {
      pulOxym.shutDown(); // Shut down MAX30102 when stopping
      if (bolDebug) {
        Serial.println("All sensors stopped.");
        delay(10); // Stabilize Serial
      }
    } else {
      manageSensorPower(); // Wake up required sensor
    }
    delay(200); // Debounce
  }
}

void loop() {
  handleButtons();

  if (!isReading) return; // Skip sensor readings if not started

  switch (currentState) {
    case SENSOR_DS18B20:
      if (millis() - lastTempRead >= RespInterval) {
        senDS18B20.begin(); // Reinitialize to ensure readiness
        senDS18B20.requestTemperatures();
        for (int i = 0; i < devDS18B20; i++) {
          if (senDS18B20.getAddress(tmpDevAdd, i)) {
            getTemp(tmpDevAdd);
          }
        }
        lastTempRead = millis();
      }
      break;
    case SENSOR_MLX90614:
      if (millis() - lastTempRead >= RespInterval) {
        readMLX90614();
        lastTempRead = millis();
      }
      break;
    case SENSOR_MAX30102:
      readMAX30102();
      break;
    case SENSOR_AUTO:
      if (millis() - lastTempRead >= RespInterval) {
        readAutoMode();
        lastTempRead = millis();
      }
      break;
    default:
      if (bolDebug) {
        Serial.println("Sensor not implemented yet.");
        delay(10); // Stabilize Serial
      }
      break;
  }
}

void prnAdd(DeviceAddress devAdd) {
  for (uint8_t i = 0; i < 8; i++) {
    if (devAdd[i] < 16) {
      if (bolDebug) Serial.print("0");
    }
    if (bolDebug) Serial.print(devAdd[i], HEX);
  }
}
