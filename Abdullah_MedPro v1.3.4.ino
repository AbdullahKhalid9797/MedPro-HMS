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
*/
#include <math.h>
#include <Wire.h>
#include "MAX30105.h" // SparkFun Library for Pulse Oximeter
#include "spo2_algorithm.h" // SparkFun library for SpO2
#include <OneWire.h> // Library for DS18B20 Temp Sensor
#include <DallasTemperature.h> // Library for DS18B20 Temp Sensor
#include <Adafruit_MLX90614.h> // Library for MLX90614 IR Temp Sensor

//====== Object Creation ======
MAX30105 pulOxym; // Object for Pulse Oximeter MAX30102
Adafruit_MLX90614 senMLX = Adafruit_MLX90614(); // Object for MLX90614
#define pinDS18B20 4  // GPIO 4 for DS18B20 Temp Sensor
OneWire oWire(pinDS18B20); // Object for DS18B20 Sensor
DallasTemperature senDS18B20(&oWire);

//--- General Variable Declaration
bool bolDebug = true; // Initialized for debug output

//--- Pulse Oximeter Variables ---
#define RespInterval 1000 // Time interval in ms for reporting
#define lenBuffer 100
#define SampleRate 100
#define varAvgSize 4
#define thsIRFinger 30000  // Lowered to detect finger with weaker signals

uint32_t irBuffer[lenBuffer];
uint32_t redBuffer[lenBuffer];
int32_t intSpO2;
int8_t intVSpO2;
int32_t intHeartRate;
int8_t intVHR;
uint32_t intLstRep = 0;

int32_t arrBufferBPM[varAvgSize] = {0}; // Initialized to avoid undefined behavior
uint8_t intIndexBPM = 0;

//--- DS18B20 Temperature Sensor Variables ---
#define TempPrec 9
int devDS18B20;
DeviceAddress tmpDevAdd;

//--- Timing Variables ---
uint32_t lastTempRead = 0; // Track last DS18B20 and MLX90614 read time

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
  while (!Serial);
  if (bolDebug) Serial.println("Initializing sensors...");

  //====== Sensors Initialization ======
  Wire.begin(); // Initialize I2C on default pins (SDA: GPIO 21, SCL: GPIO 22)
  senDS18B20.begin(); // Initialize DS18B20 sensor
  if (!senMLX.begin()) { // Initialize MLX90614 IR sensor on I2C
    if (bolDebug) Serial.println("Error: MLX90614 not found. Check wiring or I2C address (expected 0x5A).");
    while (1);
  }
  delay(500); // Increased delay for MLX90614 stabilization
  if (bolDebug) {
    Serial.print("MLX90614 Emissivity: ");
    Serial.println(senMLX.readEmissivity());
  }
  if (!pulOxym.begin(Wire, 400000)) { // Initialize MAX30102 at 400kHz
    if (bolDebug) Serial.println("Pulse Oximeter Sensor not found. Check wiring.");
    while (1);
  }

  // --- MAX30102 Configuration ---
  byte irLEDBr = 70; // Further increased IR LED brightness
  byte byteSmplAvg = 8; // Keep sample averaging
  byte irLEDMode = 2; // Red + IR mode
  int intPulseWidth = 1600; // Wider pulse width for stronger signal
  int intRange = 16384; // Keep ADC range

  pulOxym.setup(irLEDBr, byteSmplAvg, irLEDMode, SampleRate, intPulseWidth, intRange);
  pulOxym.setPulseAmplitudeRed(irLEDBr);
  pulOxym.setPulseAmplitudeIR(irLEDBr);
  pulOxym.setPulseAmplitudeGreen(0);
  pulOxym.enableDIETEMPRDY(); // Enable temperature interrupt to reduce power

  // --- DS18B20 Configuration ---
  devDS18B20 = senDS18B20.getDeviceCount();
  if (bolDebug) {
    Serial.print("Found ");
    Serial.print(devDS18B20, DEC);
    Serial.println(" DS18B20 devices.");
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
      }
    } else {
      if (bolDebug) {
        Serial.print("Device ");
        Serial.print(i, DEC);
        Serial.println(" x Ghost device with no address.");
      }
    }
  }

  // Initial 100 samples for MAX30102
  if (bolDebug) Serial.println("Collecting initial 100 samples for MAX30102...");
  for (byte i = 0; i < lenBuffer; i++) {
    while (!pulOxym.available()) {
      delay(1);
      pulOxym.check();
    }
    irBuffer[i] = pulOxym.getIR();
    redBuffer[i] = pulOxym.getRed();
    pulOxym.nextSample();
  }
  if (bolDebug) Serial.println("MAX30102 initial samples collected.");
}

void getTemp(DeviceAddress devAdd) {
  float valTemp = senDS18B20.getTempC(devAdd);
  if (valTemp == DEVICE_DISCONNECTED_C) {
    if (bolDebug) Serial.println("Error: Could not read DS18B20 temperature data");
    return;
  }
  if (bolDebug) {
    Serial.print("DS18B20\t\t ");
    Serial.print(valTemp);
    Serial.print("oC\t\t");
    Serial.print(DallasTemperature::toFahrenheit(valTemp));
    Serial.println("oF");
  }
}

void loop() {
  //===== DS18B20 and MLX90614 Temperature Data ======
  if (millis() - lastTempRead >= RespInterval) { // Read every 1000ms
    // DS18B20
    senDS18B20.requestTemperatures();
    for (int i = 0; i < devDS18B20; i++) {
      if (senDS18B20.getAddress(tmpDevAdd, i)) {
        getTemp(tmpDevAdd);
      }
    }
    // MLX90614 with retry logic
    if (bolDebug) {
      pulOxym.shutDown(); // Pause MAX30102 to reduce I2C traffic
      Wire.end(); // Reset I2C bus
      Wire.begin(); // Reinitialize I2C bus
      delay(50); // Further increased delay for stability
      // Pre-read check
      Wire.beginTransmission(0x5A);
      uint8_t error = Wire.endTransmission();
      if (error != 0) {
        Serial.print("MLX90614 Pre-read I2C Error Code: ");
        Serial.println(error);
      } else {
        float ambientTempC = NAN, objectTempC = NAN;
        for (uint8_t retry = 0; retry < 3; retry++) {
          ambientTempC = senMLX.readAmbientTempC();
          objectTempC = senMLX.readObjectTempC();
          if (!isnan(ambientTempC) && !isnan(objectTempC)) break;
          delay(50);
          if (bolDebug && retry == 2) {
            Serial.print("MLX90614 Read I2C Error Code: ");
            Serial.println(Wire.endTransmission());
          }
        }
        if (isnan(ambientTempC) || isnan(objectTempC)) {
          Serial.println("Error: MLX90614 failed to read temperature after retries.");
        } else {
          Serial.print("MLX90614\tAmbient = ");
          Serial.print(ambientTempC);
          Serial.print("oC/");
          Serial.print(senMLX.readAmbientTempF());
          Serial.print("oF\tObject = ");
          Serial.print(objectTempC);
          Serial.print("oC/");
          Serial.print(senMLX.readObjectTempF());
          Serial.println("oF");
        }
      }
      pulOxym.wakeUp(); // Resume MAX30102
      delay(10); // Allow sensor to stabilize
    }
    lastTempRead = millis();
  }

  //===== Pulse Oximeter Data ======
  // Shift 75 samples
  for (byte i = 25; i < lenBuffer; i++) {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25] = irBuffer[i];
  }

  // Add 25 new samples
  for (byte i = 75; i < lenBuffer; i++) { // Collect 25 samples
    while (!pulOxym.available()) {
      delay(1);
      pulOxym.check();
    }
    irBuffer[i] = pulOxym.getIR();
    redBuffer[i] = pulOxym.getRed();
    pulOxym.nextSample();
  }

  // Check for finger
  if (irBuffer[lenBuffer - 1] < thsIRFinger) {
    if (bolDebug && millis() - intLstRep >= RespInterval) {
      Serial.println("No finger detected.");
      Serial.print("MAX30102 IR Value: ");
      Serial.println(irBuffer[lenBuffer - 1]);
      intLstRep = millis();
    }
    return; // Allow loop() to iterate
  }

  // Cap buffer values to avoid overflow
  for (int i = 0; i < lenBuffer; i++) {
    if (irBuffer[i] > 200000) irBuffer[i] = 200000;
    if (redBuffer[i] > 200000) redBuffer[i] = 200000;
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

    // Only show valid, realistic values
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
        Serial.print("MAX30102 IR Value: ");
        Serial.println(irBuffer[lenBuffer - 1]);
      }
    } else if (bolDebug && millis() - intLstRep >= RespInterval) {
      Serial.print("MAX30102\tInvalid HR or SpO2 data. Raw HR: ");
      Serial.print(intHeartRate);
      Serial.print(" bpm, Raw SpO2: ");
      Serial.print(intSpO2);
      Serial.print(" %, Valid HR: ");
      Serial.print(intVHR);
      Serial.print(", Valid SpO2: ");
      Serial.print(intVSpO2);
      Serial.print(", IR Value: ");
      Serial.println(irBuffer[lenBuffer - 1]);
    }
    intLstRep = millis();
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
