/*
  ========== Change Log ==========
  - Files without a version number are taken from the library examples of the respective sensors
  - Ver 1.0: MAX30102 Pulse Oximeter sensor operational, capturing Heart Rate and SpO2 values
  - Ver 1.1: Added support to capture Temperature value from the MAX30102 sensor
  - Ver 1.2: Added support to capture Temperature value from DS18B20 Temperature Probe
  - Ver 1.2.1 Rectified the issue with blocking while loop associated with MAX30102 which was restricting the main loop
*/
#include <Wire.h>
#include "MAX30105.h" // SparkFun Library for Pulse Oximeter
#include "spo2_algorithm.h" // SparkFun library for SpO2
#include <OneWire.h> // Library for DS18B20 Temp Sensor
#include <DallasTemperature.h> // Library for DS18B20 Temp Sensor

//====== Object Creation ======
MAX30105 pulOxym; // Object for Pulse Oximeter MAX30102
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
#define thsIRFinger 20000  // Adjust threshold if required

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
uint32_t lastTempRead = 0; // Track last DS18B20 read time

int32_t fncCalcMovAvgBPM(int32_t newBPM) {
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
  //====== Sensors Initialization ======
  Wire.begin(); // Initialize MAX30102 on default I2C Channel (SDA 21, SCL 22)
  senDS18B20.begin(); // Initialize DS18B20 sensor

  // --- MAX30102 ---
  if (!pulOxym.begin(Wire, 400000)) { // Use 400kHz I2C speed
    if (bolDebug) {
      Serial.println("Pulse Oximeter Sensor not found. Check Wiring.");
    }
    while (1);
  }
  // Sensor configuration
  byte irLEDBr = 30; // IR LED Brightness level (0-255)
  byte byteSmplAvg = 4;
  byte irLEDMode = 2;
  int intPulseWidth = 411;
  int intRange = 16384;

  pulOxym.setup(irLEDBr, byteSmplAvg, irLEDMode, SampleRate, intPulseWidth, intRange);
  pulOxym.setPulseAmplitudeRed(irLEDBr);
  pulOxym.setPulseAmplitudeIR(irLEDBr);
  pulOxym.setPulseAmplitudeGreen(0);

  // --- DS18B20 ---
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
  for (byte i = 0; i < lenBuffer; i++) {
    while (!pulOxym.available()) {
      delay(1);
      pulOxym.check();
    }
    irBuffer[i] = pulOxym.getIR();
    redBuffer[i] = pulOxym.getRed();
    pulOxym.nextSample();
  }
}

void getTemp(DeviceAddress devAdd) {
  float valTemp = senDS18B20.getTempC(devAdd);
  if (valTemp == DEVICE_DISCONNECTED_C) {
    if (bolDebug) {
      Serial.println("Error: Could not read temperature data");
    }
    return;
  }
  if (bolDebug) {
    Serial.print("DS18B20 Temperature oC: ");
    Serial.print(valTemp);
    Serial.print(" :: oF: ");
    Serial.println(DallasTemperature::toFahrenheit(valTemp));
  }
}

void loop() {
  //===== DS18B20 Temperature Data ======
  if (millis() - lastTempRead >= RespInterval) { // Read every 1000ms
    senDS18B20.requestTemperatures();
    for (int i = 0; i < devDS18B20; i++) {
      if (senDS18B20.getAddress(tmpDevAdd, i)) {
        getTemp(tmpDevAdd);
      }
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
  for (byte i = 75; i < lenBuffer; i++) {
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
    if (intVHR && intVSpO2 && intSmoBPM >= 40 && intSmoBPM <= 120 && intSpO2 >= 65 && intSpO2 <= 100) {
      float fltTemp = pulOxym.readTemperature(); // Get Temperature from Oximeter
      if (bolDebug) {
        Serial.print("Heart Rate: ");
        Serial.print(intSmoBPM);
        Serial.print(" bpm | SpO₂: ");
        Serial.print(intSpO2);
        Serial.print(" % | Temp: ");
        Serial.print(fltTemp, 1);
        Serial.println(" °C");
      }
    }
    intLstRep = millis();
  }
}

void prnAdd(DeviceAddress devAdd) {
  for (uint8_t i = 0; i < 8; i++) {
    if (devAdd[i] < 16) {
      if (bolDebug) {
        Serial.print("0");
      }
    }
    if (bolDebug) {
      Serial.print(devAdd[i], HEX);
    }
  }
}
