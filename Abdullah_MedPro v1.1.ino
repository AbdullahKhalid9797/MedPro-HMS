/*
  ========== Change Log ==========
  - Files without a version number, are taken from the library examples of the respective sensors
  - Ver 1.0 : MAX30102 Pulse Oxymeter sensor is operational. Capturing Heart Rate and SpO2 values
  - Ver 1.1 : Added support to capture Temperature value from the sensor.
*/
#include <Wire.h>
#include "MAX30105.h" // Sparkfun Library for Pulse Oxymeter
#include "spo2_algorithm.h" // Sparkfun library for SpO2

//====== Object Creation ======
MAX30105 pulOxym; // Object creation for Pulse Oxymeter MAX30102

//--- General Variable Declaration
bool bolDebug = true;

//--- Variable Declaration for Pulse Oxymeter Sensor ---
#define RespInterval 1000 //Time Interval in ms
#define lenBuffer 100
#define SampleRate 100
#define varAvgSize 4
#define thsIRFinger 20000  // Adjust this threshold if required

uint32_t irBuffer[lenBuffer];
uint32_t redBuffer[lenBuffer];
int32_t intSpO2;
int8_t intVSpO2;
int32_t intHeartRate;
int8_t intVHR;
uint32_t intLstRep = 0;

int32_t arrBufferBPM[varAvgSize];
uint8_t intIndexBPM = 0;

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

  //====== MAX30102 Oxymeter Sensor Initialization ======
  Wire.begin(); // Initiating MAX30102 on default I2C Channel (SDA 21, SCK 22)
  if (!pulOxym.begin(Wire, I2C_SPEED_FAST)) {
    if(bolDebug){Serial.println("Pulse Oxymeter Sensor not found. Check Wiring.");}
    while (1);
  }
  // Sensor configuration
  byte irLEDBr = 30; // Sensor IR LED Brightness level (0-255)
  byte byteSmplAvg = 4;
  byte irLEDMode = 2;
  int intPulseWidth = 411;
  int intRange = 16384;

  pulOxym.setup(irLEDBr, byteSmplAvg, irLEDMode, SampleRate, intPulseWidth, intRange);
  pulOxym.setPulseAmplitudeRed(irLEDBr);
  pulOxym.setPulseAmplitudeIR(irLEDBr);
  pulOxym.setPulseAmplitudeGreen(0);
}

void loop() {

  //====== Pulse Oxymeter Data ======
  // Initial 100 samples
  for (byte i = 0; i < lenBuffer; i++) {
    while (!pulOxym.available()) {
      delay(1);
      pulOxym.check();
    }
    irBuffer[i] = pulOxym.getIR();
    redBuffer[i] = pulOxym.getRed();
    pulOxym.nextSample();
  }

  while (true) {
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
      delay(RespInterval);
      continue;
    }
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

    if (millis() - intLstRep > RespInterval) {
      int32_t intSmoBPM = fncCalcMovAvgBPM(intHeartRate);

      // Only show valid, realistic values
      if (intVHR && intVSpO2 && intSmoBPM >= 40 && intSmoBPM <= 120 && intSpO2 >= 65 && intSpO2 <= 100) {
        float fltTemp = pulOxym.readTemperature(); // Get Temperature from Oxymeter
        if(bolDebug){
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
}
