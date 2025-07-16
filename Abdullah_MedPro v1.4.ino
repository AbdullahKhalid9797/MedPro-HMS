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

//====== Pin Definitions ======
#define BUTTON_NEXT 15    // IO15 for Next button
#define BUTTON_PREV 2     // IO2 for Previous button
#define BUTTON_SELECT 0   // IO0 for Select button

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
uint32_t lastButtonCheck = 0; // Last button check time
#define BUTTON_DEBOUNCE 50 // Debounce interval in ms

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
  if (bolDebug) Serial.println("Initializing sensors and buttons...");

  //====== Button Initialization ======
  pinMode(BUTTON_NEXT, INPUT_PULLUP);
  pinMode(BUTTON_PREV, INPUT_PULLUP);
  pinMode(BUTTON_SELECT, INPUT_PULLUP);

  //====== Sensors Initialization ======
  Wire.begin(); // Initialize I2C on default pins (SDA: GPIO 21, SCL: GPIO 22)
  senDS18B20.begin(); // Initialize DS18B20 sensor
  if (!senMLX.begin()) { // Initialize MLX90614 IR sensor on I2C
    if (bolDebug) Serial.println("Error: MLX90614 not found. Check wiring or I2C address (expected 0x5A).");
    while (1);
  }
  delay(500); // Increased delay

  if (bolDebug) {
    Serial.print("MLX90614 Emissivity: ");
    Serial.println(senMLX.readEmissivity());
  }
  if (!pulOxym.begin(Wire, 400000)) { // Initialize MAX30102 at 400kHz
    if (bolDebug) Serial.println("Pulse Oximeter Sensor not found. Check wiring.");
    while (1);
  }

  // --- MAX30102 Configuration ---
  byte irLEDBr = 70;
  byte byteSmplAvg = 8;
  byte irLEDMode = 2;
  int intPulseWidth = 1600;
  int intRange = 16384;

  pulOxym.setup(irLEDBr, byteSmplAvg, irLEDMode, SampleRate, intPulseWidth, intRange);
  pulOxym.setPulseAmplitudeRed(irLEDBr);
  pulOxym.setPulseAmplitudeIR(irLEDBr);
  pulOxym.setPulseAmplitudeGreen(0);
  pulOxym.enableDIETEMPRDY();

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

  // Initial message
  if (bolDebug) Serial.println("System ready. Press Select to start DS18B20 readings.");
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

void readMLX90614() {
  pulOxym.shutDown(); // Pause MAX30102 to reduce I2C traffic
  Wire.end(); // Reset I2C bus
  Wire.begin(); // Reinitialize I2C bus
  delay(50);
  Wire.beginTransmission(0x5A);
  uint8_t error = Wire.endTransmission();
  if (error != 0) {
    if (bolDebug) {
      Serial.print("MLX90614 Pre-read I2C Error Code: ");
      Serial.println(error);
    }
    pulOxym.wakeUp();
    return;
  }
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
    if (bolDebug) Serial.println("Error: MLX90614 failed to read temperature after retries.");
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
    }
  }
  pulOxym.wakeUp();
  delay(10);
}

void readMAX30102() {
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
      Serial.print("MAX30102 IR Value: ");
      Serial.println(irBuffer[lenBuffer - 1]);
      intLstRep = millis();
    }
    return;
  }

  // Cap buffer values
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
      Serial.println(irBuffer[lenBuffer - 1]);
    }
    intLstRep = millis();
  }
}

void readAutoMode() {
  // Read DS18B20
  senDS18B20.requestTemperatures();
  for (int i = 0; i < devDS18B20; i++) {
    if (senDS18B20.getAddress(tmpDevAdd, i)) {
      getTemp(tmpDevAdd);
    }
  }
  // Read MAX30102
  readMAX30102();
}

void handleButtons() {
  if (millis() - lastButtonCheck < BUTTON_DEBOUNCE) return;
  lastButtonCheck = millis();

  if (digitalRead(BUTTON_NEXT) == LOW) {
    isReading = false; // Stop readings when switching
    currentState = (SensorState)((currentState + 1) % 6); // Circular navigation
    if (bolDebug) {
      Serial.print("Switched to state: ");
      Serial.println(currentState);
    }
    delay(200); // Debounce
  }
  if (digitalRead(BUTTON_PREV) == LOW) {
    isReading = false; // Stop readings when switching
    if (currentState == SENSOR_DS18B20) {
      currentState = SENSOR_AUTO; // Loop back to Auto Mode
    } else {
      currentState = (SensorState)(currentState - 1);
    }
    if (bolDebug) {
      Serial.print("Switched to state: ");
      Serial.println(currentState);
    }
    delay(200); // Debounce
  }
  if (digitalRead(BUTTON_SELECT) == LOW) {
    isReading = !isReading; // Toggle reading state
    if (bolDebug) {
      Serial.print("Readings ");
      Serial.println(isReading ? "started" : "stopped");
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
        pulOxym.shutDown(); // Disable MAX30102
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
        pulOxym.shutDown(); // Disable MAX30102
        readMLX90614();
        lastTempRead = millis();
      }
      break;
    case SENSOR_MAX30102:
      pulOxym.wakeUp(); // Enable MAX30102
      readMAX30102();
      break;
    case SENSOR_AUTO:
      pulOxym.wakeUp(); // Enable MAX30102
      if (millis() - lastTempRead >= RespInterval) {
        readAutoMode();
        lastTempRead = millis();
      }
      break;
    default:
      if (bolDebug) Serial.println("Sensor not implemented yet.");
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
