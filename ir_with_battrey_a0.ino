#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>

// ===== Hardware Pins =====
const int ADC_PIN = 1;
const int BattreyPin = 0;       // Hardware Analog Pin 0 for Battery Sense
const uint16_t IR_LED_PIN = 4;  // IR LED Output Pin

// ===== NTC Parameters =====
const float BALANCE_RESISTOR = 2200.0;
const float ADC_MAX = 4095.0;
const float ROOM_RESISTANCE = 10000.0;
const float ROOM_TEMP_KELVIN = 298.15;
const float BETA_COEFFICIENT = 3950.0;

// ===== AC Control Settings =====
volatile float T_set = 26.0;             
volatile float delta = 1.5;              
bool isAcOn = false;                     

// ===== Harsh Temperature Correction (Th_corr) =====
float Th_corr = 0.0;
float T_baseline = -999.0;
unsigned long baselineTime = 0;

// ===== RAW IR TIMING ARRAYS =====
const uint16_t AC_ON_RAW[] = {4405, 4344, 573, 1601, 572, 517, 573, 1604, 573, 1601, 573, 516, 573,
 517, 572, 1601, 573, 518, 573, 519, 573, 1605, 572, 517, 573, 516, 573, 1601, 573, 1600, 573,
 517, 572, 1607, 573, 518, 573, 517, 572, 1605, 573, 1601, 573, 1601, 573, 1600, 574, 1600, 573,
 1603, 573, 1606, 574, 1600, 573, 517, 573, 516, 573, 516, 573, 517, 573, 516, 573, 518, 573,
 1607, 573, 517, 574, 1605, 573, 1601, 573, 517, 572, 517, 573, 516, 573, 519, 572, 519, 573,
 1600, 573, 517, 573, 516, 573, 1601, 573, 1601, 573, 1601, 573, 1602, 581, 5194, 4416, 4345,
 573, 1601, 573, 516, 574, 1604, 574, 1600, 573, 516, 574, 516, 573, 1600, 574, 518, 573, 518,
 574, 1604, 574, 515, 574, 516, 573, 1600, 574, 1600, 574, 515, 574, 1606, 574, 517, 574, 516,
 574, 1604, 573, 1600, 574, 1600, 574, 1599, 574, 1599, 575, 1601, 574, 1606, 574, 1600, 574,
 516, 574, 515, 574, 516, 573, 516, 574, 515, 574, 518, 573, 1606, 574, 516, 575, 1605, 574,
 1600, 574, 515, 574, 516, 574, 515, 574, 518, 573, 518, 574, 1600, 574, 516, 573, 516, 574,
 1600, 574, 1600, 574, 1600, 574, 1604, 582};

const uint16_t AC_OFF_RAW[] = {4419, 4344, 578, 1596, 578, 511, 579, 1599, 578, 1595, 579, 511, 
578, 511, 578, 1596, 578, 513, 578, 513, 578, 1600, 578, 511, 578, 512, 578, 1595, 578, 1596,
 578, 511, 578, 1605, 575, 515, 579, 512, 579, 1600, 578, 1595, 579, 1595, 579, 1595, 578, 1596,
 578, 1597, 579, 1601, 578, 1596, 579, 513, 579, 512, 578, 511, 579, 511, 578, 511, 578, 513,
 578, 1602, 578, 1596, 578, 1595, 578, 512, 578, 511, 578, 1595, 578, 512, 578, 513, 578, 513,
 578, 511, 578, 512, 578, 1595, 578, 1596, 578, 511, 578, 1599, 578, 1596, 588, 5187, 4420, 
 4339, 578, 1596, 578, 511, 578, 1599, 579, 1595, 578, 511, 578, 511, 578, 1595, 578, 513, 578,
 513, 578, 1601, 579, 512, 579, 512, 578, 1595, 579, 1595, 578, 511, 579, 1601, 578, 513, 579,
 511, 578, 1599, 578, 1595, 579, 1595, 578, 1595, 578, 1595, 579, 1597, 579, 1600, 579, 1595,
 578, 511, 578, 511, 578, 511, 579, 511, 578, 511, 578, 513, 578, 1602, 578, 1595, 579, 1595,
 579, 511, 578, 511, 578, 1595, 579, 511, 578, 513, 579, 512, 579, 511, 578, 511, 578, 1596,
 578, 1595, 578, 511, 579, 1598, 579, 1598, 587};

const uint16_t AC_ON_LEN  = sizeof(AC_ON_RAW) / sizeof(AC_ON_RAW[0]);
const uint16_t AC_OFF_LEN = sizeof(AC_OFF_RAW) / sizeof(AC_OFF_RAW[0]);
const uint16_t IR_FREQUENCY = 38; 

// ===== Timing & Averaging Variables =====
const int MAX_SAMPLES = 6;
float tempSamples[MAX_SAMPLES];
int sampleIndex = 0;
int totalSamplesCollected = 0;

unsigned long lastSampleTime = 0;
const unsigned long SAMPLE_INTERVAL = 10000; 

unsigned long lastTerminalTime = 0;
const unsigned long TERMINAL_INTERVAL = 1000; 

// ===== Global Battery Metric =====
int currentBatteryPercentage = 100;

// ===== BLE UART UUIDs =====
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" 
#define RX_CHARACTERISTIC_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" 
#define TX_CHARACTERISTIC_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E" 

BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
IRsend irsend(IR_LED_PIN);
float T_current = 0.0; 

// Your custom working NTC math function
float readTemperature() {
  int adcValue = analogRead(ADC_PIN);
  if (adcValue <= 0 || adcValue >= ADC_MAX) return -999;

  float ntcResistance = BALANCE_RESISTOR * ((ADC_MAX - (float)adcValue) / (float)adcValue);
  float temperatureKelvin = 1.0 / ((1.0 / ROOM_TEMP_KELVIN) +
                            (1.0 / BETA_COEFFICIENT) * log(ntcResistance / ROOM_RESISTANCE));

  return temperatureKelvin - 279.15;
}

// ===== Non-Linear Battery Approximation Curve Function =====
int readBatteryPercentage() {
  long adcSum = 0;
  
  // Multi-sample to clear noise from high-impedance (100k) divider paths
  for (int i = 0; i < 11; i++) {
    adcSum += analogRead(BattreyPin);
    delayMicroseconds(20);
  }
  float avgAdc = (float)adcSum / 11.0;

  // Convert ADC reading to the absolute voltage at the pin
  // ESP32-C3 defaults to ~3.3V full scale range at 12-bit (4095) resolution
  float pinVoltage = (avgAdc / 4095.0) * 3.3;

  // Re-calculate the original raw battery voltage before the 1:1 (100k/100k) divider
  float batteryVoltage = pinVoltage * 2.0;

  int percentage = 0;

  // Mapping the characteristic 3-stage discharge plateau curve of a 3.7V Li-ion cell
  if (batteryVoltage >= 4.15) {
    percentage = 100;
  } else if (batteryVoltage >= 3.85) {
    // Upper tier: Curve rolls off softly from 100% to 80%
    percentage = 80 + (batteryVoltage - 3.85) * (20.0 / (4.15 - 3.85));
  } else if (batteryVoltage >= 3.65) {
    // Flat nominal plateau: Most of your usage runtime happens here (80% down to 20%)
    percentage = 20 + (batteryVoltage - 3.65) * (60.0 / (3.85 - 3.65));
  } else if (batteryVoltage >= 3.20) {
    // Voltage Cliff: Rapid drop down to safety cut-off limits (20% down to 0%)
    percentage = 0 + (batteryVoltage - 3.20) * (20.0 / (3.65 - 3.20));
  } else {
    percentage = 0; // Depleted or protective shutdown state
  }

  return constrain(percentage, 0, 100);
}

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) { deviceConnected = true; }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    BLEDevice::startAdvertising(); 
  }
};

class DataInputCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String rxValue = pCharacteristic->getValue();
    rxValue.trim();

    if (rxValue.length() > 0) {
      float parsed_T_set = 0;
      float parsed_delta = 0;
      int matched = 0;
      const char* payload = rxValue.c_str();

      if (strstr(payload, "T") != NULL || strstr(payload, "D") != NULL) {
        matched = sscanf(payload, "T%f, D%f", &parsed_T_set, &parsed_delta);
        if (matched != 2) {
          matched = sscanf(payload, "T%f,D%f", &parsed_T_set, &parsed_delta);
        }
      } else {
        matched = sscanf(payload, "%f,%f", &parsed_T_set, &parsed_delta);
      }

      if (matched == 2) {
        T_set = parsed_T_set;
        delta = parsed_delta;
      }
    }
  }
};

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(ADC_PIN, INPUT);
  pinMode(BattreyPin, INPUT); // Initialize the hardware battery sense line
  
  irsend.begin(); 

  for (int i = 0; i < MAX_SAMPLES; i++) tempSamples[i] = 0.0;

  BLEDevice::init("C3_AC_Terminal");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
                        TX_CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                                           RX_CHARACTERISTIC_UUID,
                                           BLECharacteristic::PROPERTY_WRITE
                                         );
  pRxCharacteristic->setCallbacks(new DataInputCallbacks());

  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  
  Serial.println("ESP32-C3 BLE UART Terminal Ready.");
}

void loop() {
  unsigned long currentTime = millis();
  float T_un = readTemperature(); 

  // ==================== HARSH CORRECTION ENGINE ====================
  if (!isAcOn && T_un != -999) {
    if (T_baseline == -999.0) {
      T_baseline = T_un;
      baselineTime = currentTime;
    }

    float tempDifference = T_un - T_baseline;
    unsigned long timeDifferenceMs = currentTime - baselineTime;
    float timeDifferenceMins = (float)timeDifferenceMs / 60000.0;

    if (tempDifference < 0) {
      T_baseline = T_un;
      baselineTime = currentTime;
      Th_corr = 0.0;
    } 
    else if (timeDifferenceMins > 10.0) {
      T_baseline = T_un;
      baselineTime = currentTime;
      Th_corr = 0.0;
    } 
    else if (tempDifference >= 3.0) {
      if (timeDifferenceMins <= 3.0) {
        Th_corr = 2.0; 
      } else if (timeDifferenceMins >= 10.0) {
        Th_corr = 0.0; 
      } else {
        Th_corr = 2.0 - ((2.0 / 7.0) * (timeDifferenceMins - 3.0));
      }
      Th_corr = constrain(Th_corr, 0.0, 2.0);
    }
  } else {
    Th_corr = 0.0;
    T_baseline = -999.0;
  }
  // ==================================================================

  // 1. Temperature & Battery Window (Every 10 Seconds)
  if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = currentTime;

    // Update internal battery percentage metric here safely
    currentBatteryPercentage = readBatteryPercentage();

    if (T_un != -999) {
      tempSamples[sampleIndex] = T_un;
      sampleIndex = (sampleIndex + 1) % MAX_SAMPLES;
      if (totalSamplesCollected < MAX_SAMPLES) totalSamplesCollected++;
      
      float sum = 0;
      for (int i = 0; i < totalSamplesCollected; i++) {
        sum += tempSamples[i];
      }
      T_current = sum / totalSamplesCollected;

      if (T_current > (T_set + delta - Th_corr) && !isAcOn) {
        irsend.sendRaw(AC_ON_RAW, AC_ON_LEN, IR_FREQUENCY); 
        isAcOn = true;
      } 
      else if (T_current < (T_set - delta) && isAcOn) {
        irsend.sendRaw(AC_OFF_RAW, AC_OFF_LEN, IR_FREQUENCY);
        isAcOn = false;
      }
    }
  }

  // 2. Terminal Transmission Window (Every 1 Second)
  if (currentTime - lastTerminalTime >= TERMINAL_INTERVAL) {
    lastTerminalTime = currentTime;

    if (deviceConnected) {
      char txBuffer[130]; // Expanded buffer size slightly for extra metric output
      if (T_un == -999) {
        sprintf(txBuffer, "Sensor Wiring Error! | Batt:%d%%\r\n", currentBatteryPercentage);
      } else {
        // Appended custom Battery percentage data payload seamlessly to your BLE transmission
        sprintf(txBuffer, "T_avg:%.2f C | Tun:%.2f C | Target:%.1f | D:%.1f | AC:%s | Batt:%d%%\r\n", 
                T_current, T_un, T_set, delta, isAcOn ? "ON" : "OFF", currentBatteryPercentage);
      }
      
      pTxCharacteristic->setValue(txBuffer);
      pTxCharacteristic->notify();
    }
  }
}