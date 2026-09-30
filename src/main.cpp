#include <Arduino.h>
#include <driver/i2s.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ================================================================
// ⚙️ SWITCH: เปลี่ยนเป็น true เมื่อประกอบไมค์ INMP441 และแอมป์ MAX98357A เข้าบอร์ดแล้ว
// ================================================================
#define HAS_HARDWARE false 

// พิน I2S ไมค์ และ แอมป์
#define I2S_MIC_SD 4
#define I2S_MIC_WS 5
#define I2S_MIC_SCK 6
#define I2S_AMP_DIN 7
#define I2S_AMP_LRC 15
#define I2S_AMP_BCLK 16

#define SAMPLE_RATE 16000
#define BUFFER_SIZE 512

// UUIDs สำหรับ Web Bluetooth
#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_VOL_L   "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_VOL_R   "8b00d662-1b1d-4076-96a9-0b1a0301a2d1"
#define CHARACTERISTIC_BAT_L   "09650011-8208-41f2-9596-f94d3809b44a"
#define CHARACTERISTIC_NOISE   "c82f232a-5a21-4f11-a0a1-43e5c9a0ef01"
#define CHARACTERISTIC_FOCUS   "d13f412b-6b32-5f22-b1b2-54f6d0b1fa02"

BLEServer* pServer = NULL;
BLECharacteristic* pVolLeftChar = NULL;
BLECharacteristic* pVolRightChar = NULL;
BLECharacteristic* pBatLeftChar = NULL;
BLECharacteristic* pNoiseChar = NULL;
BLECharacteristic* pFocusChar = NULL;

bool deviceConnected = false;
float gainLeft = 1.0;   // ค่า Gain ซ้าย (1.0x = 50%)
float gainRight = 1.0;  // ค่า Gain ขวา
int noiseLevel = 1;     // 0 = Low, 1 = Medium, 2 = High
String focusMode = "directional";
unsigned long lastBatUpdate = 0;

// BLE Callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("\n[BLE Status] 🟢 Web Dashboard Connected!");
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("\n[BLE Status] 🔴 Web Dashboard Disconnected!");
      BLEDevice::startAdvertising();
    }
};

class VolumeCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = pCharacteristic->getValue().c_str();
      if (rxValue.length() > 0) {
        int val = rxValue.toInt();
        float gain = (float)val / 50.0;
        if (pCharacteristic == pVolLeftChar) {
          gainLeft = gain;
          Serial.printf("[BLE Receiver] Left Gain: %.2fx (%d%%)\n", gainLeft, val);
        } else if (pCharacteristic == pVolRightChar) {
          gainRight = gain;
          Serial.printf("[BLE Receiver] Right Gain: %.2fx (%d%%)\n", gainRight, val);
        }
      }
    }
};

class NoiseCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = pCharacteristic->getValue().c_str();
      if (rxValue.length() > 0) {
        noiseLevel = rxValue.toInt();
        const char* levels[] = {"Low", "Medium", "High"};
        Serial.printf("[BLE Receiver] Noise Reduction Level: %s (%d)\n", levels[noiseLevel], noiseLevel);
      }
    }
};

class FocusCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = pCharacteristic->getValue().c_str();
      if (rxValue.length() > 0) {
        focusMode = rxValue;
        Serial.printf("[BLE Receiver] Focus Mode: %s\n", focusMode.c_str());
      }
    }
};

// -------------------------------------------------------------
// บล็อกคอนฟิก I2S ฮาร์ดแวร์ (เขียนเตรียมไว้ก่อน)
// -------------------------------------------------------------
void setupI2S() {
#if HAS_HARDWARE
  // ตั้งค่าไมค์ INMP441
  i2s_config_t i2s_mic_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = BUFFER_SIZE
  };
  i2s_pin_config_t mic_pin = { .bck_io_num = I2S_MIC_SCK, .ws_io_num = I2S_MIC_WS, .data_out_num = I2S_PIN_NO_CHANGE, .data_in_num = I2S_MIC_SD };
  i2s_driver_install(I2S_NUM_0, &i2s_mic_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &mic_pin);

  // ตั้งค่าแอมป์ MAX98357A
  i2s_config_t i2s_amp_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = BUFFER_SIZE
  };
  i2s_pin_config_t amp_pin = { .bck_io_num = I2S_AMP_BCLK, .ws_io_num = I2S_AMP_LRC, .data_out_num = I2S_AMP_DIN, .data_in_num = I2S_PIN_NO_CHANGE };
  i2s_driver_install(I2S_NUM_1, &i2s_amp_config, 0, NULL);
  i2s_set_pin(I2S_NUM_1, &amp_pin);
  Serial.println("[I2S] Audio Hardware Driver Initialized!");
#endif
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n==============================================");
  Serial.println("  ESP32-S3 Hearing Aid Firmware (Bypass Mode) ");
  Serial.println("==============================================");

  setupI2S();

  // ตั้งค่า BLE
  BLEDevice::init("ESP32_HearingAid");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pVolLeftChar = pService->createCharacteristic(CHARACTERISTIC_VOL_L, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pVolLeftChar->setCallbacks(new VolumeCallback());

  pVolRightChar = pService->createCharacteristic(CHARACTERISTIC_VOL_R, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pVolRightChar->setCallbacks(new VolumeCallback());

  pBatLeftChar = pService->createCharacteristic(CHARACTERISTIC_BAT_L, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pBatLeftChar->addDescriptor(new BLE2902());

  pNoiseChar = pService->createCharacteristic(CHARACTERISTIC_NOISE, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pNoiseChar->setCallbacks(new NoiseCallback());

  pFocusChar = pService->createCharacteristic(CHARACTERISTIC_FOCUS, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pFocusChar->setCallbacks(new FocusCallback());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("✅ BLE Services Active & Ready to Pair!");
}

void loop() {
#if HAS_HARDWARE
  // บล็อกอ่าน-ส่งเสียงจริงจากฮาร์ดแวร์
  int16_t sampleBuffer[BUFFER_SIZE];
  size_t bytesRead = 0, bytesWritten = 0;

  i2s_read(I2S_NUM_0, sampleBuffer, sizeof(sampleBuffer), &bytesRead, portMAX_DELAY);
  int count = bytesRead / sizeof(int16_t);

  for (int i = 0; i < count; i++) {
    int32_t val = sampleBuffer[i] * gainLeft;
    if (val > 32767) val = 32767;
    if (val < -32768) val = -32768;
    sampleBuffer[i] = (int16_t)val;
  }

  i2s_write(I2S_NUM_1, sampleBuffer, bytesRead, &bytesWritten, portMAX_DELAY);
#endif

  // บล็อกส่งค่าแบตเตอรี่จำลองไปยัง Web Dashboard
  if (deviceConnected && (millis() - lastBatUpdate > 5000)) {
    lastBatUpdate = millis();
    int dummyBattery = 92;
    char batStr[8];
    snprintf(batStr, sizeof(batStr), "%d", dummyBattery);

    pBatLeftChar->setValue(batStr);
    pBatLeftChar->notify();
  }

  delay(10);
}