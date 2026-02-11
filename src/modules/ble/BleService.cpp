#include "modules/ble/BleService.h"

#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <M5Unified.h>
#include <esp_bt.h>
#include <esp_bt_main.h>

#include "modules/display/DisplayController.h"

#define STERZO_SERVICE_UUID "347b0001-7635-408b-8918-8ff3949ce592"
#define CHAR14_UUID         "347b0014-7635-408b-8918-8ff3949ce592"
#define CHAR30_UUID         "347b0030-7635-408b-8918-8ff3949ce592"
#define CHAR31_UUID         "347b0031-7635-408b-8918-8ff3949ce592"
#define CHAR32_UUID         "347b0032-7635-408b-8918-8ff3949ce592"

// Shared runtime state owned in main.cpp for now.
extern BLEServer* pServer;
extern BLEService* pSvc;
extern BLECharacteristic* pChar14;
extern BLECharacteristic* pChar30;
extern BLECharacteristic* pChar31;
extern BLECharacteristic* pChar32;
extern BLE2902* p2902_14;
extern BLE2902* p2902_30;
extern BLE2902* p2902_32;
extern bool bleStackInitialized;
extern bool deviceConnected;
extern bool ind32On;
extern bool challengeOK;
extern bool zwiftConnected;
extern bool screenOn;
extern unsigned long lastButtonTime;
extern unsigned long lastBLEActivityTime;

void exitScreenOffMode();

static uint32_t rotate_left32(uint32_t value, uint32_t count) {
  const uint32_t mask = (CHAR_BIT * sizeof(value)) - 1;
  count &= mask;
  return (value << count) | (value >> (-count & mask));
}

static uint32_t hashed(uint64_t seed) {
  uint32_t ret = (uint32_t)(seed + 0x16fa5717);
  uint64_t rax = seed * 0xba2e8ba3ULL;
  uint64_t eax = (rax >> 35) * 0xb;
  uint64_t ecx = seed - eax;
  uint32_t edx = rotate_left32((uint32_t)seed, (uint32_t)(ecx & 0x0F));
  return ret ^ edx;
}

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    (void)s;
    deviceConnected = true;
    lastBLEActivityTime = millis();
    Serial.println("BLE client connected");

    if (!screenOn) {
      exitScreenOffMode();
    }
    lastButtonTime = millis();
    display::handleEvent(display::DisplayEvent::BleConnected);
  }

  void onDisconnect(BLEServer* s) override {
    (void)s;
    deviceConnected = false;
    zwiftConnected = false;
    Serial.println("BLE client disconnected");

    if (screenOn) {
      display::handleEvent(display::DisplayEvent::BleDisconnected);
    }
  }
};

class Char31Callbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    lastBLEActivityTime = millis();

    auto val = c->getValue();
    Serial.print("char31 onWrite: ");
    for (auto& b : val) {
      Serial.printf("%02X ", b);
    }
    Serial.println();

    if (val.size() >= 2 && val[0] == 0x03 && val[1] == 0x10) {
      Serial.println("-> got 0x310, sending initial challenge");
      uint8_t chal[4] = {0x03, 0x10, 0x12, 0x34};
      try {
        pChar32->setValue(chal, 4);
        pChar32->indicate();
        zwiftConnected = false;
      } catch (const std::exception& e) {
        Serial.printf("ERROR: Failed to send initial challenge: %s\n", e.what());
      }
    } else if (val.size() >= 2 && val[0] == 0x03 && val[1] == 0x11) {
      Serial.println("-> got 0x311, marking challengeOK");
      challengeOK = true;
      uint8_t resp[4] = {0x03, 0x11, 0xFF, 0xFF};
      try {
        pChar32->setValue(resp, 4);
        pChar32->indicate();
      } catch (const std::exception& e) {
        Serial.printf("ERROR: Failed to send 0x311 response: %s\n", e.what());
        challengeOK = false;
      }
    } else if (val.size() >= 6 && val[0] == 0x03 && val[1] == 0x12) {
      uint32_t seed = ((uint32_t)val[5] << 24) | ((uint32_t)val[4] << 16) | ((uint32_t)val[3] << 8) | ((uint32_t)val[2]);
      uint32_t pwd = hashed(seed);
      Serial.printf("-> got 0x312, seed=0x%08X, pwd=0x%08X\n", seed, pwd);
      uint8_t res[6];
      res[0] = 0x03;
      res[1] = 0x12;
      res[2] = pwd & 0xFF;
      res[3] = (pwd >> 8) & 0xFF;
      res[4] = (pwd >> 16) & 0xFF;
      res[5] = (pwd >> 24) & 0xFF;
      try {
        pChar32->setValue(res, 6);
        pChar32->indicate();
      } catch (const std::exception& e) {
        Serial.printf("ERROR: Failed to send 0x312 response: %s\n", e.what());
      }
    } else if (val.size() >= 2 && val[0] == 0x03 && val[1] == 0x13) {
      Serial.println("-> got 0x313, final ACK");
      uint8_t r[3] = {0x03, 0x13, 0xFF};
      try {
        pChar32->setValue(r, 3);
        pChar32->indicate();
        challengeOK = true;
        zwiftConnected = true;
        Serial.println("Zwift connection established - keeping screen on indefinitely");
      } catch (const std::exception& e) {
        Serial.printf("ERROR: Failed to send final ACK: %s\n", e.what());
        challengeOK = false;
        zwiftConnected = false;
      }
    }
  }
};

class Char32Callbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    (void)c;
    Serial.println("char32 onWrite");
  }
  void onRead(BLECharacteristic* c) override {
    (void)c;
    Serial.println("char32 onRead");
  }
};

class Char32Desc2902Callbacks : public BLEDescriptorCallbacks {
  void onWrite(BLEDescriptor* d) override {
    (void)d;
    ind32On = true;
    Serial.println("Client ENABLED indications (char32)");
  }
};

bool configureBLEStack() {
  if (bleStackInitialized) {
    return true;
  }

  Serial.println("Configuring BLE stack...");

  try {
    BLEDevice::init("STERZO");
  } catch (const std::exception& e) {
    Serial.printf("ERROR: BLE initialization failed: %s\n", e.what());
    return false;
  }

  try {
    pServer = BLEDevice::createServer();
    if (!pServer) {
      Serial.println("ERROR: Failed to create BLE server");
      return false;
    }

    pServer->setCallbacks(new MyServerCallbacks());
    pSvc = pServer->createService(STERZO_SERVICE_UUID);
    if (!pSvc) {
      Serial.println("ERROR: Failed to create BLE service");
      return false;
    }
  } catch (const std::exception& e) {
    Serial.printf("ERROR: BLE server/service setup failed: %s\n", e.what());
    return false;
  }

  try {
    pChar14 = pSvc->createCharacteristic(CHAR14_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    pChar30 = pSvc->createCharacteristic(CHAR30_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    pChar31 = pSvc->createCharacteristic(CHAR31_UUID, BLECharacteristic::PROPERTY_WRITE);
    pChar32 = pSvc->createCharacteristic(CHAR32_UUID, BLECharacteristic::PROPERTY_INDICATE);

    if (!pChar14 || !pChar30 || !pChar31 || !pChar32) {
      Serial.println("ERROR: Failed to create one or more BLE characteristics");
      return false;
    }

    p2902_14 = new BLE2902();
    pChar14->addDescriptor(p2902_14);

    p2902_30 = new BLE2902();
    p2902_30->setNotifications(true);
    pChar30->addDescriptor(p2902_30);

    p2902_32 = new BLE2902();
    p2902_32->setCallbacks(new Char32Desc2902Callbacks());
    pChar32->addDescriptor(p2902_32);

    pChar31->setCallbacks(new Char31Callbacks());
    pChar32->setCallbacks(new Char32Callbacks());

    pSvc->start();

    auto adv = BLEDevice::getAdvertising();
    if (!adv) {
      Serial.println("ERROR: Failed to get BLE advertising object");
      return false;
    }

    adv->addServiceUUID(STERZO_SERVICE_UUID);
    adv->setScanResponse(true);
    adv->setMinPreferred(0x06);
    adv->setMinPreferred(0x12);
    adv->setMinInterval(800);
    adv->setMaxInterval(1600);

    BLEDevice::startAdvertising();
  } catch (const std::exception& e) {
    Serial.printf("ERROR: BLE characteristic/advertising setup failed: %s\n", e.what());
    return false;
  }

  bleStackInitialized = true;
  Serial.println("BLE stack configured and advertising started");
  return true;
}

void startBLE() {
  Serial.println("Starting BLE...");
  esp_bt_controller_status_t status = esp_bt_controller_get_status();

  if (status == ESP_BT_CONTROLLER_STATUS_IDLE) {
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
#if CONFIG_IDF_TARGET_ESP32
    bt_cfg.mode = ESP_BT_MODE_BLE;
#endif
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
      Serial.printf("ERROR: BT controller init failed: %s (0x%x)\n", esp_err_to_name(ret), ret);
      return;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
      Serial.printf("ERROR: BT controller enable failed: %s (0x%x)\n", esp_err_to_name(ret), ret);
      esp_bt_controller_deinit();
      return;
    }
  } else if (status == ESP_BT_CONTROLLER_STATUS_INITED) {
    esp_err_t ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
      Serial.printf("ERROR: BT controller enable failed: %s (0x%x)\n", esp_err_to_name(ret), ret);
      return;
    }
  } else if (status == ESP_BT_CONTROLLER_STATUS_ENABLED) {
    Serial.println("BLE controller already enabled, resuming advertising");
    BLEDevice::startAdvertising();
  }

  setCpuFrequencyMhz(80);
  Serial.println("BLE started successfully, CPU at 80MHz");

  if (!configureBLEStack()) {
    Serial.println("ERROR: BLE stack configuration failed");
  }
}

void stopBLE() {
  Serial.println("Stopping BLE to save power...");
  esp_bt_controller_status_t status = esp_bt_controller_get_status();
  if (status == ESP_BT_CONTROLLER_STATUS_IDLE) {
    Serial.println("BLE already stopped");
    return;
  }

  try {
    BLEDevice::deinit(false);
    bleStackInitialized = false;
    pServer = nullptr;
    pSvc = nullptr;
    pChar14 = nullptr;
    pChar30 = nullptr;
    pChar31 = nullptr;
    pChar32 = nullptr;
    p2902_14 = nullptr;
    p2902_30 = nullptr;
    p2902_32 = nullptr;
  } catch (const std::exception& e) {
    Serial.printf("WARNING: BLEDevice::deinit() failed: %s\n", e.what());
  }

  if (status == ESP_BT_CONTROLLER_STATUS_ENABLED) {
    esp_err_t ret = esp_bt_controller_disable();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
      Serial.printf("WARNING: BT controller disable failed: %s (0x%x)\n", esp_err_to_name(ret), ret);
      return;
    }
  }

  status = esp_bt_controller_get_status();
  if (status == ESP_BT_CONTROLLER_STATUS_INITED) {
    esp_err_t ret = esp_bt_controller_deinit();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
      Serial.printf("WARNING: BT controller deinit failed: %s (0x%x)\n", esp_err_to_name(ret), ret);
    }
  }

  Serial.println("BLE stopped successfully");
}

void sendSteeringBin(float bin) {
  try {
    pChar30->setValue((uint8_t*)&bin, sizeof(bin));
    pChar30->notify();
    Serial.printf("NOTIFY bin: %.0f°\n", bin);
  } catch (const std::exception& e) {
    Serial.printf("ERROR: Failed to send steering bin %.0f°: %s\n", bin, e.what());
  }
}
