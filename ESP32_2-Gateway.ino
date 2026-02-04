/**
 * ESP32 GATEWAY - ULTIMATE MERGED VERSION
 * 1. Token: OLD (drLWRdn9tSKGmKQ6QHcz)
 * 2. Feature: 2-Way Communication (Upload Telemetry + RPC Downlink)
 * 3. System: Anti-Freeze (Watchdog) + WiFi Auto Reconnect
 * 4. Monitor: Clean Format & Slow Speed
 */

#if defined(ESP8266)
#include <ESP8266WiFi.h>
#define THINGSBOARD_ENABLE_PROGMEM 0
#elif defined(ESP32) || defined(RASPBERRYPI_PICO) || defined(RASPBERRYPI_PICO_W)
#include <WiFi.h>
#endif

#include "BLEDevice.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <esp_task_wdt.h> 

// ===================== ThingsBoard & MQTT =====================
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h> 

// --- THONG TIN WIFI & SERVER ---
const char* ssid = "Dimethylbenzene";
const char* pass = "Kimbokjooswag05";

constexpr char THINGSBOARD_SERVER[] = "dashboard.tyne.vn";
// [QUAN TRONG] SU DUNG TOKEN CU NHU YEU CAU
constexpr char THINGSBOARD_ACCESS_TOKEN[] = "drLWRdn9tSKGmKQ6QHcz"; 
constexpr uint16_t THINGSBOARD_PORT = 1893;
constexpr uint32_t MAX_MESSAGE_SIZE = 512U;

// [CONFIG] Watchdog & Network
#define WDT_TIMEOUT_SECONDS 20 
const unsigned long MAX_OFFLINE_TIME = 180000; // 3 phut khong co mang thi reset

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE); 

// ===================== BLE VARIABLES =====================
static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
static BLEUUID charUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;

static BLERemoteCharacteristic *pRemoteCharacteristic = nullptr;
static BLEAdvertisedDevice *myDevice = nullptr;
static BLEClient *pClient = nullptr;

// ===================== DATA VARIABLES =====================
float g_temp = 0.0;
int   g_hum  = 0;
long  g_ir   = 0;
int   g_relay= 0;
int   g_state= 0; 

volatile bool g_newData = false;
bool g_dataReceivedInThisSession = false; 

// ===================== LED CONFIG =====================
#define ON_Board_LED 2        
const int LED_ON  = HIGH; // ESP32 thuong la HIGH = Sang      
const int LED_OFF = LOW;

bool wifiWasConnected = false;
uint32_t wifiLastAttemptMs = 0;
const uint32_t WIFI_RETRY_MS = 5000;

uint32_t ledLastToggleMs = 0;
const uint32_t LED_BLINK_MS = 500; 
bool ledState = false;

// ===================== RPC HANDLERS (NHAN LENH TU CLOUD) =====================
RPC_Response processSetTemp(const RPC_Data &data) {
  float targetTemp = data; 
  Serial.print("[RPC] Set Temp: "); Serial.println(targetTemp);
  
  String packet = "T" + String(targetTemp, 1) + "#"; 
  if (pRemoteCharacteristic != nullptr && connected) {
      pRemoteCharacteristic->writeValue(packet.c_str(), packet.length());
      return RPC_Response(NULL, targetTemp); 
  }
  return RPC_Response(NULL, "BLE Disconnected");
}

RPC_Response processSetCup(const RPC_Data &data) {
  int cupThresh = data;
  Serial.print("[RPC] Set Cup: "); Serial.println(cupThresh);

  String packet = "C" + String(cupThresh) + "#";
  if (pRemoteCharacteristic != nullptr && connected) {
      pRemoteCharacteristic->writeValue(packet.c_str(), packet.length());
      return RPC_Response(NULL, cupThresh);
  }
  return RPC_Response(NULL, "BLE Disconnected");
}

RPC_Response processSetDone(const RPC_Data &data) {
  int doneThresh = data;
  Serial.print("[RPC] Set Done: "); Serial.println(doneThresh);

  String packet = "D" + String(doneThresh) + "#";
  if (pRemoteCharacteristic != nullptr && connected) {
      pRemoteCharacteristic->writeValue(packet.c_str(), packet.length());
      return RPC_Response(NULL, doneThresh);
  }
  return RPC_Response(NULL, "BLE Disconnected");
}

// Khai bao mang Callback
RPC_Callback callbacks[] = {
  { "setTemp", processSetTemp },
  { "setCup",  processSetCup },
  { "setDone", processSetDone }
};
const size_t callbacks_count = sizeof(callbacks) / sizeof(RPC_Callback);

// ===================== TIMERS =====================
const uint32_t TB_MIN_INTERVAL_MS = 2000; 
uint32_t lastTbUploadMs = 0;
uint32_t lastNetworkCheckMs = 0;
unsigned long offlineStartTime = 0;

// ===================== HELPER FUNCTIONS =====================
String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;
  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

float extractNumber(String subString) {
    int colonIndex = subString.indexOf(':');
    if (colonIndex != -1) return subString.substring(colonIndex + 1).toFloat();
    return 0.0;
}

static bool parseSTM32Payload(String payload) {
    if (!payload.startsWith("*") || !payload.endsWith("#")) return false;
    String cleanData = payload.substring(1, payload.length() - 1);
    
    String s_temp = getValue(cleanData, ',', 0);
    String s_stat = getValue(cleanData, ',', 4); 

    if(s_temp == "" || s_stat == "") return false;

    g_temp  = extractNumber(s_temp);
    g_hum   = (int)extractNumber(getValue(cleanData, ',', 1));
    g_ir    = (long)extractNumber(getValue(cleanData, ',', 2));
    g_relay = (int)extractNumber(getValue(cleanData, ',', 3));
    g_state = (int)extractNumber(s_stat);
    return true;
}

static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic,
                           uint8_t *pData, size_t length, bool isNotify) {
  if (g_dataReceivedInThisSession) return;
  esp_task_wdt_reset(); 

  String s;
  s.reserve(length + 1);
  for (size_t i = 0; i < length; i++) s += (char)pData[i];

  if (s.startsWith("*")) {
    if (parseSTM32Payload(s)) {
      g_newData = true;
      g_dataReceivedInThisSession = true; 
      
      // [MONITOR DEP - STYLE CODE CU]
      Serial.printf("[PARSE SUCCESS] T:%.1f H:%d I:%ld Relay:%d Mode:%d\n", 
                    g_temp, g_hum, g_ir, g_relay, g_state);
    } 
  }
}

// ===================== BLE Client Callbacks =====================
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient *pclient) override {
    Serial.println("[BLE] CONNECTED NODE");
    g_dataReceivedInThisSession = false; 
    connected = true;
  }
  void onDisconnect(BLEClient *pclient) override {
    connected = false;
    Serial.println("[BLE] DISCONNECTED NODE");
  }
};

bool connectToServer() {
  Serial.println("[BLE] Connecting...");
  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());
  if(!pClient->connect(myDevice)) return false;
  pClient->setMTU(517); 

  BLERemoteService *pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) { pClient->disconnect(); return false; }

  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) { pClient->disconnect(); return false; }

  if (pRemoteCharacteristic->canNotify()) pRemoteCharacteristic->registerForNotify(notifyCallback);
  return true;
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;
    }
  }
};

// ===================== WIFI & TB HANDLING =====================
void handleWiFiAndThingsBoard() {
  unsigned long now = millis();
  
  // 1. Xu ly WiFi
  if (WiFi.status() != WL_CONNECTED) {
    wifiWasConnected = false;
    
    // [LED] Nhap nhay khi mat mang
    if (now - ledLastToggleMs >= LED_BLINK_MS) {
      ledLastToggleMs = now;
      ledState = !ledState;
      digitalWrite(ON_Board_LED, ledState ? LED_ON : LED_OFF);
    }
    
    if (now - wifiLastAttemptMs >= WIFI_RETRY_MS) {
      wifiLastAttemptMs = now;
      Serial.println("[WiFi] Reconnecting...");
      WiFi.disconnect(false);
      WiFi.begin(ssid, pass);
    }

    // [CHONG TREO] Reset neu mat mang qua 3 phut
    if (offlineStartTime == 0) offlineStartTime = now;
    else if (now - offlineStartTime > MAX_OFFLINE_TIME) {
       Serial.println("[SYSTEM] Reset do mat mang qua lau!");
       ESP.restart();
    }
    return;
  }

  // 2. Khi WiFi OK
  if (!wifiWasConnected) {
    wifiWasConnected = true;
    Serial.println("[WiFi] CONNECTED.");
    // [LED] Sang dung khi co mang
    digitalWrite(ON_Board_LED, LED_ON); 
    offlineStartTime = 0; 
  }

  // 3. Xu ly ThingsBoard
  if (!tb.connected()) {
    if (now - lastNetworkCheckMs > 2000) {
        lastNetworkCheckMs = now;
        Serial.print("[TB] Connecting... ");
        if (tb.connect(THINGSBOARD_SERVER, THINGSBOARD_ACCESS_TOKEN, THINGSBOARD_PORT)) {
          Serial.println("OK");
          Serial.println("[TB] Subscribing RPC...");
          const RPC_Callback *firstCallback = callbacks;
          const RPC_Callback *lastCallback  = callbacks + callbacks_count;
          tb.RPC_Subscribe(firstCallback, lastCallback); 
        } else {
          Serial.println("Fail");
        }
    }
  } else {
    tb.loop();
  }
}

void uploadIfNewBleData() {
  if (!g_newData) return;
  if (WiFi.status() != WL_CONNECTED || !tb.connected()) return;
  if (millis() - lastTbUploadMs < TB_MIN_INTERVAL_MS) return;

  Serial.println("[TB] Uploading Telemetry...");
  tb.sendTelemetryData("temp", g_temp);     
  tb.sendTelemetryData("humidity", g_hum);
  tb.sendTelemetryData("ir_val", g_ir);     
  tb.sendTelemetryData("relay", g_relay);   
  tb.sendTelemetryData("state", g_state);   

  lastTbUploadMs = millis();
  g_newData = false; 
}

// ===================== SETUP & LOOP =====================
void setup() {
  // [CHONG TREO] On dinh nguon dien
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
  
  // [CHONG TREO] Watchdog Timer 20s
  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = WDT_TIMEOUT_SECONDS * 1000,
      .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,    
      .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL); 

  Serial.begin(115200);
  Serial.println("--- ESP32 GATEWAY - ULTIMATE MERGE ---");

  pinMode(ON_Board_LED, OUTPUT);
  digitalWrite(ON_Board_LED, LED_OFF);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  BLEDevice::init("");
  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false); 
}

void loop() {
  esp_task_wdt_reset(); // Reset cho Watchdog

  if (doConnect) {
    if (connectToServer()) Serial.println("[BLE] Connected Node.");
    else Serial.println("[BLE] Fail.");
    doConnect = false;
  }

  if (!connected && doScan) {
    BLEDevice::getScan()->clearResults();
    BLEDevice::getScan()->start(1); 
  }

  handleWiFiAndThingsBoard();
  uploadIfNewBleData();
  
  // [MONITOR] Delay 1 giay moi chay lai vong lap de de doc log
  delay(1000); 
}