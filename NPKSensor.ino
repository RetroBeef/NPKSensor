#include <esp_task_wdt.h>

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#include <TP357.h>
#include <Aroma.h>
#include <FC3000.h>
#include <RbBleDevice.h>

#include <vector>

#include "NPKSensor.h"
#include "creds.h"

StaticJsonDocument<64> doc;

typedef struct {
  String deviceName;
  uint8_t state;
  String dateString;
  float power;
  float factor;
  float voltage;
  float current;
} TasmotaState_t;

typedef struct{
  uint8_t state;
  uint8_t dimmPercentage;
  uint32_t localTime;
  uint32_t lastSeenMs;
}fc3000_state_t;

typedef struct{
  uint16_t pan;
  uint16_t tilt;
  uint32_t distance;
  uint32_t lastSeenMs;
}distance_state_t;

typedef struct{
  float weight;
  uint32_t lastSeenMs;
}weight_state_t;

TasmotaState_t lastTasmotaExhaustionState;
TasmotaState_t lastTasmotaLightState;
fc3000_state_t lastLightState = {0,0,0,0};
distance_state_t lastDistanceState = {0,0,0,0};
weight_state_t lastWeightState = {0.f,0};

uint8_t scanTime = 5;
BLEScan* pBLEScan = 0;

BLEClient* bleClientAroma = 0;
Aroma* aroma = 0;

BLEClient* bleClientLight = 0;
FC3000* light = 0;

BLEClient* bleClientDistance = 0;
RbBleDevice* distance = 0;

BLEClient* bleClientWeight = 0;
RbBleDevice* weight = 0;

char restToken[25];

std::vector<ScannableBleDevice*> scannableBleDevices;
ThermoPro* tp = 0;

std::vector<std::function<void()>> aromaActionQueue;
std::vector<std::function<void()>> distanceActionQueue;

WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);
uint32_t wifiLastSeen = 0;

int botRequestDelay = 1000;
unsigned long lastTimeBotRan;

const uint8_t rxPin = 5;//23;//5;
const uint8_t txPin = 6;//19;//6;

sensor_t sensorType = SENSORTYPE8;//SENSORTYPE7;

NPKSensor* npkSensor = 0;
npk_data_t lastNpkData;

uint32_t lastBleScan = 0;

aroma_response_t lastFogResponse;
aroma_response_t lastLedResponse;

bool autoHumidityEnabled = false;
uint8_t humidityOuterLimit = 5;
uint8_t humidityInnerLimit = 3;
uint8_t humidityTargetValue = 65;

String tasmotaToList(const TasmotaState_t& tmState) {
  return "Switch: " + String(tmState.state) + "\nPower: " + String(tmState.power, 2) + " W";
}

String tasmotaToJSON(const TasmotaState_t& tmState) {
  return "{\"deviceName\":\"" + tmState.deviceName + "\",\"state\":" + String(tmState.state) + ",\"dateString\":\"" + tmState.dateString + "\",\"power\":" + String(tmState.power) + ",\"factor\":" + String(tmState.factor) + ",\"voltage\":" + String(tmState.voltage) + ",\"current\":" + String(tmState.current) + "}";
}

String aromaToList() {
  aroma_rgb_t* rgb = (aroma_rgb_t*)lastLedResponse.data;
  return "Humidifier:\nFog: " + String(lastFogResponse.state) + "\nLed: " + String(lastLedResponse.state) + " " + String(rgb->r) + " " + String(rgb->g) + " " + String(rgb->b);
}

String aromaToJSON() {
  aroma_rgb_t* rgb = (aroma_rgb_t*)lastLedResponse.data;
  return "{\"fog\":" +
         String(lastFogResponse.state) +
         ",\"led\":{\"state\":" +
         String(lastLedResponse.state) +
         ",\"color\":{\"r\":" + String(rgb->r) +
         ",\"g\":" + String(rgb->g) +
         ",\"b\":" + String(rgb->b) +
         "}}}";
}

String lightToList(){
  return "Lights:\nState: " + String(lastLightState.state) + "\nDimmer: " + String(lastLightState.dimmPercentage) + "%\nlocaltime: " + String(lastLightState.localTime) + "\nlastSeen: " + String(lastLightState.lastSeenMs) + "ms ago\n" + tasmotaToList(lastTasmotaLightState);
}

String lightToJSON(){
  return "{\"state\":" + String(lastLightState.state) + ",\"dimmer\":" + String(lastLightState.dimmPercentage) + ",\"localtime\":" + String(lastLightState.localTime) + ",\"lastseen\":" + String(lastLightState.lastSeenMs) + ",\"plug\":" + tasmotaToJSON(lastTasmotaLightState) + "}";
}

String distanceToList(){
  return "Distance:\nPan: " + String(lastDistanceState.pan) + "\nTilt: " + String(lastDistanceState.tilt) + "\nDistance: " + String(lastDistanceState.distance) + "\nlastSeen: " + String(lastDistanceState.lastSeenMs) + "ms ago";
}

String distanceToJSON(){
  return "{\"pan\":" + String(lastDistanceState.pan) + ",\"tilt\":" + String(lastDistanceState.tilt) + ",\"distance\":" + String(lastDistanceState.distance) + ",\"lastseen\":" + String(lastDistanceState.lastSeenMs) + "}";
}

String weightToList(){
  return "Scale:\nWeight: " + String(lastWeightState.weight) + "\nlastSeen: " + String(lastDistanceState.lastSeenMs) + "ms ago";
}

String weightToJSON(){
  return "{\"weight\":" + String(lastWeightState.weight) + ",\"lastseen\":" + String(lastDistanceState.lastSeenMs) + "}";
}

class AdvertisedDeviceCallback: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      for (ScannableBleDevice* d : scannableBleDevices) {
        d->getScanCallback()(advertisedDevice);
      }
    }
};

String getSystemJSON() {
  return "{\"air\":" + tp->toJSON() + ",\"soil\":" + npkSensor->toJSON(lastNpkData) + ",\"humidifier\":" + aromaToJSON() + ",\"exhaustion\":{\"plug\":" + tasmotaToJSON(lastTasmotaExhaustionState) + "},\"lights\":" + lightToJSON() + ",\"distance\":" + distanceToJSON() + ",\"scale\":" + weightToJSON() + ",\"autohum\":" + autoHumidityEnabled + "}";
}

void pushToRestServer() {
  HTTPClient http;
  http.begin(restServerUrl);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("token", restToken);
  int httpResponseCode = http.POST(getSystemJSON());
  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println(httpResponseCode);
    Serial.println(response);
  } else {
    Serial.print("Error on sending POST: ");
    Serial.println(httpResponseCode);
  }
  http.end();
}

void pushToTasmota(const String& url) {
  HTTPClient http;
  http.begin(url);
  int httpResponseCode = http.GET();
  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println(httpResponseCode);
    Serial.println(response);
  } else {
    Serial.print("Error on sending GET: ");
    Serial.println(httpResponseCode);
  }
  http.end();
}

void pullFromTasmota(TasmotaState_t& tmState, const String& urlDevice, const String& urlSensors) {
  HTTPClient httpDevice;
  httpDevice.begin(urlDevice);
  int httpResponseCode = httpDevice.GET();
  if (httpResponseCode > 0) {
    JsonDocument docDevice;
    DeserializationError error = deserializeJson(docDevice, httpDevice.getString());
    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      return;
    }
    JsonObject status = docDevice["Status"];
    tmState.deviceName = (const char*)status["DeviceName"];
    tmState.state = status["Power"];
  } else {
    Serial.print("Error on sending GET: ");
    Serial.println(httpResponseCode);
  }
  httpDevice.end();
  HTTPClient httpSensor;
  httpSensor.begin(urlSensors);
  httpResponseCode = httpSensor.GET();
  if (httpResponseCode > 0) {
    JsonDocument docSensor;
    DeserializationError error = deserializeJson(docSensor, httpSensor.getString());
    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      return;
    }

    JsonObject statusEnergy = docSensor["StatusSNS"]["ENERGY"];
    tmState.dateString = (const char*)docSensor["StatusSNS"]["Time"];
    tmState.power = statusEnergy["Power"];
    tmState.factor = statusEnergy["Factor"];
    tmState.voltage = statusEnergy["Voltage"];
    tmState.current = statusEnergy["Current"];
  } else {
    Serial.print("Error on sending GET: ");
    Serial.println(httpResponseCode);
  }
  httpSensor.end();
}

void naiveControlLoop(const uint8_t& targetValue, const uint8_t& currentValue) {
  if (currentValue >= targetValue + humidityOuterLimit) {
    //ex on
    if (!lastTasmotaExhaustionState.state) {
      pushToTasmota(externalTasmotaExhaustUrlSwitchOn);
      bot.sendMessage(CHAT_ID, "AUTO: turning on exhaustion", "");
    }
  } else if (currentValue >= targetValue + humidityInnerLimit) {
    //fog off
    if (lastFogResponse.state) {
      aromaActionQueue.push_back([&aroma] {
        aroma->enableFog(0);
        aroma->enableLed(1);
        aroma->setLedRgbValue(255, 0, 0);
      });
      bot.sendMessage(CHAT_ID, "AUTO: turning off fog", "");
    }
  } else if (currentValue <= targetValue - humidityInnerLimit) {
    //fog on
    if (!lastFogResponse.state) {
      aromaActionQueue.push_back([&aroma] {
        aroma->enableFog(1);
        aroma->enableLed(1);
        aroma->setLedRgbValue(0, 255, 0);
      });
      bot.sendMessage(CHAT_ID, "AUTO: turning on fog", "");
    }
  } else if (currentValue <= targetValue - humidityOuterLimit) {
    //ex off
    if (lastTasmotaExhaustionState.state) {
      pushToTasmota(externalTasmotaExhaustUrlSwitchOff);
      bot.sendMessage(CHAT_ID, "AUTO: turning off exhaustion", "");
    }
  }
}

String commands = "commands: /start /state /fog_on /fog_off /ex_on /ex_off /cam /dist /json /push /auto_on /auto_off";

void handleNewMessages(int numNewMessages) {
  Serial.println("handleNewMessages");
  Serial.println(String(numNewMessages));

  for (int i = 0; i < numNewMessages; i++) {
    String chat_id = String(bot.messages[i].chat_id);
    if (chat_id != CHAT_ID) {
      bot.sendMessage(chat_id, "Unauthorized user", "");
      continue;
    }

    String text = bot.messages[i].text;
    Serial.println(text);

    String from_name = bot.messages[i].from_name;

    if (text == "/cam") {
      continue;
    }

    if (text == "/dist") {
      distanceActionQueue.push_back([&distance] {
          String cmd = "{\"cmd\":\"dist\"}";
          distance->write(cmd);
      });
      bot.sendMessage(chat_id, "trying to start distance measurement", "");
    }

    if (text == "/auto_on") {
      bot.sendMessage(chat_id, "turning on experimental auto humidity (65% target)", "");
      autoHumidityEnabled = true;
    }

    if (text == "/auto_off") {
      bot.sendMessage(chat_id, "turning off experimental auto humidity (65% target)", "");
      autoHumidityEnabled = false;
    }

    if (text == "/ex_on") {
      bot.sendMessage(chat_id, "trying to turn on exhaustion", "");
      pushToTasmota(externalTasmotaExhaustUrlSwitchOn);
    }

    if (text == "/ex_off") {
      bot.sendMessage(chat_id, "trying to turn off exhaustion", "");
      pushToTasmota(externalTasmotaExhaustUrlSwitchOff);
    }

    if (text == "/fog_on") {
      aromaActionQueue.push_back([&aroma] {
        aroma->enableFog(1);
        aroma->enableLed(1);
        aroma->setLedRgbValue(0, 255, 0);
      });
      bot.sendMessage(chat_id, "trying to turn on fog", "");
    }

    if (text == "/fog_off") {
      aromaActionQueue.push_back([&aroma] {
        aroma->enableFog(0);
        aroma->enableLed(1);
        aroma->setLedRgbValue(255, 0, 0);
      });
      bot.sendMessage(chat_id, "trying to turn off fog", "");
    }

    if (text == "/state") {
      String msg = tp->toList() + "\n" + npkSensor->toList(lastNpkData) + "\n" + aromaToList() + "\n\nExhaustion:\n" + tasmotaToList(lastTasmotaExhaustionState) + "\n\n" + lightToList() + "\n\n" + distanceToList() + "\n\n" + weightToList() + "\n\nauto humidity: " + String(autoHumidityEnabled);
      bot.sendMessage(chat_id, msg, "");
    }

    if (text == "/json") {
      bot.sendMessage(chat_id, getSystemJSON(), "");
    }

    if (text == "/push") {
      bot.sendMessage(chat_id, "pushing " + getSystemJSON() + " to " + String(restServerUrl), "");
      pushToRestServer();
    }

    if (text == "/start") {
      String welcome = "Welcome, " + from_name + ".\n";
      welcome += "You can use the following commands\n\n";
      welcome += "/state to get latest data as list \n";
      welcome += "/fog_on to turn on humidifier \n";
      welcome += "/fog_off to turn off humidifier \n";
      welcome += "/ex_on to turn on exhaustion \n";
      welcome += "/ex_off to turn off exhaustion \n";
      welcome += "/cam to get a picture \n";
      welcome += "/dist to measure distance to canopy \n";
      welcome += "/json to get latest data as JSON \n";
      welcome += "/push to push latest data to rest server \n";
      welcome += "/auto_on to turn on experimental auto humidity \n";
      welcome += "/auto_off to turn off experimental auto humidity \n";
      bot.sendMessage(chat_id, welcome, "");
    } else {
      bot.sendMessage(chat_id, commands, "");
    }
  }
}

void setup() {
  esp_task_wdt_init(480, false);
  Serial.begin(115200);
  WiFi.setAutoConnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  uint8_t mac[6];
  WiFi.macAddress(mac);
  for (uint8_t i = 0; i < 6; i++) {
    mac[i] ^= 0xBF;
  }
  sprintf(restToken, "%02x%02x%02x%02x%02x%02x\0", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.printf("token for rest %s\r\n", restToken);

  client.setCACert(TELEGRAM_CERTIFICATE_ROOT);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println(WiFi.localIP());
  wifiLastSeen = millis();

  npkSensor = new NPKSensor(1, rxPin, txPin, sensorType);

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  tp = new ThermoPro(externalTempHumidAddress);
  scannableBleDevices.push_back(tp);
  pBLEScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallback());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(40);
  pBLEScan->setWindow(30);

  bleClientAroma = BLEDevice::createClient();
  bleClientLight = BLEDevice::createClient();
  bleClientDistance = BLEDevice::createClient();
  bleClientWeight = BLEDevice::createClient();

  aroma = new Aroma(bleClientAroma, externalHumidControlAddress, [](BLERemoteCharacteristic * pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
    aroma_response_t* response = (aroma_response_t*)pData;
    switch (response->type) {
      case stateFog: {
          memcpy(&lastFogResponse, response, sizeof(aroma_response_t));
          if (response->state) {
            Serial.println("fog(on)");
          } else {
            Serial.println("fog(off)");
          }
        } break;
      case stateLed: {
          memcpy(&lastLedResponse, response, sizeof(aroma_response_t));
          if (response->state) {
            Serial.print("led(on), ");
          } else {
            Serial.print("led(off), ");
          }
          aroma_rgb_t* rgb = (aroma_rgb_t*)response->data;
          Serial.printf("rgb(%u, %u, %u)", rgb->r, rgb->g, rgb->b);
          Serial.println();
        }
    }
  });

  light = new FC3000(bleClientLight, externalLightAddress, [](BLERemoteCharacteristic * pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
    char data[length + 1];
    data[length] = '\0';
    memcpy(data, pData, length);
    char* cmdStart = strstr(data, "cmd");
    if (cmdStart != NULL) {//no status if timer disabled?
      cmdStart += 4;
      int state, dimmPercentage, dc1, dc2;
      unsigned long localTime;
      unsigned long dc3;
      sscanf(cmdStart, "%d|%d|%d|%d|%ld|%lu", &state, &dimmPercentage, &dc1, &dc2, &localTime, &dc3);
      lastLightState.state = state;
      lastLightState.dimmPercentage = dimmPercentage;
      lastLightState.localTime = localTime;
      lastLightState.lastSeenMs = millis() - lastLightState.lastSeenMs;
      Serial.printf("state(%u), dimmPercentage(%u), localUnixTime(%u)\r\n", state, dimmPercentage, localTime);
    } else {
      Serial.println(data);
    }
  });

  distance = new RbBleDevice(bleClientDistance, externalDistanceAddress, [](BLERemoteCharacteristic * pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
    DeserializationError error = deserializeJson(doc, pData, length);
    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      return;
    }

    uint16_t pan = doc["pan"];
    uint16_t tilt = doc["tilt"];
    uint32_t dist = doc["dist"];
    lastDistanceState.pan = pan;
    lastDistanceState.tilt = tilt;
    lastDistanceState.distance = dist;
    lastDistanceState.lastSeenMs = millis() - lastDistanceState.lastSeenMs;
    Serial.printf("pan(%u), tilt(%u), dist(%u)\r\n", pan, tilt, dist);
  });

  weight = new RbBleDevice(bleClientWeight, externalWeightAddress, [](BLERemoteCharacteristic * pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
    DeserializationError error = deserializeJson(doc, pData, length);
    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      return;
    }

    float weight = doc["weight"];
    lastWeightState.weight = weight;
    lastWeightState.lastSeenMs = millis() - lastWeightState.lastSeenMs;
    Serial.printf("weight(%f)\r\n", weight);
  });

  /*if (aroma->connectToDevice() != 0) {
    Serial.println("could not connect to aroma");
  } else {
    Serial.println("connected to aroma");
  }

  if (light->connectToDevice() != 0) {
    Serial.println("could not connect to light");
  } else {
    Serial.println("connected to light");
  }

  if (distance->connectToDevice() != 0) {
    Serial.println("could not connect to distance");
  } else {
    Serial.println("connected to distance");
  }

  if (weight->connectToDevice() != 0) {
    Serial.println("could not connect to weight");
  } else {
    Serial.println("connected to weight");
  }

  npkSensor->update(lastNpkData);*/
  Serial.flush();
}

void loop() {
  if (millis() > lastTimeBotRan + botRequestDelay)  {
    if (millis() - wifiLastSeen > 5 * 1000 * 60) {
      ESP.restart();
    }
    if (WiFi.status() == WL_CONNECTED) {
      wifiLastSeen = millis();
      /*if (!npkSensor->update(lastNpkData)) {
        Serial.println(npkSensor->toJSON(lastNpkData));
      }*/

      int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

      while (numNewMessages) {
        handleNewMessages(numNewMessages);
        numNewMessages = bot.getUpdates(bot.last_message_received + 1);
      }
      pullFromTasmota(lastTasmotaExhaustionState, externalTasmotaExhaustUrlStatusDevice, externalTasmotaExhaustUrlStatusSensors);
      pullFromTasmota(lastTasmotaLightState, externalTasmotaLightUrlStatusDevice, externalTasmotaLightUrlStatusSensors);
      lastTimeBotRan = millis();
    } else {
      WiFi.begin(ssid, password);
    }
  } else if (millis() - lastBleScan > 30000) {
    return;
    BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
    pBLEScan->clearResults();
    lastBleScan = millis();
    if (!aroma->isConnected()) {
      aroma->connectToDevice();
    }
    if (aroma->isConnected()) {
      for (std::function<void()> f : aromaActionQueue) {
        f();
      }
      aromaActionQueue.clear();
      aroma->queryFogStatus();
      aroma->queryLedStatus();
    }
    if (autoHumidityEnabled) {
      if (tp->getHumidity() > 1) {
        naiveControlLoop(humidityTargetValue, tp->getHumidity());
      }
    }
    if (!light->isConnected()) {
      if (light->connectToDevice() != 0) {
        Serial.println("could not connect to light");
      } else {
        Serial.println("connected to light");
      }
    }
    if (!distance->isConnected()) {
      if (distance->connectToDevice() != 0) {
        Serial.println("could not connect to distance");
      } else {
        Serial.println("connected to distance");
      }
      if(distance->isConnected()){
        for (std::function<void()> f : distanceActionQueue) {
          f();
        }
        distanceActionQueue.clear();
      }
    }
    if (!weight->isConnected()) {
      if (weight->connectToDevice() != 0) {
        Serial.println("could not connect to weight");
      } else {
        Serial.println("connected to weight");
      }
    }
  } else if (millis() % 300000 == 0) {
    return;
    pushToRestServer();
  }
}
