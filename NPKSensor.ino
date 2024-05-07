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

#include <vector>

#include "NPKSensor.h"
#include "creds.h"

uint8_t scanTime = 5;
BLEScan* pBLEScan = 0;
BLEClient* bleClient = 0;
Aroma* aroma = 0;

char restToken[25];

std::vector<ScannableBleDevice*> scannableBleDevices;
ThermoPro* tp = 0;

std::vector<std::function<void()>> aromaActionQueue;

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

String aromaToList() {
  aroma_rgb_t* rgb = (aroma_rgb_t*)lastLedResponse.data;
  return "Humidifier:\nFog: " + String(lastFogResponse.state) + "\nLed: " + String(lastLedResponse.state) + " " + String(rgb->r) + " " + String(rgb->g) + " " + String(rgb->b);
}

String aromaToJSON() {
  aroma_rgb_t* rgb = (aroma_rgb_t*)lastLedResponse.data;
  return "{\"aroma550\":{\"fog\":" +
         String(lastFogResponse.state) +
         ",\"led\":{\"state\":" +
         String(lastLedResponse.state) +
         ",\"color\":{\"r\":" + String(rgb->r) +
         ",\"g\":" + String(rgb->g) +
         ",\"b\":" + String(rgb->b) +
         "}}}}";
}

class AdvertisedDeviceCallback: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      for (ScannableBleDevice* d : scannableBleDevices) {
        d->getScanCallback()(advertisedDevice);
      }
    }
};

String getSystemJSON() {
  return "{\"air\":" + tp->toJSON() + ",\"soil\":" + npkSensor->toJSON(lastNpkData) + ",\"humidifier\":" + aromaToJSON() + "}";
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

String commands = "commands: /start /state /fog_on /fog_off /cam /json /push";

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
      String msg = tp->toList() + "\n" + npkSensor->toList(lastNpkData) + "\n" + aromaToList();
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
      welcome += "/cam to get a picture \n";
      welcome += "/json to get latest data as JSON \n";
      welcome += "/push to push latest data to rest server \n";
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

  bleClient = BLEDevice::createClient();

  aroma = new Aroma(bleClient, externalHumidControlAddress, [](BLERemoteCharacteristic * pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
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
  aroma->connectToDevice();

  npkSensor->update(lastNpkData);
}

void loop() {
  if (millis() > lastTimeBotRan + botRequestDelay)  {
    if (millis() - wifiLastSeen > 5 * 1000 * 60) {
      ESP.restart();
    }
    if (WiFi.status() == WL_CONNECTED) {
      wifiLastSeen = millis();
      if (!npkSensor->update(lastNpkData)) {
        Serial.println(npkSensor->toJSON(lastNpkData));
      }

      int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

      while (numNewMessages) {
        handleNewMessages(numNewMessages);
        numNewMessages = bot.getUpdates(bot.last_message_received + 1);
      }
      lastTimeBotRan = millis();
    } else {
      WiFi.begin(ssid, password);
    }
  } else if (millis() - lastBleScan > 30000) {
    BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
    pBLEScan->clearResults();
    Serial.println(tp->toList());
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
  }else if(millis() % 300000 == 0){
    pushToRestServer();
  }
}
