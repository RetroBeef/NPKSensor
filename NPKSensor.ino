#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#include "NPKSensor.h"
#include "creds.h"

#pragma pack(push,1)
typedef struct {
  uint8_t unk1;
  uint8_t temperature;
  uint8_t unk2;
  uint8_t humidity;
  uint8_t unk3;
  uint8_t unk4;
} ble_temphumid_t;
#pragma pack(pop)
const uint8_t offsetInPayload = 19;
const uint8_t expectedPayloadSize = 39;

uint8_t scanTime = 5;
BLEScan* pBLEScan;
BLEAddress extTempHumid(externalTempHumidAddress);

WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);

SemaphoreHandle_t wifi_ble_mutex = NULL;

int botRequestDelay = 1000;
unsigned long lastTimeBotRan;

const uint8_t rxPin = 23;//5;
const uint8_t txPin = 19;//6;

sensor_t sensorType = SENSORTYPE8;//SENSORTYPE7;

NPKSensor* npkSensor = 0;
npk_data_t npkTestWarn = {6, 6, 1200, 1500, 2.5f, 100, 100, 160};
npk_data_t lastNpkDefaultCalibrated = {20, 20, 3200, 2500, 5.5f, 270, 200, 360};
npk_data_t lastNpkData;
npk_data_t lastCalibData;

float lastAirTemperature = 0.f;
uint8_t lastAirHumidity = 0;

// Variables for tracking last message time for each value
unsigned long lastHourlyMessageTimeSoilTemperature = 0;
unsigned long lastHourlyMessageTimeSoilMoisture = 0;
unsigned long lastHourlyMessageTimeSoilSalinity = 0;
unsigned long lastHourlyMessageTimeSoilConductivity = 0;
unsigned long lastHourlyMessageTimePH = 0;
unsigned long lastHourlyMessageTimeSoilNitrogenContent = 0;
unsigned long lastHourlyMessageTimeSoilPhosphorus = 0;
unsigned long lastHourlyMessageTimeSoilPotassiumContent = 0;
const unsigned long hourlyMessageInterval = 3600000; // 1 hour in milliseconds

class advertisedDeviceCallback: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      if (advertisedDevice.getAddress() == extTempHumid) {
        if (advertisedDevice.getPayloadLength() == expectedPayloadSize) {
          ble_temphumid_t* tempHumidData = (ble_temphumid_t*)(advertisedDevice.getPayload() + offsetInPayload);
          lastAirTemperature = tempHumidData->temperature / 10.f;
          lastAirHumidity = tempHumidData->humidity;
        }
      }
    }
};

void checkForDifferences(const npk_data_t *data1, const npk_data_t *data2, unsigned long lastCheckMs = 0) {
  // Get current time
  unsigned long currentTime = lastCheckMs ? lastCheckMs : millis();

  // Check if enough time has passed since the last message for each value
  if (currentTime - lastHourlyMessageTimeSoilTemperature >= hourlyMessageInterval) {
    if (fabs(data1->soilTemperature - data2->soilTemperature) / data1->soilTemperature > 0.3) {
      bot.sendMessage(CHAT_ID, "WARNING: >30% Difference in soilTemperature!", "");
    }
    lastHourlyMessageTimeSoilTemperature = currentTime;
  }

  if (currentTime - lastHourlyMessageTimeSoilMoisture >= hourlyMessageInterval) {
    if (fabs(data1->soilMoisture - data2->soilMoisture) / data1->soilMoisture > 0.3) {
      bot.sendMessage(CHAT_ID, "WARNING: >30% difference in moisture!", "");
    }
    lastHourlyMessageTimeSoilMoisture = currentTime;
  }

  if (currentTime - lastHourlyMessageTimeSoilSalinity >= hourlyMessageInterval && sensorType >= SENSORTYPE8) {
    if (fabs(data1->soilSalinity - data2->soilSalinity) / data1->soilSalinity > 0.3) {
      bot.sendMessage(CHAT_ID, "WARNING: >30% difference in salinity!", "");
    }
    lastHourlyMessageTimeSoilSalinity = currentTime;
  }

  if (currentTime - lastHourlyMessageTimeSoilConductivity >= hourlyMessageInterval) {
    if (fabs(data1->soilConductivity - data2->soilConductivity) / data1->soilConductivity > 0.3) {
      bot.sendMessage(CHAT_ID, "WARNING: >30% difference in conductivity!", "");
    }
    lastHourlyMessageTimeSoilConductivity = currentTime;
  }

  if (currentTime - lastHourlyMessageTimePH >= hourlyMessageInterval) {
    if (fabs(data1->pH - data2->pH) / data1->pH > 0.1) {
      bot.sendMessage(CHAT_ID, "WARNING: >10% difference in pH!", "");
    }
    lastHourlyMessageTimePH = currentTime;
  }

  if (currentTime - lastHourlyMessageTimeSoilNitrogenContent >= hourlyMessageInterval) {
    if (fabs(data1->soilNitrogenContent - data2->soilNitrogenContent) / data1->soilNitrogenContent > 0.3) {
      bot.sendMessage(CHAT_ID, "WARNING: >30% difference in Nitrogen!", "");
    }
    lastHourlyMessageTimeSoilNitrogenContent = currentTime;
  }

  if (currentTime - lastHourlyMessageTimeSoilPhosphorus >= hourlyMessageInterval) {
    if (fabs(data1->soilPhosphorus - data2->soilPhosphorus) / data1->soilPhosphorus > 0.3) {
      bot.sendMessage(CHAT_ID, "WARNING: >30% difference in Phosphorus!", "");
    }
    lastHourlyMessageTimeSoilPhosphorus = currentTime;
  }

  if (currentTime - lastHourlyMessageTimeSoilPotassiumContent >= hourlyMessageInterval) {
    if (fabs(data1->soilPotassiumContent - data2->soilPotassiumContent) / data1->soilPotassiumContent > 0.3) {
      bot.sendMessage(CHAT_ID, "WARNING: >30% difference in Potassium!", "");
    }
    lastHourlyMessageTimeSoilPotassiumContent = currentTime;
  }
}

String commands = "commands: /start /calib /state /json /testwarn";

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

    if (text == "/calib") {
      String msg = "Air:\nTemperature: " + String(lastAirTemperature, 2)  + "°C\nHumidity" + String(lastAirHumidity) + "%rH\n\nSoil:\n" +  npkSensor->toList(lastCalibData);
      bot.sendMessage(chat_id, msg, "");
    }

    if (text == "/state") {
      String msg = "Air:\nTemperature: " + String(lastAirTemperature, 2)  + "°C\nHumidity" + String(lastAirHumidity) + "%rH\n\nSoil:\n" +  npkSensor->toList(lastNpkData);
      bot.sendMessage(chat_id, msg, "");
    }

    if (text == "/json") {
      bot.sendMessage(chat_id, npkSensor->toJSON(lastNpkData), "");
    }

    if (text == "/testwarn") {
      bot.sendMessage(chat_id, "initiated warning test procedure", "");
      lastHourlyMessageTimeSoilTemperature = 0;
      lastHourlyMessageTimeSoilMoisture = 0;
      lastHourlyMessageTimeSoilSalinity = 0;
      lastHourlyMessageTimeSoilConductivity = 0;
      lastHourlyMessageTimePH = 0;
      lastHourlyMessageTimeSoilNitrogenContent = 0;
      lastHourlyMessageTimeSoilPhosphorus = 0;
      lastHourlyMessageTimeSoilPotassiumContent = 0;
      checkForDifferences(&npkTestWarn, &lastNpkDefaultCalibrated, hourlyMessageInterval + 1);
    }

    if (text == "/start") {
      String welcome = "Welcome, " + from_name + ".\n";
      welcome += "You can use the following commands\n\n";
      welcome += "/calib to get latest calibrated data as list \n";
      welcome += "/state to get latest data as list \n";
      welcome += "/json to get latest data as JSON \n";
      welcome += "/testwarn to test the warning feature \n";
      bot.sendMessage(chat_id, welcome, "");
    }else{
      bot.sendMessage(chat_id, commands, "");
    }
  }
}

void calib() {
  lastCalibData.soilSalinity *= 10;
  lastCalibData.soilConductivity *= 10;
  lastCalibData.soilNitrogenContent *= 10;
  lastCalibData.soilPhosphorus *= 10;
  lastCalibData.soilPotassiumContent *= 10;

  lastCalibData.soilPhosphorus /= 1.8f;
  lastCalibData.soilPotassiumContent /= 2.528f;

  lastCalibData.soilSalinity -= 1000;
  lastCalibData.soilConductivity -= 1000;
  lastCalibData.pH += 1;
  lastCalibData.soilNitrogenContent += 40;
  lastCalibData.soilPhosphorus += 30;
  lastCalibData.soilPotassiumContent += 50;
}

void ble_task(void *parameter) {
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new advertisedDeviceCallback());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(40);
  pBLEScan->setWindow(30);
  while (1) {
    if (xSemaphoreTake(wifi_ble_mutex, portMAX_DELAY) == pdTRUE) {
      WiFi.mode(WIFI_OFF);
      btStart();
      BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
      pBLEScan->clearResults();
      BLEDevice::deinit(true);
      btStop();
      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid, password);
      xSemaphoreGive(wifi_ble_mutex);
      delay(10000);
    }
  }

  vTaskDelete(NULL);
}

void setup() {
  Serial.begin(115200);
  npkSensor = new NPKSensor(1, rxPin, txPin, sensorType);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  client.setCACert(TELEGRAM_CERTIFICATE_ROOT);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  wifi_ble_mutex = xSemaphoreCreateMutex();
  
  xTaskCreate(ble_task, "ble_task", 10240, NULL, 1, NULL);
  npkSensor->update(lastNpkData);
}

void loop() {
  if (millis() > lastTimeBotRan + botRequestDelay)  {
    if (xSemaphoreTake(wifi_ble_mutex, portMAX_DELAY) == pdTRUE) {
      if (!npkSensor->update(lastNpkData)) {
        memcpy(&lastCalibData, &lastNpkData, sizeof(npk_data_t));
        calib();
        Serial.println(npkSensor->toJSON(lastNpkData));
        //Serial.println(npkSensor->toJSON(lastCalibData));
        checkForDifferences(&lastCalibData, &lastNpkDefaultCalibrated);
      }
  
      int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
  
      while (numNewMessages) {
        handleNewMessages(numNewMessages);
        numNewMessages = bot.getUpdates(bot.last_message_received + 1);
      }
      lastTimeBotRan = millis();
      xSemaphoreGive(wifi_ble_mutex); 
    }
  }
}
