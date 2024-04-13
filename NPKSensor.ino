#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>

#include "NPKSensor.h"
#include "creds.h"

WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);

int botRequestDelay = 1000;
unsigned long lastTimeBotRan;

const uint8_t rxPin = 23;
const uint8_t txPin = 19;

NPKSensor* npkSensor = 0;
npk_data_t lastNpkData;

void handleNewMessages(int numNewMessages) {
  Serial.println("handleNewMessages");
  Serial.println(String(numNewMessages));

  for (int i=0; i<numNewMessages; i++) {
    String chat_id = String(bot.messages[i].chat_id);
    if (chat_id != CHAT_ID){
      bot.sendMessage(chat_id, "Unauthorized user", "");
      continue;
    }
    
    String text = bot.messages[i].text;
    Serial.println(text);

    String from_name = bot.messages[i].from_name;

    if (text == "/start") {
      String welcome = "Welcome, " + from_name + ".\n";
      welcome += "You can use the following commands\n\n";
      welcome += "/state to get latest data as list \n";
      welcome += "/json to get latest data as JSON \n";
      bot.sendMessage(chat_id, welcome, "");
    }
    
    if (text == "/state") {
      bot.sendMessage(chat_id, npkSensor->toList(lastNpkData), "");
    }

    if (text == "/json") {
      bot.sendMessage(chat_id, npkSensor->toJSON(lastNpkData), "");
    }
  }
}

void setup() {
  Serial.begin(115200);

  Serial.begin(115200);
  npkSensor = new NPKSensor(1, rxPin, txPin);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  client.setCACert(TELEGRAM_CERTIFICATE_ROOT);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
}

void loop() {
  if (millis() > lastTimeBotRan + botRequestDelay)  {
    if(!npkSensor->update(lastNpkData)){
      Serial.println(npkSensor->toJSON(lastNpkData));
    }
    
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

    while(numNewMessages) {
      Serial.println("got response");
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    lastTimeBotRan = millis();
  }
}
