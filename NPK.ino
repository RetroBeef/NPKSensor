#include "NPKSensor.h"

const uint8_t rxPin = 23;
const uint8_t txPin = 19;

NPKSensor* npkSensor = 0;
npk_data_t lastNpkData;

void setup() {
  Serial.begin(115200);
  npkSensor = new NPKSensor(1, rxPin, txPin);
}

void loop() {
  if(!npkSensor->update(lastNpkData)){
    Serial.println(npkSensor->toJSON(lastNpkData));
  }
  delay(2000);
}
