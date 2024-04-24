#pragma once

#include <Arduino.h>
#include <stdint.h>

typedef enum{
    SENSORTYPE7 = 7,
    SENSORTYPE8 = 8
} sensor_t;

#pragma pack(push,1)
typedef struct {
  uint8_t address;
  uint8_t function;
  uint8_t startRegisterHigh;
  uint8_t startRegisterLow;
  uint8_t registerLengthHigh;
  uint8_t registerLengthLow;
  uint8_t crcLow;
  uint8_t crcHigh;
} npk_request_t;

typedef struct {
  uint8_t address;
  uint8_t function;
  uint8_t dataLength;
  uint8_t reg0DataHigh;
  uint8_t reg0DataLow;
  uint8_t reg1DataHigh;
  uint8_t reg1DataLow;
  uint8_t reg2DataHigh;
  uint8_t reg2DataLow;
  uint8_t reg3DataHigh;
  uint8_t reg3DataLow;
  uint8_t reg4DataHigh;
  uint8_t reg4DataLow;
  uint8_t reg5DataHigh;
  uint8_t reg5DataLow;
  uint8_t reg6DataHigh;
  uint8_t reg6DataLow;
  uint8_t crcLow;
  uint8_t crcHigh;
} npk7_response_t;

typedef struct {
  uint8_t address;
  uint8_t function;
  uint8_t dataLength;
  uint8_t reg0DataHigh;
  uint8_t reg0DataLow;
  uint8_t reg1DataHigh;
  uint8_t reg1DataLow;
  uint8_t reg2DataHigh;
  uint8_t reg2DataLow;
  uint8_t reg3DataHigh;
  uint8_t reg3DataLow;
  uint8_t reg4DataHigh;
  uint8_t reg4DataLow;
  uint8_t reg5DataHigh;
  uint8_t reg5DataLow;
  uint8_t reg6DataHigh;
  uint8_t reg6DataLow;
  uint8_t reg7DataHigh;
  uint8_t reg7DataLow;
  uint8_t crcLow;
  uint8_t crcHigh;
} npk8_response_t;
#pragma push(pop)

typedef struct {
  float soilTemperature;          // Soil temperature in °C
  float soilMoisture;             // Soil moisture percentage
  int soilSalinity;               // Soil salinity value
  int soilConductivity;           // Soil conductivity value in µS/cm
  float pH;                       // pH value
  int soilNitrogenContent;        // Soil nitrogen content in ppm (mg/kg)
  int soilPhosphorus;             // Soil phosphorus content in ppm (mg/kg)
  int soilPotassiumContent;       // Soil potassium content in ppm (mg/kg)
} npk_data_t;

class NPKSensor {
  protected:
    npk_request_t npkRequest = {0x01, 0x03, 0x00, 0x00, 0x00, 0x08, 0x44, 0x0c};
    uint8_t pinRx;
    uint8_t pinTx;
    sensor_t type;
  public:
    NPKSensor(const uint8_t& address, const uint8_t& pinRx, const uint8_t& pinTx, const sensor_t type);
    virtual ~NPKSensor() {}
    int8_t update(npk_data_t& npkData);
    String toJSON(const npk_data_t& npkData);
    String toList(const npk_data_t& npkData);
};
