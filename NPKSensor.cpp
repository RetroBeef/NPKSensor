#include "NPKSensor.h"
#include "crc.h"

namespace{
  void interpret_npk8_response(const npk8_response_t *res, npk_data_t *interp) {
      interp->soilTemperature = (int16_t)((res->reg0DataHigh << 8) | res->reg0DataLow) / 10.0;
      interp->soilMoisture = ((res->reg1DataHigh << 8) | res->reg1DataLow) / 10.0;
      interp->soilSalinity = (int16_t)((res->reg2DataHigh << 8) | res->reg2DataLow);
      interp->soilConductivity = (int16_t)((res->reg3DataHigh << 8) | res->reg3DataLow);
      interp->pH = (int16_t)((res->reg4DataHigh << 8) | res->reg4DataLow) / 100.0;
      interp->soilNitrogenContent = (int16_t)((res->reg5DataHigh << 8) | res->reg5DataLow);
      interp->soilPhosphorus = (int16_t)((res->reg6DataHigh << 8) | res->reg6DataLow);
      interp->soilPotassiumContent = (int16_t)((res->reg7DataHigh << 8) | res->reg7DataLow);
  } 

  void interpret_npk7_response(const npk7_response_t *res, npk_data_t *interp) {
      interp->soilTemperature = (int16_t)((res->reg0DataHigh << 8) | res->reg0DataLow) / 10.0;
      interp->soilMoisture = ((res->reg1DataHigh << 8) | res->reg1DataLow) / 10.0;
      interp->soilSalinity = 0;//todo
      interp->soilConductivity = (int16_t)((res->reg2DataHigh << 8) | res->reg2DataLow);
      interp->pH = (int16_t)((res->reg3DataHigh << 8) | res->reg3DataLow) / 100.0;
      interp->soilNitrogenContent = (int16_t)((res->reg4DataHigh << 8) | res->reg4DataLow);
      interp->soilPhosphorus = (int16_t)((res->reg5DataHigh << 8) | res->reg5DataLow);
      interp->soilPotassiumContent = (int16_t)((res->reg6DataHigh << 8) | res->reg6DataLow);
  } 

  String getRatios(const uint16_t& n, const uint16_t& p, const uint16_t& k) {
      // Find the smallest value among n, p, and k
      float smallest = min(min(n, p), k);
      
      // Calculate ratios based on the smallest value
      float ratio_n = (float)n / smallest;
      float ratio_p = (float)p / smallest;
      float ratio_k = (float)k / smallest;
  
      // Round the ratios to the nearest integers
      uint8_t rounded_n = round(ratio_n);
      uint8_t rounded_p = round(ratio_p);
      uint8_t rounded_k = round(ratio_k);
  
      // Create the ratio string
      String ratioString = String(rounded_n) + ":" + String(rounded_p) + ":" + String(rounded_k);
  
      return ratioString;
  }
}

NPKSensor::NPKSensor(const uint8_t& address, const uint8_t& pinRx, const uint8_t& pinTx, const sensor_t type) : pinRx(pinRx), pinTx(pinTx), type(type){
  npkRequest.address = address;
  npkRequest.registerLengthLow = (uint8_t)type;
  if(type==SENSORTYPE7){
    npkRequest.crcLow = 0x04;
    npkRequest.crcHigh = 0x08;
  }
  Serial1.begin(9600, SERIAL_8N1, pinRx, pinTx);
}

int8_t NPKSensor::update(npk_data_t& npkData){
  size_t response_data_size = sizeof(npk8_response_t);
  if(type==SENSORTYPE7) response_data_size = sizeof(npk7_response_t);
  uint8_t response_data[response_data_size];
  uint16_t expected_crc, received_crc;
  Serial1.write((uint8_t*)&npkRequest, sizeof(npk_request_t));
  ssize_t total_bytes_read = 0;
  while (total_bytes_read < response_data_size) {
    ssize_t bytes_read = Serial1.read(response_data + total_bytes_read, response_data_size - total_bytes_read);
    if (bytes_read < 0) {
      Serial.println("Error reading from serial port");
      return -1;
    }
    total_bytes_read += bytes_read;
  }

  // Check CRC16
  expected_crc = (response_data[response_data_size - 2] << 8) | response_data[response_data_size - 1];
  received_crc = __builtin_bswap16(calculate_crc16(response_data, response_data_size - 2));

  if (expected_crc != received_crc) {
    Serial.printf("CRC16 check failed! Expected: 0x%04X, Received: 0x%04X\r\n", expected_crc, received_crc);
    return -1;
  }
  if(type==SENSORTYPE7){
    interpret_npk7_response((npk7_response_t*)response_data, &npkData);
  }else{
    interpret_npk8_response((npk8_response_t*)response_data, &npkData);
  }
  
  return 0;
}

String NPKSensor::toJSON(const npk_data_t& npkData) {
    String jsonStr = "{";
    jsonStr += "\"add\":" + String(npkRequest.address) + ",";
    jsonStr += "\"fun\":" + String(npkRequest.function) + ",";
    jsonStr += "\"soil\":{";
    jsonStr += "\"temp\":{\"v\":" + String(npkData.soilTemperature, 1) + ",\"u\":\"°C\"},";
    jsonStr += "\"hum\":{\"v\":" + String(npkData.soilMoisture, 1) + ",\"u\":\"% rH\"},";
    if(type==SENSORTYPE8)jsonStr += "\"sal\":{\"v\":" + String(npkData.soilSalinity) + ",\"u\":\"µS/cm\"},";
    jsonStr += "\"EC\":{\"v\":" + String(npkData.soilConductivity) + ",\"u\":\"µS/cm\"},";
    jsonStr += "\"pH\":{\"v\":" + String(npkData.pH, 1) + ",\"u\":\"-log(H+)\"},";
    jsonStr += "\"N\":{\"v\":" + String(npkData.soilNitrogenContent) + ",\"u\":\"mg/kg\"},";
    jsonStr += "\"P\":{\"v\":" + String(npkData.soilPhosphorus) + ",\"u\":\"mg/kg\"},";
    jsonStr += "\"K\":{\"v\":" + String(npkData.soilPotassiumContent) + ",\"u\":\"mg/kg\"}";
    jsonStr += "}}";

    return jsonStr;
}

String NPKSensor::toList(const npk_data_t& npkData) {
    String str = "Soil:\n";
    str += "Temperature: " + String(npkData.soilTemperature, 1) + " °C\n";
    str += "Humidity: " + String(npkData.soilMoisture, 1) + "% rH\n";
    if(type==SENSORTYPE8)str += "Salinity: " + String(npkData.soilSalinity) + " µS/cm\n";
    str += "Electrical Conductivity: " + String(npkData.soilConductivity) + " µS/cm\n";
    str += "pH: " + String(npkData.pH, 1) + " -log(H+)\n";
    str += "Nitrogen(N): " + String(npkData.soilNitrogenContent) + " mg/kg\n";
    str += "Phosphorus(P): " + String(npkData.soilPhosphorus) + " mg/kg\n";
    str += "Potassium(K): " + String(npkData.soilPotassiumContent) + " mg/kg\n";
    str += "NPK Ratios: " + getRatios(npkData.soilNitrogenContent, npkData.soilPhosphorus, npkData.soilPotassiumContent) + "\n";
    return str;
}
